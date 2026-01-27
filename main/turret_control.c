#include "turret_control.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"     // esp_rom_delay_us

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "pd_runner.h"       // pd_get_last()

static const char *TAG = "TURRET";

// =======================
// Config / Estado global
// =======================
static turret_pins_t s_pins;
static turret_cfg_t  s_cfg;

static TaskHandle_t  s_task = NULL;
static bool          s_running = false;

// Stepper state
static int32_t s_pos_steps = 0;
static int     s_scan_dir = +1;
// Bloqueo temporal de dirección cuando se golpea un endstop
static int64_t s_block_left_until_ms  = 0;   // no permitir mover a la izquierda hasta este tiempo
static int64_t s_block_right_until_ms = 0;   // no permitir mover a la derecha hasta este tiempo

// Servo state
static int     s_servo_deg = 90;

// Tracking time
static int64_t s_last_seen_ms = 0;

// Frame size real (set desde simple_video_server_example.c)
static int s_frame_w = 640;
static int s_frame_h = 480;
// =======================
// Lock / anti-jitter
// =======================

// Zona para "considerar centrado" (más pequeña)
#define LOCK_IN_X   0.02f
#define LOCK_IN_Y   0.02f

// Zona para "salir del lock" (más grande) -> histéresis
#define LOCK_OUT_X  0.05f
#define LOCK_OUT_Y  0.05f

// Tiempo mínimo centrado para bloquear (ms)
#define LOCK_STABLE_MS  250

// Límite de movimiento por tick (suaviza)
#define SERVO_MAX_DEG_PER_TICK   3    // prueba 2..5
#define STEPPER_MAX_STEPS_TICK   30   // extra (aparte de step_max)

// =======================================
// Endstops: NO a 3V3 => presionado = 1
// (input pulldown, al presionar sube a 3V3)
// =======================================
#define ENDSTOP_PRESSED_LEVEL 1

// =======================
// Helpers de tiempo
// =======================
static inline int64_t now_ms(void)
{
    return (int64_t)(esp_timer_get_time() / 1000ULL);
}

// =======================
// Helpers Endstops (ACTIVE-HIGH)
// =======================
static inline bool endstop_left_hit(void)
{
    if (s_pins.limit_left_gpio < 0) return false;
    return gpio_get_level((gpio_num_t)s_pins.limit_left_gpio) == ENDSTOP_PRESSED_LEVEL;
}

static inline bool endstop_right_hit(void)
{
    if (s_pins.limit_right_gpio < 0) return false;
    return gpio_get_level((gpio_num_t)s_pins.limit_right_gpio) == ENDSTOP_PRESSED_LEVEL;
}

// =======================
// Stepper low-level
// =======================
static void step_pulse(uint32_t delay_us)
{
    gpio_set_level((gpio_num_t)s_pins.step_gpio, 1);
    esp_rom_delay_us(delay_us);
    gpio_set_level((gpio_num_t)s_pins.step_gpio, 0);
    esp_rom_delay_us(delay_us);
}

/**
 * Mueve el stepper `steps` pasos en dirección dir:
 * dir < 0: izquierda
 * dir > 0: derecha
 *
 * Protección: si el endstop correspondiente está presionado, no avanza a ese lado.
 */
static void stepper_move_steps(int dir, int steps)
{
    if (steps <= 0) return;

    int64_t t = now_ms();

    // ✅ Si ese lado está bloqueado temporalmente, no lo intentes
    if (dir < 0 && t < s_block_left_until_ms)  return;
    if (dir > 0 && t < s_block_right_until_ms) return;

    // ✅ Si el endstop está presionado, bloquea ese lado un rato y no avances hacia ahí
    if (dir < 0 && endstop_left_hit())  { s_block_left_until_ms  = t + 400; return; }
    if (dir > 0 && endstop_right_hit()) { s_block_right_until_ms = t + 400; return; }

    gpio_set_level((gpio_num_t)s_pins.dir_gpio, (dir > 0) ? 1 : 0);

    for (int i = 0; i < steps; i++) {

        // Si se activa durante el movimiento, bloquea y corta
        if (dir < 0 && endstop_left_hit())  { s_block_left_until_ms  = now_ms() + 400; break; }
        if (dir > 0 && endstop_right_hit()) { s_block_right_until_ms = now_ms() + 400; break; }

        step_pulse(s_cfg.step_delay_us);
        s_pos_steps += (dir > 0) ? 1 : -1;

        if (s_cfg.scan_limit_steps > 0) {
            if (s_pos_steps <= -s_cfg.scan_limit_steps) { s_scan_dir = +1; break; }
            if (s_pos_steps >=  s_cfg.scan_limit_steps) { s_scan_dir = -1; break; }
        }
    }
}


// =======================
// Servo (LEDC)
// =======================
static bool servo_init(int gpio)
{
    ledc_timer_config_t tcfg = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_16_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    if (ledc_timer_config(&tcfg) != ESP_OK) return false;

    ledc_channel_config_t ccfg = {
        .gpio_num       = gpio,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0,
        .intr_type      = LEDC_INTR_DISABLE
    };
    return ledc_channel_config(&ccfg) == ESP_OK;
}

/**
 * Mueve servo en grados (clamp con min/max).
 * Convierte grados a pulso típico 500..2500us (aprox 0..270°).
 */
static void servo_set_deg(int deg)
{
    if (deg < s_cfg.servo_min_deg) deg = s_cfg.servo_min_deg;
    if (deg > s_cfg.servo_max_deg) deg = s_cfg.servo_max_deg;
    s_servo_deg = deg;

    const int min_us = 500;
    const int max_us = 2500;

    // Asumiendo servo 0..270°
    int pulse_us = min_us + (deg * (max_us - min_us)) / 270;

    // Periodo 20ms => 20000us, duty 16-bit (0..65535)
    uint32_t duty = (uint32_t)((pulse_us * 65535ULL) / 20000ULL);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// =======================
// Tracking (PD)
// =======================
static bool pick_best_box(const pd_result_t *r, pd_box_t *best)
{
    if (!r || r->count <= 0) return false;

    int best_i = 0;
    int best_area = r->boxes[0].w * r->boxes[0].h;

    for (int i = 1; i < r->count; i++) {
        int area = r->boxes[i].w * r->boxes[i].h;
        if (area > best_area) {
            best_area = area;
            best_i = i;
        }
    }

    *best = r->boxes[best_i];
    return true;
}

/**
 * Lee PD, calcula error (ex, ey) y aplica:
 * - Stepper (pan) según ex
 * - Servo (tilt) según ey
 * - MOSFET: prende solo si está centrado estable con histéresis
 */
static void turret_track_from_pd(void)
{
    pd_result_t r;
    pd_get_last(&r);

    int64_t t = now_ms();

    // 1) si no hay detección => apaga MOSFET y no muevas
    if (r.count <= 0) {
        turret_mosfet_set(false);
        return;
    }

    // 2) descarta detección vieja
    if (s_cfg.valid_age_ms > 0 && (t - (int64_t)r.ts_ms) > (int64_t)s_cfg.valid_age_ms) {
        return;
    }

    // 3) escoger el box principal (mayor área)
    int best = 0;
    int best_area = r.boxes[0].w * r.boxes[0].h;
    for (int i = 1; i < r.count; i++) {
        int area = r.boxes[i].w * r.boxes[i].h;
        if (area > best_area) { best_area = area; best = i; }
    }

    pd_box_t b = r.boxes[best];

    // 4) centro del bbox en pixeles
    float cx = (float)(b.x + b.w * 0.5f);
    float cy = (float)(b.y + b.h * 0.5f);

    // 5) normalizar 0..1 con el frame real
    float W = (float)((s_frame_w > 0) ? s_frame_w : 640);
    float H = (float)((s_frame_h > 0) ? s_frame_h : 480);

    float nx = cx / W;  // 0..1
    float ny = cy / H;  // 0..1

    // 6) error respecto al centro (0.5,0.5)
    float ex = 0.5f - nx;  // + => está a la izquierda del centro
    float ey = 0.5f - ny;  // + => está arriba del centro
    
    // 7) deadzone (zona muerta para que no tiemble)
    const float DEAD_X = 0.03f;  // 3% ancho
    const float DEAD_Y = 0.03f;  // 3% alto

    // --------- MOSFET: solo ON cuando está centrado ---------
    const float ON_TOL  = 0.02f;
    const float OFF_TOL = 0.05f;
    static bool mosfet_on = false;
    static int64_t centered_since_ms = 0;

    float ax = fabsf(ex);
    float ay = fabsf(ey);
// --------- LOCK: si está centrado y estable, nos quedamos quietos ----------
static bool locked = false;
static int64_t lock_since_ms = 0;

bool in_lock_zone  = (ax < LOCK_IN_X)  && (ay < LOCK_IN_Y);
bool out_lock_zone = (ax > LOCK_OUT_X) || (ay > LOCK_OUT_Y);

if (!locked) {
    if (in_lock_zone) {
        if (lock_since_ms == 0) lock_since_ms = t;
        if ((t - lock_since_ms) >= LOCK_STABLE_MS) {
            locked = true;         // ✅ queda quieto
        }
    } else {
        lock_since_ms = 0;
    }
} else {
    // Si ya está locked, solo desbloquea si te sales bastante
    if (out_lock_zone) {
        locked = false;
        lock_since_ms = 0;
    }
}

// Si está locked => no mover stepper ni servo
if (locked) {
    s_last_seen_ms = t;
    return;
}

    bool in_on_zone   = (ax < ON_TOL) && (ay < ON_TOL);
    bool out_off_zone = (ax > OFF_TOL) || (ay > OFF_TOL);

    if (!mosfet_on) {
        if (in_on_zone) {
            if (centered_since_ms == 0) centered_since_ms = t;
            if ((t - centered_since_ms) >= 250) {
                mosfet_on = true;
                //turret_mosfet_set(true);
            }
        } else {
            centered_since_ms = 0;
        }
    } else {
        if (out_off_zone) {
            mosfet_on = false;
            centered_since_ms = 0;
            //turret_mosfet_set(false);
        }
    }

// --------- X => STEPPER ---------
if (ax > DEAD_X) {

    // tu convención: ex>0 => target a la izq => gira hacia izq
    int dir = (ex > 0) ? -1 : +1;

    int steps = (int)(ax * s_cfg.kp_step);

    if (steps < s_cfg.step_min) steps = s_cfg.step_min;
    if (steps > s_cfg.step_max) steps = s_cfg.step_max;

    // límite extra por tick (anti-locos)
    if (steps > STEPPER_MAX_STEPS_TICK) steps = STEPPER_MAX_STEPS_TICK;

    stepper_move_steps(dir, steps);
}

    // --------- Y => SERVO ---------
if (ay > DEAD_Y) {
    int delta = (int)(ey * s_cfg.kp_servo);

    // límite por tick (anti-saltos)
    if (delta >  SERVO_MAX_DEG_PER_TICK) delta =  SERVO_MAX_DEG_PER_TICK;
    if (delta < -SERVO_MAX_DEG_PER_TICK) delta = -SERVO_MAX_DEG_PER_TICK;

    // si el delta quedó muy chiquito, ni te muevas
    if (delta != 0) {
        int new_deg = s_servo_deg + delta;
        servo_set_deg(new_deg);
    }
}


    s_last_seen_ms = t;
}


// =======================
// Scan (sin target)
// =======================
static void turret_scan_tick(void)
{
    // Si está golpeado un endstop, cambia dirección
    if (endstop_left_hit())  s_scan_dir = +1;
    if (endstop_right_hit()) s_scan_dir = -1;

    stepper_move_steps(s_scan_dir, s_cfg.scan_steps_per_tick);
}

/**
 * Seguridad: si el motor DC empuja y pega un endstop,
 * apagamos MOSFET y nos movemos al lado contrario hasta liberar.
 */
static void turret_escape_from_limits(void)
{
    // Si pega el izquierdo: apagar DC + mover a la derecha hasta soltar
    if (endstop_left_hit()) {
        turret_mosfet_set(false);
        s_scan_dir = +1;                 // escaneo se va a la derecha
        s_block_left_until_ms = now_ms() + 800;  // bloquea volver a la izquierda

        // soltamos el switch con pasitos
        int guard = 2000; // por si acaso, para no quedarnos infinitos
        while (endstop_left_hit() && guard-- > 0) {
            stepper_move_steps(+1, 2);
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // backoff extra para separarnos
        stepper_move_steps(+1, 200);
        return;
    }

    // Si pega el derecho: apagar DC + mover a la izquierda hasta soltar
    if (endstop_right_hit()) {
        turret_mosfet_set(false);
        s_scan_dir = -1;
        s_block_right_until_ms = now_ms() + 800;

        int guard = 2000;
        while (endstop_right_hit() && guard-- > 0) {
            stepper_move_steps(-1, 2);
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        stepper_move_steps(-1, 200);
        return;
    }
}

// =======================
// Task principal
// =======================
static void turret_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "turret task start");

    s_last_seen_ms = now_ms();

    while (s_running) {

        // ✅ Siempre primero seguridad física
        turret_escape_from_limits();

        int64_t t = now_ms();

        // Tracking
        int64_t before = s_last_seen_ms;
        turret_track_from_pd();
        bool saw_recent = (s_last_seen_ms != before);

        // Si no vimos nada en esta iteración, y pasó el timeout => escanear
        if (!saw_recent) {
            if ((t - s_last_seen_ms) > (int64_t)s_cfg.lost_timeout_ms) {
                turret_scan_tick();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(s_cfg.scan_tick_ms));
    }

    ESP_LOGI(TAG, "turret task stop");
    s_task = NULL;
    vTaskDelete(NULL);
}

// =======================
// API pública
// =======================
bool turret_init(const turret_pins_t *pins, const turret_cfg_t *cfg)
{
    if (!pins || !cfg) return false;
    s_pins = *pins;
    s_cfg  = *cfg;

    // -----------------------
    // GPIO Outputs (step/dir/en)
    // -----------------------
    gpio_config_t out = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };

    out.pin_bit_mask |= (1ULL << s_pins.step_gpio);
    out.pin_bit_mask |= (1ULL << s_pins.dir_gpio);
    if (s_pins.en_gpio >= 0) out.pin_bit_mask |= (1ULL << s_pins.en_gpio);

    ESP_ERROR_CHECK(gpio_config(&out));
    gpio_set_level((gpio_num_t)s_pins.step_gpio, 0);
    gpio_set_level((gpio_num_t)s_pins.dir_gpio, 0);

    // enable activo LOW (DRV8825 típico)
    if (s_pins.en_gpio >= 0) {
        gpio_set_level((gpio_num_t)s_pins.en_gpio, 0);
    }

    // -----------------------
    // MOSFET output
    // -----------------------
    if (s_pins.mosfet_gpio >= 0) {
        gpio_reset_pin((gpio_num_t)s_pins.mosfet_gpio);
        gpio_set_direction((gpio_num_t)s_pins.mosfet_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)s_pins.mosfet_gpio, 0); // OFF por defecto
        gpio_set_drive_capability((gpio_num_t)s_pins.mosfet_gpio, GPIO_DRIVE_CAP_3);
    }

    // -----------------------
    // Microstepping pins (si existen)
    // -----------------------
    if (s_pins.m0_gpio >= 0) gpio_set_direction((gpio_num_t)s_pins.m0_gpio, GPIO_MODE_OUTPUT);
    if (s_pins.m1_gpio >= 0) gpio_set_direction((gpio_num_t)s_pins.m1_gpio, GPIO_MODE_OUTPUT);
    if (s_pins.m2_gpio >= 0) gpio_set_direction((gpio_num_t)s_pins.m2_gpio, GPIO_MODE_OUTPUT);

    // -----------------------
    // Endstops: INPUT + PULLDOWN
    // (NO a 3V3 => presionado = 1)
    // -----------------------
    gpio_config_t in = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 0,
        .pull_down_en = 1,
        .intr_type = GPIO_INTR_DISABLE
    };

    if (s_pins.limit_left_gpio >= 0)  in.pin_bit_mask |= (1ULL << s_pins.limit_left_gpio);
    if (s_pins.limit_right_gpio >= 0) in.pin_bit_mask |= (1ULL << s_pins.limit_right_gpio);

    if (in.pin_bit_mask) {
        ESP_ERROR_CHECK(gpio_config(&in));
    }

    // -----------------------
    // Servo init
    // -----------------------
    if (!servo_init(s_pins.servo_gpio)) {
        ESP_LOGE(TAG, "servo init failed");
        return false;
    }

    s_servo_deg = s_cfg.servo_center_deg;
    servo_set_deg(s_servo_deg);

    ESP_LOGI(TAG, "turret init ok (step=%d dir=%d servo=%d L=%d R=%d mosfet=%d)",
             s_pins.step_gpio, s_pins.dir_gpio, s_pins.servo_gpio,
             s_pins.limit_left_gpio, s_pins.limit_right_gpio, s_pins.mosfet_gpio);

    return true;
}

void turret_start(void)
{
    if (s_running) return;
    s_running = true;

    if (!s_task) {
        xTaskCreatePinnedToCore(turret_task, "turret_task", 4096, NULL, 6, &s_task, 1);
    }
}

void turret_stop(void)
{
    s_running = false;
}

// Ajuste de frame size real desde video server
void turret_set_frame_size(int w, int h)
{
    if (w > 0) s_frame_w = w;
    if (h > 0) s_frame_h = h;
}

void turret_get_frame_size(int *w, int *h)
{
    if (w) *w = s_frame_w;
    if (h) *h = s_frame_h;
}

// MOSFET control
void turret_mosfet_set(bool on)
{
    if (s_pins.mosfet_gpio < 0) return;
    gpio_set_level((gpio_num_t)s_pins.mosfet_gpio, on ? 1 : 0);
}

/**
 * API alternativa (si la usas desde otro lado):
 * nx, ny son normalizados 0..1
 */
void turret_update_target(bool has_target, float nx, float ny)
{
    const float cx = 0.5f;
    const float cy = 0.5f;
    const float center_tol = 0.02f;

    if (!has_target) {
        turret_mosfet_set(false);
        return;
    }

    float ex = nx - cx;
    float ey = ny - cy;

    float ax = fabsf(ex);
    float ay = fabsf(ey);

    bool centered = (ax < center_tol) && (ay < center_tol);
    turret_mosfet_set(centered);
}
