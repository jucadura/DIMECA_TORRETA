#include "turret_control.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "pd_runner.h"

static const char *TAG = "TURRET";

// =====================================================
// Config / Estado global
// =====================================================
static turret_pins_t s_pins;
static turret_cfg_t  s_cfg;

static TaskHandle_t  s_task = NULL;
static bool          s_running = false;

// Frame size real
static int s_frame_w = 640;
static int s_frame_h = 480;

// Stepper
static int32_t s_pos_steps = 0;
static int     s_scan_dir  = +1;

// Endstop blocks
static int64_t s_block_left_until_ms  = 0;
static int64_t s_block_right_until_ms = 0;

// Servo
static int s_servo_deg = 90;

// Tracking
static int64_t s_last_seen_ms = 0;
static int64_t s_track_lost_since = 0;

// Flags
static bool s_homing_done = false;

// =====================================================
// MOSFET (motor DC vibración) - GPIO 4
// =====================================================

// Pulsos (ajusta según vibración real)
#define MOSFET_ON_MS       120     // encendido corto
#define MOSFET_OFF_MS      280     // apagado para descansar
#define MOSFET_MAX_MS     1500     // máximo vibrando continuo mientras locked
#define MOSFET_COOLDOWN   1200     // descanso antes de volver a vibrar si sigue locked
#define HOMING_DELAY_US  250   // ejemplo: más rápido que 500

static bool    s_mosfet_state = false;
static int64_t s_mosfet_next_toggle_ms = 0;
static int64_t s_mosfet_lock_start_ms = 0; // >0 vibrando, <0 cooldown_until, 0 idle

// =====================================================
// Máquina de estados
// =====================================================
typedef enum {
    TSTATE_HOMING_FIND_FIRST = 0,
    TSTATE_HOMING_FIND_SECOND,
    TSTATE_HOMING_GO_CENTER,
    TSTATE_SEARCH_STEPPER,
    TSTATE_SEARCH_SERVO,
    TSTATE_CONFIRM,
    TSTATE_TRACK
} turret_state_t;

static turret_state_t s_state = TSTATE_HOMING_FIND_FIRST;

// Homing data
static int32_t s_range_steps = 0;
static int32_t s_steps_from_first = 0;
static int     s_first_is_left = -1;   // 1=LEFT, 0=RIGHT

// Search
static int32_t s_search_step_size = 0;
static int32_t s_search_remaining = 0;
static int     s_search_servo_phase = 0;

// Confirm
static int     s_confirm_hits = 0;
static int64_t s_confirm_start_ms = 0;

// Hold / settle
static int64_t s_hold_until_ms = 0;

// =====================================================
// Ajustes
// =====================================================
#define ENDSTOP_PRESSED_LEVEL 1

#define STEPPER_SETTLE_MS   60
#define SERVO_SETTLE_MS     180   // ✅ más lento (antes 120)

// Deadzones
#define DEAD_X  0.04f
#define DEAD_Y  0.06f            // ✅ un poquito más deadzone para no vibrar

// Lock anti-jitter
#define LOCK_IN_X      0.035f
#define LOCK_IN_Y      0.045f
#define LOCK_OUT_X     0.070f
#define LOCK_OUT_Y     0.090f
#define LOCK_STABLE_MS 400

// ✅ Servo más suave
#define SERVO_MAX_DEG_PER_TICK   1   // antes 3
#define STEPPER_MAX_STEPS_TICK   30

// PD filtro
#define PD_MIN_SCORE 0.55f
#define PD_MIN_AREA  (45 * 45)

// =====================================================
// Forward declarations
// =====================================================
static void turret_homing_tick(void);
static void turret_search_tick(void);
static void turret_confirm_tick(void);
static void turret_track_from_pd(void);

static bool  pd_has_valid_target(pd_box_t *best_out);
static void  turret_escape_from_limits(void);
static void  release_current_endstop(int hit_side);

// =====================================================
static inline int64_t now_ms(void)
{
    return (int64_t)(esp_timer_get_time() / 1000ULL);
}

// =====================================================
// Endstops
// =====================================================
static inline bool endstop_left_hit(void)
{
    return (s_pins.limit_left_gpio >= 0 &&
            gpio_get_level((gpio_num_t)s_pins.limit_left_gpio) == ENDSTOP_PRESSED_LEVEL);
}

static inline bool endstop_right_hit(void)
{
    return (s_pins.limit_right_gpio >= 0 &&
            gpio_get_level((gpio_num_t)s_pins.limit_right_gpio) == ENDSTOP_PRESSED_LEVEL);
}

// devuelve: 1 si LEFT, 0 si RIGHT, -1 si ninguno
static inline int which_endstop_hit(void)
{
    if (endstop_left_hit())  return 1;
    if (endstop_right_hit()) return 0;
    return -1;
}

// =====================================================
// Stepper
// =====================================================
static void step_pulse(uint32_t delay_us)
{
    gpio_set_level((gpio_num_t)s_pins.step_gpio, 1);
    esp_rom_delay_us(delay_us);
    gpio_set_level((gpio_num_t)s_pins.step_gpio, 0);
    esp_rom_delay_us(delay_us);
}

static int stepper_move_steps(int dir, int steps)
{
    if (steps <= 0) return 0;

    int moved = 0;
    int64_t t = now_ms();

    if (dir < 0 && t < s_block_left_until_ms)  return 0;
    if (dir > 0 && t < s_block_right_until_ms) return 0;

    // Si ya está presionado, no empujes hacia adentro
    if (dir < 0 && endstop_left_hit())  return 0;
    if (dir > 0 && endstop_right_hit()) return 0;

    gpio_set_level((gpio_num_t)s_pins.dir_gpio, (dir > 0) ? 1 : 0);

    const int YIELD_EVERY = 40;

    for (int i = 0; i < steps; i++) {
        if (dir < 0 && endstop_left_hit())  break;
        if (dir > 0 && endstop_right_hit()) break;

uint32_t delay = s_cfg.step_delay_us;

// Si estoy en estados de homing, uso delay más rápido
if (s_state == TSTATE_HOMING_FIND_FIRST || s_state == TSTATE_HOMING_FIND_SECOND) {
    delay = HOMING_DELAY_US;
}

step_pulse(delay);        s_pos_steps += (dir > 0) ? 1 : -1;
        moved++;

        // ✅ deja respirar WiFi/PD/HTTP
        if ((moved % YIELD_EVERY) == 0) vTaskDelay(1);
    }

    return moved;
}

// =====================================================
// Servo
// =====================================================
static bool servo_init(int gpio)
{
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    if (ledc_timer_config(&tcfg) != ESP_OK) return false;

    ledc_channel_config_t ccfg = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE
    };
    return ledc_channel_config(&ccfg) == ESP_OK;
}

static void servo_set_deg(int deg)
{
    if (deg < s_cfg.servo_min_deg) deg = s_cfg.servo_min_deg;
    if (deg > s_cfg.servo_max_deg) deg = s_cfg.servo_max_deg;
    s_servo_deg = deg;

    // 500us => 0°, 2500us => 270°
    int pulse_us = 500 + (deg * 2000) / 270;
    uint32_t duty = (uint32_t)((pulse_us * 65535UL) / 20000UL);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static inline void mosfet_write(bool on)
{
    s_mosfet_state = on;

    if (s_pins.mosfet_gpio < 0) return; // por si no quieres usar MOSFET
    gpio_set_level((gpio_num_t)s_pins.mosfet_gpio, on ? 1 : 0);
}

static void mosfet_stop(void)
{
    mosfet_write(false);
    s_mosfet_next_toggle_ms = 0;
    s_mosfet_lock_start_ms = 0;
}

static void mosfet_pulse_when_locked(bool locked_now)
{
    int64_t t = now_ms();

    if (!locked_now) {
        mosfet_stop();
        return;
    }

    // si recién entré a locked -> arranco ciclo
    if (s_mosfet_lock_start_ms == 0) {
        s_mosfet_lock_start_ms = t;
        s_mosfet_next_toggle_ms = t;   // toggle ya
        mosfet_write(false);
    }

    // cooldown
    if (s_mosfet_lock_start_ms < 0) {
        int64_t cooldown_until = -s_mosfet_lock_start_ms;
        if (t < cooldown_until) {
            mosfet_write(false);
            return;
        }
        // terminó cooldown, reinicio vibración
        s_mosfet_lock_start_ms = t;
        s_mosfet_next_toggle_ms = t;
        mosfet_write(false);
    }

    // límite de vibración continua
    int64_t elapsed = t - s_mosfet_lock_start_ms;
    if (elapsed >= MOSFET_MAX_MS) {
        mosfet_write(false);
        s_mosfet_lock_start_ms = -(t + MOSFET_COOLDOWN); // entro a cooldown
        s_mosfet_next_toggle_ms = 0;
        return;
    }

    // toggle ON/OFF por tiempo
    if (t >= s_mosfet_next_toggle_ms) {
        if (!s_mosfet_state) {
            mosfet_write(true);
            s_mosfet_next_toggle_ms = t + MOSFET_ON_MS;
        } else {
            mosfet_write(false);
            s_mosfet_next_toggle_ms = t + MOSFET_OFF_MS;
        }
    }
}

// =====================================================
// PD helpers
// =====================================================
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

static bool pd_has_valid_target(pd_box_t *best_out)
{
    pd_result_t r;
    pd_get_last(&r);

    if (r.count <= 0) return false;

    pd_box_t b;
    if (!pick_best_box(&r, &b)) return false;

    int area = b.w * b.h;
    if (b.score < PD_MIN_SCORE || area < PD_MIN_AREA) return false;

    if (best_out) *best_out = b;
    return true;
}

// =====================================================
// Seguridad: escape de endstops (runtime)
// =====================================================
static void turret_escape_from_limits(void)
{
    int64_t t = now_ms();

    if (endstop_left_hit()) {
        s_scan_dir = +1;
        s_block_left_until_ms = t + 500;

        int guard = 2000;
        while (endstop_left_hit() && guard-- > 0) {
            (void)stepper_move_steps(+1, 2);
            vTaskDelay(1);
        }
        (void)stepper_move_steps(+1, 80);
    }

    if (endstop_right_hit()) {
        s_scan_dir = -1;
        s_block_right_until_ms = t + 500;

        int guard = 2000;
        while (endstop_right_hit() && guard-- > 0) {
            (void)stepper_move_steps(-1, 2);
            vTaskDelay(1);
        }
        (void)stepper_move_steps(-1, 80);
    }
}

/**
 * Release REAL del endstop + backoff extra
 * hit_side: 1=LEFT, 0=RIGHT
 */
static void release_current_endstop(int hit_side)
{
    int dir_off = (hit_side == 1) ? +1 : -1;

    int guard = 3000;
    while (guard-- > 0) {
        if (hit_side == 1 && !endstop_left_hit())  break;
        if (hit_side == 0 && !endstop_right_hit()) break;

        (void)stepper_move_steps(dir_off, 2);
        vTaskDelay(1);
    }

    (void)stepper_move_steps(dir_off, 120);
}

// =====================================================
// HOMING (robusto)
// =====================================================
static void turret_homing_tick(void)
{
    int64_t t = now_ms();
    if (t < s_hold_until_ms) return;

    mosfet_pulse_when_locked(false);

    int hit = which_endstop_hit();

    switch (s_state)
    {
    case TSTATE_HOMING_FIND_FIRST:
        if (hit == -1) {
            (void)stepper_move_steps(s_scan_dir, s_cfg.scan_steps_per_tick);
            s_hold_until_ms = t + STEPPER_SETTLE_MS;
            return;
        }

        s_first_is_left = (hit == 1) ? 1 : 0;
        s_steps_from_first = 0;

        ESP_LOGI(TAG, "HOMING: first endstop = %s", s_first_is_left ? "LEFT" : "RIGHT");

        release_current_endstop(hit);

        s_scan_dir = (s_first_is_left ? +1 : -1);
        s_state = TSTATE_HOMING_FIND_SECOND;
        s_hold_until_ms = t + 120;
        return;

    case TSTATE_HOMING_FIND_SECOND:
        if (hit != -1) {
            int expected_second = s_first_is_left ? 0 : 1;

            if (hit != expected_second) {
                release_current_endstop(hit);
                s_hold_until_ms = t + 120;
                return;
            }

            s_range_steps = (s_steps_from_first > 0) ? s_steps_from_first : 0;

            ESP_LOGI(TAG, "HOMING: second endstop hit, range_steps=%d", (int)s_range_steps);

            release_current_endstop(hit);

            // prepara segmento de búsqueda
            {
                int parts = 10;
                int32_t seg = (s_range_steps > 0) ? (s_range_steps / parts) : 0;
                if (seg < 60)  seg = 60;
                if (seg > 600) seg = 600;
                s_search_step_size = seg;
                s_search_remaining = 0;
                ESP_LOGI(TAG, "SEARCH: segment=%d", (int)s_search_step_size);
            }

            s_scan_dir = (s_first_is_left ? -1 : +1);
            s_state = TSTATE_HOMING_GO_CENTER;
            s_hold_until_ms = t + 120;
            return;
        }

        s_steps_from_first += stepper_move_steps(s_scan_dir, s_cfg.scan_steps_per_tick);
        s_hold_until_ms = t + STEPPER_SETTLE_MS;
        return;

    case TSTATE_HOMING_GO_CENTER:
    {
        int32_t half = s_range_steps / 2;

        if (half > 0) {
            int32_t rem = half;
            const int CHUNK = 200;
            while (rem > 0) {
                int c = (rem > CHUNK) ? CHUNK : rem;
                (void)stepper_move_steps(s_scan_dir, c);
                rem -= c;
                vTaskDelay(1);
            }
        }

        servo_set_deg(s_cfg.servo_center_deg);

        // ✅ importantísimo: ya NO vuelvas a homing jamás
        s_homing_done = true;

        s_state = TSTATE_SEARCH_STEPPER;
        s_hold_until_ms = t + 150;

        ESP_LOGI(TAG, "HOMING DONE -> SEARCH");
        return;
    }

    default:
        return;
    }
}

// =====================================================
// SEARCH
// =====================================================
static void turret_search_tick(void)
{
    int64_t t = now_ms();
    if (t < s_hold_until_ms) return;

    mosfet_pulse_when_locked(false);

    pd_box_t b;
    if (pd_has_valid_target(&b)) {
        mosfet_pulse_when_locked(false);

        s_confirm_hits = 0;
        s_confirm_start_ms = t;
        s_state = TSTATE_CONFIRM;
        s_hold_until_ms = t + 80;
        ESP_LOGI(TAG, "SEARCH: target seen -> CONFIRM");
        return;
    }

    turret_escape_from_limits();
    if (endstop_left_hit() || endstop_right_hit()) {
        s_search_remaining = 0;
    }

    if (s_state == TSTATE_SEARCH_STEPPER) {

        if (endstop_left_hit())  s_scan_dir = +1;
        if (endstop_right_hit()) s_scan_dir = -1;

        int32_t seg = (s_search_step_size > 0) ? s_search_step_size : s_cfg.scan_steps_per_tick;
        if (s_search_remaining <= 0) s_search_remaining = seg;

        int chunk = s_cfg.scan_steps_per_tick;
        if (chunk > s_search_remaining) chunk = s_search_remaining;

        (void)stepper_move_steps(s_scan_dir, chunk);
        s_search_remaining -= chunk;

        s_hold_until_ms = t + STEPPER_SETTLE_MS;

        if (s_search_remaining <= 0) {
            s_state = TSTATE_SEARCH_SERVO;
        }
        return;
    }

    if (s_state == TSTATE_SEARCH_SERVO) {
        int center = s_cfg.servo_center_deg;

        int target_deg = center;
        switch (s_search_servo_phase % 3) {
            case 0: target_deg = center + 12; break;
            case 1: target_deg = center - 12; break;
            default: target_deg = center; break;
        }
        s_search_servo_phase++;

        servo_set_deg(target_deg);
        s_hold_until_ms = t + SERVO_SETTLE_MS;

        s_state = TSTATE_SEARCH_STEPPER;
        return;
    }

    s_state = TSTATE_SEARCH_STEPPER;
}

// =====================================================
// CONFIRM
// =====================================================
static void turret_confirm_tick(void)
{
    int64_t t = now_ms();

    const int64_t CONFIRM_TIMEOUT_MS = 900;
    const int     CONFIRM_NEED_HITS  = 3;

    if (s_confirm_start_ms == 0) s_confirm_start_ms = t;

    pd_box_t b;
    bool ok = pd_has_valid_target(&b);

    if (ok) {
        s_confirm_hits++;
        s_hold_until_ms = t + 80;

        if (s_confirm_hits >= CONFIRM_NEED_HITS) {
            s_state = TSTATE_TRACK;
            s_hold_until_ms = t + 120;

            s_last_seen_ms = t;
            s_track_lost_since = 0;

            ESP_LOGI(TAG, "CONFIRM: OK -> TRACK");
        }
        return;
    }

    s_confirm_hits = 0;

    if ((t - s_confirm_start_ms) > CONFIRM_TIMEOUT_MS) {
        mosfet_pulse_when_locked(false);

        s_state = TSTATE_SEARCH_STEPPER;
        s_confirm_start_ms = 0;
        s_confirm_hits = 0;
        s_hold_until_ms = t + 120;
        ESP_LOGI(TAG, "CONFIRM: timeout -> SEARCH");
    }
}

// =====================================================
// TRACK
// =====================================================
static void turret_track_from_pd(void)
{
    int64_t t = now_ms();
    if (t < s_hold_until_ms) return;

    pd_result_t r;
    pd_get_last(&r);

    if (r.count <= 0) {
        mosfet_pulse_when_locked(false);

        if (s_track_lost_since == 0) s_track_lost_since = t;

        if ((t - s_track_lost_since) > (int64_t)s_cfg.lost_timeout_ms) {
            s_state = TSTATE_SEARCH_STEPPER;
            s_track_lost_since = 0;
            s_hold_until_ms = t + 150;
            ESP_LOGI(TAG, "TRACK: lost -> SEARCH");
        }
        return;
    }

    pd_box_t b;
    if (!pick_best_box(&r, &b)) return;

    int area = b.w * b.h;
    if (b.score < PD_MIN_SCORE || area < PD_MIN_AREA) {
        mosfet_pulse_when_locked(false);

        if (s_track_lost_since == 0) s_track_lost_since = t;
        return;
    }

    s_last_seen_ms = t;
    s_track_lost_since = 0;

    float W = (float)((s_frame_w > 0) ? s_frame_w : 640);
    float H = (float)((s_frame_h > 0) ? s_frame_h : 480);

    float nx = (float)(b.x + b.w * 0.5f) / W;
    float ny = (float)(b.y + b.h * 0.5f) / H;

    float ex = 0.5f - nx;
    float ey = 0.5f - ny;

    float ax = fabsf(ex);
    float ay = fabsf(ey);

    static bool locked = false;
    static int64_t lock_since = 0;

    bool in_lock  = (ax < LOCK_IN_X) && (ay < LOCK_IN_Y);
    bool out_lock = (ax > LOCK_OUT_X) || (ay > LOCK_OUT_Y);

    if (!locked) {
        if (in_lock) {
            if (lock_since == 0) lock_since = t;
            if ((t - lock_since) >= LOCK_STABLE_MS) locked = true;
        } else {
            lock_since = 0;
        }
    } else {
        if (out_lock) {
            locked = false;
            lock_since = 0;
        }
    }

    if (endstop_left_hit() || endstop_right_hit()) {
        locked = false;
        lock_since = 0;
    }

    if (locked) {
        mosfet_pulse_when_locked(true);
        return;
    } else {
        mosfet_pulse_when_locked(false);
    }

    // X => stepper
    if (ax > DEAD_X) {
        int dir = (ex > 0) ? -1 : +1;

        if (endstop_left_hit())  dir = +1;
        if (endstop_right_hit()) dir = -1;

        int steps = (int)(ax * s_cfg.kp_step);
        if (steps < s_cfg.step_min) steps = s_cfg.step_min;
        if (steps > s_cfg.step_max) steps = s_cfg.step_max;
        if (steps > STEPPER_MAX_STEPS_TICK) steps = STEPPER_MAX_STEPS_TICK;

        (void)stepper_move_steps(dir, steps);
        s_hold_until_ms = t + STEPPER_SETTLE_MS;
    }

    // Y => servo
    if (ay > DEAD_Y) {
        int delta = (int)(ey * s_cfg.kp_servo * 0.35f);
        if (delta >  SERVO_MAX_DEG_PER_TICK) delta =  SERVO_MAX_DEG_PER_TICK;
        if (delta < -SERVO_MAX_DEG_PER_TICK) delta = -SERVO_MAX_DEG_PER_TICK;

        if (delta != 0) {
            servo_set_deg(s_servo_deg + delta);
            s_hold_until_ms = t + SERVO_SETTLE_MS;
        }
    }
}

// =====================================================
// TASK
// =====================================================
static void turret_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "turret task start");

    while (s_running) {

        // HOMING solo una vez
        if (!s_homing_done) {
            turret_homing_tick();
            vTaskDelay(pdMS_TO_TICKS(s_cfg.scan_tick_ms));
            continue;
        }

        // settle/hold
        if (now_ms() < s_hold_until_ms) {
            vTaskDelay(pdMS_TO_TICKS(s_cfg.scan_tick_ms));
            continue;
        }

        // Máquina principal
        if (s_state == TSTATE_SEARCH_STEPPER || s_state == TSTATE_SEARCH_SERVO) {
            turret_search_tick();
        } else if (s_state == TSTATE_CONFIRM) {
            turret_confirm_tick();
        } else if (s_state == TSTATE_TRACK) {
            turret_track_from_pd();
        } else {
            s_state = TSTATE_SEARCH_STEPPER;
        }

        vTaskDelay(pdMS_TO_TICKS(s_cfg.scan_tick_ms));
    }

    ESP_LOGI(TAG, "turret task stop");
    s_task = NULL;
    mosfet_stop();

    vTaskDelete(NULL);
}

// =====================================================
// API
// =====================================================
bool turret_init(const turret_pins_t *pins, const turret_cfg_t *cfg)
{
    if (!pins || !cfg) return false;
    s_pins = *pins;
    s_cfg  = *cfg;

    // GPIO step/dir
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

    // enable (si existe): típico DRV8825 activo LOW
    if (s_pins.en_gpio >= 0) {
        gpio_set_level((gpio_num_t)s_pins.en_gpio, 0);
    }

    // MOSFET (GPIO4) salida
    if (s_pins.mosfet_gpio >= 0) {
        gpio_reset_pin((gpio_num_t)s_pins.mosfet_gpio);
        gpio_set_direction((gpio_num_t)s_pins.mosfet_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)s_pins.mosfet_gpio, 0);
    }
    mosfet_stop();

    // endstops input
    gpio_config_t in = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 0,
        .pull_down_en = 1,
        .intr_type = GPIO_INTR_DISABLE
    };
    if (s_pins.limit_left_gpio >= 0)  in.pin_bit_mask |= (1ULL << s_pins.limit_left_gpio);
    if (s_pins.limit_right_gpio >= 0) in.pin_bit_mask |= (1ULL << s_pins.limit_right_gpio);
    if (in.pin_bit_mask) ESP_ERROR_CHECK(gpio_config(&in));

    // servo
    if (!servo_init(s_pins.servo_gpio)) {
        ESP_LOGE(TAG, "servo init failed");
        return false;
    }
    servo_set_deg(s_cfg.servo_center_deg);

    // reset state
    s_state = TSTATE_HOMING_FIND_FIRST;
    s_scan_dir = +1;
    s_pos_steps = 0;

    s_range_steps = 0;
    s_steps_from_first = 0;
    s_first_is_left = -1;

    s_search_step_size = 0;
    s_search_remaining = 0;
    s_search_servo_phase = 0;

    s_confirm_hits = 0;
    s_confirm_start_ms = 0;

    s_hold_until_ms = 0;
    s_block_left_until_ms = 0;
    s_block_right_until_ms = 0;

    s_last_seen_ms = 0;
    s_track_lost_since = 0;

    s_homing_done = false;

    ESP_LOGI(TAG, "turret init OK");
    return true;
}

void turret_start(void)
{
    // ✅ evita doble task
    if (s_running) return;
    s_running = true;

    if (!s_task) {
        xTaskCreatePinnedToCore(
            turret_task,
            "turret_task",
            8192,
            NULL,
            4,
            &s_task,
            1
        );
    }
}

void turret_stop(void)
{
    s_running = false;
    mosfet_stop();
}

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
