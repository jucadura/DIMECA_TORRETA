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

static turret_pins_t s_pins;
static turret_cfg_t  s_cfg;

static TaskHandle_t  s_task = NULL;
static bool          s_running = false;

// Estado
static int32_t s_pos_steps = 0;
static int     s_scan_dir = +1;
static int     s_servo_deg = 90;
static int64_t s_last_seen_ms = 0;

// ✅ Frame size real (lo setea simple_video_server_example.c)
static int s_frame_w = 640;
static int s_frame_h = 480;

// ✅ finales de carrera como tú lo usas: PULLDOWN y presionado=3V3 => level=1
#define ENDSTOP_PRESSED_LEVEL 1

// ---------- Helpers ----------
static inline int64_t now_ms(void) {
    return (int64_t)(esp_timer_get_time() / 1000ULL);
}

// ✅ ACTIVE-HIGH
static inline bool endstop_left_hit(void) {
    if (s_pins.limit_left_gpio < 0) return false;
    return gpio_get_level((gpio_num_t)s_pins.limit_left_gpio) == ENDSTOP_PRESSED_LEVEL;
}

static inline bool endstop_right_hit(void) {
    if (s_pins.limit_right_gpio < 0) return false;
    return gpio_get_level((gpio_num_t)s_pins.limit_right_gpio) == ENDSTOP_PRESSED_LEVEL;
}

static void step_pulse(uint32_t delay_us)
{
    gpio_set_level((gpio_num_t)s_pins.step_gpio, 1);
    esp_rom_delay_us(delay_us);
    gpio_set_level((gpio_num_t)s_pins.step_gpio, 0);
    esp_rom_delay_us(delay_us);
}

static void stepper_move_steps(int dir, int steps)
{
    if (steps <= 0) return;

    if (dir < 0 && endstop_left_hit())  return;
    if (dir > 0 && endstop_right_hit()) return;

    gpio_set_level((gpio_num_t)s_pins.dir_gpio, (dir > 0) ? 1 : 0);

    for (int i = 0; i < steps; i++) {
        if (dir < 0 && endstop_left_hit())  break;
        if (dir > 0 && endstop_right_hit()) break;

        step_pulse(s_cfg.step_delay_us);
        s_pos_steps += (dir > 0) ? 1 : -1;

        if (s_cfg.scan_limit_steps > 0) {
            if (s_pos_steps <= -s_cfg.scan_limit_steps) { s_scan_dir = +1; break; }
            if (s_pos_steps >=  s_cfg.scan_limit_steps) { s_scan_dir = -1; break; }
        }
    }
}

// ---------- Servo (LEDC) ----------
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

static void servo_set_deg(int deg)
{
    if (deg < s_cfg.servo_min_deg) deg = s_cfg.servo_min_deg;
    if (deg > s_cfg.servo_max_deg) deg = s_cfg.servo_max_deg;
    s_servo_deg = deg;

    const int min_us = 500;
    const int max_us = 2500;
    int pulse_us = min_us + (deg * (max_us - min_us)) / 180;

    uint32_t duty = (uint32_t)((pulse_us * 65535ULL) / 20000ULL);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// ---------- Tracking ----------
static bool pick_best_box(const pd_result_t *r, pd_box_t *best)
{
    if (!r || r->count <= 0) return false;
    int best_i = 0;
    int best_area = r->boxes[0].w * r->boxes[0].h;

    for (int i = 1; i < r->count; i++) {
        int area = r->boxes[i].w * r->boxes[i].h;
        if (area > best_area) { best_area = area; best_i = i; }
    }
    *best = r->boxes[best_i];
    return true;
}

static void turret_track_from_pd(void)
{
    pd_result_t r;
    pd_get_last(&r);

    int64_t t = now_ms();
    if (r.count <= 0) return;

    if (s_cfg.valid_age_ms > 0 && (t - (int64_t)r.ts_ms) > (int64_t)s_cfg.valid_age_ms) {
        return;
    }

    pd_box_t b;
    if (!pick_best_box(&r, &b)) return;

    float cx = (float)(b.x + b.w * 0.5f);
    float cy = (float)(b.y + b.h * 0.5f);

    // ✅ Usa resolución real
    float W = (float)((s_frame_w > 0) ? s_frame_w : 640);
    float H = (float)((s_frame_h > 0) ? s_frame_h : 480);

    float nx = cx / W; // 0..1
    float ny = cy / H; // 0..1

    float ex = (0.5f - nx);
    float ey = (0.5f - ny);

    int dir = (ex >= 0) ? -1 : +1;
    float mag = fabsf(ex);
    int steps = (int)(mag * s_cfg.kp_step);

    if (steps < s_cfg.step_min) steps = s_cfg.step_min;
    if (steps > s_cfg.step_max) steps = s_cfg.step_max;

    if (mag < 0.03f) steps = 0;
    if (steps > 0) stepper_move_steps(dir, steps);

    int delta = (int)(ey * s_cfg.kp_servo);
    int new_deg = s_servo_deg + delta;
    if (fabsf(ey) < 0.03f) new_deg = s_servo_deg;

    servo_set_deg(new_deg);

    s_last_seen_ms = t;
}

// ---------- Scan ----------
static void turret_scan_tick(void)
{
    if (endstop_left_hit())  s_scan_dir = +1;
    if (endstop_right_hit()) s_scan_dir = -1;

    stepper_move_steps(s_scan_dir, s_cfg.scan_steps_per_tick);
}

// ---------- Task ----------
static void turret_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "turret task start");
    s_last_seen_ms = now_ms();

    while (s_running) {
        int64_t t = now_ms();

        int64_t before = s_last_seen_ms;
        turret_track_from_pd();
        bool saw_recent = (s_last_seen_ms != before);

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

// ---------- API ----------
bool turret_init(const turret_pins_t *pins, const turret_cfg_t *cfg)
{
    if (!pins || !cfg) return false;
    s_pins = *pins;
    s_cfg  = *cfg;

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

    if (s_pins.en_gpio >= 0) {
        gpio_set_level((gpio_num_t)s_pins.en_gpio, 0); // enable activo LOW
    }

    if (s_pins.m0_gpio >= 0) gpio_set_direction((gpio_num_t)s_pins.m0_gpio, GPIO_MODE_OUTPUT);
    if (s_pins.m1_gpio >= 0) gpio_set_direction((gpio_num_t)s_pins.m1_gpio, GPIO_MODE_OUTPUT);
    if (s_pins.m2_gpio >= 0) gpio_set_direction((gpio_num_t)s_pins.m2_gpio, GPIO_MODE_OUTPUT);

    // ✅ finales: INPUT + PULLDOWN (porque van a 3V3 al presionar)
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
    // 5        ESP_ERROR_CHECK(gpio_config(&in));
    }

    if (!servo_init(s_pins.servo_gpio)) {
        ESP_LOGE(TAG, "servo init failed");
        return false;
    }
    s_servo_deg = s_cfg.servo_center_deg;
    servo_set_deg(s_servo_deg);

    ESP_LOGI(TAG, "turret init ok (step=%d dir=%d servo=%d L=%d R=%d)",
             s_pins.step_gpio, s_pins.dir_gpio, s_pins.servo_gpio,
             s_pins.limit_left_gpio, s_pins.limit_right_gpio);

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

// ✅ NUEVO (para corregir tu error del build)
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
