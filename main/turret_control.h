#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int step_gpio;
    int dir_gpio;
    int en_gpio;              // -1 si no se usa

    int m0_gpio;              // -1 si no se usa
    int m1_gpio;              // -1 si no se usa
    int m2_gpio;              // -1 si no se usa

    int servo_gpio;

    int limit_left_gpio;      // -1 si no se usa
    int limit_right_gpio;     // -1 si no se usa

    int mosfet_gpio;          // GPIO para activar MOSFET (ej: 4)
} turret_pins_t;

typedef struct {
    // Stepper timing
    uint32_t step_delay_us;

    // Scan
    int32_t  scan_limit_steps;      // (si no lo usas, pon 0)
    int32_t  scan_steps_per_tick;
    uint32_t scan_tick_ms;

    // Tracking stepper
    float    kp_step;
    int32_t  step_min;
    int32_t  step_max;

    // Tracking servo
    float    kp_servo;
    int32_t  servo_min_deg;
    int32_t  servo_max_deg;
    int32_t  servo_center_deg;

    // Detección
    uint32_t lost_timeout_ms;
    uint32_t valid_age_ms;
} turret_cfg_t;

bool turret_init(const turret_pins_t *pins, const turret_cfg_t *cfg);

void turret_start(void);
void turret_stop(void);

/** Fuerza a re-hacer homing desde cero */
void turret_home(void);

// Frame size real del stream
void turret_set_frame_size(int w, int h);
void turret_get_frame_size(int *w, int *h);

void turret_mosfet_set(bool on);

// (Opcional, si lo estás usando desde otro lado)
void turret_update_target(bool has_target, float nx, float ny);

#ifdef __cplusplus
}
#endif
