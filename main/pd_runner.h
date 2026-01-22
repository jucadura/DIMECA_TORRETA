#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int x;
    int y;
    int w;
    int h;
    float score;
} pd_box_t;

typedef struct {
    int count;
    pd_box_t boxes[8];   // máximo 8 detecciones (ajustable)
    uint32_t frame_id;
    uint32_t ts_ms;
} pd_result_t;

// Inicializa el detector y arranca task interna
bool pd_init(void);

// Enviar frame al detector (no bloquea; si está ocupado, lo ignora)
bool pd_submit_rgb565(const uint16_t *rgb565, int width, int height, uint32_t frame_id);

bool pd_run_frame(uint8_t *frame, uint32_t w, uint32_t h, uint32_t pixfmt);

// Leer el último resultado disponible
void pd_get_last(pd_result_t *out);

#ifdef __cplusplus
}
#endif
