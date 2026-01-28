#include <inttypes.h>
#include <string.h>
#include <stdint.h>

#include "pd_runner.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

// C++: componente pedestrian_detect (usa esp-dl)
#include "pedestrian_detect.hpp"
#include "dl_image_define.hpp"   // dl::image::img_t + DL_IMAGE_PIX_TYPE_RGB565

// Si no tienes linux/videodev2.h disponible en tu build, usa estos FOURCC:
#ifndef V4L2_PIX_FMT_RGB565
#define v4l2_fourcc(a, b, c, d) ((uint32_t)(a) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))
#define V4L2_PIX_FMT_RGB565 v4l2_fourcc('R','G','B','P') // "RGBP"
#define V4L2_PIX_FMT_RGB24  v4l2_fourcc('R','G','B','3') // "RGB3"
#define V4L2_PIX_FMT_JPEG   v4l2_fourcc('J','P','E','G')
#endif

static const char *TAG = "PD";

typedef struct {
    const uint16_t *rgb565;   // apunta a nuestro buffer propio (pool)
    int w;
    int h;
    uint32_t frame_id;
    int buf_idx;              // cual buffer del pool es
} pd_job_t;

static QueueHandle_t s_q = NULL;
static SemaphoreHandle_t s_mutex = NULL;
static SemaphoreHandle_t s_pool_mutex = NULL;

static pd_result_t s_last = {};
static PedestrianDetect *s_pd = nullptr;

// Pool doble para no pisar memoria mientras corre inferencia
static uint8_t *s_pool[2] = {nullptr, nullptr};
static bool s_pool_in_use[2] = {false, false};
static size_t s_pool_bytes = 0;
static int s_next_id = 0;

static bool pool_ensure(size_t bytes)
{
    if (s_pool_bytes == bytes && s_pool[0] && s_pool[1]) {
        return true;
    }

    // (Re)alloc pool
    for (int i = 0; i < 2; i++) {
        if (s_pool[i]) {
            heap_caps_free(s_pool[i]);
            s_pool[i] = nullptr;
        }
        s_pool_in_use[i] = false;
    }

    s_pool[0] = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_pool[1] = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_pool[0] || !s_pool[1]) {
        ESP_LOGE(TAG, "pool alloc failed (%u bytes each)", (unsigned)bytes);
        if (s_pool[0]) heap_caps_free(s_pool[0]), s_pool[0] = nullptr;
        if (s_pool[1]) heap_caps_free(s_pool[1]), s_pool[1] = nullptr;
        s_pool_bytes = 0;
        return false;
    }

    s_pool_bytes = bytes;
    ESP_LOGI(TAG, "RGB565 pool allocated: %u bytes each", (unsigned)bytes);
    return true;
}

static int pool_acquire(void)
{
    int idx = -1;
    xSemaphoreTake(s_pool_mutex, portMAX_DELAY);
    for (int i = 0; i < 2; i++) {
        if (!s_pool_in_use[i]) {
            s_pool_in_use[i] = true;
            idx = i;
            break;
        }
    }
    xSemaphoreGive(s_pool_mutex);
    return idx; // -1 si no hay libre
}

static void pool_release(int idx)
{
    if (idx < 0 || idx > 1) return;
    xSemaphoreTake(s_pool_mutex, portMAX_DELAY);
    s_pool_in_use[idx] = false;
    xSemaphoreGive(s_pool_mutex);
}

// Task de inferencia
static void pd_task(void *arg)
{
    (void)arg;

    pd_job_t job;

    // throttling de logs
    const uint32_t LOG_EVERY_MS = 500;   // log base 2 veces por segundo
    const uint32_t VERBOSE_EVERY_MS = 2500; // detalles completos cada 2.5s (opcional)

    uint32_t last_log_ms = 0;
    uint32_t last_verbose_ms = 0;

    while (true) {
        if (xQueueReceive(s_q, &job, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!s_pd || !job.rgb565 || job.w <= 0 || job.h <= 0) {
            ESP_LOGW(TAG, "invalid job or detector not initialized");
            pool_release(job.buf_idx);
            continue;
        }

        // Construir img_t para esp-dl (RGB565)
        dl::image::img_t img = {};
        img.data = (void *)job.rgb565;
        img.width = (uint16_t)job.w;
        img.height = (uint16_t)job.h;
        img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565;

        int64_t t0 = esp_timer_get_time();
        std::list<dl::detect::result_t> &results = s_pd->run(img);
        int64_t t1 = esp_timer_get_time();

        uint32_t infer_ms = (uint32_t)((t1 - t0) / 1000);
        uint32_t now_ms   = (uint32_t)(esp_timer_get_time() / 1000ULL);

        pd_result_t out = {};
        out.frame_id = job.frame_id;
        out.ts_ms = now_ms;

        int n = 0;
        for (auto &r : results) {
            if (n >= 8) break;
            if (r.box.size() < 4) continue;

            int x1 = r.box[0];
            int y1 = r.box[1];
            int x2 = r.box[2];
            int y2 = r.box[3];

            out.boxes[n].x = x1;
            out.boxes[n].y = y1;
            out.boxes[n].w = x2 - x1;
            out.boxes[n].h = y2 - y1;
            out.boxes[n].score = r.score;
            n++;
        }
        out.count = n;

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_last = out;
        xSemaphoreGive(s_mutex);

        // ---------------- LOGS CONTROLADOS ----------------
        if ((now_ms - last_log_ms) >= LOG_EVERY_MS) {
            last_log_ms = now_ms;
            ESP_LOGI(TAG, "detected=%d frame=%" PRIu32 " infer=%" PRIu32 "ms ts=%" PRIu32 "ms",
                     out.count, out.frame_id, infer_ms, out.ts_ms);
        }

        // Solo si hay detección, saca el BEST box pero NO spamees todo
        if (out.count > 0 && (now_ms - last_verbose_ms) >= VERBOSE_EVERY_MS) {
            last_verbose_ms = now_ms;

            int best = 0;
            int best_area = out.boxes[0].w * out.boxes[0].h;
            for (int i = 1; i < out.count; i++) {
                int area = out.boxes[i].w * out.boxes[i].h;
                if (area > best_area) { best_area = area; best = i; }
            }

            pd_box_t b = out.boxes[best];
            int cx = b.x + b.w / 2;
            int cy = b.y + b.h / 2;

            float nx = (float)cx / (float)job.w;
            float ny = (float)cy / (float)job.h;

            ESP_LOGI(TAG,
                     "BEST: x=%d y=%d w=%d h=%d s=%.2f | c=(%d,%d) | n=(%.3f,%.3f) | img=%dx%d",
                     b.x, b.y, b.w, b.h, b.score, cx, cy, nx, ny, job.w, job.h);
        }
        // --------------------------------------------------

        pool_release(job.buf_idx);

        // ✅ cede CPU para que WiFi/HTTP no se mueran
        vTaskDelay(1);
    }
}


bool pd_init(void)
{
    if (s_q) return true;

    ESP_LOGI(TAG, "pd_init() starting...");

    s_mutex = xSemaphoreCreateMutex();
    s_pool_mutex = xSemaphoreCreateMutex();
    s_q = xQueueCreate(1, sizeof(pd_job_t)); // siempre el frame más reciente si hay espacio

    if (!s_mutex || !s_q || !s_pool_mutex) {
        ESP_LOGE(TAG, "no mem for queue/mutex");
        return false;
    }

    s_pd = new PedestrianDetect();
    if (!s_pd) {
        ESP_LOGE(TAG, "PedestrianDetect new failed");
        return false;
    }

    // Opcional:
    // s_pd->set_score_thr(0.6f);
    // s_pd->set_nms_thr(0.5f);

    xTaskCreatePinnedToCore(pd_task, "pd_task", 8192, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "pd_init ok");
    return true;
}

// API segura para tu simple_video_server_example.c
bool pd_run_frame(uint8_t *frame, uint32_t w, uint32_t h, uint32_t pixfmt)
{
    if (!frame || w == 0 || h == 0) return false;
    if (!s_q || !s_pd) return false;

    // Solo soportamos RGB565 directo (RGBP) por ahora
    if (pixfmt == V4L2_PIX_FMT_JPEG) {
        // no procesamos JPEG crudo aquí
        return false;
    }

    // bytes esperados para RGB565
    size_t need = (size_t)w * (size_t)h * 2u;

    // Prepara pool si hace falta
    if (!pool_ensure(need)) return false;

    // Si la cola está ocupada, NO sobreescribas (evita pisar buffers)
    if (uxQueueMessagesWaiting(s_q) > 0) {
        return false; // drop frame
    }

    int idx = pool_acquire();
    if (idx < 0) {
        return false; // no hay buffer libre
    }

    // Si pixfmt es RGB565 (RGBP), copiamos directo
    if (pixfmt == V4L2_PIX_FMT_RGB565) {
        memcpy(s_pool[idx], frame, need);
    } else if (pixfmt == V4L2_PIX_FMT_RGB24) {
        // Convert RGB24 -> RGB565
        uint8_t *src = frame;
        uint16_t *dst = (uint16_t *)s_pool[idx];
        size_t pixels = (size_t)w * (size_t)h;
        for (size_t i = 0; i < pixels; i++) {
            uint8_t r = *src++;
            uint8_t g = *src++;
            uint8_t b = *src++;
            uint16_t rgb565 = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
            dst[i] = rgb565;
        }
    } else {
        // Formato no soportado
        pool_release(idx);
        return false;
    }

    pd_job_t job = {};
    job.rgb565 = (const uint16_t *)s_pool[idx];
    job.w = (int)w;
    job.h = (int)h;
    job.frame_id = (uint32_t)(++s_next_id);
    job.buf_idx = idx;

    if (xQueueSend(s_q, &job, 0) != pdTRUE) {
        pool_release(idx);
        return false;
    }
    return true;
}

void pd_get_last(pd_result_t *out)
{
    if (!out) return;

    memset(out, 0, sizeof(*out));
    if (!s_mutex) return;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_last;
    xSemaphoreGive(s_mutex);
}
