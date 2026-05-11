#include "custom_lcd_display.h"

#include "lcd_display.h"

#include <vector>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_lvgl_port.h>
#include "assets/lang_config.h"
#include <cstring>
#include "settings.h"

#include "esp_lcd_panel_io.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "config.h"

#include "board.h"

#define TAG "CustomLcdDisplay"

static SemaphoreHandle_t trans_done_sem = NULL;
static uint16_t *trans_buf_1;


bool CustomLcdDisplay::lvgl_port_flush_io_ready_callback(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    BaseType_t taskAwake = pdFALSE;
    lv_display_t *disp_drv = (lv_display_t *)user_ctx;
    assert(disp_drv != NULL);
    if (trans_done_sem) {
        xSemaphoreGiveFromISR(trans_done_sem, &taskAwake);
    }
    return false;
}

void CustomLcdDisplay::lvgl_port_flush_callback(lv_display_t *drv, const lv_area_t *area, uint8_t *color_map) {
    assert(drv != NULL);
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(drv);
    assert(panel_handle != NULL);

    lv_draw_sw_rgb565_swap(color_map, lv_area_get_width(area) * lv_area_get_height(area));

#if (DISPLAY_ROTATION_90 == true)
    uint16_t *from = (uint16_t*)color_map;
    const int src_width = lv_area_get_width(area);
    const int src_height = lv_area_get_height(area);
    const int max_chunk_cols = LVGL_DMA_BUFF_LEN / (src_height * sizeof(uint16_t));
    assert(max_chunk_cols > 0);

    xSemaphoreGive(trans_done_sem);

    for (int x = area->x1; x <= area->x2; x += max_chunk_cols) {
        int chunk_cols = area->x2 - x + 1;
        if (chunk_cols > max_chunk_cols) {
            chunk_cols = max_chunk_cols;
        }

        uint16_t *to = trans_buf_1;
        const int src_x_offset = x - area->x1;
        for (int src_y = 0; src_y < src_height; src_y++) {
            for (int src_x = 0; src_x < chunk_cols; src_x++) {
                to[src_x * src_height + (src_height - src_y - 1)] =
                    from[src_y * src_width + src_x_offset + src_x];
            }
        }

        xSemaphoreTake(trans_done_sem,portMAX_DELAY);
        esp_lcd_panel_draw_bitmap(panel_handle,
            DISPLAY_HEIGHT - area->y2 - 1,
            x,
            DISPLAY_HEIGHT - area->y1,
            x + chunk_cols,
            trans_buf_1);
    }
    xSemaphoreTake(trans_done_sem,portMAX_DELAY);
#else
    uint16_t *map = (uint16_t*)color_map;
    const int draw_width = lv_area_get_width(area);
    const int max_chunk_lines = LVGL_DMA_BUFF_LEN / (draw_width * sizeof(uint16_t));
    assert(max_chunk_lines > 0);

    xSemaphoreGive(trans_done_sem);

    for (int y = area->y1; y <= area->y2; y += max_chunk_lines) {
        int chunk_lines = area->y2 - y + 1;
        if (chunk_lines > max_chunk_lines) {
            chunk_lines = max_chunk_lines;
        }
        const size_t chunk_bytes = draw_width * chunk_lines * sizeof(uint16_t);
        xSemaphoreTake(trans_done_sem,portMAX_DELAY);
        memcpy(trans_buf_1, map, chunk_bytes);
        esp_lcd_panel_draw_bitmap(panel_handle, area->x1, y, area->x2 + 1, y + chunk_lines, trans_buf_1);
        map += draw_width * chunk_lines;
    }
    xSemaphoreTake(trans_done_sem,portMAX_DELAY);
#endif
    lv_disp_flush_ready(drv);
}


CustomLcdDisplay::CustomLcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                  int width, int height, int offset_x, int offset_y,
                  bool mirror_x, bool mirror_y, bool swap_xy)
    : LcdDisplay(panel_io, panel, width, height) {

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    ESP_LOGI(TAG, "Initialize LVGL port");
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_priority = 2;
    port_cfg.timer_period_ms = 50;
    lvgl_port_init(&port_cfg);
    trans_done_sem = xSemaphoreCreateBinary();
    trans_buf_1 = (uint16_t *)heap_caps_malloc(LVGL_DMA_BUFF_LEN, MALLOC_CAP_DMA);

    uint32_t buffer_size = 0;
    lv_color_t *buf1 = NULL;
    lvgl_port_lock(0);
    uint8_t color_bytes = lv_color_format_get_size(LV_COLOR_FORMAT_RGB565);
    display_ = lv_display_create(width_, height_);
    lv_display_set_flush_cb(display_, lvgl_port_flush_callback);
    buffer_size = width_ * height_;
    buf1 = (lv_color_t *)heap_caps_aligned_alloc(1, buffer_size * color_bytes, MALLOC_CAP_SPIRAM);
    lv_display_set_buffers(display_, buf1, NULL, buffer_size * color_bytes, LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_user_data(display_, panel_);
    lvgl_port_unlock();

    esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = lvgl_port_flush_io_ready_callback,
    };
    /* Register done callback */
    esp_lcd_panel_io_register_event_callbacks(panel_io_, &cbs, display_);

    if (display_ == nullptr) {
        ESP_LOGE(TAG, "Failed to add display");
        return;
    }

    if (offset_x != 0 || offset_y != 0) {
        lv_display_set_offset(display_, offset_x, offset_y);
    }

    // Note: SetupUI() should be called by Application::Initialize(), not in constructor
    // to ensure lvgl objects are created after the display is fully initialized.
}