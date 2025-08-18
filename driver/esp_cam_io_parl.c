#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "portmacro.h"
#include "time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/parlio_rx.h"
#include "hal/parlio_types.h"
#include "soc/gpio_num.h"
#include "esp_cam_io_parl.h"

static const char *TAG = "esp_cam_io_parl";

#define ESP_CAM_IO_PARL_CHECK_ISR(condition, err) if (!(condition)) { return err; }

struct esp_cam_io_parl_t {
    uint32_t alloc_size;
    uint32_t alloc_heap_caps;
    uint16_t payload_size;
    uint8_t *payload;
    uint32_t use_soft_delimiter : 1;
    volatile struct {
        esp_cam_io_parl_trans_t frame;
        uint8_t last_byte;
        uint32_t index;
        uint32_t captured : 1;
    } info;
    QueueHandle_t queue_handle;
    parlio_rx_unit_handle_t rx_unit;
    parlio_rx_delimiter_handle_t rx_delimiter;
};

#ifndef CONFIG_CAMERA_PAYLOAD_BUFFER_SIZE
#define CONFIG_CAMERA_PAYLOAD_BUFFER_SIZE 0x8000
#endif

static bool IRAM_ATTR on_partial_receive_callback(parlio_rx_unit_handle_t rx_unit, const parlio_rx_event_data_t *edata, void *user_data) {
    const uint8_t *data = edata->data;
    const uint32_t received_bytes = edata->recv_bytes;
    esp_cam_io_parl_handle_t esp_cam_io_parl = (esp_cam_io_parl_handle_t)user_data;
    for (uint16_t index = 0; index < received_bytes; index++) {
        uint8_t current_byte = data[index];
        if (esp_cam_io_parl->info.captured && esp_cam_io_parl->info.frame.buffer) {
            if (esp_cam_io_parl->info.index < esp_cam_io_parl->info.frame.length) {
                esp_cam_io_parl->info.frame.buffer[esp_cam_io_parl->info.index++] = current_byte;
            }
            else {
                esp_cam_io_parl->info.captured = false;
                free(esp_cam_io_parl->info.frame.buffer);
                esp_cam_io_parl->info.frame.buffer = NULL;
                esp_cam_io_parl->info.frame.length = 0;
                ESP_EARLY_LOGW(TAG, "JPEG buffer overflow");
                esp_cam_io_parl->info.last_byte = current_byte;
                break;
            }
            if (esp_cam_io_parl->info.last_byte == 0xFF && current_byte == 0xD9 && esp_cam_io_parl->info.index < esp_cam_io_parl->info.frame.length) {
                //ESP_EARLY_LOGI(TAG, "Received JPEG EOI at offset %u / %u (%u bytes)", index, received_bytes - 1, jpeg_capture_data.length);
                esp_cam_io_parl->info.captured = false;

                esp_cam_io_parl_trans_t frame = esp_cam_io_parl->info.frame;
                frame.length = esp_cam_io_parl->info.index;

                esp_cam_io_parl->info.frame.buffer = NULL;
                esp_cam_io_parl->info.frame.length = 0;
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                if (!xQueueSendFromISR(esp_cam_io_parl->queue_handle, &frame, &xHigherPriorityTaskWoken)) {
                    // Drop old and append new frame(s)
                    esp_cam_io_parl_trans_t old_frame;
                    if (xQueueReceiveFromISR(esp_cam_io_parl->queue_handle, &old_frame, &xHigherPriorityTaskWoken)) {
                        free(old_frame.buffer);
                    }
                    xQueueSendFromISR(esp_cam_io_parl->queue_handle, &frame, &xHigherPriorityTaskWoken);
                    //ESP_EARLY_LOGW(TAG, "JPEG queue full, dropping frames");
                }

                if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
            }
        }
        if (!esp_cam_io_parl->info.captured && esp_cam_io_parl->info.last_byte == 0xFF && current_byte == 0xD8) {
            if (!esp_cam_io_parl->info.frame.buffer) {
                esp_cam_io_parl->info.frame.length = esp_cam_io_parl->alloc_size;
                esp_cam_io_parl->info.frame.buffer = heap_caps_malloc(esp_cam_io_parl->info.frame.length, esp_cam_io_parl->alloc_heap_caps);
                if (!esp_cam_io_parl->info.frame.buffer) {
                    esp_cam_io_parl->info.last_byte = current_byte;
                    continue;
                };
                //ESP_EARLY_LOGI(TAG, "Allocated %u bytes", esp_cam_io_parl->alloc_size);
            }
            esp_cam_io_parl->info.captured = true;
            esp_cam_io_parl->info.index = 0;
            esp_cam_io_parl->info.frame.buffer[esp_cam_io_parl->info.index++] = esp_cam_io_parl->info.last_byte;
            esp_cam_io_parl->info.frame.buffer[esp_cam_io_parl->info.index++] = current_byte;
            //ESP_EARLY_LOGI(TAG, "Received JPEG SOI at offset %u / %u", index, received_bytes - 1);
        }
        esp_cam_io_parl->info.last_byte = current_byte;
    }
    return true;
}

static esp_err_t esp_cam_destroy_io_parl(esp_cam_io_parl_handle_t esp_cam_io_parl) {
    if (esp_cam_io_parl->queue_handle) {
        vQueueDeleteWithCaps(esp_cam_io_parl->queue_handle);
    }
    if (esp_cam_io_parl->info.frame.buffer) {
        free(esp_cam_io_parl->info.frame.buffer);
    }
    if (esp_cam_io_parl->payload) {
        free(esp_cam_io_parl->payload);
    }
    if (esp_cam_io_parl->rx_delimiter) {
        parlio_del_rx_delimiter(esp_cam_io_parl->rx_delimiter);
    }
    if (esp_cam_io_parl->rx_unit) {
        parlio_del_rx_unit(esp_cam_io_parl->rx_unit);
    }
    free(esp_cam_io_parl);
    return ESP_OK;
}

esp_err_t esp_cam_new_io_parl(const esp_cam_io_parl_config_t *config, esp_cam_io_parl_handle_t *ret_handle) {
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(config && ret_handle, ESP_ERR_INVALID_ARG, TAG, "Invalid argument");
    ESP_RETURN_ON_FALSE(__builtin_popcount(config->data_width) == 1, ESP_ERR_INVALID_ARG, TAG, "Data line number should be the power of 2 without counting valid signal");
    esp_cam_io_parl_handle_t esp_cam_io_parl = heap_caps_calloc(1, sizeof(esp_cam_io_parl_t), MALLOC_CAP_DEFAULT);
    ESP_GOTO_ON_FALSE(esp_cam_io_parl, ESP_ERR_NO_MEM, err, TAG, "No memory for allocating rx unit");
    esp_cam_io_parl->payload_size = CONFIG_ESP_CAM_IO_PARL_PAYLOAD_SIZE;
    gpio_num_t valid_gpio = config->de_io != -1 ? config->de_io : (config->hsync_io != -1 ? config->hsync_io : -1);
    parlio_rx_unit_config_t rx_unit_config = {
        .trans_queue_depth = 8,
        .max_recv_size = 0xFFFF,
        .data_width = config->data_width,
        .clk_src = PARLIO_CLK_SRC_EXTERNAL,
        .ext_clk_freq_hz = 120 * 1000 * 1000,
        .clk_in_gpio_num = config->pclk_io,
        .clk_out_gpio_num = -1,
        .valid_gpio_num = valid_gpio,
        .flags = {
            .free_clk = config->flags.free_clk,
            .allow_pd = config->flags.allow_pd,
        },
    };
    memcpy(rx_unit_config.data_gpio_nums, config->data_io, PARLIO_RX_UNIT_MAX_DATA_WIDTH * sizeof(gpio_num_t));
    ESP_GOTO_ON_ERROR(parlio_new_rx_unit(&rx_unit_config, &esp_cam_io_parl->rx_unit), err, TAG, "Failed to initialize rx unit");

    if (esp_cam_io_parl->payload) {
        free(esp_cam_io_parl->payload);
        esp_cam_io_parl->payload = NULL;
    }

    if (valid_gpio == -1) {
        parlio_rx_soft_delimiter_config_t rx_delimiter_config = {
            .sample_edge = config->pclk_sample_edge,
            .eof_data_len = esp_cam_io_parl->payload_size,
            .timeout_ticks = 0,
        };
        ESP_GOTO_ON_ERROR(parlio_new_rx_soft_delimiter(&rx_delimiter_config, &esp_cam_io_parl->rx_delimiter), err, TAG, "Failed to initialize rx software delimiter");
        esp_cam_io_parl->use_soft_delimiter = true;
    }
    else if (config->data_width <= 8 && PARLIO_RX_UNIT_MAX_DATA_WIDTH > 8) {
        if (config->de_io != 1) {
            parlio_rx_level_delimiter_config_t rx_delimiter_config = {
                .valid_sig_line_id = PARLIO_RX_UNIT_MAX_DATA_WIDTH - 1,
                .sample_edge = config->pclk_sample_edge,
                .eof_data_len = esp_cam_io_parl->payload_size,
                .timeout_ticks = 0,
                .flags = {
                    .active_low_en = config->flags.invert_de,
                },
            };
            ESP_GOTO_ON_ERROR(parlio_new_rx_level_delimiter(&rx_delimiter_config, &esp_cam_io_parl->rx_delimiter), err, TAG, "Failed to initialize rx level delimiter");
        }
        else if (config->hsync_io != 1) {
            parlio_rx_pulse_delimiter_config_t rx_delimiter_config = {
                .valid_sig_line_id = PARLIO_RX_UNIT_MAX_DATA_WIDTH - 1,
                .sample_edge = config->pclk_sample_edge,
                .eof_data_len = esp_cam_io_parl->payload_size,
                .timeout_ticks = 0,
                .flags = {
                    .start_bit_included = false,
                    .end_bit_included = false,
                    .has_end_pulse = true,
                    .pulse_invert = config->flags.invert_hsync,
                },
            };
            ESP_GOTO_ON_ERROR(parlio_new_rx_pulse_delimiter(&rx_delimiter_config, &esp_cam_io_parl->rx_delimiter), err, TAG, "Failed to initialize rx pulse delimiter");
        }
        esp_cam_io_parl->use_soft_delimiter = false;
    }
    else {
        ESP_LOGE(TAG, "Valid signal can not be used");
        ret = ESP_ERR_INVALID_ARG;
        goto err;
    }
    
    parlio_rx_event_callbacks_t cbs = {
        .on_partial_receive = on_partial_receive_callback,
    };
    ESP_GOTO_ON_ERROR(parlio_rx_unit_register_event_callbacks(esp_cam_io_parl->rx_unit, &cbs, esp_cam_io_parl), err, TAG, "Failed to initialize partial receive callback");

    esp_cam_io_parl->queue_handle = xQueueCreateWithCaps(config->queue_frames, sizeof(esp_cam_io_parl_trans_t), MALLOC_CAP_DEFAULT);
    ESP_GOTO_ON_FALSE(esp_cam_io_parl->queue_handle, ESP_ERR_NO_MEM, err, TAG, "No memory for transaction queue");

    esp_cam_io_parl->payload = heap_caps_malloc(esp_cam_io_parl->payload_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(esp_cam_io_parl->payload, ESP_ERR_NO_MEM, err, TAG, "No memory for payload buffer");
    esp_cam_io_parl->alloc_heap_caps = MALLOC_CAP_INTERNAL;
    
    *ret_handle = esp_cam_io_parl;

    return ESP_OK;

err:
    if (esp_cam_io_parl) {
        esp_cam_destroy_io_parl(esp_cam_io_parl);
    }
    return ret;
}

esp_err_t esp_cam_del_io_parl(esp_cam_io_parl_handle_t esp_cam_io_parl) {
    ESP_RETURN_ON_FALSE(esp_cam_io_parl, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    return esp_cam_destroy_io_parl(esp_cam_io_parl);
}

esp_err_t esp_cam_io_parl_set_alloc_size(esp_cam_io_parl_handle_t esp_cam_io_parl, uint32_t alloc_size, uint32_t heap_caps) {
    ESP_RETURN_ON_FALSE(esp_cam_io_parl && alloc_size > 160, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    esp_cam_io_parl->alloc_size = alloc_size;
    if (heap_caps) {
        esp_cam_io_parl->alloc_heap_caps = heap_caps;
    }
    if (esp_cam_io_parl->info.frame.buffer) { // To avoid misalignment
        esp_cam_io_parl->info.captured = false;
        free(esp_cam_io_parl->info.frame.buffer);
        esp_cam_io_parl->info.frame.buffer = NULL;
        esp_cam_io_parl->info.frame.length = 0;
    }
    return ESP_OK;
}

esp_err_t esp_cam_io_parl_enable(esp_cam_io_parl_handle_t esp_cam_io_parl, bool reset_queue) {
    ESP_RETURN_ON_FALSE(esp_cam_io_parl, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    ESP_RETURN_ON_ERROR(parlio_rx_unit_enable(esp_cam_io_parl->rx_unit, reset_queue), TAG, "Failed to enable PARLIO RX unit");
    if (esp_cam_io_parl->use_soft_delimiter) {
        ESP_RETURN_ON_ERROR(parlio_rx_soft_delimiter_start_stop(esp_cam_io_parl->rx_unit, esp_cam_io_parl->rx_delimiter, true), TAG, "Failed to start PARLIO RX soft delimiter");
    }
    const parlio_receive_config_t receive_config = {
        .delimiter = esp_cam_io_parl->rx_delimiter,
        .flags.partial_rx_en = true,
    };
    ESP_RETURN_ON_ERROR(parlio_rx_unit_receive(esp_cam_io_parl->rx_unit, esp_cam_io_parl->payload, esp_cam_io_parl->payload_size, &receive_config), TAG, "Failed to receive from PARLIO RX");
    return ESP_OK;
}

esp_err_t esp_cam_io_parl_disable(esp_cam_io_parl_handle_t esp_cam_io_parl) {
    if (esp_cam_io_parl->use_soft_delimiter) {
        ESP_RETURN_ON_ERROR(parlio_rx_soft_delimiter_start_stop(esp_cam_io_parl->rx_unit, esp_cam_io_parl->rx_delimiter, false), TAG, "Failed to stop PARLIO RX soft delimiter");
    }
    ESP_RETURN_ON_ERROR(parlio_rx_unit_disable(esp_cam_io_parl->rx_unit), TAG, "Failed to disable PARLIO RX unit");
    return ESP_OK;
}

esp_err_t esp_cam_io_parl_receive(esp_cam_io_parl_handle_t esp_cam_io_parl, esp_cam_io_parl_trans_t *frame, int32_t timeout_ms) {
    ESP_RETURN_ON_FALSE(esp_cam_io_parl && frame, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    TickType_t ticks = timeout_ms < 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    BaseType_t ret = xQueueReceive(esp_cam_io_parl->queue_handle, frame, ticks);
    if (ret == pdFALSE) {
        frame->buffer = NULL;
        frame->length = 0;
    }
    return ret == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t esp_cam_io_parl_receive_from_isr(esp_cam_io_parl_handle_t esp_cam_io_parl, esp_cam_io_parl_trans_t *frame, bool *hp_task_woken) {
    ESP_CAM_IO_PARL_CHECK_ISR(esp_cam_io_parl && frame, ESP_ERR_INVALID_ARG);
    ESP_CAM_IO_PARL_CHECK_ISR(xPortInIsrContext() == pdTRUE, ESP_ERR_INVALID_STATE);
    BaseType_t _hp_task_woken = 0;
    BaseType_t ret = xQueueReceiveFromISR(esp_cam_io_parl->queue_handle, frame, &_hp_task_woken);
    if (hp_task_woken) {
        *hp_task_woken = _hp_task_woken != 0;
    }
    return ret == pdTRUE ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_cam_io_parl_free_buffer(esp_cam_io_parl_trans_t frame) {
    ESP_RETURN_ON_FALSE(frame.buffer, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    free(frame.buffer);
    frame.buffer = NULL;
    frame.length = 0;
    return ESP_OK;
}