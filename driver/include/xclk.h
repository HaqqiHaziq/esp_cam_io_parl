#ifndef __CAMERA_XCLK_H__
#define __CAMERA_XCLK_H__
#pragma once

#include "esp_camera_sensor.h"

esp_err_t xclk_timer_conf(int ledc_timer, int xclk_freq_hz);
esp_err_t camera_enable_out_clock(const esp_camera_sensor_config_t *config);
void camera_disable_out_clock(void);
#endif /* __CAMERA_XCLK_H__ */