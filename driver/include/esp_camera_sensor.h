#ifndef _ESP_CAMERA_SENSOR_H_
#define _ESP_CAMERA_SENSOR_H_

#pragma once

#include "esp_err.h"
#include "soc/gpio_num.h"
#include "driver/ledc.h"
#include "sensor.h"
#include "sys/time.h"
#include "sdkconfig.h"

/**
 * @brief Configuration structure for camera sensor initialization
 */
typedef struct {
    gpio_num_t pwdn_io;             /*!< GPIO pin for camera power down line */
    gpio_num_t reset_io;            /*!< GPIO pin for camera reset line */
    gpio_num_t xclk_io;             /*!< GPIO pin for camera XCLK line */

    gpio_num_t sda_io;              /*!< GPIO pin for camera SDA line */
    gpio_num_t scl_io;              /*!< GPIO pin for camera SCL line */

    uint32_t xclk_hz;               /*!< Frequency of XCLK signal, in Hz */

    camera_pixformat_t pixel_format;       /*!< Format of the pixel data: PIXFORMAT_ + YUV422|GRAYSCALE|RGB565|JPEG  */
    camera_framesize_t frame_size;         /*!< Size of the output image: FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA  */

    ledc_timer_t ledc_timer;        /*!< LEDC timer to be used for generating XCLK  */
    ledc_channel_t ledc_channel;    /*!< LEDC channel to be used for generating XCLK  */

    int jpeg_quality;               /*!< Quality of JPEG output. 0-63 lower means higher quality  */
    int sccb_i2c_port;              /*!< If pin_sccb_sda is -1, use the already configured I2C bus by number */
} esp_camera_sensor_config_t;

/**
 * @brief Data structure of image
 */
typedef struct {
    size_t width;               /*!< Width of the image in pixels */
    size_t height;              /*!< Height of the image in pixels */
    camera_pixformat_t format;         /*!< Format of the pixel data */
} image_info_t;

#define ESP_ERR_CAMERA_BASE 0x20000
#define ESP_ERR_CAMERA_NOT_DETECTED             (ESP_ERR_CAMERA_BASE + 1)
#define ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE (ESP_ERR_CAMERA_BASE + 2)
#define ESP_ERR_CAMERA_FAILED_TO_SET_OUT_FORMAT (ESP_ERR_CAMERA_BASE + 3)
#define ESP_ERR_CAMERA_NOT_SUPPORTED            (ESP_ERR_CAMERA_BASE + 4)

/**
 * @brief Initialize the camera driver
 *
 * This function detects and configures camera over I2C interface,
 *
 * Currently this function can only be called once and there is
 * no way to de-initialize this module.
 *
 * @param[in] config  Camera configuration parameters
 * @return ESP_OK on success
 */
esp_err_t esp_camera_sensor_init(const esp_camera_sensor_config_t *config);

/**
 * @brief Deinitialize the camera driver
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the driver hasn't been initialized yet
 */
esp_err_t esp_camera_sensor_deinit(void);

/**
 * @brief Obtain image resolution for frame allocation.
 *
 * @return pointer to the image resolution
 */
image_info_t *esp_camera_sensor_get_image(void);

/**
 * @brief Get a pointer to the image sensor control structure
 *
 * @return pointer to the sensor
 */
camera_sensor_t *esp_camera_sensor_get(void);

/**
 * @brief Save camera settings to non-volatile-storage (NVS)
 *
 * @param[in] key   A unique nvs key name for the camera settings
 */
esp_err_t esp_camera_sensor_save_to_nvs(const char *key);

/**
 * @brief Load camera settings from non-volatile-storage (NVS)
 *
 * @param[in] key   A unique nvs key name for the camera settings
 */
esp_err_t esp_camera_sensor_load_from_nvs(const char *key);

#endif /* _ESP_CAMERA_SENSOR_H_ */