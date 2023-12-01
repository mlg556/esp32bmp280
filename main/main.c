/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <inttypes.h>
#include <stdio.h>

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include "bmp280.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

// SCL IO19, SDA IO18
#define SCL_GPIO 19
#define SDA_GPIO 18
#define LED_GPIO 3

static const char* TAG = "Hello";

BMP280 bmp;
static uint8_t s_led_state = 0;

void app_main(void) {
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = SDA_GPIO,
        .scl_io_num       = SCL_GPIO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_port_t i2c_num = I2C_NUM_0;

    i2c_param_config(i2c_num, &conf);
    i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0);

    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    if (bmp280_init(&bmp, i2c_num))
        ESP_LOGI(TAG, "BMP280 Init SUCCESS!\n");
    else
        ESP_LOGE(TAG, "BMP280 Init ERR!!!\n");

    while (1) {
        gpio_set_level(LED_GPIO, s_led_state);
        s_led_state = !s_led_state;

        bmp280_read_data(&bmp);
        ESP_LOGI(TAG, "%.02f C | %.02f ATM\n", bmp.temperature, bmp.pressure * BMP280_CONV_PA_ATM);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
