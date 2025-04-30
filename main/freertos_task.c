/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2025 Jeongyeham
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_lcd_panel_st7789.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "iot_sensor_hub.h"
#include <inttypes.h>
#include <stdio.h>

/////////////////////////////////////////////////////////////////////////SENSOR_HUB////////////////////////////////////////////////////////////////
#define I2C_MASTER_SCL_IO           33           /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           34           /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000      /*!< I2C master clock frequency */

static i2c_bus_handle_t i2c_bus = NULL;
static sensor_handle_t sht4x_handle = NULL;
static sensor_event_handler_instance_t sht4x_handler_handle = NULL;
static void sensor_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);

static const char* sensor_task_TAG = "sensor_hub_task";

static void sensor_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    sensor_data_t *sensor_data = (sensor_data_t *)event_data;

    switch (id) {
    case SENSOR_STARTED:
        ESP_LOGI(sensor_task_TAG, "Timestamp = %llu - %s_0x%x STARTED",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr);
        break;
    case SENSOR_STOPED:
        ESP_LOGI(sensor_task_TAG, "Timestamp = %llu - %s_0x%x STOPPED",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr);
        break;
    case SENSOR_HUMI_DATA_READY:
        ESP_LOGI(sensor_task_TAG, "Timestamp = %llu - %s_0x%x HUMI_DATA_READY - "
                 "humiture=%.2f",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->humidity);
        break;
    case SENSOR_TEMP_DATA_READY:
        ESP_LOGI(sensor_task_TAG, "Timestamp = %llu - %s_0x%x TEMP_DATA_READY - "
                 "temperature=%.2f\n",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->temperature);
        break;
    case SENSOR_ACCE_DATA_READY:
        ESP_LOGI(sensor_task_TAG, "Timestamp = %llu - %s_0x%x ACCE_DATA_READY - "
                 "acce_x=%.2f, acce_y=%.2f, acce_z=%.2f\n",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->acce.x, sensor_data->acce.y, sensor_data->acce.z);
        break;
    case SENSOR_GYRO_DATA_READY:
        ESP_LOGI(sensor_task_TAG, "Timestamp = %llu - %s_0x%x GYRO_DATA_READY - "
                 "gyro_x=%.2f, gyro_y=%.2f, gyro_z=%.2f\n",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->gyro.x, sensor_data->gyro.y, sensor_data->gyro.z);
        break;
    case SENSOR_LIGHT_DATA_READY:
        ESP_LOGI(sensor_task_TAG, "Timestamp = %llu - %s_0x%x LIGHT_DATA_READY - "
                 "light=%.2f",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->light);
        break;
    case SENSOR_RGBW_DATA_READY:
        ESP_LOGI(sensor_task_TAG, "Timestamp = %llu - %s_0x%x RGBW_DATA_READY - "
                 "r=%.2f, g=%.2f, b=%.2f, w=%.2f\n",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->rgbw.r, sensor_data->rgbw.r, sensor_data->rgbw.b, sensor_data->rgbw.w);
        break;
    case SENSOR_UV_DATA_READY:
        ESP_LOGI(sensor_task_TAG, "Timestamp = %llu - %s_0x%x UV_DATA_READY - "
                 "uv=%.2f, uva=%.2f, uvb=%.2f\n",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->uv.uv, sensor_data->uv.uva, sensor_data->uv.uvb);
        break;
    default:
        ESP_LOGI(sensor_task_TAG, "Timestamp = %" PRIi64 " - event id = %" PRIi32, sensor_data->timestamp, id);
        break;
    }
}

void sensor_hub_task(void *pvParameters) {
  const i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);

  const sensor_config_t sht4x_config = {
        .bus = i2c_bus,
        .addr = 0x44,
        .type = HUMITURE_ID,
        .mode = MODE_POLLING,
        .min_delay = 1000,
    };
    iot_sensor_create("sht4x", &sht4x_config, &sht4x_handle);

    sensors_event_loop_create();

    iot_sensor_handler_register(sht4x_handle, sensor_event_handler, &sht4x_handler_handle);

    iot_sensor_start(sht4x_handle);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////LVGL_UI///////////////////////////////////////////////////////

#define DISP_WIDTH 120
#define DISP_HEIGHT 120

static esp_lcd_panel_io_spi_config_t io_config = {};
static esp_lcd_panel_dev_config_t panel_config = {};

void lvgl_ui_task(void *pvParameters) {

    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    esp_err_t err = lvgl_port_init(&lvgl_cfg);

    static lv_disp_t * disp_handle;

    /* LCD IO */
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) 1, &io_config, &io_handle));

    /* LCD driver initialization */
    esp_lcd_panel_handle_t lcd_panel_handle;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &lcd_panel_handle));

    /* Add LCD screen */
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = lcd_panel_handle,
        .buffer_size = DISP_WIDTH*DISP_HEIGHT,
        .double_buffer = true,
        .hres = DISP_WIDTH,
        .vres = DISP_HEIGHT,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
            .swap_bytes = false,
        }
    };
    disp_handle = lvgl_port_add_disp(&disp_cfg);




}
