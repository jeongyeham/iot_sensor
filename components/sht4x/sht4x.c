/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "i2c_bus.h"
#include "esp_log.h"
#include "esp_system.h"
#include "iot_sensor_hub.h"
#include "sht4x.h"

#define MEASURE_MODE  SHT4x_MEASURE_HPM

static const char *TAG = "SHT4X";

typedef struct {
    i2c_bus_device_handle_t i2c_dev;
    uint8_t dev_addr;
} sht4x_sensor_t;

static const uint8_t measure_cmd_delay_ms[][2] = {
    {SHT4x_MEASURE_HPM, 9}, //
    {SHT4x_MEASURE_MPM, 5}, //
    {SHT4x_MEASURE_LPM, 2}, //
    {SHT4x_MEASURE_HPM_200mW_1s, 9}, //
    {SHT4x_MEASURE_HPM_200mW_0_1s, 9}, //
    {SHT4x_MEASURE_HPM_110mW_1s, 9}, //
    {SHT4x_MEASURE_HPM_110mW_0_1s, 9}, //
    {SHT4x_MEASURE_HPM_20mW_1s, 9}, //
    {SHT4x_MEASURE_HPM_20mW_0_1s, 9} //
};

sht4x_handle_t sht4x_create(i2c_bus_handle_t bus, uint8_t dev_addr) {
    sht4x_sensor_t *sens = (sht4x_sensor_t *) calloc(1, sizeof(sht4x_sensor_t));
    sens->i2c_dev = i2c_bus_device_create(bus, dev_addr, i2c_bus_get_current_clk_speed(bus));
    if (sens->i2c_dev == NULL) {
        free(sens);
        return NULL;
    }
    sens->dev_addr = dev_addr;
    return (sht4x_handle_t) sens;
}

esp_err_t sht4x_delete(sht4x_handle_t *sensor) {
    if (*sensor == NULL) {
        return ESP_OK;
    }
    sht4x_sensor_t *sens = (sht4x_sensor_t *) (*sensor);
    esp_err_t err = i2c_bus_device_delete(&sens->i2c_dev);
    free(sens);
    *sensor = NULL;
    if (err != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t sht4x_write_cmd(sht4x_handle_t sensor, sht4x_cmd_measure_t sht4x_cmd) {
    sht4x_sensor_t *sens = (sht4x_sensor_t *) sensor;
    uint8_t cmd_buffer;
    cmd_buffer = sht4x_cmd;
    esp_err_t ret = i2c_bus_write_bytes(sens->i2c_dev, NULL_I2C_MEM_ADDR, 1, &cmd_buffer);
    return ret;
}

static esp_err_t sht4x_get_data(sht4x_handle_t sensor, uint8_t data_len, uint8_t *data_arr) {
    sht4x_sensor_t *sens = (sht4x_sensor_t *) sensor;
    esp_err_t ret = i2c_bus_read_bytes(sens->i2c_dev, NULL_I2C_MEM_ADDR, data_len, data_arr);
    return ret;
}

static uint8_t CheckCrc8(const uint8_t *message, uint8_t initial_value) {
    uint8_t crc = 0;
    int i = 0, j = 0;
    crc = initial_value;

    for (j = 0; j < 2; j++) {
        crc ^= message[j];
        for (i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31; /*!< 0x31 is Polynomial for 8-bit CRC checksum*/
            } else {
                crc = (crc << 1);
            }
        }
    }

    return crc;
}

esp_err_t sht4x_get_single_shot(sht4x_handle_t sensor, sht4x_cmd_measure_t measure_cmd, float *Tem_val,
                                float *Hum_val) {
    if (!Tem_val || !Hum_val) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buff[6];
    uint16_t tem = 0, hum = 0;
    static float Temperature = 0;
    static float Humidity = 0;

    esp_err_t ret = sht4x_write_cmd(sensor, measure_cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t delay_ms = 0;
    for (size_t i = 0; i < 3; i++) {
        if (measure_cmd == measure_cmd_delay_ms[i][0]) {
            delay_ms = measure_cmd_delay_ms[i][1];
            break;
        } else
            delay_ms = 9;
    }
    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    ret = sht4x_get_data(sensor, 6, buff);
    /* check crc */
    if (ret != ESP_OK || CheckCrc8(buff, 0xFF) != buff[2] || CheckCrc8(&buff[3], 0xFF) != buff[5]) {
        return ESP_FAIL;
    }

    tem = (((uint16_t) buff[0] << 8) | buff[1]);
    Temperature = (175.0f * (float) tem / (float) ((2 << 15) - 1) - 45.0f);
    /*!< T = -45 + 175 * tem / (2^16-1), this temperature conversion formula is for Celsius °C */
    hum = (((uint16_t) buff[3] << 8) | buff[4]);
    Humidity = (125.0f * (float) hum / (float) ((2 << 15) - 1) - 6.0f); /*!< RH = -6 + 125 * hum / (2^16-1) */

    if ((Temperature >= -20) && (Temperature <= 125) && (Humidity >= 0) && (Humidity <= 100)) {
        *Tem_val = Temperature;
        *Hum_val = Humidity;
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

esp_err_t sht4x_soft_reset(sht4x_handle_t sensor) {
    esp_err_t ret = sht4x_write_cmd(sensor, SOFT_RESET_CMD);
    return ret;
}

#ifdef CONFIG_SENSOR_INCLUDED_HUMITURE

static sht4x_handle_t sht4x = NULL;
static bool is_init = false;

esp_err_t humiture_sht4x_init(i2c_bus_handle_t i2c_bus, uint8_t addr) {
    if (is_init || !i2c_bus) {
        return ESP_FAIL;
    }

    sht4x = sht4x_create(i2c_bus, addr);

    if (!sht4x) {
        return ESP_FAIL;
    }

    is_init = true;
    return ESP_OK;
}

esp_err_t humiture_sht4x_deinit(void) {
    if (!is_init) {
        return ESP_FAIL;
    }

    esp_err_t ret = sht4x_delete(&sht4x);

    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    is_init = false;
    return ESP_OK;
}

esp_err_t humiture_sht4x_test(void) {
    if (!is_init) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t humiture_sht4x_acquire_humidity(float *h) {
    if (!is_init) {
        return ESP_FAIL;
    }

    float temperature = 0;
    float humidity = 0;
    esp_err_t ret = sht4x_get_single_shot(sht4x, MEASURE_MODE, &temperature, &humidity);

    if (ret == ESP_OK) {
        *h = humidity;
        return ESP_OK;
    }

    *h = 0;
    return ESP_FAIL;
}

esp_err_t humiture_sht4x_acquire_temperature(float *t) {
    if (!is_init) {
        return ESP_FAIL;
    }

    float temperature = 0;
    float humidity = 0;
    esp_err_t ret = sht4x_get_single_shot(sht4x, MEASURE_MODE, &temperature, &humidity);

    if (ret == ESP_OK) {
        *t = temperature;
        return ESP_OK;
    }

    *t = 0;
    return ESP_FAIL;
}

/**
 *
 */
static humiture_impl_t sht4x_impl = {
    .init = humiture_sht4x_init,
    .deinit = humiture_sht4x_deinit,
    .test = humiture_sht4x_test,
    .acquire_humidity = humiture_sht4x_acquire_humidity,
    .acquire_temperature = humiture_sht4x_acquire_temperature,
};

SENSOR_HUB_DETECT_FN(HUMITURE_ID, sht4x, &sht4x_impl);

#endif
