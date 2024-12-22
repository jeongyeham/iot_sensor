/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SHT4x_H_
#define _SHT4x_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "freertos/FreeRTOS.h"
#include "i2c_bus.h"
#include "math.h"

typedef enum {

    SOFT_RESET_CMD = 0x94,                   /*!< Command to soft reset*/

    SHT4x_MEASURE_HPM = 0xFD,
    SHT4x_MEASURE_MPM = 0xF6,
    SHT4x_MEASURE_LPM = 0xE0,

    SHT4x_MEASURE_HPM_200mW_1s = 0x39,
    SHT4x_MEASURE_HPM_200mW_0_1s = 0x32,
    SHT4x_MEASURE_HPM_110mW_1s = 0x2F,
    SHT4x_MEASURE_HPM_110mW_0_1s = 0x24,
    SHT4x_MEASURE_HPM_20mW_1s = 0x1E,
    SHT4x_MEASURE_HPM_20mW_0_1s = 0x15,

    SHT4x_READ_SERIAL = 0x89
} sht4x_cmd_measure_t;

typedef enum {
    SHT4x_ADDR_AD = 0x44,
    SHT4x_ADDR_BD = 0x45,
    SHT4x_ADDR_CD = 0x46
} sht4x_set_address_t;

typedef void *sht4x_handle_t;

/**
 * @brief Create sht4x handle_t
 *
 * @param bus sensorice object handle of sht4x
 * @param dev_addr sensorice address
 *
 * @return
 *     - sht4x handle_t
 */
sht4x_handle_t sht4x_create(i2c_bus_handle_t bus, uint8_t dev_addr);

/**
 * @brief Delete sht4x handle_t
 *
 * @param sensor point to sensorice object handle of sht4x
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t sht4x_delete(sht4x_handle_t *sensor);

/**
 * @brief Get temperature and humidity just once
 *
 * @param sensor object handle of shd4x
 * @param measure_cmd measure mode
 * @param Tem_val temperature data
 * @param Hum_val humidity data
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t sht4x_get_single_shot(sht4x_handle_t sensor, sht4x_cmd_measure_t measure_cmd, float *Tem_val, float *Hum_val);

/**
 * @brief Soft reset for sht4x
 *
 * @param sensor object handle of sht4x
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t sht4x_soft_reset(sht4x_handle_t sensor);

/***implements of humiture hal interface****/
#ifdef CONFIG_SENSOR_HUMITURE_INCLUDED_SHT4X

/**
 * @brief initialize sht4x with default configurations
 *
 * @param handle i2c bus handle the sensor will be attached to
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t humiture_sht4x_init(i2c_bus_handle_t handle);

/**
 * @brief de-initialize sht4x
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t humiture_sht4x_deinit(void);

/**
 * @brief test if sht4x is active
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t humiture_sht4x_test(void);

/**
 * @brief acquire relative humidity result one time.
 *
 * @param h point to result data (unit:percentage)
 * @return esp_err_t
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t humiture_sht4x_acquire_humidity(float *h);

/**
 * @brief acquire temperature result one time.
 *
 * @param t point to result data (unit:dCelsius)
 * @return esp_err_t
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t humiture_sht4x_acquire_temperature(float *t);

#endif

#ifdef __cplusplus
}
#endif

#endif
