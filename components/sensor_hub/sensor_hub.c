#include <sys/cdefs.h>
//
// Created by jeong on 2024/7/26.
//

#include <esp_log.h>
#include "iot_sensor_hub.h"
#include "i2c_bus.h"

// 定义I2C端口号，例如I2C_NUM_0或I2C_NUM_1
#define I2C_NUM I2C_NUM_0

// 定义I2C通信的SCL和SDA GPIO引脚编号
#define I2C_SCL_IO_NUM 33
#define I2C_SDA_IO_NUM 34

// 定义I2C时钟频率（Hz），例如100000表示100kHz
#define I2C_CLK_FREQ 100000

#define SENSOR_PERIOD  1000

const char *TAG = "sensor_hub";

_Noreturn void sensor_hub_task(void *pvParameters) {

// 静态初始化i2c_config_t结构体
    static const i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,        // 主模式
            .sda_io_num = I2C_SDA_IO_NUM,   // SDA GPIO编号
            .scl_io_num = I2C_SCL_IO_NUM,   // SCL GPIO编号
            .sda_pullup_en = GPIO_PULLUP_DISABLE,    // SDA上拉使能
            .scl_pullup_en = GPIO_PULLUP_DISABLE,    // SCL上拉使能
            .master.clk_speed = I2C_CLK_FREQ,       // I2C时钟频率
            // 其他成员可以根据需要进行配置，例如:
            // .clk_flags = 0, // 时钟配置标志，通常默认为0
    };
    bus_handle_t i2c_bus_handle = i2c_bus_create(I2C_NUM_0, &conf);


/*create sensors based on sensor scan result*/
    sensor_info_t *sensor_infos[10];
    sensor_handle_t sensor_handle[10] = {NULL};
    sensor_config_t sensor_config = {
            .bus = i2c_bus_handle, /*which bus sensors will connect to*/
            .mode = MODE_POLLING, /*data acquire mode*/
            .min_delay = SENSOR_PERIOD /*data acquire period*/
    };
    int num = iot_sensor_scan(i2c_bus_handle, sensor_infos, 10); /*scan for valid sensors based on active i2c address*/
    for (size_t i = 0; i < num && i < 10; i++) {

        if (ESP_OK != iot_sensor_create(sensor_infos[i]->sensor_id, &sensor_config,
                                        &sensor_handle[i])) { /*create a sensor with specific sensor_id and configurations*/
            ESP_LOGE(TAG, "Create error!");
        }

        iot_sensor_start(
                sensor_handle[i]); /*start a sensor, data ready events will be posted once data acquired successfully*/
        ESP_LOGI(TAG, "%s (%s) created", sensor_infos[i]->name, sensor_infos[i]->desc);
    }

    while (1) {
        vTaskDelay(1000);
    }
}