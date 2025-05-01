/*
 * SPDX-FileCopyrightText: 2022-2024 Jeongyeham
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_event.h"
#include "freertos_task.h"


static const char *TAG = "main";

_Noreturn void app_main(void) {
    TaskHandle_t net_task_handle = NULL, mqtt5_handle = NULL, sensor_hub_handle = NULL, lvgl_ui_handle = NULL;
    EventGroupHandle_t s_sys_event_group = xEventGroupCreate();

    ESP_LOGI(TAG, "Ready to create task(s)");

    xTaskCreatePinnedToCore(wifi_task, "wifi_task", 4096, s_sys_event_group, 0, &net_task_handle, PRO_CPU_NUM);
    //xTaskCreatePinnedToCore(mqtt5_start_task, "mqtt5_task", 4096, s_sys_event_group, 0, &mqtt5_handle, APP_CPU_NUM);
    //xTaskCreatePinnedToCore(sensor_hub_task,"sensor_hub_task", 4096,NULL,0,&sensor_hub_handle,APP_CPU_NUM);
    //xTaskCreatePinnedToCore(lvgl_ui_task,"lvgl_ui_task", 4096,NULL,0,&lvgl_ui_handle,APP_CPU_NUM);

    ESP_LOGI(TAG, "Already created %d task(s)", uxTaskGetNumberOfTasks());

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

