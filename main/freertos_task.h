//
// Created by jeong on 2025/4/27.
//

#ifndef FREERTOS_TASK_H
#define FREERTOS_TASK_H

void sensor_hub_task(void *pvParameters);
void lvgl_ui_task(void *pvParameters);
void mqtt5_start_task(void *pvParameters);
void wifi_task(void *pvParameters);
void smart_config_task(void *pvParameters);

#endif //FREERTOS_TASK_H
