idf_component_register(SRCS "app_main.c" "freertos_task.c"
                    INCLUDE_DIRS "."
                    REQUIRES sensor_hub esp_lvgl_port lvgl)

