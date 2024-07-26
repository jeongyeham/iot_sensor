idf_component_register(SRCS "smart_config.c"
        INCLUDE_DIRS include
        REQUIRES wpa_supplicant nvs_flash esp_wifi)
