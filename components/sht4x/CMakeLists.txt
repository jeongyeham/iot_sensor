idf_component_register(SRCS "sht4x.c"
                        INCLUDE_DIRS include
                        REQUIRES i2c_bus esp_timer sensor_hub)

if(CONFIG_SENSOR_INCLUDED_HUMITURE)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u humiture_sht4x_init")
endif()
