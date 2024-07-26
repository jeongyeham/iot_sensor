idf_component_register(SRCS "sht4x_test.c"
                        INCLUDE_DIRS .
                        REQUIRES unity sht3x i2c_bus)
