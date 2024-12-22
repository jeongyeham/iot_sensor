set(SRC_FILE "i2c_bus_v2.c")
set(REQ esp_driver_i2c)

idf_component_register(SRCS ${SRC_FILE}
                        INCLUDE_DIRS "include"
                        REQUIRES ${REQ})
