#common requires
set(requires i2c_bus driver)
#imu sensor requires
#list(APPEND requires mpu6050 lis2dh12)
#humiture sensor requires
list(APPEND requires  sht4x)
#light sensor requires
#list(APPEND requires bh1750 veml6040 veml6075)

idf_component_register(SRC_DIRS "." "hal"
                        INCLUDE_DIRS "include"
                        REQUIRES "${requires}" "esp_timer" "esp_event")
