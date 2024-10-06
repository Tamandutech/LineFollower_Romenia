idf_component_register(
    REQUIRES "Service" "I2Cdev" "LSM6DSR" "LEDsService"
    SRCS "src/IMUService.cpp" 
    INCLUDE_DIRS "src"
)