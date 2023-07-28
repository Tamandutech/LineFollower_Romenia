idf_component_register(
    REQUIRES "Service" "I2Cdev" "LSM6DS3"
    SRCS "src/IMUService.cpp" 
    INCLUDE_DIRS "src"
)