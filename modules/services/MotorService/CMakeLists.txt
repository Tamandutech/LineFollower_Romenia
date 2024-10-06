idf_component_register(
    REQUIRES "Service" "ESP32MotorControl" "LEDsService" "driver" "DriveEncoder"
    SRCS "src/MotorService.cpp" 
    INCLUDE_DIRS "src"
)