idf_component_register(
    REQUIRES "Service" "QTRSensors" "ESP32MotorControl" "LEDsService" "driver"
    SRCS "src/DriveMotors.cpp" 
    INCLUDE_DIRS "src"
)