idf_component_register(
    REQUIRES "Service" "QTRSensors" "DShotRMT" "ESP32MotorControl" "LEDsService"
    SRCS "src/DriveMotors.cpp" 
    INCLUDE_DIRS "src"
)