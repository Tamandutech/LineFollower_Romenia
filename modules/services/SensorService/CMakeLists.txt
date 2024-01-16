idf_component_register(
    REQUIRES "Service" "QTRSensors" "QTRwithMUX" "LEDsService" "DriveEncoder" "DriveMotors" 
    SRCS "src/SensorService.cpp" 
    INCLUDE_DIRS "src"
)