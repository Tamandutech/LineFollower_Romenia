idf_component_register(
    REQUIRES "Service" "QTRSensors" "QTRwithMUX" "LEDsService" "MotorService" 
    SRCS "src/SensorService.cpp" 
    INCLUDE_DIRS "src"
)