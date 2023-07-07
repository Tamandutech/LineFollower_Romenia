idf_component_register(
    REQUIRES "Service" "MotorService" "RPMService" "QTRSensors" "QTRwithMUX"
    SRCS "src/SensorService.cpp" 
    INCLUDE_DIRS "src"
)