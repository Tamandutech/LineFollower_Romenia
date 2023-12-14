idf_component_register(
    REQUIRES "Service" "MotorService" "RPMService" "SensorService" "StatusService"
    SRCS "src/ControlService.cpp" 
    INCLUDE_DIRS "src"
)