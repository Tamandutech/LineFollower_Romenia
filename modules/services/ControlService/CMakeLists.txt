idf_component_register(
    REQUIRES "Service" "MotorService" "SensorService"
    SRCS "src/ControlService.cpp" 
    INCLUDE_DIRS "src"
)