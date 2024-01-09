idf_component_register(
    REQUIRES "Service" "MotorService" "RPMService" "SensorService" "DriveEncoder"
    SRCS "src/ControlService.cpp" 
    INCLUDE_DIRS "src"
)