idf_component_register(
    REQUIRES "Service" "SensorService" "DriveEncoder" "DriveMotors"
    SRCS "src/ControlService.cpp" 
    INCLUDE_DIRS "src"
)