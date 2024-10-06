idf_component_register(
    REQUIRES "Service" "SensorService" "IMUService" "DriveEncoder" "MotorService" "CMDWrapper"
    SRCS "src/ControlService.cpp" 
    INCLUDE_DIRS "src"
)