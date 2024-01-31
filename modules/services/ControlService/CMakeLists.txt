idf_component_register(
    REQUIRES "Service" "SensorService" "IMUService" "DriveEncoder" "MotorService"
    SRCS "src/ControlService.cpp" 
    INCLUDE_DIRS "src"
)