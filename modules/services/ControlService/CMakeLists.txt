idf_component_register(
    REQUIRES "Service" "SensorService" "IMUService" "DriveEncoder" "DriveMotors" "MotorService"
    SRCS "src/ControlService.cpp" 
    INCLUDE_DIRS "src"
)