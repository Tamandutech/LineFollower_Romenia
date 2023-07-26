idf_component_register(
    REQUIRES "Service" "QTRSensors" "DShotRMT"
    SRCS "src/MotorService.cpp" 
    INCLUDE_DIRS "src"
)