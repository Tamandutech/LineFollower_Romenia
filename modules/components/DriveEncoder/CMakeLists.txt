idf_component_register(
    REQUIRES "Service" "ESP32Encoder"
    SRCS "src/DriveEncoder.cpp" 
    INCLUDE_DIRS "src"
)