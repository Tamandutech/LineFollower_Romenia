idf_component_register(
    REQUIRES "Service" "ESP32Encoder" "driver"
    SRCS "src/DriveEncoder.cpp" 
    INCLUDE_DIRS "src"
)