idf_component_register(
    REQUIRES "Service" "ESP32Encoder"
    SRCS "src/RPMService.cpp" 
    INCLUDE_DIRS "src"
)