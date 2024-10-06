idf_component_register(SRCS "src/QTRSensors.cpp"
INCLUDE_DIRS "src"
REQUIRES "MCP3008" "esp_adc" "esp_timer" "freertos" "driver")