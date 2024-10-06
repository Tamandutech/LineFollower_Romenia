idf_component_register(
    REQUIRES "Service" "MappingService" "LEDsService" "esp_timer"
    SRCS "src/StatusService.cpp" 
    INCLUDE_DIRS "src"
)