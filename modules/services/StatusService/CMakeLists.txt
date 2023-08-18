idf_component_register(
    REQUIRES "Service" "MappingService" "LEDsService"
    SRCS "src/StatusService.cpp" 
    INCLUDE_DIRS "src"
)