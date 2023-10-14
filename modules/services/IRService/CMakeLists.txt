idf_component_register(
    REQUIRES "Service" "IRRemote_EspIDFLib" "LEDsService"
    SRCS "src/IRService.cpp" 
    INCLUDE_DIRS "src"
)