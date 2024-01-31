idf_component_register(
    REQUIRES "Service" "LEDsService" "IR_nec_transceiver"
    SRCS "src/IRService.cpp" 
    INCLUDE_DIRS "src"
)