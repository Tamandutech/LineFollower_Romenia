idf_component_register(
    SRCS 
        "src/RobotData.cpp"
        "src/Motor/dataMotor.cpp" 
        "src/Specifications/dataSpec.cpp" 

    REQUIRES
        "DataObjects"

    INCLUDE_DIRS 
        "src"
        "src/Motor"
        "src/Specifications"
)