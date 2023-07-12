idf_component_register(
    SRCS 
        "src/RobotData.cpp"
        "src/Motor/dataMotor.cpp" 
        "src/Specifications/dataSpec.cpp" 
        "src/PID/dataPID.cpp" 
        "src/Status/dataStatus.cpp" 

    REQUIRES
        "DataObjects"

    INCLUDE_DIRS 
        "src"
        "src/Motor"
        "src/Specifications"
        "src/PID"
        "src/Status"
)