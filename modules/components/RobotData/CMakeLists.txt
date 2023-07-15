idf_component_register(
    SRCS 
        "src/RobotData.cpp"
        "src/Motor/dataMotor.cpp" 
        "src/Specifications/dataSpec.cpp" 
        "src/PID/dataPID.cpp" 
        "src/Status/dataStatus.cpp"
        "src/SLatMarks/dataSLatMarks.cpp" 

    REQUIRES
        "DataObjects" "MappingService"

    INCLUDE_DIRS 
        "src"
        "src/Motor"
        "src/Specifications"
        "src/PID"
        "src/Status"
        "src/SLatMarks"
)