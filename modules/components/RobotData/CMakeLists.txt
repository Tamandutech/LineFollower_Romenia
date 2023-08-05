idf_component_register(
    SRCS 
        "src/RobotData.cpp"
        "src/Motor/dataMotor.cpp" 
        "src/Specifications/dataSpec.cpp" 
        "src/PID/dataPID.cpp" 
        "src/Status/dataStatus.cpp"
        "src/SLatMarks/dataSLatMarks.cpp" 
        "src/uint16Vector/dataUint16.cpp" 
        "src/int32Vector/dataInt32.cpp" 

    REQUIRES
        "DataObjects" "MappingService"

    INCLUDE_DIRS 
        "src"
        "src/Motor"
        "src/Specifications"
        "src/PID"
        "src/Status"
        "src/SLatMarks"
        "src/uint16Vector" 
        "src/int32Vector"
)