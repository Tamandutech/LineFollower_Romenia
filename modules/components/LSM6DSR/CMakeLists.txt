idf_component_register(
    SRCS 
        "src/LSM6DSR.cpp"
        "src/lsm6dsr_reg.c"
    INCLUDE_DIRS 
        "src"
    REQUIRES "I2Cdev"
)