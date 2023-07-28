#ifndef IMU_SERVICE_HPP
#define IMU_SERVICE_HPP

// Bibliotecas para criacao do servico
#include "thread.hpp"
#include "esp_log.h"
#include "Injector/singleton.hpp"
// Bibliotecas de componentes/dados
#include "I2Cdev.h"
#include "LSM6DS3.h"
#include "RobotData.h"

#define I2C_ADDR 0x6A //CONFIG_I2C_ADDRESS, configurar depois

using namespace cpp_freertos;

class IMUService : public Thread, public Singleton<IMUService>
{
public:
    IMUService(std::string name, uint32_t stackDepth, UBaseType_t priority);

    void Run() override;

private:

    // Atalhos para facilitar a escrita do código
    Robot *robot;
    dataMotor *get_Vel;
    dataSpec *get_Spec;
    dataStatus *get_Status;
    dataSLatMarks *get_Marks;
    dataSensor *get_latArray;
    dataSensor *get_centerArray;

    /* IMU Data */
    float accX = 0, accY = 0, accZ = 0; // double ?
    float gyroX = 0, gyroY = 0, gyroZ = 0; // double ?
    
    LSM6DS3 imu = LSM6DS3(I2C_ADDR);

    void updateLSM6DS3();
    void start_i2c(void);
};

#endif