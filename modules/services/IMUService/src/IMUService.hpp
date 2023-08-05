#ifndef IMU_SERVICE_HPP
#define IMU_SERVICE_HPP

// Bibliotecas para criacao do servico
#include "thread.hpp"
#include "esp_log.h"
#include "Injector/singleton.hpp"
// Bibliotecas de componentes/dados
#include "I2Cdev.h"
#include "LSM6DSR.h"
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
    dataUint16 *get_arrayAcc;
    dataUint16 *get_arrayGyr;

    /* IMU Data */
    int32_t acc[3];
	int32_t gyr[3];

    LSM6DSR imu = LSM6DSR(I2C_ADDR);

    void updateLSM6DS3();
    void saveData();
};

#endif