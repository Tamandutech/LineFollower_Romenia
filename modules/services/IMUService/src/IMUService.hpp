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
    IMUService(std::string name, uint32_t stackDepth, UBaseType_t priority, BaseType_t coreid);

    void Run() override;

private:

    // Atalhos para facilitar a escrita do código
    Robot *robot;
    dataInt32 *get_arrayAcc;
    dataInt32 *get_arrayGyr;

    /* IMU Data */
    int32_t acc[3];
	int32_t gyr[3];

    //LSM6DSR imu = LSM6DSR(I2C_ADDR);

    void updateIMU();
    void saveData();
};

#endif