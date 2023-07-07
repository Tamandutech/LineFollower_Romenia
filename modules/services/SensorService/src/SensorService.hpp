#ifndef SENSOR_SERVICE_HPP
#define SENSOR_SERVICE_HPP

#include "thread.hpp"
#include "esp_log.h"
#include "Injector/singleton.hpp"
#include "QTRSensors.h"
#include "bitset"// biblioteca que transforma um número decimal para binário
#include "QTRwithMUX.h" // biblioteca própria
#include "MotorService.hpp"
#include "RPMService.hpp"
#include "RobotData.h"


using namespace cpp_freertos;

class SensorService : public Thread, public Singleton<SensorService>
{
public:
    SensorService(std::string name, uint32_t stackDepth, UBaseType_t priority);

    void Run() override;
    void AngleError();

private:

    // Atalhos para facilitar a escrita do código
    Robot *robot;
    dataMotor *get_Vel;
    dataSpec *get_Spec;
    MotorService *motor;
    RPMService *rpm;
    
    // Objetos usados no serviço:
    QTRSensors sArray[sQuant];
    QTRSensors sLat;
    QTRwithMUX MUX;

    // Variaveis do servico
    uint16_t MPR_Mot = 0;
    uint16_t rev = 0;
    uint16_t gear = 0;


    // Variaveis
    int tempo_cal = 7000; // tempo de calibracao, passar para RobotData

    void auto_calibrate();
};

#endif