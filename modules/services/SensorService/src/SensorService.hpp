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
#include "cmath" // biblioteca do C++ para funcoes trigonometricas


using namespace cpp_freertos;

class SensorService : public Thread, public Singleton<SensorService>
{
public:
    SensorService(std::string name, uint32_t stackDepth, UBaseType_t priority);

    void Run() override;
    void AngleError();

    float AngleArray[sQuantReading]; // array que salva as N ultimas leituras do sensor

private:

    // Atalhos para facilitar a escrita do código
    Robot *robot;
    dataMotor *get_Vel;
    dataSpec *get_Spec;
    MotorService *motor;
    RPMService *rpm;
    
    // Objetos usados no serviço:
    QTRSensors sArray[sQuant]; // sensores frontais
    QTRSensors sLat; // sensores laterais
    QTRSensors sCenter; // sensores no centro do corpo
    QTRwithMUX MUX; // objeto para acessar as funcoes do QTRwithMUX


    // Variaveis do servico
    uint16_t MPR_Mot = 0; // MPR atual do robo
    


    // Variaveis
    int tempo_cal = 7000; // tempo de calibracao, passar para RobotData

    void auto_calibrate();
};

#endif