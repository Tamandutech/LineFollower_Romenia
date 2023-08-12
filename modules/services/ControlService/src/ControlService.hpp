#ifndef CONTROL_SERVICE_HPP
#define CONTROL_SERVICE_HPP

// Bibliotecas para criacao do servico
#include "thread.hpp"
#include "esp_log.h"
#include "Injector/singleton.hpp"
// Bibliotecas de componentes/dados
#include "RobotData.h"
// Bibliotecas de servicos
#include "MotorService.hpp"
#include "SensorService.hpp"


using namespace cpp_freertos;

class ControlService : public Thread, public Singleton<ControlService>
{
public:
    ControlService(std::string name, uint32_t stackDepth, UBaseType_t priority);

    void Run() override;

private:

    // Atalhos para facilitar a escrita do código
    Robot *robot;
    dataMotor *get_Vel;
    dataSpec *get_Spec;
    dataPID *get_PID;
    dataStatus *get_Status;
    dataFloat *get_Angle;
    MotorService *control_motor;
    SensorService *from_sensor;

    float erro_anterior = 0;

    void ControlePID();
    float CalculatePD(float K_p, float K_d, float errof);
    
};

#endif