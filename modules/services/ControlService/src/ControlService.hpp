#ifndef CONTROL_SERVICE_HPP
#define CONTROL_SERVICE_HPP

// Bibliotecas para criacao do servico
#include "thread.hpp"
#include "esp_log.h"
#include "Injector/singleton.hpp"
// Bibliotecas de componentes/dados
#include "RobotData.h"
#include "DriveEncoder.h"
// Bibliotecas do C++
#include <stdlib.h>
// Bibliotecas de servicos
#include "DriveMotors.h"
#include "SensorService.hpp"


using namespace cpp_freertos;

class ControlService : public Thread, public Singleton<ControlService>
{
public:
    ControlService(std::string name, uint32_t stackDepth, UBaseType_t priority, BaseType_t coreid);

    void Run() override;

private:

    // Atalhos para facilitar a escrita do código
    Robot *robot;
    dataMotor *get_Speed;
    dataSpec *get_Spec;
    dataPID *get_PID;
    dataStatus *get_Status;
    dataFloat *get_Angle;
    //MotorService *control_motor;
    SensorService *from_sensor;

    DriveEncoder encs;
    DriveMotors motors;

    int8_t speedMin = 0;
    int8_t speedMax = 0;
    uint8_t nloops = 0;

    float erro_anterior = 0;
    float vel_base = 0;
    float lastPulseRight = 0;
    float lastPulseLeft = 0;
    uint16_t deltaTimeMS_inst = 10;
    TickType_t lastTicksRevsCalc = 0;
    int count =0;


    bool brushless_started = false;

    void ControlePID();
    float CalculatePD(float K_p, float K_d, float errof);
    void StopCar();
    void NewSpeed(int16_t left_wheel, int16_t right_wheel);
    void SaveRPM();
};

#endif