#ifndef RPM_SERVICE_H
#define RPM_SERVICE_H

#include "thread.hpp"
#include "Injector/singleton.hpp"
#include "RobotData.h"

#include "ESP32Encoder.h"
#include "math.h" 

using namespace cpp_freertos;

#include "esp_log.h"

class RPMService : public Thread, public Singleton<RPMService>
{
public:
    RPMService(std::string name, uint32_t stackDepth, UBaseType_t priority, BaseType_t coreid);

    ESP32Encoder enc_motEsq;
    ESP32Encoder enc_motDir;

    void Run() override;
    void ResetCount();
    void ReadBoth();

private:
    // Atalhos para facilitar a escrita do código
    Robot *robot;
    dataMotor *get_Vel;
    dataSpec *get_Spec;

    int32_t lastPulseRight = 0;
    int32_t lastPulseLeft = 0;

    // Componente de gerenciamento dos encoders
    void SaveEncData(int32_t pulseRight, int32_t pulseLeft);
};

#endif