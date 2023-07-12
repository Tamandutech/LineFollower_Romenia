#ifndef CONTROL_SERVICE_HPP
#define CONTROL_SERVICE_HPP

#include "thread.hpp"
#include "esp_log.h"
#include "Injector/singleton.hpp"
#include "RobotData.h"


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
    
};

#endif