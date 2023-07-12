#include "ControlService.hpp"

ControlService::ControlService(std::string name, uint32_t stackDepth, UBaseType_t priority) : Thread(name, stackDepth, priority)
{
    // Atalhos:
    this->robot = Robot::getInstance();
    this->get_Vel = robot->getMotorData();
    this->get_Spec = robot->getSpecification();
};

void ControlService::Run()
{
    // Loop do servico, em desenvolvimento
    
    // Variavel necerraria para funcionaliade do vTaskDelayUtil, guarda a contagem de pulsos da CPU
    //TickType_t xLastWakeTime = xTaskGetTickCount();

    // Loop
    for (;;)
    {
        
    }
}

