#include "RPMService.hpp"

RPMService::RPMService(std::string name, uint32_t stackDepth, UBaseType_t priority) : Thread(name, stackDepth, priority)
{
    // Atalhos:
    this->robot = Robot::getInstance();
    this->get_Vel = robot->getMotorData();
    this->get_Spec = robot->getSpecification();

    // GPIOs dos encoders dos encoders dos motores
    enc_motEsq.attachFullQuad(enc_eq_B, enc_eq_A);
    enc_motDir.attachFullQuad(enc_dir_B, enc_dir_A);
};

void RPMService::Run()
{
    // Loop do servico, em desenvolvimento
    
    // Variavel necerraria para funcionaliade do vTaskDelayUtil, guarda a contagem de pulsos da CPU
    //TickType_t xLastWakeTime = xTaskGetTickCount();

    // Loop
    for (;;)
    {
        
    }
}

void RPMService::ReadBoth()
{
    lastPulseRight = enc_motDir.getCount();
    lastPulseLeft = enc_motEsq.getCount();

    SaveEncData(lastPulseRight, lastPulseLeft);
}

void RPMService::ResetCount()
{
    enc_motEsq.clearCount();
    enc_motDir.clearCount();
    
    SaveEncData(0, 0);
}

void RPMService::SaveEncData(int32_t pulseRight, int32_t pulseLeft)
{
    get_Vel->EncRight->setData(pulseRight);
    get_Vel->EncLeft->setData(pulseLeft);
    get_Vel->EncMedia->setData( (pulseRight+pulseLeft)/2 );
}