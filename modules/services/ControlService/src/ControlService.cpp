#include "ControlService.hpp"

ControlService::ControlService(std::string name, uint32_t stackDepth, UBaseType_t priority) : Thread(name, stackDepth, priority)
{
    // Atalhos de dados:
    this->robot = Robot::getInstance();
    this->get_Vel = robot->getMotorData();
    this->get_Spec = robot->getSpecification();
    this->get_PID = robot->getPID();
    this->get_Status = robot->getStatus();
    // Atalhos de servicos:
    this->control_motor = MotorService::getInstance();
    this->from_sensor = SensorService::getInstance();
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

float ControlService::CalcularPID(float K_p, float K_d, float errof){
    float PID_now;
    PID_now = (K_p * (errof)) + (K_d * (errof - erro_anterior));
    erro_anterior = errof;
    return PID_now;
}

void ControlService::ControlePID(){
    TrackState state = (TrackState) get_Status->robotState->getData();
    float erro = from_sensor->AngleArray[0];

    if(state == CAR_STOPPED){
        control_motor->StopMotors();
        
    }else{
        float Kp = get_PID->Kp(state)->getData();
        float Kd = get_PID->Kd(state)->getData();

        float PID = CalcularPID(Kp, Kd, erro);
        float vel_base = get_Vel->Setpoint(state)->getData();
        control_motor->ControlMotors((vel_base + PID), (vel_base - PID));
    }
}