#include "ControlService.hpp"

ControlService::ControlService(std::string name, uint32_t stackDepth, UBaseType_t priority) : Thread(name, stackDepth, priority)
{
    // Atalhos de dados:
    this->robot = Robot::getInstance();
    this->get_Vel = robot->getMotorData();
    this->get_Spec = robot->getSpecification();
    this->get_PID = robot->getPID();
    this->get_Status = robot->getStatus();
    this->get_Angle = robot->getFrontSensors();
    // Atalhos de servicos:
    this->control_motor = MotorService::getInstance();
    this->from_sensor = SensorService::getInstance();
};

void ControlService::Run()
{// Loop do servico   
    // Variavel necerraria para funcionalidade do vTaskDelayUtil, guarda a conGetName().c_str()em de pulsos da CPU
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Loop
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
        if(get_Status->robotState->getData() != CAR_STOPPED){
            ControlePID();
        }
    }
}

float ControlService::CalculatePD(float K_p, float K_d, float errof){
    float PID_now;
    PID_now = (K_p * (errof)) + (K_d * (errof - erro_anterior));
    erro_anterior = errof;
    return PID_now;
}

void ControlService::ControlePID(){
    CarState state = (CarState) get_Status->robotState->getData();
    TrackState line_state = (TrackState) get_Status->TrackStatus->getData();
    float erro = get_Angle->getChannel(0);

    if(state == CAR_STOPPED){
        control_motor->StopMotors();
        
    }else{
        float Kp = get_PID->Kp(line_state)->getData();
        float Kd = get_PID->Kd(line_state)->getData();

        float PID = CalculatePD(Kp, Kd, erro);
        float vel_base = get_Vel->Setpoint(line_state)->getData();
        control_motor->ControlMotors((vel_base + PID), (vel_base - PID));
    }
}