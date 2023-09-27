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
    this->rpm = RPMService::getInstance();

    esp_log_level_set(name.c_str(), ESP_LOG_INFO);
};

void ControlService::Run()
{// Loop do servico   
    //ESP_LOGI(GetName().c_str(), "Início ControlService");
    // Variavel necerraria para funcionalidade do vTaskDelayUtil, guarda a conGetName().c_str()em de pulsos da CPU
    TickType_t xLastWakeTime = xTaskGetTickCount();
    vel_base = 0;

    // Loop
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_PERIOD_MS);
        if(get_Status->robotState->getData() != CAR_STOPPED){
            ControlePID();
        }else{
            control_motor->StopMotors();
            rpm->ResetCount();
            erro_int_linear_r = 0;
            erro_int_linear_l = 0;
            erro_ant_linear_r = 0;
            erro_ant_linear_l = 0;
        }
    }
}

float ControlService::CalculatePD(float K_p, float K_d, float errof){
    float PID_now;
    PID_now = (K_p * (errof)) + (K_d * (errof - erro_anterior));
    erro_anterior = errof;
    return PID_now;
}

float ControlService::ControlMotors(float RPM_insta, float vel_motor, bool right){
    float K_p = get_PID->Kp_Linear->getData();
    float K_d = get_PID->Kd_Linear->getData();
    float K_i = get_PID->Ki_Linear->getData();
    float PID_now;
    float erro = vel_motor - RPM_insta;
    if(right){
        erro_int_linear_r += erro;
        erro_int_linear_r = constrain(erro_int_linear_r, -100, 100);
        PID_now = (K_p * (erro)) + (K_d * (erro - erro_ant_linear_r)) + (K_i * erro_int_linear_r);
        erro_ant_linear_r = erro;
    }else{
        erro_int_linear_l += erro;
        erro_int_linear_l = constrain(erro_int_linear_l, -100, 100);
        PID_now = (K_p * (erro)) + (K_d * (erro - erro_ant_linear_l)) + (K_i * erro_int_linear_l);
        erro_ant_linear_l = erro;
    }
    
    return (PID_now);
}

void ControlService::ControlePID(){
    //ESP_LOGI(GetName().c_str(), "Início Controle PID.");
    CarState state = (CarState) get_Status->robotState->getData();
    TrackState line_state = (TrackState) get_Status->TrackStatus->getData();

    from_sensor->AngleError();
    float erro = get_Angle->getChannel(0); // em graus
    get_PID->erro->setData(erro);
    get_PID->erroquad->setData((erro*erro));
    
    //float erro = from_sensor->AngleArray[0];

    if(state == CAR_STOPPED){
        control_motor->StopMotors();
        
    }else{
        float Kp = get_PID->Kp(line_state)->getData();
        float Kd = get_PID->Kd(line_state)->getData();
        uint16_t MPR_Mot = get_Spec->MPR->getData();
        deltaTimeMS_inst = (xTaskGetTickCount() - lastTicksRevsCalc) * portTICK_PERIOD_MS;
        lastTicksRevsCalc = xTaskGetTickCount();
        float PID = CalculatePD(Kp, Kd, erro);
        get_PID->output->setData(PID);
        //vel_base += 1;
        rpm->ReadBoth();
        int32_t enc_right = get_Vel->EncRight->getData();
        int32_t enc_left = get_Vel->EncLeft->getData();

        float RPM_Right = (((float)enc_right - lastPulseRight)*(float)60000) / ((float)MPR_Mot*(float)deltaTimeMS_inst);
        float RPM_Left  = (((float)enc_left  - lastPulseLeft)*(float)60000)  / ((float)MPR_Mot*(float)deltaTimeMS_inst);
        
        get_Vel->RPMRight_inst->setData(RPM_Right);
        get_Vel->RPMLeft_inst->setData(RPM_Left);
        get_Vel->RPMCar_media->setData((RPM_Left+RPM_Right)/2);

        lastPulseRight = enc_right;
        lastPulseLeft = enc_left;
        
        vel_base = get_Vel->Setpoint(line_state)->getData();
        get_PID->setpoint->setData(vel_base);

        float vel_right = vel_base + PID;
        float vel_left = vel_base - PID;

        vel_right = ControlMotors(RPM_Right, vel_right, 1);
        vel_left = ControlMotors(RPM_Left, vel_left, 0);

        if(count > 50){
            //ESP_LOGI(GetName().c_str(), "RPM_R = %.2f, RPM_L = %.2f", RPM_Right, RPM_Left);
            count = 0;
        }else{ count++; }
        //ESP_LOGI(GetName().c_str(), "Erro = %.2f", erro);
        control_motor->ControlMotors(vel_left, vel_right);
    }
}