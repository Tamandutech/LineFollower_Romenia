#include "ControlService.hpp"

ControlService::ControlService(std::string name, uint32_t stackDepth, UBaseType_t priority, BaseType_t coreid) : Thread(name, stackDepth, priority, coreid)
{
    // Atalhos de dados:
    this->robot = Robot::getInstance();
    this->get_Speed = robot->getMotorData();
    this->get_Spec = robot->getSpecification();
    this->get_PID = robot->getPID();
    this->get_Status = robot->getStatus();
    this->get_Angle = robot->getFrontSensors();

    // Atalhos de servicos:
    this->from_sensor = SensorService::getInstance();
    this->motors = MotorService::getInstance();

    esp_log_level_set(name.c_str(), ESP_LOG_INFO);
    // Multiplica a quantidade de revolucoes pela reducao da roda, salvando na variavel MPR
    uint16_t rev = get_Spec->Revolution->getData(); // criada para facilitar a leitura
    uint16_t gear = get_Spec->GearRatio->getData(); // idem
    uint16_t MPR = rev*gear;

    get_Spec->MPR->setData(MPR);
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
        vTaskDelayUntil(&xLastWakeTime, deltaTimeMS_inst / portTICK_PERIOD_MS);
        //ESP_LOGI(GetName().c_str(), "RPMService: %d", eTaskGetState(this->rpm->GetHandle()));
        //ESP_LOGI(GetName().c_str(), "StatusService: %d", eTaskGetState(StatusService::getInstance()->GetHandle()));
        from_sensor->processSLat();
        update_voltage();
        if(get_Status->robotState->getData() != CAR_STOPPED){
            int speed = get_Speed->Brushless_TargetSpeed->getData();
            if(brushless_started){
                ControlePID();
                motors->ControlBrushless(speed);
                //uint32_t time = (uint32_t) (esp_timer_get_time() - lastTime);
                //ESP_LOGI(GetName().c_str(), "Tempo: %lu", time);
            }else{
                brushless_started = motors->StartBrushless(speed);
            }
            
        }else if(brushless_started){
            motors->stop_car();
        }
        
    }
}

void ControlService::ControlePID(){
    //ESP_LOGI(GetName().c_str(), "Início Controle PID.");
    CarState state = (CarState) get_Status->robotState->getData();
    TrackSegment line_state = (TrackSegment) get_Status->TrackStatus->getData();
    TrackSegment RealLine_state = (TrackSegment) get_Status->RealTrackStatus->getData();

    if(get_Status->MappingTuningParam->getData()){
        state = (CarState) CAR_TUNING;
    }

    from_sensor->AngleError();
    float erro = get_Angle->getChannel(0); // em graus
    get_PID->erro->setData(erro);
    get_PID->erroquad->setData((erro*erro));

    if(state == CAR_STOPPED){
        //ESP_LOGI(GetName().c_str(), "Parando motores");
        motors->StopMotors();
    }else{
        PID_Consts PD = get_PID->PD_values(line_state, state);
        
        float Kp = PD.Kp;
        float Kd = PD.Kd;

        float PID = CalculatePD(Kp, Kd, erro);
        get_PID->output->setData(PID);
        
        if(get_Status->VelCalculated->getData()){
            vel_base = get_Speed->vel_mapped->getData();
        }else{
            vel_base = get_Speed->getSpeed(line_state, state)->getData();
        }
        
        get_PID->setpoint->setData(vel_base);
        get_Speed->TargetSpeed->setData(vel_base);

        SaveRPM();
        float RobotLinearSpeed = CalculateRobotLinearSpeed();
        int16_t PositionError = get_Speed->PositionError->getData();
        double kpAcceleration = get_PID->Kp_Acceleration->getData();
        AccelerationControl(RobotLinearSpeed, PositionError, kpAcceleration);
        if((AccelerationStep || DesaccelerationStep) && RealLine_state != SPECIAL_TRACK)
        {   
            if(DesaccelerationStep)
                kpAcceleration = get_PID->Kp_Deceleration->getData();
            vel_base = AccelerationControl(RobotLinearSpeed, PositionError, kpAcceleration);
        }

        float max_angle = get_Spec->MaxAngle_Center->getData();
        bool OpenLoopControl = get_Status->OpenLoopControl->getData();
        float limite = get_Spec->Malha_Aberta->getData();
        if(abs(PositionError) >= 6500 && OpenLoopControl && RealLine_state != SPECIAL_TRACK)
        {
            int8_t min = get_Speed->min->getData();
            int8_t max = get_Speed->max->getData();
            if(PositionError >= 0){
                NewSpeed(min, max);
            }else{
                NewSpeed(max, min);
            }
        }else{
            if(RealLine_state == SPECIAL_TRACK)
            {
                float kpRot = get_PID->Kp_Rotational->getData();
                float kiRot = get_PID->Ki_Rotational->getData();
                float kdRot = get_PID->Kd_Rotational->getData();
                float speedRight = get_Speed->RPMRight_inst->getData();
                float speedLeft = get_Speed->RPMLeft_inst->getData();
                float RotationalSpeed = (speedRight - speedLeft)/ 2.0;
                get_Speed->RotationalSpeed->setData(RotationalSpeed);
                float erroRotation = -RotationalSpeed;
                PID = kpRot * erroRotation + kiRot * somaIntegradorRotacional + kdRot * (erroRotation - erroAnteriorRotacional);
                erroAnteriorRotacional = erroRotation;
                somaIntegradorRotacional += erroRotation;
            }
            else
                somaIntegradorRotacional = 0;
            NewSpeed((vel_base - PID), (vel_base + PID));
        }
        //ESP_LOGI(GetName().c_str(), "RPM_Right = %d, RPM_Left = %d", get_Speed->RPMRight_inst->getData(), get_Speed->RPMLeft_inst->getData());
        //ESP_LOGI(GetName().c_str(), "Erro = %.2f, VelEsq = %.2f, RPMEsq = %.2f, VelDir = %.2f, RPMDir = %.2f", erro, (vel_base - PID), RPM_Left, (vel_base + PID), RPM_Right);
        if(iloop > 50)
        {
            //ESP_LOGI(GetName().c_str(), "TargetSpeed = %d, RobotSpeed = %.2f, PwmBase = %.2f", get_Speed->TargetSpeed->getData(), RobotLinearSpeed, vel_base);
            iloop = 0;
        }
        iloop++;
    }
}

float ControlService::CalculatePD(float K_p, float K_d, float errof){
    float PID_now;
    PID_now = (K_p * (errof)) + (K_d * (errof - erro_anterior));
    erro_anterior = errof;
    return PID_now;
}

void ControlService::NewSpeed(int16_t left_wheel, int16_t right_wheel){
    get_Speed->PWM_right->setData(right_wheel);
    get_Speed->PWM_left->setData(left_wheel);
    //ESP_LOGI(GetName().c_str(), "Chamando MotorService");
    motors->ControlMotors(left_wheel, right_wheel);
}
void ControlService::SaveRPM(){

    int64_t deltaTimeUs = (esp_timer_get_time() - lastTimeWheelsSpeedMeasured);
    lastTimeWheelsSpeedMeasured = esp_timer_get_time();

    uint16_t MPR_Mot = get_Spec->MPR->getData();
    
    motors->read_both();
    int32_t enc_right = get_Speed->EncRight->getData();
    int32_t enc_left = get_Speed->EncLeft->getData();

    float RPM_Right = (((enc_right - lastPulseRight) / (float)MPR_Mot) / ((float)deltaTimeUs / (float)60000000) );
    float RPM_Left  = (((enc_left  - lastPulseLeft)  / (float)MPR_Mot) / ((float)deltaTimeUs / (float)60000000) );
    lastPulseRight = enc_right;
    lastPulseLeft = enc_left;

    get_Speed->RPMRight_inst->setData(RPM_Right);
    get_Speed->RPMLeft_inst->setData(RPM_Left);
    get_Speed->RPMCar_media->setData((RPM_Left+RPM_Right)/2);
}
int16_t ControlService::CalculateRobotLinearSpeed(){
    float MaxMotorSpeed = get_Speed->MotorMaxSpeed->getData();
    int16_t RightWheelSpeed =  get_Speed->RPMRight_inst->getData();
    int16_t LeftWheelSpeed = get_Speed->RPMLeft_inst->getData();
    return  100.0 * ((RightWheelSpeed + LeftWheelSpeed) / (2.0 * MaxMotorSpeed));
}

void ControlService::setAccelerationDirection(float SpeedError)
{
    if(!AccelerationStep && !DesaccelerationStep)
    {
        if(SpeedError > 0)
            AccelerationStep = true;
        else
            DesaccelerationStep = true;
    }
}
void ControlService::EnableAccelerationControlIfNeeded(float linearSpeed, float SpeedError, int16_t PositionError)
{
    int16_t SpeedErrorTreshold = get_Speed->SpeedErrorTreshold->getData();
    int16_t SafePositionError = get_Speed->SafePositionError->getData();
    if(abs(SpeedError) > SpeedErrorTreshold && abs(PositionError) < SafePositionError)
        setAccelerationDirection(SpeedError);
}
void ControlService::DisableAccelerationWhenEnded(float SpeedError, int16_t PositionError)
{
    int16_t SafePositionError = get_Speed->SafePositionError->getData();
    if((AccelerationStep && SpeedError < 0) || abs(PositionError) >= SafePositionError)
        AccelerationStep = false;

    else if(DesaccelerationStep && SpeedError > 0)
        DesaccelerationStep = false;
}

float ControlService::AccelerationControl(int16_t RobotLinearSpeed, int16_t PositionError, double kpAccelerationControl)
{
    float LinearSpeed = get_Speed->TargetSpeed->getData();
    float SpeedError = LinearSpeed - RobotLinearSpeed;
    float newLinearSpeed = LinearSpeed + kpAccelerationControl*SpeedError;

    EnableAccelerationControlIfNeeded(LinearSpeed, SpeedError, PositionError);
    DisableAccelerationWhenEnded(SpeedError, PositionError);

    return newLinearSpeed;

}