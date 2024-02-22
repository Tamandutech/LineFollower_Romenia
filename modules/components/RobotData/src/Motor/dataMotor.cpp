#include "dataMotor.h"

dataMotor::dataMotor(std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    //ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);
    dataManager = dataManager->getInstance();

    // Contagem atual dos encoders
    EncRight = new DataAbstract<int32_t>("EncRight", name, 0);
    EncLeft = new DataAbstract<int32_t>("EncLeft", name, 0);
    EncMedia = new DataAbstract<int32_t>("EncMedia", name, 0);

    
    // Valocidades atuais
    RPMRight_inst = new DataAbstract<int16_t>("RPMRight_inst", name, 0);
    dataManager->registerRuntimeData(RPMRight_inst);
    RPMLeft_inst = new DataAbstract<int16_t>("RPMLeft_inst", name, 0);
    dataManager->registerRuntimeData(RPMLeft_inst);
    RPMCar_media = new DataAbstract<int16_t>("RPMCar_media", name, 0);
    dataManager->registerRuntimeData(RPMCar_media);

    // PWM mandado ao motor
    PWM_right = new DataAbstract<int16_t>("PWM_right", name, 0);
    dataManager->registerRuntimeData(PWM_right);
    PWM_left = new DataAbstract<int16_t>("PWM_left", name, 0);
    dataManager->registerRuntimeData(PWM_left);

    TargetSpeed = new DataAbstract<int16_t>("TargetSpeed", name, 0);
    dataManager->registerRuntimeData(TargetSpeed);

    Brushless_TargetSpeed = new DataAbstract<int16_t>("Brushless_TargetSpeed", name, 205);
    dataManager->registerParamData(Brushless_TargetSpeed);

    PositionError = new DataAbstract<int16_t>("PositionError", name, 0);

    MotorMaxSpeed = new DataAbstract<float>("MotorMaxSpeed", name, 3000);
    dataManager->registerParamData(MotorMaxSpeed);

    SpeedErrorTreshold = new DataAbstract<int16_t>("SpeedErrorTreshold", name, 14);
    dataManager->registerParamData(SpeedErrorTreshold);

    SafePositionError = new DataAbstract<int16_t>("SafePositionError", name, 3500);
    dataManager->registerParamData(SafePositionError);

    // Vel. máximo e mínimo
    max = new DataAbstract<int8_t>("max", name, 100);
    dataManager->registerParamData(max);
    min = new DataAbstract<int8_t>("min", name, 0);
    dataManager->registerParamData(min);
    
    DecelerationOffsetGain = new DataAbstract<float>("DecelerationOffsetGain", name, 0.2);
    dataManager->registerParamData(DecelerationOffsetGain);

    // Vel. base para calibracao
    vel_calibrate = new DataAbstract<float>("Velocidade_Calibracao", name, 46);
    dataManager->registerParamData(vel_calibrate);

    // Variável para guardar a velocidade do trecho
    vel_mapped = new DataAbstract<float>("Velocidade_mapeada", name, 0);
    dataManager->registerRuntimeData(vel_mapped);

    // Velocidade rotacional do robô
    RotationalSpeed = new DataAbstract<float>("RotationalSpeed", name, 0);
    dataManager->registerRuntimeData(RotationalSpeed);

    //Setpoints para os tipos de trechos
    Initial_Speed = new DataAbstract<float>("PWM_initial_speed", name, 1100);
    dataManager->registerParamData(Initial_Speed);

    XLong_Line = new DataAbstract<float>("PWM_XLong_line", name, 1000);
    dataManager->registerParamData(XLong_Line);
    Long_Line = new DataAbstract<float>("PWM_Long_line", name, 1000);
    dataManager->registerParamData(Long_Line);
    Medium_Line = new DataAbstract<float>("PWM_Medium_line", name, 1000);
    dataManager->registerParamData(Medium_Line);
    Short_Line = new DataAbstract<float>("PWM_Short_line", name, 1000);
    dataManager->registerParamData(Short_Line);

    XLong_Curve = new DataAbstract<float>("PWM_XLong_curve", name, 1000);
    dataManager->registerParamData(XLong_Curve);
    Long_Curve = new DataAbstract<float>("PWM_Long_curve", name, 1000);
    dataManager->registerParamData(Long_Curve);
    Medium_Curve = new DataAbstract<float>("PWM_Medium_curve", name, 1000);
    dataManager->registerParamData(Medium_Curve);
    Short_Curve = new DataAbstract<float>("PWM_Short_curve", name, 1000);
    dataManager->registerParamData(Short_Curve);

    Zig_zag = new DataAbstract<float>("PWM_Zig_Zag", name, 1000);
    dataManager->registerParamData(Zig_zag);
    Special_Track = new DataAbstract<float>("PWM_Special_Track", name, 1000);
    dataManager->registerParamData(Special_Track);
    Default_speed = new DataAbstract<float>("PWM_Default_Speed", name, 1000);
    dataManager->registerParamData(Default_speed);
    Tunning_speed = new DataAbstract<float>("PWM_Tunning_speed", name, 1000);
    dataManager->registerParamData(Tunning_speed);
    
}

DataAbstract<float> *dataMotor::Track_Setpoint(TrackSegment state)
{// Retorna o valor do setpoint para cada tipo de trecho de pista
    switch(state)
    {
        case LONG_LINE:
            return Long_Line;
            break;
        case MEDIUM_LINE:
            return Medium_Line;
            break;
        case SHORT_LINE:
            return Short_Line;
            break;
        case LONG_CURVE:
            return Long_Curve;
            break;
        case MEDIUM_CURVE:
            return Medium_Curve;
            break;
        case SHORT_CURVE:
            return Short_Curve;
            break;
        case ZIGZAG_TRACK:
            return Zig_zag;
            break;
        case SPECIAL_TRACK:
            return Special_Track;
            break;
        case XLONG_LINE:
            return XLong_Line;
            break;
        case XLONG_CURVE:
            return XLong_Curve;
            break;
        default:
            return Default_speed;
            break;
    }

    //ESP_LOGE(tag, "Estado do Robô ou Objeto PID inválido para esse método: %s:%d para obter o Kp do PID, retornando valor null.", name.c_str(),state);
    return nullptr;
}

DataAbstract<float> *dataMotor::getSpeed(TrackSegment trackSegment, CarState state)
{
    DataAbstract<float> *speedTarget = Default_speed;

    switch (state)
    {
    case CAR_ENC_READING:
        speedTarget = Track_Setpoint(trackSegment);
        break;
    case CAR_ENC_READING_BEFORE_FIRSTMARK:
        speedTarget = Initial_Speed;
        break;
    case CAR_TUNING:
        speedTarget = Tunning_speed;
    default:
        break;
    }
    return speedTarget;
}