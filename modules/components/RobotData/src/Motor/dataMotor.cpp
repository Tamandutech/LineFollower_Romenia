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

    Brushless_TargetSpeed = new DataAbstract<int16_t>("Brushless_TargetSpeed", name, 205);


    // Vel. máximo e mínimo
    max = new DataAbstract<int8_t>("max", name, 100);
    dataManager->registerParamData(max);
    min = new DataAbstract<int8_t>("min", name, 0);
    dataManager->registerParamData(min);
    
    // Vel. base para calibracao
    vel_calibrate = new DataAbstract<float>("Velocidade_Calibracao", name, 46); // inicia com valor 46
    dataManager->registerParamData(vel_calibrate);

    // Variável para guardar a velocidade do trecho
    vel_mapped = new DataAbstract<float>("Velocidade_mapeada", name, 0);
    dataManager->registerRuntimeData(vel_mapped);

    //Setpoints para os tipos de trechos
    Long_Line = new DataAbstract<float>("PWM_Long_line", name, 80);
    dataManager->registerParamData(Long_Line);
    Medium_Line = new DataAbstract<float>("PWM_Medium_line", name, 80);
    dataManager->registerParamData(Medium_Line);
    Short_Line = new DataAbstract<float>("PWM_Short_line", name, 80);
    dataManager->registerParamData(Short_Line);
    Long_Curve = new DataAbstract<float>("PWM_Long_curve", name, 80);
    dataManager->registerParamData(Long_Curve);
    Medium_Curve = new DataAbstract<float>("PWM_Medium_curve", name, 80);
    dataManager->registerParamData(Medium_Curve);
    Short_Curve = new DataAbstract<float>("PWM_Short_curve", name, 80);
    dataManager->registerParamData(Short_Curve);
    Zigzag = new DataAbstract<float>("PWM_ZigZag", name, 80);
    dataManager->registerParamData(Zigzag);
    Special_Track = new DataAbstract<float>("PWM_SpecialTrack", name, 80);
    dataManager->registerParamData(Special_Track);
    Default_Track = new DataAbstract<float>("PWM_Default_Track", name, 80);
    dataManager->registerParamData(Default_Track);
    Tunning = new DataAbstract<float>("PWM_Tunning", name, 500);
    dataManager->registerParamData(Tunning);
    
}

DataAbstract<float> *dataMotor::Setpoint(TrackState state)
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
        case ZIGZAG:
            return Zigzag;
            break;
        case TUNNING:
            return Tunning;
            break;
        default:
            return Default_Track;
            break;
    }

    //ESP_LOGE(tag, "Estado do Robô ou Objeto PID inválido para esse método: %s:%d para obter o Kp do PID, retornando valor null.", name.c_str(),state);
    return nullptr;
}