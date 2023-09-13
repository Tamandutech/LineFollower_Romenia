#include "dataMotor.h"

dataMotor::dataMotor(std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    //ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);

    // Contagem atual dos encoders
    EncRight = new DataAbstract<int32_t>("EncRight", name, 0);
    EncLeft = new DataAbstract<int32_t>("EncLeft", name, 0);
    EncMedia = new DataAbstract<int32_t>("EncMedia", name, 0);

    
    // Valocidades atuais
    RPMRight_inst = new DataAbstract<int16_t>("RPMRight_inst", name, 0);
    RPMLeft_inst = new DataAbstract<int16_t>("RPMLeft_inst", name, 0);
    RPMCar_media = new DataAbstract<int16_t>("RPMCar_media", name, 0);
    
    // Vel. base para calibracao
    vel_calibrate = new DataAbstract<float>("Velocidade Calibracao", name, 46); // inicia com valor 46

    //Setpoints para os tipos de trechos
    Long_Line = new DataAbstract<float>("PWM_Long_line", name, 80);
    Medium_Line = new DataAbstract<float>("PWM_Medium_line", name, 80);
    Short_Line = new DataAbstract<float>("PWM_Short_line", name, 80);
    Long_Curve = new DataAbstract<float>("PWM_Long_curve", name, 80);
    Medium_Curve = new DataAbstract<float>("PWM_Medium_curve", name, 80);
    Short_Curve = new DataAbstract<float>("PWM_Short_curve", name, 80);
    Zigzag = new DataAbstract<float>("PWM_ZigZag", name, 80);
    Special_Track = new DataAbstract<float>("PWM_SpecialTrack", name, 80);
    Default_Track = new DataAbstract<float>("PWM_Default_Track", name, 80);
    Tunning = new DataAbstract<float>("PWM_Tunning", name, 80);
    
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