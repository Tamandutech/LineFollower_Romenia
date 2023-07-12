#include "dataMotor.h"

dataMotor::dataMotor(std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);

    // Contagem atual dos encoders
    EncRight = new DataAbstract<int32_t>("EncRight", name, 0);
    EncLeft = new DataAbstract<int32_t>("EncLeft", name, 0);
    EncMedia = new DataAbstract<int32_t>("EncMedia", name, 0);

    
    // Valocidades atuais
    RPMRight_inst = new DataAbstract<int16_t>("RPMRight_inst", name, 0);
    RPMLeft_inst = new DataAbstract<int16_t>("RPMLeft_inst", name, 0);
    RPMCar_media = new DataAbstract<int16_t>("RPMCar_media", name, 0);
    
    // Vel. base para calibracao
    vel_calibrate = new DataAbstract<float>("Velocidade Calibracao", name, 50); // inicia com valor 50

    //Setpoints translacionais para os tipos de trechos
    Long_Line = new DataAbstract<float>("PWM_Long_line", name, 100);
    Medium_Line = new DataAbstract<float>("PWM_Medium_line", name, 100);
    Short_Line = new DataAbstract<float>("PWM_Short_line", name, 100);
    Long_Curve = new DataAbstract<float>("PWM_Long_curve", name, 100);
    Medium_Curve = new DataAbstract<float>("PWM_Medium_curve", name, 100);
    Short_Curve = new DataAbstract<float>("PWM_Short_curve", name, 100);
    Zigzag = new DataAbstract<float>("PWM_ZigZag", name, 100);
    Special_Track = new DataAbstract<float>("PWM_SpecialTrack", name, 100);
    Default_Track = new DataAbstract<float>("PWM_Default_Track", name, 100);
    Tunning = new DataAbstract<float>("PWM_Tunning", name, 100);
    
}

DataAbstract<float> *dataMotor::Setpoint(TrackState state)
{
    if(state == LONG_LINE){
        return Long_Line;
    }
    else if(state == MEDIUM_LINE){
        return Medium_Line;
    }
    else if(state == SHORT_LINE){
        return Short_Line;
    }
    else if(state == LONG_CURVE){
        return Long_Curve;
    }
    else if(state == MEDIUM_CURVE){
        return Medium_Curve;
    }
    else if(state == SHORT_CURVE){
        return Short_Curve;
    }
    else if(state == ZIGZAG){
        return Zigzag;
    }
    else if(state == TUNNING){
        return Tunning;
    }
    else{
        return Default_Track;
    }
    ESP_LOGE(tag, "Estado do Robô ou Objeto PID inválido para esse método: %s:%d para obter o Kp do PID, retornando valor null.", name.c_str(),state);
    return nullptr;
}