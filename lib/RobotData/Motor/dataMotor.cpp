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
    vel_calibracao = new DataAbstract<float>("Velocidade Calibracao", name, 50); // inicia com valor 50

    //Setpoints translacionais para os tipos de trechos
    Long_Line = new DataAbstract<float>("PWM_Long_line", name, 1000);
    Medium_Line = new DataAbstract<float>("PWM_Medium_line", name, 1000);
    Short_Line = new DataAbstract<float>("PWM_Short_line", name, 1000);
    Long_Curve = new DataAbstract<float>("PWM_Long_curve", name, 1000);
    Medium_Curve = new DataAbstract<float>("PWM_Medium_curve", name, 1000);
    Short_Curve = new DataAbstract<float>("PWM_Short_curve", name, 1000);
    ZIGZAG = new DataAbstract<float>("PWM_ZigZag", name, 1000);
    Special_Track = new DataAbstract<float>("PWM_SpecialTrack", name, 1000);
    Default_Track = new DataAbstract<float>("PWM_Default_Track", name, 1000);
    Tunning = new DataAbstract<float>("PWM_Tunning", name, 1000);
    
}