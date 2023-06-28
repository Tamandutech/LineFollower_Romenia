#include "dataMotor.h"

dataMotor::dataMotor(std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);

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