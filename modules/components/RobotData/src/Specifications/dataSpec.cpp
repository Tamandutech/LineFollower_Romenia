#include "dataSpec.h"

dataSpec::dataSpec(std::string name,bool PID_Select)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);


    /*
     * Variaveis que contempla relacao de Revloucoes e reducao
     * dos motores, entrada eh ((Qtd de pulsos para uma volta) * (Reducao do motor))
     * */
    GearRatio = new DataAbstract<uint16_t>("GearRatio", name, 5);
    Revolution = new DataAbstract<uint16_t>("Revolution", name, 12);
    MPR = new DataAbstract<uint16_t>("MPR", name, 0);

    WheelDiameter = new DataAbstract<uint8_t>("WheelDiameter", name, 0);
    RobotDiameter = new DataAbstract<uint16_t>("RobotDiameter", name, 112); //mm


}