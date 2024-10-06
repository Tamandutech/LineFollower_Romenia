#include "dataSpec.h"

dataSpec::dataSpec(std::string name,bool PID_Select)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    //ESP_LOGI(tag, "Criando objeto: %s (%p)", name.c_str(), this);
    dataManager = dataManager->getInstance();

    /*
     * Variaveis que contempla relacao de Revoluções e redução dos
     * motores, entrada é ((Qtd de pulsos para uma volta) * (Reducao do motor))
     * */
    GearRatio = new DataAbstract<uint16_t>("GearRatio", name, 10);  // verificar depois se é o motor 10:1
    dataManager->registerParamData(GearRatio);

    Revolution = new DataAbstract<uint16_t>("Revolution", name, 12);
    dataManager->registerParamData(Revolution);
    
    MPR = new DataAbstract<uint16_t>("MPR", name, 0);
    dataManager->registerParamData(MPR);

    WheelDiameter = new DataAbstract<uint8_t>("WheelDiameter", name, 23); //mm  // mudar depois
    dataManager->registerParamData(WheelDiameter);
    
    RobotDiameter = new DataAbstract<uint8_t>("RobotDiameter", name, 126); //mm  // mudar depois
    dataManager->registerParamData(RobotDiameter);
    
    SensorToCenter = new DataAbstract<uint16_t>("SensorToCenter", name, 131); //mm  // mudar depois
    dataManager->registerParamData(SensorToCenter);
    
    RadiusSensor = new DataAbstract<uint16_t>("RadiusSensor", name, 164); //mm  // mudar depois
    dataManager->registerParamData(RadiusSensor);
    
    MaxAngle = new DataAbstract<float>("MaxAngle", name, 13.8); //graus  // mudar depois
    dataManager->registerParamData(MaxAngle);

    MaxAngle_Center = new DataAbstract<float>("MaxAngle_Center", name, 13.8); //graus  // mudar depois
    dataManager->registerParamData(MaxAngle_Center);

    Friction_Angle = new DataAbstract<float>("Friction_Angle", name, 20); //graus  // mudar depois
    dataManager->registerParamData(Friction_Angle);

    Friction_Coef = new DataAbstract<float>("Friction_Coef", name, 0);
    dataManager->registerParamData(Friction_Coef);

    Acceleration = new DataAbstract<float>("Acceleration", name, 0);
    dataManager->registerParamData(Acceleration);

    MaxRPM = new DataAbstract<float>("Max_RPM", name, 3000);  // mudar depois
    dataManager->registerParamData(MaxRPM);

    Mass = new DataAbstract<float>("Mass", name, 0);  // mudar depois
    dataManager->registerParamData(Mass);

    Mass_BrushON = new DataAbstract<float>("Mass_Brushless_ON", name, 0);  // mudar depois
    dataManager->registerParamData(Mass_BrushON);

    Malha_Aberta = new DataAbstract<float>("Malha_Aberta", name, 1.0);  // mudar depois
    dataManager->registerParamData(Malha_Aberta);
}