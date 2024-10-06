#ifndef DATA_SPEC_H
#define DATA_SPEC_H

#include <stdint.h>
#include <stddef.h>
#include <string>

#include "dataEnums.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

#include "DataAbstract.hpp"
#include "DataStorage.hpp"
#include "DataManager.hpp"

#include "esp_log.h"

class dataSpec
{
public:
    dataSpec(std::string name = "dataSpec",bool PID_Select = false);

    
    DataAbstract<uint16_t> *GearRatio; // Reducao dos motores
    
    DataAbstract<uint16_t> *Revolution; // Numero de Revolucoes dos motores

    DataAbstract<uint16_t> *MPR;  // Variavel que contempla relacao de Revloucoes e reducao dos motores
                                  // , entrada eh ((Qtd de pulsos para uma volta) * (Reducao do motor))
    
    DataAbstract<uint8_t> *WheelDiameter; // (mm) Diâmetro das rodas
    
    DataAbstract<uint8_t> *RobotDiameter; // (mm) Distancia entre as rodas
    
    DataAbstract<uint16_t> *SensorToCenter; // Distancia do centro do robô até a ponta dos sensores
    
    DataAbstract<uint16_t> *RadiusSensor; // (mm) Raio da angulacao dos sensores frontais
    
    DataAbstract<float> *MaxAngle; // (graus) Angulo maximo de leitura = (1/2) * (tamanho do arco)/(raio do arco)

    DataAbstract<float> *MaxAngle_Center; // (graus) Angulo maximo com relação ao centro de movimento

    DataAbstract<float> *Friction_Angle; // (graus) Coeficiente de fricção em graus

    DataAbstract<float> *Friction_Coef; // Coeficiente de fricção calculado a partir do Friction_Angle

    DataAbstract<float> *Acceleration; // (mm/s^2) Aceleração do robô

    DataAbstract<float> *MaxRPM; // RPM máximo estimado

    DataAbstract<float> *Mass; // Massa com brushless desligado

    DataAbstract<float> *Mass_BrushON; // Massa quando o brushless está ligado

    DataAbstract<float> *Malha_Aberta; // Angulo para ligar a malha aberta

private:
    std::string name;
    const char *tag = "RobotData";

    DataManager *dataManager;
};

#endif