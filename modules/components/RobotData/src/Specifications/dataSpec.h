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

    // Reducao dos motores
    DataAbstract<uint16_t> *GearRatio;
    // Num Revolucoes
    DataAbstract<uint16_t> *Revolution;
    // Variavel que contempla relacao de Revloucoes e reducao dos motores, entrada eh ((Qtd de pulsos para uma volta) * (Reducao do motor))
    DataAbstract<uint16_t> *MPR;

    // Diâmetro das rodas
    DataAbstract<uint8_t> *WheelDiameter; // mm
    // Distancia entre as rodas
    DataAbstract<uint8_t> *RobotDiameter; // mm
    // Distancia do centro do robô até a ponta dos sensores
    DataAbstract<uint16_t> *SensorToCenter;
    // Raio da angulacao dos sensores frontais
    DataAbstract<uint16_t> *RadiusSensor; // mm
    // Angulo maximo de leitura = (1/2) * (tamanho do arco)/(raio do arco)
    DataAbstract<float> *MaxAngle; // graus

    DataAbstract<float> *MaxAngle_Center; // graus

    DataAbstract<float> *Friction_Angle; // graus

    DataAbstract<float> *Friction_Coef; // graus

    DataAbstract<float> *Acceleration; // mm/s^2

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