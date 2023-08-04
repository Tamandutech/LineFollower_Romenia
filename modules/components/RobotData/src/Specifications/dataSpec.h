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

    // Cor da linha
    DataAbstract<bool> *WhiteLine;

    // Reducao dos motores
    DataAbstract<uint16_t> *GearRatio;
    // Num Revolucoes
    DataAbstract<uint16_t> *Revolution;
    // Variavel que contempla relacao de Revloucoes e reducao dos motores, entrada eh ((Qtd de pulsos para uma volta) * (Reducao do motor))
    DataAbstract<uint16_t> *MPR;

    // Diâmetro das rodas
    DataAbstract<uint8_t> *WheelDiameter;
    // Distancia entre as rodas
    DataAbstract<uint8_t> *RobotDiameter; // mm
    // Distancia do centro do robô até a ponta dos sensores
    DataAbstract<uint16_t> *SensorToCenter;
    // Raio da angulacao dos sensores frontais
    DataAbstract<uint16_t> *RadiusSensor; // mm
    // Angulo maximo de leitura = (1/2) * (tamanho do arco)/(raio do arco)
    DataAbstract<uint16_t> *MaxAngle; // graus

private:
    std::string name;
    const char *tag = "RobotData";

    DataManager *dataManager;
};

#endif