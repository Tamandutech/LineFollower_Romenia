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
    dataSpec(std::string name = "dataSpeed",bool PID_Select = false);

    // Reducao dos motores
    DataAbstract<uint16_t> *GearRatio;
    // Num Revolucoes
    DataAbstract<uint16_t> *Revolution;
    // Variavel que contempla relacao de Revloucoes e reducao dos motores, entrada eh ((Qtd de pulsos para uma volta) * (Reducao do motor))
    DataAbstract<uint16_t> *MPR;

    // Diâmetro das rodas
    DataAbstract<uint8_t> *WheelDiameter;
    DataAbstract<uint16_t> *RobotDiameter;

private:
    std::string name;
    const char *tag = "RobotData";

    DataManager *dataManager;
};

#endif