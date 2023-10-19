#ifndef MAPPING_SERVICE_H
#define MAPPING_SERVICE_H

#include <atomic>
#include <mutex>
#include <math.h>
#include <limits.h>

#include "thread.hpp"
#include "singleton.hpp"
#include "RobotData.h"
#include "LEDsService.hpp"

#include "driver/gpio.h"

using namespace cpp_freertos;

#include "esp_log.h"

#define ManualMap

class MappingService : public Thread, public Singleton<MappingService>
{
public:

    MappingService(std::string name, uint32_t stackDepth, UBaseType_t priority);
    
    void Run() override;

    esp_err_t startNewMapping(uint16_t leftMarksToStop = INT16_MAX, int32_t mediaPulsesToStop = LONG_MAX, uint32_t timeToStop = (portMAX_DELAY / portTICK_PERIOD_MS));
    esp_err_t stopNewMapping();

    esp_err_t loadMapping();
    esp_err_t saveMapping();

    esp_err_t createNewMark();

private:
    std::string name;

    // Atalhos para o RobotData
    Robot *robot;
    dataMotor *speedMapping;
    dataSLatMarks *latMarks;
    dataStatus *status;
    dataSpec *get_Spec;
    // Atalhos para serviços
    LEDsService *LED;

    // Variáveis

    struct MapData tempActualMark;
    struct MapData tempPreviousMark;

    // atributos de filtro
    uint16_t leftMarksToStop;
    uint16_t rightMarksToStop;
    int32_t mediaPulsesToStop;
    TickType_t ticksToStop;

    // atributos para offsets iniciais
    int32_t initialRightPulses = 0;
    int32_t initialLeftPulses = 0;
    int32_t initialMediaPulses = 0;
    TickType_t initialTicks = 0;

    // variáveis de calculos temporárias
    uint32_t tempDeltaPulses = 0;
    uint32_t tempMilimiterInPulses = 0;
    uint32_t tempDeltaDist = 0;

    led_position_t LEDposition[NUM_LEDS] = {LED_POSITION_NONE};

    void MappingWithMarks();
    void MappingWithoutMarks(TickType_t *xLastWakeTime);
};

#endif