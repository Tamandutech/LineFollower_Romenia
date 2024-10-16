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

    MappingService(std::string name, uint32_t stackDepth, UBaseType_t priority, BaseType_t coreid);
    
    void Run() override;

    esp_err_t startNewMapping(uint16_t leftMarksToStop = INT16_MAX, int32_t mediaPulsesToStop = LONG_MAX, uint32_t timeToStop = (portMAX_DELAY / portTICK_PERIOD_MS));
    esp_err_t stopNewMapping();

    esp_err_t loadMapping();
    esp_err_t saveMapping();

    esp_err_t createNewMark();

    bool track_is_a_line(uint8_t track);
    bool track_is_a_curve(uint8_t track);

private:
    std::string name;

    // Atalhos para o RobotData
    Robot *robot;
    dataMotor *get_Speed;
    dataSLatMarks *get_latMarks;
    dataStatus *get_Status;
    dataSpec *get_Spec;
    // Atalhos para serviços
    LEDsService *LED;

    // Variáveis

    struct MapData tempActualMark;

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
    int32_t EncLeft = 0, EncRight = 0;
    int32_t lastEncLeft = 0, lastEncRight = 0, lastEncMedia = 0;
    uint8_t lastTrack = 0;

    void MappingWithMarks();
    void MappingWithoutMarks(TickType_t *xLastWakeTime);
    void AtualizarLEDs();
    bool finished_mapping();
};

#endif