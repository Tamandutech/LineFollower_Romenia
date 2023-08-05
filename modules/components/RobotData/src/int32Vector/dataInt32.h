#ifndef DATA_SENSOR_H 
#define DATA_SENSOR_H

#include <stdint.h>
#include <stddef.h>
#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

#include "dataEnums.h"
#include "DataStorage.hpp"

#include "esp_log.h"

class dataInt32
{
public:
    dataInt32(uint16_t qtdChannels, std::string name = "dataInt32");

    int setLine(int32_t value);
    int32_t getLine();

    int setChannel(uint16_t channelNumber, int32_t value, std::vector<int32_t> *channel, SemaphoreHandle_t *xSemaphoreOfArg);
    int setChannel(uint16_t channelNumber, int32_t value);
    int32_t getChannel(uint16_t channelNumber, std::vector<int32_t> *channel, SemaphoreHandle_t *xSemaphoreOfArg);
    int32_t getChannel(uint16_t channelNumber);

    int setChannels(std::vector<int32_t> values, std::vector<int32_t> *channel, SemaphoreHandle_t *xSemaphoreOfArg);
    int setChannels(std::vector<int32_t> values);
    std::vector<int32_t> getChannels(std::vector<int32_t> *channel, SemaphoreHandle_t *xSemaphoreOfArg);
    std::vector<int32_t> getChannels();

    int setChannelMin(uint16_t channelNumber, int32_t value);
    int32_t getChannelMin(uint16_t channelNumber);

    int setChannelMax(uint16_t channelNumber, int32_t value);
    int32_t getChannelMax(uint16_t channelNumber);

    int setChannelsMins(std::vector<int32_t> values);
    std::vector<int32_t> getChannelsMins();

    int setChannelsMaxes(std::vector<int32_t> values);
    std::vector<int32_t> getChannelsMaxes();

private:
    std::string name;
    const char *tag = "RobotData";

    // Valores
    SemaphoreHandle_t xSemaphorechannel;
    std::vector<int32_t> channel;

    SemaphoreHandle_t xSemaphoreline;
    int32_t line;

    // Parâmetros
    uint16_t qtdChannels = 0;

    SemaphoreHandle_t xSemaphoremaxChannel;
    std::vector<int32_t> maxChannel;

    SemaphoreHandle_t xSemaphoreminChannel;
    std::vector<int32_t> minChannel;
};

#endif