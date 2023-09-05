#ifndef DATA_FLOAT_SENSOR_H 
#define DATA_FLOAT_SENSOR_H

#include <stdint.h>
#include <stddef.h>
#include <string>
#include <vector>
#include <mutex>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

#include "dataEnums.h"
#include "DataStorage.hpp"

#include "esp_log.h"

class dataFloat
{
public:
    dataFloat(uint16_t qtdChannels, std::string name = "dataFloat");

    int set_Channels_mutex(std::vector<float> values, std::vector<float> *channel);
    int get_Channel_mutex(uint16_t channelNumber, std::vector<float> *channel);

    int setLine(float value);
    float getLine();

    int setChannel(uint16_t channelNumber, float value, std::vector<float> *channel, SemaphoreHandle_t *xSemaphoreOfArg);
    int setChannel(uint16_t channelNumber, float value);
    float getChannel(uint16_t channelNumber, std::vector<float> *channel, SemaphoreHandle_t *xSemaphoreOfArg);
    float getChannel(uint16_t channelNumber);

    int setChannels(std::vector<float> values, std::vector<float> *channel, SemaphoreHandle_t *xSemaphoreOfArg);
    int setChannels(std::vector<float> values);
    std::vector<float> getChannels(std::vector<float> *channel, SemaphoreHandle_t *xSemaphoreOfArg);
    std::vector<float> getChannels();

    int setChannelMin(uint16_t channelNumber, float value);
    float getChannelMin(uint16_t channelNumber);

    int setChannelMax(uint16_t channelNumber, float value);
    float getChannelMax(uint16_t channelNumber);

    int setChannelsMins(std::vector<float> values);
    std::vector<float> getChannelsMins();

    int setChannelsMaxes(std::vector<float> values);
    std::vector<float> getChannelsMaxes();

private:
    std::string name;
    const char *tag = "RobotData";

    // Valores
    SemaphoreHandle_t xSemaphorechannel;
    std::mutex channel_mutex;
    std::vector<float> channel;

    SemaphoreHandle_t xSemaphoreline;
    float line;

    // Parâmetros
    uint16_t qtdChannels = 0;

    bool created = false;

    SemaphoreHandle_t xSemaphoremaxChannel;
    std::vector<float> maxChannel;

    SemaphoreHandle_t xSemaphoreminChannel;
    std::vector<float> minChannel;
};

#endif