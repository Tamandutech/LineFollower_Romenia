#ifndef IR_SERVICE_HPP
#define IR_SERVICE_HPP

// Bibliotecas para criacao do servico
#include "thread.hpp"
#include "esp_log.h"
#include "Injector/singleton.hpp"
// Bibliotecas de componentes/dados
#include <stdio.h>
#include <string.h>
#include "ir_tools.h"
#include "driver/rmt.h"
#include "RobotData.h"
#include "LEDsService.hpp"

using namespace cpp_freertos;

class IRService : public Thread, public Singleton<IRService>
{
public:
    IRService(std::string name, uint32_t stackDepth, UBaseType_t priority);
    
    void Run() override;
private:

    std::string tag;

    // Atalhos:
    Robot *robot;
    dataStatus *status;
    LEDsService *LED;

    rmt_channel_t rx_channel = RMT_CHANNEL_2;

    uint32_t addr = 0;
    uint32_t cmd = 0;
    size_t length = 0;
    bool repeat = false;
    RingbufHandle_t rb = NULL;
    rmt_item32_t *items = NULL;
    
};

#endif