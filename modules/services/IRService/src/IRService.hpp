#ifndef IR_SERVICE_HPP
#define IR_SERVICE_HPP

// Bibliotecas para criacao do servico
#include "thread.hpp"
#include "esp_log.h"
#include "Injector/singleton.hpp"
// Bibliotecas de componentes/dados
#include <stdio.h>
#include <string.h>
#include "driver/rmt_rx.h"
#include "RobotData.h"
#include "LEDsService.hpp"

using namespace cpp_freertos;


class IRService : public Thread, public Singleton<IRService>
{
public:
    IRService(std::string name, uint32_t stackDepth, UBaseType_t priority, BaseType_t coreid);
    
    void Run() override;
private:

    std::string tag;

    // Atalhos:
    Robot *robot;
    dataStatus *get_Status;
    LEDsService *LED;

    uint32_t addr = 0;
    uint32_t cmd = 0;
    size_t length = 0;
    bool repeat = false;
    rmt_symbol_word_t *items = NULL;

    static bool example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data);
    
};

#endif