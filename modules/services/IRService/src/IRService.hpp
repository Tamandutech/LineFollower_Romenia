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
    rmt_channel_handle_t rx_channel = NULL;
    QueueHandle_t receive_queue;
    rmt_receive_config_t receive_config;
    rmt_symbol_word_t raw_symbols[64]; // 64 symbols should be sufficient for a standard NEC frame
    rmt_rx_done_event_data_t rx_data; // save the received RMT symbols

    static bool rx_callback_finished(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data);
    void create_rx_channel();
    void register_rx_callback();
    void config_rx_receive();
    void start_rx_receive();
    BaseType_t rx_signal_received();
    void switch_robot_off();
};

#endif