#include "IRService.hpp"

IRService::IRService(std::string name, uint32_t stackDepth, UBaseType_t priority, BaseType_t coreid):Thread(name, stackDepth, priority,  coreid)
{
    tag = name;
    // Atalhos para o RobotData:
    this->robot = Robot::getInstance();
    this->get_Status = robot->getStatus();
    this->LED = LEDsService::getInstance();

    esp_log_level_set(name.c_str(), ESP_LOG_INFO);
}

void IRService::Run()
{
    ESP_LOGI(GetName().c_str(), "Início IRService");
    // Variavel necerraria para funcionalidade do vTaskDelayUtil, guarda a conGetName().c_str()em de pulsos da CPU
    TickType_t xLastWakeTime = xTaskGetTickCount();

    ESP_LOGI(GetName().c_str(), "create RMT RX channel");
    rmt_channel_handle_t rx_channel = NULL;
    rmt_rx_channel_config_t rx_chan_config = {
        .gpio_num = GPIO_NUM_15,          // GPIO number
        .clk_src = RMT_CLK_SRC_DEFAULT,   // select source clock
        .resolution_hz = 1 * 1000 * 1000, // 1 MHz tick resolution, i.e., 1 tick = 1 µs
        .mem_block_symbols = 64,          // memory block size, 64 * 4 = 256 Bytes
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_channel));

    ESP_LOGI(GetName().c_str(), "register RX done callback");
    QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = example_rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));

    // the following timing requirement is based on NEC protocol
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1250,     // the shortest duration for NEC signal is 560 µs, 1250 ns < 560 µs, valid signal is not treated as noise
        .signal_range_max_ns = 12000000, // the longest duration for NEC signal is 9000 µs, 12000000 ns > 9000 µs, the receive does not stop early
    };

    ESP_LOGI(GetName().c_str(), "enable RMT RX channel");
    ESP_ERROR_CHECK(rmt_enable(rx_channel));

    // save the received RMT symbols
    rmt_symbol_word_t raw_symbols[64]; // 64 symbols should be sufficient for a standard NEC frame
    rmt_rx_done_event_data_t rx_data;
    // ready to receive
    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_PERIOD_MS);

        // wait for RX done signal
        if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS) {

            LED->LedComandSend(LED_POSITION_FRONT, COLOR_PINK, 1);
            get_Status->ControlOff->setData(true);
            // start receive again
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
        }
    }
}

bool IRService::example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    // return whether any task is woken up
    return high_task_wakeup == pdTRUE;
}