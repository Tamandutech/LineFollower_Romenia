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
    //ESP_LOGI(GetName().c_str(), "Início IRService");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();

    create_rx_channel();
    register_rx_callback();
    config_rx_receive();

    //ESP_LOGI(GetName().c_str(), "enable RMT RX channel");
    ESP_ERROR_CHECK(rmt_enable(rx_channel));
    
    start_rx_receive();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_PERIOD_MS);
        
        if (rx_signal_received()) {
            uint16_t command = nec.parse_nec_frame(rx_data.received_symbols, rx_data.num_symbols);
            
            if(command == STOP_COMMAND) switch_robot_off();
            
            start_rx_receive();
        }
    }
}

void IRService::create_rx_channel(){
    //ESP_LOGI(GetName().c_str(), "create RMT RX channel");
    rmt_rx_channel_config_t rx_chan_config = {
        .gpio_num = GPIO_NUM_15,          // GPIO number
        .clk_src = RMT_CLK_SRC_DEFAULT,   // select source clock
        .resolution_hz = 1 * 1000 * 1000, // 1 MHz tick resolution, i.e., 1 tick = 1 µs
        .mem_block_symbols = 64,          // memory block size, 64 * 4 = 256 Bytes
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_channel));
}

void IRService::register_rx_callback(){
    //ESP_LOGI(GetName().c_str(), "register RX done callback");
    receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rx_callback_finished,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));
}

bool IRService::rx_callback_finished(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    // return whether any task is woken up
    return high_task_wakeup == pdTRUE;
}

void IRService::config_rx_receive(){
    // the following timing requirement is based on NEC protocol
    receive_config = {
        .signal_range_min_ns = 1250,     // the shortest duration for NEC signal is 560 µs, 1250 ns < 560 µs, valid signal is not treated as noise
        .signal_range_max_ns = 12000000, // the longest duration for NEC signal is 9000 µs, 12000000 ns > 9000 µs, the receive does not stop early
    };
}

void IRService::start_rx_receive(){
    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
}

BaseType_t IRService::rx_signal_received(){
    return xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000));
}

void IRService::switch_robot_off(){
    //ESP_LOGI(GetName().c_str(), "Signal received, stopping robot");
    get_Status->ControlOff->setData(true);
}

