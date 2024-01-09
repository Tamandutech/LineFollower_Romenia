#include "IRService.hpp"

IRService::IRService(std::string name, uint32_t stackDepth, UBaseType_t priority, BaseType_t coreid):Thread(name, stackDepth, priority,  coreid)
{
    tag = name;
    // Atalhos para o RobotData:
    this->robot = Robot::getInstance();
    this->get_Status = robot->getStatus();

    esp_log_level_set(name.c_str(), ESP_LOG_INFO);
}

void IRService::Run()
{
    //ESP_LOGI(GetName().c_str(), "Início IRService");
    // Variavel necerraria para funcionalidade do vTaskDelayUtil, guarda a conGetName().c_str()em de pulsos da CPU
    TickType_t xLastWakeTime = xTaskGetTickCount();

    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(GPIO_NUM_15, rx_channel);
    rmt_config(&rmt_rx_config);
    rmt_driver_install(rx_channel, 1000, 0);
    ir_parser_config_t ir_parser_config = IR_PARSER_DEFAULT_CONFIG((ir_dev_t)rx_channel);
    ir_parser_config.flags |= IR_TOOLS_FLAGS_PROTO_EXT; // Using extended IR protocols (both NEC and RC5 have extended version)
    ir_parser_t *ir_parser = NULL;
    ir_parser = ir_parser_rmt_new_nec(&ir_parser_config);

    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(rx_channel, &rb);
    assert(rb != NULL);
    // Start receive
    rmt_rx_start(rx_channel, true);

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_PERIOD_MS);

        items = (rmt_item32_t *) xRingbufferReceive(rb, &length, portMAX_DELAY);
        if (items) {
            length /= 4; // one RMT = 4 Bytes
            if (ir_parser->input(ir_parser, items, length) == ESP_OK) {
                if (ir_parser->get_scan_code(ir_parser, &addr, &cmd, &repeat) == ESP_OK) {

                    get_Status->ControlOff->setData(true);
                    
                }
            }
            //after parsing the data, return spaces to ringbuffer.
            vRingbufferReturnItem(rb, (void *) items);
        }
    }
    ir_parser->del(ir_parser);
    rmt_driver_uninstall(rx_channel);
}