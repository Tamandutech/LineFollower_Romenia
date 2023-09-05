#include "LEDsService.hpp"

QueueHandle_t LEDsService::queueLedCommands;
led_command_t LEDsService::ledCommand;

uint32_t LEDsService::ws2812_t0h_ticks;
uint32_t LEDsService::ws2812_t1h_ticks;
uint32_t LEDsService::ws2812_t0l_ticks;
uint32_t LEDsService::ws2812_t1l_ticks;

LEDsService::LEDsService(std::string name, uint32_t stackDepth, UBaseType_t priority) : Thread(name, stackDepth, priority)
{// Construtor do serviço
    esp_log_level_set(name.c_str(), ESP_LOG_INFO);
    ESP_LOGI("LEDsService", "Constructor Start");

    queueLedCommands = xQueueCreate(10, sizeof(ledCommand)); // cria uma fila de espera com o tipo de pacote 'led_command_t'

    ESP_LOGI("LEDsService", "Constructor END");
}

void LEDsService::Run()
{
    ESP_LOGI("LEDsService", "Run");

    ESP_LOGI("LEDsService", "GPIO: %d, Canal: %d", config.gpio_num, config.channel);

    this->config.clk_div = 2;

#ifndef ESP32_QEMU
    ESP_ERROR_CHECK(rmt_config(&this->config));
    ESP_ERROR_CHECK(rmt_driver_install(this->config.channel, 0, 0)); // Instala e verifica o driver RMT

#endif

    this->strip_config.max_leds = NUM_LEDS; // Definindo a quantidade máx de LEDs
    this->strip_config.dev = (led_strip_dev_t)config.channel; // Definindo o RMT channel como LED strip device

    this->strip = led_strip_new_rmt_ws2812(&strip_config); // Criando o LED strip driver

    if (!strip) // Se falhar em criar o driver
        ESP_LOGE(GetName().c_str(), "Falha ao iniciar driver do LED.");

    ESP_ERROR_CHECK(this->strip->clear(this->strip, 100));

    for (;;)
    {// Loop do serviço
        vTaskDelay(0);
        xQueueReceive(queueLedCommands, &ledCommand, portMAX_DELAY); // Recebe um pacote de dados da fila pela porta definida
        // O resto do código só roda quando xQueueReceive recebe um pacote
        
        // ESP_LOGD(GetName().c_str(), "Run: ledCommand.effect = %d", ledCommand.effect);

        switch (ledCommand.effect)
        {// Verifica qual efeito desejado na LED
        case LED_EFFECT_SET: // Ligar ou desligar a LED
            led_effect_set();
            break;

        case LED_EFFECT_BLINK: // Ligar e desligar logo em seguida
            // led_effect_blink(); // Não implementada
            break;

        case LED_EFFECT_FADE: // Ligar e ir diminuindo o brilho gradativamente
            // led_effect_fade(); // Não implementada
            break;

        default:
            break;
        }
    }
}

void LEDsService::config_LED(led_position_t position[NUM_LEDS], led_color_t color, led_effect_t effect, float brigh)
{// Salva os dados recebidos numa variável tipo led_command_t e a adiciona na fila
    led_command_t command;
    for(int i=0; i < NUM_LEDS; i++){
        command.led[i] = position[i];
    }
    command.color = color;
    command.effect = effect;
    command.brightness = brigh;
    queueCommand(command); // adiciona na fila
}

esp_err_t LEDsService::queueCommand(led_command_t command)
{// Manda o pacote de dados 'led_command_t' para a fila pela porta portMAX_DELAY
    ESP_LOGI(GetName().c_str(), "queueCommand: command.effect = %d", command.effect);
    return xQueueSend(queueLedCommands, &command, portMAX_DELAY);
}

void LEDsService::led_effect_set()
{// Liga ou desliga a LED conforme indicado em ledCommand
    ESP_LOGI("LEDsService", "led_effect_set");
    vTaskDelay(1);
    for (size_t i = 0; i < NUM_LEDS; i++)
    {
        if (ledCommand.led[i] >= 0)
        {
            ESP_LOGI(GetName().c_str(), "led_effect_set: ledCommand.led[%d] = %d, R = %d, G = %d, B = %d", i, ledCommand.led[i], (*((uint8_t *)(&ledCommand.color) + 2)), (*((uint8_t *)(&ledCommand.color) + 1)), (*(uint8_t *)(&ledCommand.color)));
#ifndef ESP32_QEMU
            ESP_ERROR_CHECK(this->strip->set_pixel(this->strip, ledCommand.led[i], ledCommand.brightness * (*((uint8_t *)(&ledCommand.color) + 2)), ledCommand.brightness * (*((uint8_t *)(&ledCommand.color) + 1)), ledCommand.brightness * (*(uint8_t *)(&ledCommand.color))));
#endif
        }
        else
        {// Se ledCommand.led[i] for igual a LED_EFFECT_NONE:
            break;
        }
    }
    ESP_ERROR_CHECK(this->strip->refresh(strip, 100));
}

/**
 * @brief Converte RGB para o formato RMT.
 *
 * @note Para WS2812, cada valor R,G,B contem 256 diferentes valores (aka. uint8_t)
 *
 * @param[in] src: origem dos dados, para converter para RMT
 * @param[in] dest: lugar para armazenar os dados convertidos
 * @param[in] src_size: tamanho dos dados a serem convertidos
 * @param[in] wanted_num: número de itens RMT que serão convertidos
 * @param[out] translated_size: número de itens convertidos
 * @param[out] item_num: número de itens RMT que foram convertidos da origem
 */
void IRAM_ATTR LEDsService::ws2812_rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size,
                                               size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    if (src == NULL || dest == NULL)
    {
        *translated_size = 0;
        *item_num = 0;
        return;
    }

    const rmt_item32_t bit0 = {{{ws2812_t0h_ticks, 1, ws2812_t0l_ticks, 0}}}; // Logical 0
    const rmt_item32_t bit1 = {{{ws2812_t1h_ticks, 1, ws2812_t1l_ticks, 0}}}; // Logical 1
    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t *pdest = dest;

    while (size < src_size && num < wanted_num)
    {
        for (int i = 0; i < 8; i++)
        {
            // MSB primeiro
            if (*psrc & (1 << (7 - i)))
            {
                pdest->val = bit1.val;
            }
            else
            {
                pdest->val = bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }

    *translated_size = size;
    *item_num = num;
}

esp_err_t LEDsService::ws2812_set_pixel(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue)
{
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);

    if (index >= ws2812->strip_len)
    {
        //ESP_LOGE(LEDsService::getInstance()->GetName().c_str(), "Índice %d maior que o número total de LEDs.", index);
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t start = index * 3;

    // Na ordem GRB
    ws2812->buffer[start + 0] = green & 0xFF;
    ws2812->buffer[start + 1] = red & 0xFF;
    ws2812->buffer[start + 2] = blue & 0xFF;
    return ESP_OK;
}

esp_err_t LEDsService::ws2812_refresh(led_strip_t *strip, uint32_t timeout_ms)
{
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);

#ifndef ESP32_QEMU
    if (ESP_OK != rmt_write_sample(ws2812->rmt_channel, ws2812->buffer, ws2812->strip_len * 3, true))
    {
        //ESP_LOGE(LEDsService::getInstance()->GetName().c_str(), "Falha ao transmitir pacotes do RMT.");
        return ESP_FAIL;
    }
    return rmt_wait_tx_done(ws2812->rmt_channel, pdMS_TO_TICKS(timeout_ms));
#else
    return ESP_OK;
#endif
}

esp_err_t LEDsService::ws2812_clear(led_strip_t *strip, uint32_t timeout_ms)
{
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);

    // Escreve 0 em todos os LEDs para apagar
    memset(ws2812->buffer, 0, ws2812->strip_len * 3);
    return ws2812_refresh(strip, timeout_ms);
}

esp_err_t LEDsService::ws2812_del(led_strip_t *strip)
{
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);
    free(ws2812);
    return ESP_OK;
}

led_strip_t *LEDsService::led_strip_new_rmt_ws2812(const led_strip_config_t *config)
{
    if (!config)
    {
        //ESP_LOGE(GetName().c_str(), "A configuração não pode ser nula");
        return NULL;
    }

    // 24 bits por LED
    uint32_t ws2812_size = sizeof(ws2812_t) + config->max_leds * 3;
    ws2812_t *ws2812 = (ws2812_t *)calloc(1, ws2812_size);

    if (!ws2812)
    {
        //ESP_LOGE(GetName().c_str(), "Falha ao alocar memória para o driver.");
        return NULL;
    }

    uint32_t counter_clk_hz = 0;
#ifndef ESP32_QEMU
    if (ESP_OK != rmt_get_counter_clock((rmt_channel_t)config->dev, &counter_clk_hz))
    {
        //ESP_LOGE(GetName().c_str(), "Falha ao obter contagem de clock do RMT.");
        return NULL;
    }
#endif

    float ratio = (float)counter_clk_hz / 1e9;
    ws2812_t0h_ticks = (uint32_t)(ratio * WS2812_T0H_NS);
    ws2812_t0l_ticks = (uint32_t)(ratio * WS2812_T0L_NS);
    ws2812_t1h_ticks = (uint32_t)(ratio * WS2812_T1H_NS);
    ws2812_t1l_ticks = (uint32_t)(ratio * WS2812_T1L_NS);

#ifndef ESP32_QEMU
    rmt_translator_init((rmt_channel_t)config->dev, (sample_to_rmt_t)ws2812_rmt_adapter);
#endif

    ws2812->rmt_channel = (rmt_channel_t)config->dev;
    ws2812->strip_len = config->max_leds;

    ws2812->parent.set_pixel = ws2812_set_pixel;
    ws2812->parent.refresh = ws2812_refresh;
    ws2812->parent.clear = ws2812_clear;
    ws2812->parent.del = ws2812_del;

    return &ws2812->parent;
}