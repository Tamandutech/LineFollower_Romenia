#include "dataInt32.h"

dataInt32::dataInt32(uint16_t qtdChannels, std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);

    this->qtdChannels = qtdChannels;
    // Alocando espaço para as variáveis
    ESP_LOGD(tag, "Alocando espaço na memória paras as variáveis, quantidade de canais: %d", qtdChannels);
    channel.reserve(qtdChannels);
    maxChannel.reserve(qtdChannels);
    minChannel.reserve(qtdChannels);

    ESP_LOGD(tag, "Criando Semáforos");
    (xSemaphorechannel) = xSemaphoreCreateMutex();
    (xSemaphoreline) = xSemaphoreCreateMutex();
    (xSemaphoremaxChannel) = xSemaphoreCreateMutex();
    (xSemaphoreminChannel) = xSemaphoreCreateMutex();
}

int dataInt32::setLine(int32_t value)
{
    if (xSemaphoreTake(xSemaphoreline, (TickType_t)10) == pdTRUE)
    {
        this->line = value;
        xSemaphoreGive(xSemaphoreline);
        return RETORNO_OK;
    }
    else
    {
        ESP_LOGE(tag, "Variável Line ocupada, não foi possível definir valor.");
        return RETORNO_VARIAVEL_OCUPADA;
    }
}
int32_t dataInt32::getLine()
{
    int16_t tempLine = 0;
    for (;;)
    {
        if (xSemaphoreTake(xSemaphoreline, (TickType_t)10) == pdTRUE)
        {
            tempLine = this->line;
            xSemaphoreGive(xSemaphoreline);
            return tempLine;
        }
        else
        {
            ESP_LOGE(tag, "Variável Output ocupada, não foi possível ler valor. Tentando novamente...");
        }
    }
}

int dataInt32::setChannel(uint16_t channelNumber, int32_t value, std::vector<int32_t> *channel, SemaphoreHandle_t *xSemaphoreOfArg)
{
    if (channelNumber > (qtdChannels - 1))
    {
        ESP_LOGE(tag, "O canal informado \"%ud\" não existe, máximo: %ud", channelNumber, (qtdChannels - 1));
        return RETORNO_ARGUMENTO_INVALIDO;
    }

    if (xSemaphoreTake((*xSemaphoreOfArg), (TickType_t)10) == pdTRUE)
    {
        (*channel)[channelNumber] = value;
        xSemaphoreGive((*xSemaphoreOfArg));
        return RETORNO_OK;
    }
    else
    {
        ESP_LOGE(tag, "Vetor de canais ocupado, não foi possível definir valor.");
        return RETORNO_VARIAVEL_OCUPADA;
    }
}
int dataInt32::setChannel(uint16_t channelNumber, int32_t value)
{
    return this->setChannel(channelNumber, value, &this->channel, &xSemaphorechannel);
}

int32_t dataInt32::getChannel(uint16_t channelNumber, std::vector<int32_t> *channel, SemaphoreHandle_t *xSemaphoreOfArg)
{
    if (channelNumber > (qtdChannels - 1))
    {
        ESP_LOGE(tag, "O canal informado \"%ud\" não existe, máximo: %ud", channelNumber, (qtdChannels - 1));
        return RETORNO_ARGUMENTO_INVALIDO;
    }

    int32_t tempChannel;
    for (;;)
    {
        if (xSemaphoreTake((*xSemaphoreOfArg), (TickType_t)10) == pdTRUE)
        {
            tempChannel = (*channel)[channelNumber];
            xSemaphoreGive((*xSemaphoreOfArg));
            return tempChannel;
        }
        else
        {
            ESP_LOGE(tag, "Vetor de canais ocupado, não foi possível ler valor. Tentando novamente...");
        }
    }
}
int32_t dataInt32::getChannel(uint16_t channelNumber)
{
    return this->getChannel(channelNumber, &this->channel, &xSemaphorechannel);
}

int dataInt32::setChannels(std::vector<int32_t> values, std::vector<int32_t> *channel, SemaphoreHandle_t *xSemaphoreOfArg)
{
    if (xSemaphoreTake((*xSemaphoreOfArg), (TickType_t)10) == pdTRUE)
    {
        (*channel) = values;
        xSemaphoreGive((*xSemaphoreOfArg));
        return RETORNO_OK;
    }
    else
    {
        ESP_LOGE(tag, "Vetor de canais ocupado, não foi possível definir valores.");
        return RETORNO_VARIAVEL_OCUPADA;
    }
}
int dataInt32::setChannels(std::vector<int32_t> values)
{
    return this->setChannels(values, &this->channel, &xSemaphorechannel);
}

std::vector<int32_t> dataInt32::getChannels(std::vector<int32_t> *channel, SemaphoreHandle_t *xSemaphoreOfArg)
{
    std::vector<int32_t> tempChannels;
    for (;;)
    {
        if (xSemaphoreTake((*xSemaphoreOfArg), (TickType_t)10) == pdTRUE)
        {
            tempChannels = (*channel);
            xSemaphoreGive((*xSemaphoreOfArg));
        }
        else
        {
            ESP_LOGE(tag, "Vetor de canais ocupado, não foi possível ler valores. Tentando novamente...");
        }

        return tempChannels;
    }
}
std::vector<int32_t> dataInt32::getChannels()
{
    return this->getChannels(&this->channel, &xSemaphorechannel);
}

int dataInt32::setChannelMin(uint16_t channelNumber, int32_t value)
{
    return this->setChannel(channelNumber, value, &this->minChannel, &xSemaphoreminChannel);
}
int32_t dataInt32::getChannelMin(uint16_t channelNumber)
{
    return this->getChannel(channelNumber, &this->minChannel, &xSemaphoreminChannel);
}

int dataInt32::setChannelMax(uint16_t channelNumber, int32_t value)
{
    return this->setChannel(channelNumber, value, &this->maxChannel, &xSemaphoremaxChannel);
}
int32_t dataInt32::getChannelMax(uint16_t channelNumber)
{
    return this->getChannel(channelNumber, &this->maxChannel, &xSemaphoremaxChannel);
}

int dataInt32::setChannelsMins(std::vector<int32_t> values)
{
    return this->setChannels(values, &this->minChannel, &xSemaphoreminChannel);
}
std::vector<int32_t> dataInt32::getChannelsMins()
{
    return this->getChannels(&this->minChannel, &xSemaphoreminChannel);
}

int dataInt32::setChannelsMaxes(std::vector<int32_t> values)
{
    return this->setChannels(values, &this->maxChannel, &xSemaphoremaxChannel);
}
std::vector<int32_t> dataInt32::getChannelsMaxes()
{
    return this->getChannels(&this->maxChannel, &xSemaphoremaxChannel);
}