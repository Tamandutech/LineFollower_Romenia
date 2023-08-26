#include "dataFloat.h"

dataFloat::dataFloat(uint16_t qtdChannels, std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    //ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);

    this->qtdChannels = qtdChannels;
    // Alocando espaço para as variáveis
    //ESP_LOGD(tag, "Alocando espaço na memória paras as variáveis, quantidade de canais: %d", qtdChannels);
    channel.reserve(qtdChannels);
    maxChannel.reserve(qtdChannels);
    minChannel.reserve(qtdChannels);

    //ESP_LOGD(tag, "Criando Semáforos");
    (xSemaphorechannel) = xSemaphoreCreateMutex();
    (xSemaphoreline) = xSemaphoreCreateMutex();
    (xSemaphoremaxChannel) = xSemaphoreCreateMutex();
    (xSemaphoreminChannel) = xSemaphoreCreateMutex();
}

int dataFloat::setLine(float value)
{
    if (xSemaphoreTake(xSemaphoreline, (TickType_t)10) == pdTRUE)
    {
        this->line = value;
        xSemaphoreGive(xSemaphoreline);
        return RETORNO_OK;
    }
    else
    {
        ////ESP_LOGE(tag, "Variável Line ocupada, não foi possível definir valor.");
        return RETORNO_VARIAVEL_OCUPADA;
    }
}
float dataFloat::getLine()
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
            //ESP_LOGE(tag, "Variável Output ocupada, não foi possível ler valor. Tentando novamente...");
        }
    }
}

int dataFloat::setChannel(uint16_t channelNumber, float value, std::vector<float> *channel, SemaphoreHandle_t *xSemaphoreOfArg)
{
    if (channelNumber > (qtdChannels - 1))
    {
        //ESP_LOGE(tag, "O canal informado \"%ud\" não existe, máximo: %ud", channelNumber, (qtdChannels - 1));
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
        //ESP_LOGE(tag, "Vetor de canais ocupado, não foi possível definir valor.");
        return RETORNO_VARIAVEL_OCUPADA;
    }
}
int dataFloat::setChannel(uint16_t channelNumber, float value)
{
    return this->setChannel(channelNumber, value, &this->channel, &xSemaphorechannel);
}

float dataFloat::getChannel(uint16_t channelNumber, std::vector<float> *channel, SemaphoreHandle_t *xSemaphoreOfArg)
{
    if (channelNumber > (qtdChannels - 1))
    {
        //ESP_LOGE(tag, "O canal informado \"%ud\" não existe, máximo: %ud", channelNumber, (qtdChannels - 1));
        return RETORNO_ARGUMENTO_INVALIDO;
    }

    float tempChannel;
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
            //ESP_LOGE(tag, "Vetor de canais ocupado, não foi possível ler valor. Tentando novamente...");
        }
    }
}
float dataFloat::getChannel(uint16_t channelNumber)
{
    return this->getChannel(channelNumber, &this->channel, &xSemaphorechannel);
}

int dataFloat::setChannels(std::vector<float> values, std::vector<float> *channel, SemaphoreHandle_t *xSemaphoreOfArg)
{
    if (xSemaphoreTake((*xSemaphoreOfArg), (TickType_t)10) == pdTRUE)
    {
        (*channel) = values;
        xSemaphoreGive((*xSemaphoreOfArg));
        return RETORNO_OK;
    }
    else
    {
        //ESP_LOGE(tag, "Vetor de canais ocupado, não foi possível definir valores.");
        return RETORNO_VARIAVEL_OCUPADA;
    }
}
int dataFloat::setChannels(std::vector<float> values)
{
    return this->setChannels(values, &this->channel, &xSemaphorechannel);
}

std::vector<float> dataFloat::getChannels(std::vector<float> *channel, SemaphoreHandle_t *xSemaphoreOfArg)
{
    std::vector<float> tempChannels;
    for (;;)
    {
        if (xSemaphoreTake((*xSemaphoreOfArg), (TickType_t)10) == pdTRUE)
        {
            tempChannels = (*channel);
            xSemaphoreGive((*xSemaphoreOfArg));
        }
        else
        {
            //ESP_LOGE(tag, "Vetor de canais ocupado, não foi possível ler valores. Tentando novamente...");
        }

        return tempChannels;
    }
}
std::vector<float> dataFloat::getChannels()
{
    return this->getChannels(&this->channel, &xSemaphorechannel);
}

int dataFloat::setChannelMin(uint16_t channelNumber, float value)
{
    return this->setChannel(channelNumber, value, &this->minChannel, &xSemaphoreminChannel);
}
float dataFloat::getChannelMin(uint16_t channelNumber)
{
    return this->getChannel(channelNumber, &this->minChannel, &xSemaphoreminChannel);
}

int dataFloat::setChannelMax(uint16_t channelNumber, float value)
{
    return this->setChannel(channelNumber, value, &this->maxChannel, &xSemaphoremaxChannel);
}
float dataFloat::getChannelMax(uint16_t channelNumber)
{
    return this->getChannel(channelNumber, &this->maxChannel, &xSemaphoremaxChannel);
}

int dataFloat::setChannelsMins(std::vector<float> values)
{
    return this->setChannels(values, &this->minChannel, &xSemaphoreminChannel);
}
std::vector<float> dataFloat::getChannelsMins()
{
    return this->getChannels(&this->minChannel, &xSemaphoreminChannel);
}

int dataFloat::setChannelsMaxes(std::vector<float> values)
{
    return this->setChannels(values, &this->maxChannel, &xSemaphoremaxChannel);
}
std::vector<float> dataFloat::getChannelsMaxes()
{
    return this->getChannels(&this->maxChannel, &xSemaphoremaxChannel);
}