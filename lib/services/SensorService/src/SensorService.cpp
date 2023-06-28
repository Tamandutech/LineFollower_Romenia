#include "SensorService.hpp"

SensorService::SensorService(std::string name, uint32_t stackDepth, UBaseType_t priority):Thread(name, stackDepth, priority)
{
    esp_log_level_set(name.c_str(), ESP_LOG_INFO);

    // Inicializacao dos sensores frontais
    // Todos sao controlados na mesma porta, porém cada objeto do vetor representa um sensor diferente
    // Para controlar cada sensor, é preciso mudar as portas digitais com o selectMuxPin()
    for(int i=0; i < sQuant; i++)
    {
        sArray[i].setTypeAnalogESP();
        sArray[i].setSensorPins((const adc1_channel_t[]){(adc1_channel_t)sInput}, 1);
        sArray[i].setSamplesPerSensor(5);
    }
    // Inicializacao dos sensores laterais
    sLat.setTypeAnalogESP();
    sLat.setSensorPins((const adc1_channel_t[]){(adc1_channel_t)s_lat_esq, (adc1_channel_t)s_lat_dir}, 2);
    sLat.setSamplesPerSensor(5);
    // Calibracao
    auto_calibrate();
}

void SensorService::Run()
{
    ESP_LOGE("Sensor", "Inicio.");
    // Loop do servico
    
}

void SensorService::auto_calibrate()
{
    auto get_Vel = Robot::getInstance()->getMotorData();
    // Calibracao automatica, em desenvolvimento
    // Calibração dos sensores frontais
    for(int t=0; t < tempo_cal; t++)
    {
        ESP_LOGD(GetName().c_str(), "(%p) | sArray: (%p)", this, &sArray);
        
        // Anda para uma direcao por 1/4 do tempo de calibracao
        // Anda pra direcao contraria por 2/4 do tempo
        // Volta a posicao original, andando o último 1/4 do tempo de volta
        if((t<tempo_cal/4)||((t > 3*tempo_cal/4)))
        {
            MotorService::getInstance()->WalkStraight(get_Vel->vel_calibracao->getData(), 0);
        }
        else
        {
            MotorService::getInstance()->WalkStraight(get_Vel->vel_calibracao->getData(), 1);
        }
        MUX.calibrate_all(sArray, sQuant);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    // Calibração dos sensores laterais
    for(int t=0; t < tempo_cal; t++)
    {
        ESP_LOGD(GetName().c_str(), "(%p) | sLat: (%p)", this, &sLat);
        if((t<tempo_cal/4)||((t > 3*tempo_cal/4)))
        {
            MotorService::getInstance()->WalkStraight(get_Vel->vel_calibracao->getData(), 0);
        }
        else
        {
            MotorService::getInstance()->WalkStraight(get_Vel->vel_calibracao->getData(), 1);
        }
        sLat.calibrate();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
