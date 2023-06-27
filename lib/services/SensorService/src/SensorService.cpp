#include "SensorService.hpp"

SensorService::SensorService(std::string name, uint32_t stackDepth, UBaseType_t priority):Thread(name, stackDepth, priority)
{
    esp_log_level_set(name.c_str(), ESP_LOG_INFO);

    // Inicializacao dos sensores
    // Todos sao controlados na mesma porta, porém cada objeto do vetor representa um sensor diferente
    // Para controlar cada sensor, é preciso mudar as portas digitais com o selectMuxPin()
    for(int i=0; i < sQuant; i++)
    {
        sArray[i].setTypeAnalogESP();
        sArray[i].setSensorPins((const adc1_channel_t[]){(adc1_channel_t)sInput}, 1);
        sArray[i].setSamplesPerSensor(5);
    }
    auto_calibrate();
}

void SensorService::Run()
{
    ESP_LOGE("Sensor", "Inicio.");
    // Loop do servico
    
}

void SensorService::auto_calibrate()
{
    // Calibracao automatica, em desenvolvimento
    
}
