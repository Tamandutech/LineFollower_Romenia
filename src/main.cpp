// Espressif (ESP-IDF)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

// Servicos
#include "SensorService.hpp"
#include "MotorService.hpp"
#include "MappingService.hpp"

// Data Objects

// Criando os objetos dos serviços:
SensorService *sensorService;
MotorService *motorService;

extern "C"
{
  void app_main(void);
}

// Necessario criar os servicos

void app_main() 
{
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("main", ESP_LOG_INFO);

    // Necessario iniciar os servicos
    sensorService = new SensorService("SensorService", 4096, 5);
    sensorService->Start();
    motorService = new MotorService("MotorService", 4096, 5);
    sensorService->Start();

    for(;;)
    {
        // Comentarios e delay
    }
}