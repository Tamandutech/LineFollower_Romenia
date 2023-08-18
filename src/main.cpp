/*
---------------- BLOCO DE NOTAS ------------------
 - Ajeitar a inicialização dos serviços na mais
 - Tirar os // dos ESP_LOGs quando arrumar o VSCode
*/

// Espressif (ESP-IDF)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

// Servicos
#include "RobotData.h"
#include "DataAbstract.hpp"
#include "ControlService.hpp"
#include "MappingService.hpp"
#include "MotorService.hpp"
#include "RPMService.hpp"
#include "SensorService.hpp"
#include "StatusService.hpp"

// Data Objects

// Criando os objetos dos serviços:
Robot *robot;
ControlService *controlService;
MappingService *mappingService;
MotorService *motorService;
RPMService *rpmService;
SensorService *sensorService;
StatusService *statusService;

extern "C"
{
  void app_main(void);
}

// Necessario criar os servicos

void app_main() 
{
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("main", ESP_LOG_INFO);

    //ESP_LOGD("Main", "Instanciando Robô...");
    robot = Robot::getInstance("TT_LF_ROMENIA");

    // Configuracao dos servicos
    //ESP_LOGD("Main", "Configurando Serviços...");
    mappingService = MappingService::getInstance("MappingService", 8192, 8);
    statusService = StatusService::getInstance("StatusService", 10000, 8);
    rpmService = RPMService::getInstance("RPMService", 4096, 9);
    motorService = MotorService::getInstance("MotorService", 4096, 5);
    sensorService = SensorService::getInstance("SensorService", 4096, 5);
    controlService = ControlService::getInstance("ControlService", 4096, 5);

    mappingService->Start();
    statusService->Start();
    rpmService->Start();
    motorService->Start();
    sensorService->Start();
    controlService->Start();

    for(;;)
    {
        // Comentarios e delay
    }
}