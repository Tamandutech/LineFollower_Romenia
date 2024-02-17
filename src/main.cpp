
// Servicos
#include "RobotData.h"
#include "DataAbstract.hpp"
#include "ControlService.hpp"
#include "IMUService.hpp"
#include "LEDsService.hpp"
#include "MappingService.hpp"
#include "MotorService.hpp"
#include "SensorService.hpp"
#include "StatusService.hpp"
#include "BLEServerService.hpp"
#include "IRService.hpp"

// Espressif (ESP-IDF)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "cmd_system.hpp"
#include "cmd_param.hpp"
#include "cmd_datamanager.hpp"
#include "cmd_runtime.hpp"
#include "better_console.hpp"



// C/C++
#include <stdbool.h>
#include <string>

// Data Objects

// Criando os objetos dos serviços:
Robot *robot;
ControlService *controlService;
IMUService *imuService;
LEDsService *ledsService;
MappingService *mappingService;
MotorService *motorService;
SensorService *sensorService;
StatusService *statusService;
BLEServerService *bleServerService;
IRService *irService;

extern "C"
{
  void app_main(void);
}

// Necessario criar os servicos

void app_main() 
{
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("Main", ESP_LOG_INFO);

    //ESP_LOGI("Main", "Instanciando Robô...");
    robot = Robot::getInstance("TT_LF_ROMENIA");

    // Configuracao dos servicos
    //ESP_LOGI("Main", "Configurando Serviços...");
    ledsService = LEDsService::getInstance("LEDsService", 4096, 9);
    ledsService->Start();

    ledsService->LedComandSend(LED_POSITION_FRONT, COLOR_RED, 1);

    //ESP_LOGI("Main", "Configurando LOGs...");
    esp_log_level_set("*", ESP_LOG_ERROR);
    //esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set("LEDsService", ESP_LOG_ERROR);
    esp_log_level_set("StatusService", ESP_LOG_INFO);
    esp_log_level_set("BLEServerService", ESP_LOG_DEBUG);
    esp_log_level_set("DataManager", ESP_LOG_ERROR);
    esp_log_level_set("DataStorage", ESP_LOG_ERROR);
    esp_log_level_set("Main", ESP_LOG_DEBUG);
    esp_log_level_set("MappingService", ESP_LOG_ERROR);
    esp_log_level_set("BLEServerService", ESP_LOG_DEBUG);
    esp_log_level_set("DataManager", ESP_LOG_ERROR);
    esp_log_level_set("DataStorage", ESP_LOG_ERROR);
    esp_log_level_set("QTRSensorMUX", ESP_LOG_DEBUG);

    //ESP_LOGD("Main", "Configurando Comandos...");
    better_console_config_t console_config;
    console_config.max_cmdline_args = 8;
    console_config.max_cmdline_length = 256;
    ESP_ERROR_CHECK(better_console_init(&console_config));
    //ESP_LOGD("Main", "Registrando Comandos...");
    register_system(robot->getADC_handle());
    register_cmd_param();
    register_cmd_datamanager();
    register_cmd_runtime();

    //ESP_LOGI("Main", "Configurando Serviços...");

    bleServerService = BLEServerService::getInstance("BLEServerService", 4096, 7, PRO_CPU_NUM);
    bleServerService->Start();
    
    mappingService = MappingService::getInstance("MappingService", 8192, 8);
    //ESP_LOGI(MappingService::getInstance()->GetName().c_str(), "Mapeamento");
    
    //imuService = IMUService::getInstance("IMUService", 4096, 5, PRO_CPU_NUM);
    //ESP_LOGI(IMUService::getInstance()->GetName().c_str(), "IMUService");
    
    statusService = StatusService::getInstance("StatusService", 10000, 8, PRO_CPU_NUM);
    //ESP_LOGI(StatusService::getInstance()->GetName().c_str(), "StatusService");
    
    sensorService = SensorService::getInstance("SensorService", 8192, 9, PRO_CPU_NUM);
    //ESP_LOGI(SensorService::getInstance()->GetName().c_str(), "SensorService");

    motorService = MotorService::getInstance("MotorsService", 4096, 8);
    //ESP_LOGI(StatusService::getInstance()->GetName().c_str(), "MotorsService");
    
    controlService = ControlService::getInstance("ControlService", 8192, 10, APP_CPU_NUM);
    ESP_LOGI(ControlService::getInstance()->GetName().c_str(), "ControlService");

    irService = IRService::getInstance("IRService", 4096, 9);
    //ESP_LOGI(RPMService::getInstance()->GetName().c_str(), "IRService");

    //mappingService->Start();
    //imuService->Start();
    motorService->Start();
    statusService->Start();
    sensorService->Start();
    controlService->Start();
    irService->Start();

    //ESP_LOGI("Main", "Ligando LEDs");
    ledsService->LedComandSend(LED_POSITION_FRONT, COLOR_PURPLE, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    //ESP_LOGI("Main", "Apagando LEDs");
    ledsService->LedComandSend(LED_POSITION_FRONT, COLOR_BLACK, 1);

    

    for (;;)
    {
      //ESP_LOGD("Main", "StatusService: %d", eTaskGetState(statusService->GetHandle()));
      //ESP_LOGD("main", "mappingService: %d", eTaskGetState(mappingService->GetHandle()));
      //ESP_LOGD("main", "rpmService: %d", eTaskGetState(rpmService->GetHandle()));
      //ESP_LOGD("main", "motorService: %d", eTaskGetState(motorService->GetHandle()));
      //ESP_LOGD("main", "sensorService: %d", eTaskGetState(sensorService->GetHandle()));
      //ESP_LOGD("Main", "controlService: %d", eTaskGetState(controlService->GetHandle()));
      //ESP_LOGD("main", "ledsService: %d", eTaskGetState(ledsService->GetHandle()));

      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}