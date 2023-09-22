/*
---------------- BLOCO DE NOTAS ------------------
 - Tirar os // dos ESP_LOGs quando arrumar o VSCode
 - Inicializar os brushless no serviço MotorService
*/

// Servicos
#include "RobotData.h"
#include "DataAbstract.hpp"
#include "ControlService.hpp"
#include "IMUService.hpp"
#include "LEDsService.hpp"
#include "MappingService.hpp"
#include "MotorService.hpp"
#include "RPMService.hpp"
#include "SensorService.hpp"
#include "StatusService.hpp"
#include "BLEServerService.hpp"

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
RPMService *rpmService;
SensorService *sensorService;
StatusService *statusService;
BLEServerService *bleServerService;

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

    led_position_t LEDposition[NUM_LEDS] = {LED_POSITION_NONE};
    LEDposition[0] = LED_POSITION_FRONT;
    LEDposition[1] = LED_POSITION_NONE;
    ledsService->config_LED(LEDposition, COLOR_RED, LED_EFFECT_SET, 1);

    //ESP_LOGI("Main", "Configurando LOGs...");
    esp_log_level_set("*", ESP_LOG_ERROR);
    //esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set("LEDsService", ESP_LOG_ERROR);
    esp_log_level_set("StatusService", ESP_LOG_ERROR);
    esp_log_level_set("Main", ESP_LOG_DEBUG);
    esp_log_level_set("BLEServerService", ESP_LOG_DEBUG);
    esp_log_level_set("DataManager", ESP_LOG_ERROR);
    esp_log_level_set("DataStorage", ESP_LOG_ERROR);
    esp_log_level_set("QTRSensorMUX", ESP_LOG_DEBUG);

    //ESP_LOGD("Main", "Configurando Comandos...");
    better_console_config_t console_config;
    console_config.max_cmdline_args = 8;
    console_config.max_cmdline_length = 256;
    ESP_ERROR_CHECK(better_console_init(&console_config));
    ESP_LOGD("Main", "Registrando Comandos...");
    register_system();
    register_cmd_param();
    register_cmd_datamanager();
    register_cmd_runtime();

    //ESP_LOGI("Main", "Configurando Serviços...");

    bleServerService = BLEServerService::getInstance("BLEServerService", 4096, 7);
    bleServerService->Start();
    
    mappingService = MappingService::getInstance("MappingService", 8192, 8);
    //ESP_LOGI(MappingService::getInstance()->GetName().c_str(), "Mapeamento");
    
    //imuService = IMUService::getInstance("IMUService", 4096, 5);
    //ESP_LOGI(IMUService::getInstance()->GetName().c_str(), "IMUService");
    
    statusService = StatusService::getInstance("StatusService", 10000, 8);
    //ESP_LOGI(StatusService::getInstance()->GetName().c_str(), "StatusService");
    
    rpmService = RPMService::getInstance("RPMService", 4096, 9);
    //ESP_LOGI(RPMService::getInstance()->GetName().c_str(), "RPMService");
    
    motorService = MotorService::getInstance("MotorService", 4096, 5);
    //ESP_LOGI(MotorService::getInstance()->GetName().c_str(), "MotorService");
    
    sensorService = SensorService::getInstance("SensorService", 8192, 5);
    //ESP_LOGI(SensorService::getInstance()->GetName().c_str(), "SensorService");
    
    controlService = ControlService::getInstance("ControlService", 8192, 10);
    //ESP_LOGI(ControlService::getInstance()->GetName().c_str(), "ControlService");

    mappingService->Start();
    //imuService->Start();
    statusService->Start();
    rpmService->Start();
    motorService->Start();
    sensorService->Start();
    controlService->Start();

    ESP_LOGI("Main", "Ligando LEDs");
    ledsService->config_LED(LEDposition, COLOR_PURPLE, LED_EFFECT_SET, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI("Main", "Apagando LEDs");
    ledsService->config_LED(LEDposition, COLOR_BLACK, LED_EFFECT_SET, 1);

    

    for (;;)
    {
      //ESP_LOGD("main", "StatusService: %d", eTaskGetState(statusService->GetHandle()));
      //ESP_LOGD("main", "mappingService: %d", eTaskGetState(mappingService->GetHandle()));
      //ESP_LOGD("main", "rpmService: %d", eTaskGetState(rpmService->GetHandle()));
      //ESP_LOGD("main", "motorService: %d", eTaskGetState(motorService->GetHandle()));
      //ESP_LOGD("main", "sensorService: %d", eTaskGetState(sensorService->GetHandle()));
      //ESP_LOGD("main", "controlService: %d", eTaskGetState(controlService->GetHandle()));
      //ESP_LOGD("main", "ledsService: %d", eTaskGetState(ledsService->GetHandle()));

      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}