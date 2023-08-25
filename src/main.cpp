/*
---------------- BLOCO DE NOTAS ------------------
 - Ajeitar a inicialização dos serviços na main
 - Parametros na inicalização do IMUService na main
 - Tirar os // dos ESP_LOGs quando arrumar o VSCode
 - Verificar se é necessário um delay na biblioteca MUTEX
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
#include "IMUService.hpp"
#include "LEDsService.hpp"
#include "MappingService.hpp"
#include "MotorService.hpp"
#include "RPMService.hpp"
#include "SensorService.hpp"
#include "StatusService.hpp"

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

extern "C"
{
  void app_main(void);
}

// Necessario criar os servicos

void app_main() 
{
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("Main", ESP_LOG_INFO);

    //ESP_LOGD("Main", "Instanciando Robô...");
    robot = Robot::getInstance("TT_LF_ROMENIA");

    // Configuracao dos servicos
    //ESP_LOGD("Main", "Configurando Serviços...");
    ledsService = LEDsService::getInstance("LEDsService", 4096, 9);
    ledsService->Start();

    led_position_t LEDposition[NUM_LEDS] = {LED_POSITION_NONE};
    LEDposition[0] = LED_POSITION_FRONT;
    ledsService->config_LED(LEDposition, COLOR_RED, LED_EFFECT_SET, 1);

    //ESP_LOGD("Main", "Configurando LOGs...");
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("LEDsService", ESP_LOG_ERROR);
    esp_log_level_set("StatusService", ESP_LOG_ERROR);
    esp_log_level_set("Main", ESP_LOG_DEBUG);

    //ESP_LOGD("Main", "Configurando Serviços...");
    
    mappingService = MappingService::getInstance("MappingService", 8192, 8);
    imuService = IMUService::getInstance("IMUService", 4096, 5);
    statusService = StatusService::getInstance("StatusService", 10000, 8);
    rpmService = RPMService::getInstance("RPMService", 4096, 9);
    motorService = MotorService::getInstance("MotorService", 4096, 5);
    sensorService = SensorService::getInstance("SensorService", 4096, 5);
    controlService = ControlService::getInstance("ControlService", 4096, 5);

    mappingService->Start();
    imuService->Start();
    statusService->Start();
    rpmService->Start();
    motorService->Start();
    sensorService->Start();
    controlService->Start();

    ledsService->config_LED(LEDposition, COLOR_PURPLE, LED_EFFECT_SET, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    //ESP_LOGD("Main", "Apagando LEDs");
    ledsService->config_LED(LEDposition, COLOR_BLACK, LED_EFFECT_SET, 1);

    for(;;)
    {
        // Comentarios e delay
    }
}