#ifndef CAR_STATUS_SERVICE_H
#define CAR_STATUS_SERVICE_H

#include "thread.hpp"
#include "singleton.hpp"
#include "RobotData.h"
#include "dataEnums.h"
#include "driver/gpio.h"

#include "MappingService.hpp"
#include "LEDsService.hpp"


using namespace cpp_freertos;

#include "esp_log.h"

#define ManualMap

class StatusService : public Thread, public Singleton<StatusService>
{
public:
    
    StatusService(std::string name, uint32_t stackDepth, UBaseType_t priority, BaseType_t coreid);

    void Run() override;
    static QueueHandle_t gpio_evt_queue;

private:

    // Atalhos do RobotData:
    Robot *robot;
    dataStatus *get_Status;
    dataMotor *get_Speed;
    dataSLatMarks *get_latMarks;
    dataSpec *get_Spec;
    // Atalhos de outros serviços:
    LEDsService *LED;
    MappingService *mappingService;

    // Variáveis
    
    CarState actualCarState;
    TrackState TrackLen = SHORT_CURVE;

    int numMarks = 0; // Número total de marcações laterais na pista

    bool stateChanged; // verifica se o carrinho mudou seu estado quanto ao mapeamento
    bool lastTransition = false;

    TrackState lastTrack = SHORT_LINE; // armazena último tipo de trecho da pista percorrido
    uint8_t lastState; //armazena último estado do mapeamento
    bool lastMappingState;

    bool started_in_Tuning = false; // se o robo esta em modo de teste
    int32_t mediaEncActual = 0;
    int32_t mediaEncFinal = 0; // leitura de encoder final do trecho
    int32_t initialmediaEnc = 0; // leitura feita na linha de partida
    int32_t pulsesBeforeCurve = 200;
    int32_t pulsesAfterCurve = 200;
    bool firstmark = false;
    bool in_transition = false;
    int16_t offset_transition = 0;
    int mark_in_transition = 0;

    CarState trackType;
    TrackState trackLen;
    float trackSpeed;


    static void IRAM_ATTR gpio_isr_handler(void *arg);
    void mappingStatus(bool is_reading, bool is_mapping);
    void loadTrackMapped(int section, int section_speed);
    float calculate_speed(int section);
    float convert_RPM_to_speed(float RPM);
    int16_t calculate_offset(int section);
    void actualize_friction();
};

#endif