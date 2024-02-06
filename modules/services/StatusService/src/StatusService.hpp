#ifndef CAR_STATUS_SERVICE_H
#define CAR_STATUS_SERVICE_H

#include "thread.hpp"
#include "singleton.hpp"
#include "esp_timer.h"
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
    static SemaphoreHandle_t SemaphoreButton;

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
    
    CarState actualCarState, initialRobotState;
    TrackSegment TrackLen = SHORT_CURVE;

    int numMarks = 0; // Número total de marcações laterais na pista

    bool stateChanged; // verifica se o carrinho mudou seu estado quanto ao mapeamento
    bool lastTransition = false;

    TrackSegment lastTrack = SHORT_LINE; // armazena último tipo de trecho da pista percorrido
    uint8_t lastState; //armazena último estado do mapeamento
    bool lastPaused = false;
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
    int64_t lastTime = 0;

    CarState trackType;
    TrackSegment trackLen;
    float trackSpeed;


    static void IRAM_ATTR start_robot_with_boot_button(void *arg);
    void config_extern_interrupt_to_read_button(gpio_num_t gpio_num);
    void define_if_will_start_mapping();
    void start_following_defined_map();
    void wait_press_boot_button_to_start();
    void delete_mapping_if_boot_button_is_pressed();
    void start_mapping_the_track();
    void set_tuning_mode();
    bool check_if_passed_first_mark();
    void reset_enconder_value();
    bool track_segment_changed();
    void set_LEDs();
    void stop_tunning_mode();
    void load_track_mapped(int mark);
    float calculate_speed(int section);
    float convert_RPM_to_speed(float RPM);
    int16_t calculate_offset(int section);
    void actualize_friction();
};

#endif