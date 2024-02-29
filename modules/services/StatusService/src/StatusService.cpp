#include "StatusService.hpp"

SemaphoreHandle_t StatusService::SemaphoreButton;

void IRAM_ATTR StatusService::start_robot_with_boot_button(void *arg)
{
    BaseType_t high_task_awoken = pdFALSE;
    xSemaphoreGiveFromISR(SemaphoreButton, &high_task_awoken);
}

// Construtor do servico
StatusService::StatusService(std::string name, uint32_t stackDepth, UBaseType_t priority, BaseType_t coreid) : Thread(name, stackDepth, priority, coreid)
{
    // Atalhos para o RobotData:
    this->robot = Robot::getInstance();
    this->get_Status = robot->getStatus();
    this->get_Speed = robot->getMotorData();
    this->get_latMarks = robot->getSLatMarks();
    this->get_Spec = robot->getSpecification();
    // Atalhos para servicos:
    mappingService = MappingService::getInstance();
    this->LED = LEDsService::getInstance();
    esp_log_level_set(name.c_str(), ESP_LOG_INFO); // enable dos prints desse serviço

    get_Status->robotState->setData(CAR_STOPPED);
    get_Status->RealTrackStatus->setData(DEFAULT_TRACK);
    get_Status->TrackStatus->setData(DEFAULT_TRACK);
    get_Speed->vel_mapped->setData(get_Speed->getSpeed(DEFAULT_TRACK, CAR_ENC_READING_BEFORE_FIRSTMARK)->getData());

    lastPaused = get_Status->robotPaused->getData();
    lastState = get_Status->robotState->getData();
    lastTrack = (TrackSegment)get_Status->TrackStatus->getData();

    SemaphoreButton = xSemaphoreCreateBinary();
    config_extern_interrupt_to_read_button(GPIO_NUM_0);

    if(!(get_Status->TunningMode->getData())) // Se o robo nao estiver em modo de teste
    {
        define_if_will_start_mapping();
    }

    actualize_friction();
}

void StatusService::config_extern_interrupt_to_read_button(gpio_num_t interruptPort)
{
    gpio_config_t interruptConfig = {};
    interruptConfig.intr_type = GPIO_INTR_NEGEDGE;
    interruptConfig.pin_bit_mask = (1ULL << interruptPort);
    interruptConfig.mode = GPIO_MODE_INPUT;
    interruptConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    interruptConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;

    gpio_config(&interruptConfig);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(interruptPort, start_robot_with_boot_button, NULL);
}

void StatusService::define_if_will_start_mapping(){
    get_latMarks->marks->loadData();
    initialRobotState = CAR_MAPPING;

    if (get_latMarks->marks->getSize() > 0)
        start_following_defined_map();
}

void StatusService::start_following_defined_map(){
    get_Status->TrackStatus->setData(SHORT_LINE);
    get_Status->RealTrackStatus->setData(SHORT_LINE);

    initialRobotState = CAR_ENC_READING_BEFORE_FIRSTMARK;
    numMarks = get_latMarks->marks->getSize();
    mediaEncFinal = get_latMarks->marks->getData(numMarks - 1).MapEncMedia;
}

// Main do servico
void StatusService::Run()
{
    // Variavel necerraria para funcionalidade do vTaskDelayUtil, guarda a conGetName().c_str()em de pulsos da CPU
    TickType_t xLastWakeTime = xTaskGetTickCount();

    wait_press_boot_button_to_start();
    
    LED->LedComandSend(LED_POSITION_FRONT, COLOR_RED, 1);
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    
    if(!gpio_get_level(GPIO_NUM_0) && get_latMarks->marks->getSize() > 0 && !get_Status->TunningMode->getData() && get_Status->HardDeleteMap->getData())
    {
        delete_mapping_if_boot_button_is_pressed();
    }
    //ESP_LOGI(GetName().c_str(), "Iniciando delay de 1500ms");
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    if (initialRobotState == CAR_MAPPING && !get_Status->TunningMode->getData())
        start_mapping_the_track();

    started_in_Tuning = false;
    if (get_Status->TunningMode->getData())
        set_tuning_mode();

    get_Status->robotState->setData(initialRobotState);
    get_Status->FirstMark->setData(false); // Indica que nao passou pela primeira marcaçao

    //Loop
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, 30 / portTICK_PERIOD_MS);
        //lastTime = esp_timer_get_time();
        // Carregando status atual
        TrackLen = (TrackSegment)get_Status->TrackStatus->getData();
        pulsesAfterCurve = get_latMarks->PulsesAfterCurve->getData();
        actualCarState = (CarState) get_Status->robotState->getData();

        if (get_Status->robotPaused->getData())
            lastPaused = true;

        if(check_if_passed_first_mark())
            reset_enconder_value();
        
        if(track_segment_changed()){
            set_LEDs();
        }
        if(get_Status->ControlOff->getData() && actualCarState != CAR_STOPPED){
            mappingService->createNewMark();
            robot->getStatus()->robotState->setData(CAR_STOPPED);
            DataManager::getInstance()->saveAllParamDataChanged();
            LEDsService::getInstance()->LedComandSend(LED_POSITION_FRONT, COLOR_PINK, 1);
            LED->LedComandSend(LED_POSITION_LEFT, COLOR_PINK, 1);
            LED->LedComandSend(LED_POSITION_RIGHT, COLOR_PINK, 1);
        }

        mediaEncActual = (get_Speed->EncRight->getData() + get_Speed->EncLeft->getData()) / 2; // calcula media dos encoders
        
        if (actualCarState == CAR_TUNING && !get_Status ->TunningMode->getData())
            stop_tunning_mode();

        if (actualCarState == CAR_ENC_READING && (!get_Status->TunningMode->getData() || !started_in_Tuning))
        {
            if ((mediaEncActual - initialmediaEnc) >= mediaEncFinal)
            {
                //ESP_LOGD(GetName().c_str(), "Parando o robô");

                robot->getStatus()->robotState->setData(CAR_STOPPED);
                DataManager::getInstance()->saveAllParamDataChanged();
                LEDsService::getInstance()->LedComandSend(LED_POSITION_FRONT, COLOR_BLACK, 1);
                LED->LedComandSend(LED_POSITION_LEFT, COLOR_BLACK, 1);
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_BLACK, 1);
            }
            if ((mediaEncActual - initialmediaEnc) < mediaEncFinal)
            {
                // define o status do carrinho se o mapeamento não estiver ocorrendo
                int mark = 0;
                for (mark = 0; mark < numMarks - 1; mark++)
                {
                    // Verifica a contagem do encoder e atribui o estado ao robô
                    int32_t Manualmedia = get_latMarks->marks->getData(mark).MapEncMedia;        // Média dos encoders na chave mark
                    int32_t ManualmediaNxt = get_latMarks->marks->getData(mark + 1).MapEncMedia; // Média dos encoders na chave mark + 1

                    if ((mediaEncActual - initialmediaEnc) >= Manualmedia && (mediaEncActual - initialmediaEnc) <= ManualmediaNxt) // análise do valor das médias dos encoders
                    {

                        trackLen = (TrackSegment)get_latMarks->marks->getData(mark+1).MapTrackStatus;
                        get_Status->RealTrackStatus->setData(trackLen);
                        load_track_mapped(mark+1);
                        transition = false;

                        offset = get_latMarks->marks->getData(mark).MapOffset;
                        offsetnxt = get_latMarks->marks->getData(mark+1).MapOffset;

                        // Verifica se o robô precisa reduzir a velocidade, entrando no modo curva
                        if (mark + 2 < numMarks)
                        {

                            if (mappingService->track_is_a_line((TrackSegment)get_latMarks->marks->getData(mark + 1).MapTrackStatus) && mappingService->track_is_a_curve((TrackSegment)get_latMarks->marks->getData(mark + 2).MapTrackStatus) && offsetnxt == 0)
                            {
                                offsetnxt = -calculate_offset(mark+1);
                            }
                            if (offsetnxt < 0)
                            {
                                if (((mediaEncActual - initialmediaEnc) > (ManualmediaNxt + offsetnxt)))
                                {
                                    transition = true;
                                    load_track_mapped(mark+2);
                                }
                            }
                        }

                        if (mappingService->track_is_a_curve((TrackSegment)get_latMarks->marks->getData(mark).MapTrackStatus) && mappingService->track_is_a_line((TrackSegment)get_latMarks->marks->getData(mark + 1).MapTrackStatus))
                        {
                            offset += pulsesAfterCurve;
                        }
                        if (offset > 0)
                        {
                            if (((mediaEncActual - initialmediaEnc) < (Manualmedia + offset)))
                            {
                                transition = true;
                            
                                load_track_mapped(mark);
                            }
                        }
                        // Atualiza estado do robô
                        get_Status->Transition->setData(transition);
                        get_Status->TrackStatus->setData(trackLen);
                        break;
                    }
                }
            }
        }

        //uint32_t time = (uint32_t) (esp_timer_get_time() - lastTime);
        //ESP_LOGI(GetName().c_str(), "Tempo: %lu", time);
    }
}

void StatusService::wait_press_boot_button_to_start()
{
    //ESP_LOGD(GetName().c_str(), "Aguardando pressionamento do botão.");
    xSemaphoreTake(SemaphoreButton, portMAX_DELAY);
}

void StatusService::delete_mapping_if_boot_button_is_pressed()
{
    DataStorage::getInstance()->delete_data("Marcacoes_sLatMarks.marks");
    initialRobotState = CAR_MAPPING;
    //ESP_LOGD(GetName().c_str(), "Mapeamento Deletado");
    LEDsService::getInstance()->LedComandSend(LED_POSITION_FRONT, COLOR_YELLOW, 1);
}

void StatusService::start_mapping_the_track(){
    //ESP_LOGD(GetName().c_str(), "Mapeamento inexistente, iniciando robô em modo mapemaneto.");
    LED->LedComandSend(LED_POSITION_FRONT, COLOR_YELLOW, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // Começa mapeamento
    get_Status->RealTrackStatus->setData(DEFAULT_TRACK);
    get_Status->TrackStatus->setData(DEFAULT_TRACK);
    get_Speed->vel_mapped->setData(get_Speed->getSpeed(DEFAULT_TRACK, CAR_MAPPING)->getData());
    mappingService->startNewMapping();
}

void StatusService::set_tuning_mode(){
    started_in_Tuning = true;
    initialRobotState = CAR_TUNING;
    get_latMarks->marks->clearAllData();
    get_Speed->vel_mapped->setData(get_Speed->getSpeed(DEFAULT_TRACK, CAR_TUNING)->getData());
    numMarks = 0;
    mediaEncFinal = 0;
    LEDsService::getInstance()->LedComandSend(LED_POSITION_FRONT, COLOR_WHITE, 0.5);
    LED->LedComandSend(LED_POSITION_LEFT, COLOR_WHITE, 0.5);
    LED->LedComandSend(LED_POSITION_RIGHT, COLOR_WHITE, 0.5);
}

bool StatusService::check_if_passed_first_mark()
{
    return get_latMarks->rightMarks->getData() >= 1 && actualCarState == CAR_ENC_READING_BEFORE_FIRSTMARK;
}

void StatusService::reset_enconder_value()
{
    actualCarState = CAR_ENC_READING;
    get_Status->robotState->setData(actualCarState);
    initialmediaEnc = (get_Speed->EncRight->getData() + get_Speed->EncLeft->getData()) / 2;
}

bool StatusService::track_segment_changed()
{
    return lastTrack != (TrackSegment)get_Status->TrackStatus->getData() 
        || lastTransition != get_Status->Transition->getData();
}

void StatusService::set_LEDs()
{
    lastTrack =  (TrackSegment)get_Status->TrackStatus->getData();
    lastTransition = get_Status->Transition->getData();
    if (mappingService->track_is_a_line(lastTrack) && !lastTransition)
    {
        //ESP_LOGI(GetName().c_str(), "Alterando os leds para modo inLine.");
        //gpio_set_level((gpio_num_t)buzzer_pin, 1);
        switch (TrackLen)
        {
            case SHORT_LINE:
                LED->LedComandSend(LED_POSITION_LEFT, COLOR_GREEN, 1);
                LED->LedComandSend(LED_POSITION_FRONT, COLOR_GREEN, 1);
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_GREEN, 1);
                break;
            case MEDIUM_LINE:
                LED->LedComandSend(LED_POSITION_LEFT, COLOR_GREEN, 1);
                LED->LedComandSend(LED_POSITION_FRONT, COLOR_GREEN, 1);
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_GREEN, 1);
                break;
            case LONG_LINE:
                LED->LedComandSend(LED_POSITION_LEFT, COLOR_GREEN, 1);
                LED->LedComandSend(LED_POSITION_FRONT, COLOR_GREEN, 1);
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_GREEN, 1);
                break;
            case XLONG_LINE:
                LED->LedComandSend(LED_POSITION_LEFT, COLOR_GREEN, 1);
                LED->LedComandSend(LED_POSITION_FRONT, COLOR_GREEN, 1);
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_GREEN, 1);
                break;
            case SPECIAL_TRACK:
                LED->LedComandSend(LED_POSITION_LEFT, COLOR_YELLOW, 1);
                LED->LedComandSend(LED_POSITION_FRONT, COLOR_YELLOW, 1);
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_YELLOW, 1);
                break;
            default:
                LED->LedComandSend(LED_POSITION_LEFT, COLOR_WHITE, 1);
                LED->LedComandSend(LED_POSITION_FRONT, COLOR_WHITE, 1);
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_WHITE, 1);
                break;
        }
    }
    else if(!lastTransition)
    {
        //ESP_LOGI(GetName().c_str(), "Alterando os leds para modo inCurve.");
        switch (TrackLen)
        {
            case SHORT_CURVE:
                LED->LedComandSend(LED_POSITION_LEFT, COLOR_RED, 1);
                LED->LedComandSend(LED_POSITION_FRONT, COLOR_RED, 1);
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_RED, 1);
                break;
            case MEDIUM_CURVE:
                LED->LedComandSend(LED_POSITION_FRONT, COLOR_PINK, 0.7);
                LED->LedComandSend(LED_POSITION_LEFT, COLOR_PINK, 0.7);
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_PINK, 0.7);
                break;
            case LONG_CURVE:
                LED->LedComandSend(LED_POSITION_FRONT, COLOR_PINK, 1);
                LED->LedComandSend(LED_POSITION_LEFT, COLOR_PINK, 1);
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_PINK, 1);
                break;
            case XLONG_CURVE:
                LED->LedComandSend(LED_POSITION_FRONT, COLOR_PINK, 1);
                LED->LedComandSend(LED_POSITION_LEFT, COLOR_PINK, 1);
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_PINK, 1);
                break;
            case ZIGZAG_TRACK:
                LED->LedComandSend(LED_POSITION_LEFT, COLOR_WHITE, 1);
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_WHITE, 1);
                LED->LedComandSend(LED_POSITION_FRONT, COLOR_WHITE, 1);
                break;
            case SPECIAL_TRACK:
                LED->LedComandSend(LED_POSITION_LEFT, COLOR_YELLOW, 1);
                LED->LedComandSend(LED_POSITION_FRONT, COLOR_YELLOW, 1);
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_YELLOW, 1);
                break;
            default:
                LED->LedComandSend(LED_POSITION_LEFT, COLOR_WHITE, 1);
                LED->LedComandSend(LED_POSITION_FRONT, COLOR_WHITE, 1);
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_WHITE, 1);
                break;
        }
    }
    else if(lastTransition)
    {
        //gpio_set_level((gpio_num_t)buzzer_pin,1);
        LED->LedComandSend(LED_POSITION_LEFT, COLOR_BLUE, 1);
        LED->LedComandSend(LED_POSITION_RIGHT, COLOR_BLUE, 1);
        LED->LedComandSend(LED_POSITION_FRONT, COLOR_BLUE, 1);
    }
}

void StatusService::stop_tunning_mode(){
    robot->getStatus()->robotState->setData(CAR_STOPPED);
    vTaskDelay(0);
    DataManager::getInstance()->saveAllParamDataChanged();
    LED->LedComandSend(LED_POSITION_LEFT, COLOR_BLACK, 1);
    LED->LedComandSend(LED_POSITION_RIGHT, COLOR_BLACK, 1);
    LEDsService::getInstance()->LedComandSend(LED_POSITION_FRONT, COLOR_BLACK, 1);
}

void StatusService::load_track_mapped(int mark)
{
    trackLen = (TrackSegment)get_latMarks->marks->getData(mark).MapTrackStatus;
    trackSpeed = get_Speed->getSpeed(trackLen, CAR_ENC_READING)->getData();
    get_Speed->vel_mapped->setData(trackSpeed);
}

/* float StatusService::calculate_speed(int section){
    float vel;
    if(get_Status->VelCalculated->getData()){
        int32_t delta_right = (get_latMarks->marks->getData(section).MapEncRight - get_latMarks->marks->getData(section-1).MapEncRight);
        int32_t delta_left = (get_latMarks->marks->getData(section).MapEncLeft - get_latMarks->marks->getData(section-1).MapEncLeft);
        
        float radius;
        if(delta_right != delta_left) radius = std::abs(((float)get_Spec->RobotDiameter->getData()/2)*((float)(delta_right+delta_left)/(float)(delta_right-delta_left)));
        else radius = 0;

        float accel;
        if(get_Status->BrushlessON->getData()){
            accel = (get_Spec->Mass->getData()/get_Spec->Mass_BrushON->getData())* 9806.65;
        }else{
            accel = 9806.65;
        }

        if(get_Status->LineInMaxSpeed->getData()){
            if(get_latMarks->marks->getData(section).MapStatus == CAR_IN_LINE){ 
                vel = 100;
            }else{
                vel = std::sqrt(radius * accel * get_Spec->Friction_Coef->getData()); // em mm/s
                vel = (vel*60)/((float)get_Spec->WheelDiameter->getData()*M_PI); // conversão RPM
                if(vel <= get_Spec->MaxRPM->getData()) vel = (vel/get_Spec->MaxRPM->getData())*100;
                else vel = 100;
            }
        }else{
            vel = std::sqrt(radius * accel * get_Spec->Friction_Coef->getData()); // em mm/s
            vel = (vel*60)/((float)get_Spec->WheelDiameter->getData()*M_PI); // conversão RPM
            if(vel <= get_Spec->MaxRPM->getData()) vel = (vel/get_Spec->MaxRPM->getData())*100; // RPM para porcentagem
            else vel = 100;
        }
    }else{
        TrackSegment line_state = (TrackSegment)get_latMarks->marks->getData(section).MapTrackStatus;
        vel = get_Speed->Setpoint(line_state)->getData();
    }
    
    return vel;
} */

int16_t StatusService::calculate_offset(int section){
    float actual_speed;
    float next_speed;

    TrackSegment actual_track = (TrackSegment)get_latMarks->marks->getData(section).MapTrackStatus;
    TrackSegment next_track = (TrackSegment)get_latMarks->marks->getData(section+1).MapTrackStatus;
    
    //actual_speed = (float)get_Speed->RPMCar_media->getData();
    actual_speed = get_Speed->getSpeed(actual_track, CAR_ENC_READING)->getData();
    next_speed = get_Speed->getSpeed(next_track, CAR_ENC_READING)->getData();

    //actual_speed = (actual_speed*100.0)/get_Spec->MaxRPM->getData(); // conversão para %
    //actual_speed = convert_RPM_to_speed(actual_speed);
    //next_speed = (next_speed*get_Spec->MaxRPM->getData())/100;
    //next_speed = convert_RPM_to_speed(next_speed);
    
    float offset = (std::pow(actual_speed, 2) - std::pow(next_speed, 2)) * get_Speed->DecelerationOffsetGain->getData();
    offset = std::abs(offset);
    return (int16_t)offset;
}

float StatusService::convert_RPM_to_speed(float RPM){
    float speed; // mm/s
    speed = (M_PI * get_Spec->WheelDiameter->getData() * RPM)/60;
    return speed;
}

void StatusService::actualize_friction(){
    // Calculo do coeficiente de atrito
    float friction_angle = get_Spec->Friction_Angle->getData();
    float friction = (friction_angle)*M_PI/180; // de graus para rad
    friction = tan(friction);
    get_Spec->Friction_Coef->setData(friction);
    // Calculo da aceleracao máx
    if(get_Status->BrushlessON->getData()){
        get_Spec->Acceleration->setData(friction * 9806.65 * (get_Spec->Mass->getData()/get_Spec->Mass_BrushON->getData())); // g = 9806,65
    }else{
        get_Spec->Acceleration->setData(friction*9806.65); // g = 9806,65
    }
}