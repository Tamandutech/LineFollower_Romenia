#include "StatusService.hpp"

QueueHandle_t StatusService::gpio_evt_queue; // variavel de fila

void IRAM_ATTR StatusService::gpio_isr_handler(void *arg) // se precionar o botão
{
    // Salva o estado do robo em uma fila
    // carstate soh diferencia entre curva e reta
    uint32_t gpio_num = (uint32_t)arg;
    uint8_t carstate = CAR_IN_CURVE;
    if (gpio_num == GPIO_NUM_0)
    {
        carstate = CAR_IN_LINE;
    }
    xQueueSendFromISR(gpio_evt_queue, &carstate, NULL); // ( xQueue, pvItemToQueue, pxHigherPriorityTaskWoken )
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

    if(!(get_Status->TunningMode->getData())) // Se o robo nao estiver em modo de teste
    {
        get_latMarks->marks->loadData(); // carrega as marcações laterais salvas

        if (get_latMarks->marks->getSize() <= 0) // Se o robo não tem mapeamento salvo
        {
            // Muda o status para iniciar o mapeamento
            mappingStatus(false, true); // (bool is_reading, bool is_mapping)
        }
        else
        {// Se tem mapeamento salvo
            // Muda o status para ler o mapeamento
            mappingStatus(true, false); // (bool is_reading, bool is_mapping)
            numMarks = get_latMarks->marks->getSize();
            mediaEncFinal = get_latMarks->marks->getData(numMarks - 1).MapEncMedia;
        }
    }

    // Status inicial do robo (parado)
    get_Status->robotState->setData(CAR_STOPPED);

    stateChanged = true;
    lastMappingState = false;
    // Carregando o final da pista
    lastState = get_Status->robotState->getData();
    lastTrack = (TrackState) get_Status->TrackStatus->getData();

    firstmark = false; // Não passou pela linha de partida

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_0);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_NUM_0, gpio_isr_handler, (void *)GPIO_NUM_0);

    actualize_friction();
}

// Main do servico
void StatusService::Run()
{
    // Variavel necerraria para funcionalidade do vTaskDelayUtil, guarda a conGetName().c_str()em de pulsos da CPU
    TickType_t xLastWakeTime = xTaskGetTickCount();

    //ESP_LOGI(GetName().c_str(), "Aguardando pressionamento do botão.");
    uint8_t num;
    do
    {// Aguarda o precionar do botao
        xQueueReceive(gpio_evt_queue, &num, portMAX_DELAY);
        //ESP_LOGI(GetName().c_str(), "Aguardando inicialização");
        if(get_Status->robotState->getData() != CAR_STOPPED) break;
    } while (num != CAR_IN_LINE); // enquanto o botão não é precionado
    
    // Acendendo a luz vermelha na LED da frente:
    LED->set_LED(LED_POSITION_FRONT, COLOR_BLACK, LED_EFFECT_SET, 1);


    vTaskDelay(1500 / portTICK_PERIOD_MS);
    // Deletar o mapeamento caso o botão de boot seja mantido pressionado e exista mapeamento na flash
    if(!gpio_get_level(GPIO_NUM_0) && get_latMarks->marks->getSize() > 0 && !get_Status->TunningMode->getData() && get_Status->HardDeleteMap->getData())
    {
        DataStorage::getInstance()->delete_data("Marcacoes_sLatMarks.marks");
        mappingStatus(false, true); // (bool is_reading, bool is_mapping)
        //ESP_LOGI(GetName().c_str(), "Mapeamento Deletado");

        // Mudando a led frontal para amarelo:
        LED->set_LED(LED_POSITION_FRONT, COLOR_YELLOW, LED_EFFECT_SET, 1);
    }
    //ESP_LOGI(GetName().c_str(), "Iniciando delay de 1500ms");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    

    if(!get_Status->TunningMode->getData())
    { // Se nao estiver em modo de teste
        if(get_Status->robotIsMapping->getData())
        { // Se nao houver mapeamento salvo
            //ESP_LOGI(GetName().c_str(), "Mapeamento inexistente, iniciando robô em modo mapemaneto.");
            
            // Mudando a led frontal para amarelo:
            LED->set_LED(LED_POSITION_FRONT, COLOR_YELLOW, LED_EFFECT_SET, 1);

            vTaskDelay(1000 / portTICK_PERIOD_MS);
            // Começa mapeamento
            get_Status->RealTrackStatus->setData(UNDEFINED);
            get_Status->TrackStatus->setData(UNDEFINED);
            get_Speed->vel_mapped->setData(get_Speed->Setpoint(UNDEFINED)->getData());
            mappingService->startNewMapping();
        }
        else
        {// Se nao estiver mapeando, muda o status para linha curta (mais lento)
            get_Status->TrackStatus->setData(SHORT_LINE);
            get_Status->RealTrackStatus->setData(SHORT_LINE);
            get_Speed->vel_mapped->setData(get_Speed->Setpoint(SHORT_LINE)->getData());
        }

        get_Status->robotState->setData(CAR_IN_LINE); // Carro sai do estado parado
        started_in_Tuning = false;
    }
    else
    {// Se estiver em modo de teste
        started_in_Tuning = true;
        get_Status->robotState->setData(CAR_TUNING);
        get_Status->TrackStatus->setData(TUNNING);
        get_Status->RealTrackStatus->setData(TUNNING);
        trackSpeed = get_Speed->Setpoint(TUNNING)->getData();
        get_Speed->vel_mapped->setData(trackSpeed);
        mappingStatus(false, false); // (bool is_reading, bool is_mapping)
        get_latMarks->marks->clearAllData();
        numMarks = 0;
        mediaEncFinal = 0;
        
        // Muda a LED frontal para branco com brilho 50%
        LED->set_LED(LED_POSITION_FRONT, COLOR_WHITE, LED_EFFECT_SET, 0.5);
    }
    get_Status->FirstMark->setData(false); // Indica que nao passou pela primeira marcaçao

    //Loop
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, 30 / portTICK_PERIOD_MS);
        // Carregando status atual
        ////ESP_LOGI(GetName().c_str(), "Abulabuleh3333333333333333");
        get_Status->stateMutex.lock();
        TrackLen = (TrackState)get_Status->TrackStatus->getData();
        pulsesBeforeCurve = get_latMarks->PulsesBeforeCurve->getData();
        pulsesAfterCurve = get_latMarks->PulsesAfterCurve->getData();
        actualCarState = (CarState) get_Status->robotState->getData();
        if(started_in_Tuning && get_Status->TunningMode->getData() && get_Status->robotState->getData() != CAR_TUNING && get_Status->robotState->getData() != CAR_STOPPED &&  !get_Status->encreading->getData() && !get_Status->robotIsMapping->getData()) 
        {// Se iniciar em modo de teste
            get_Status->robotState->setData(CAR_TUNING);
            get_Status->TrackStatus->setData(TUNNING);
            get_Status->RealTrackStatus->setData(TUNNING);
            trackSpeed = get_Speed->Setpoint(TUNNING)->getData();
            get_Speed->vel_mapped->setData(trackSpeed);
            
            // LED frontal branca com brilho 50%
            LED->set_LED(LED_POSITION_FRONT, COLOR_WHITE, LED_EFFECT_SET, 0.5);
        }
        if(get_latMarks->rightMarks->getData() >= 1 && !firstmark)
        {// Se passar pela linha de largada e nao tiver essa informacao salva
            firstmark = true;
            get_Status->FirstMark->setData(true);
            initialmediaEnc = (get_Speed->EncRight->getData() + get_Speed->EncLeft->getData()) / 2;
        }
        if (lastMappingState != get_Status->robotIsMapping->getData() && get_Status->robotIsMapping->getData())
        {// Caso esteja mapeando
            lastMappingState = get_Status->robotIsMapping->getData();

            //ESP_LOGI(GetName().c_str(), "Alterando velocidades para modo mapeamento.");
            // LED frontal branca com brilho 50%
            LED->set_LED(LED_POSITION_FRONT, COLOR_YELLOW, LED_EFFECT_SET, 1);
        }
        else if ((lastState != get_Status->robotState->getData() || lastTrack != (TrackState)get_Status->TrackStatus->getData() || lastTransition != get_Status->Transition->getData()) && !lastMappingState && get_Status->robotState->getData() != CAR_STOPPED && get_Status->robotState->getData() != CAR_TUNING)
        {// Caso tenha mudado o trecho em que o robô se encontra
            lastState = get_Status->robotState->getData();
            lastTrack =  (TrackState)get_Status->TrackStatus->getData();
            lastTransition = get_Status->Transition->getData();
            gpio_set_level((gpio_num_t)buzzer_pin, 0);
            if (lastState == CAR_IN_LINE && !lastTransition)
            {
                //ESP_LOGI(GetName().c_str(), "Alterando os leds para modo inLine.");
                switch (TrackLen)
                {
                    case SHORT_LINE:
                        LED->set_LED(LED_POSITION_FRONT, COLOR_GREEN, LED_EFFECT_SET, 0.05);
                        break;
                    case MEDIUM_LINE:
                        LED->set_LED(LED_POSITION_FRONT, COLOR_GREEN, LED_EFFECT_SET, 0.3);
                        break;
                    case LONG_LINE:
                        LED->set_LED(LED_POSITION_FRONT, COLOR_GREEN, LED_EFFECT_SET, 1);
                        break;
                    case SPECIAL_TRACK:
                        LED->set_LED(LED_POSITION_FRONT, COLOR_PURPLE, LED_EFFECT_SET, 0.05);
                        break;
                    default:
                        LED->set_LED(LED_POSITION_FRONT, COLOR_WHITE, LED_EFFECT_SET, 1);
                        break;
                }
            }
            else if(lastState == CAR_IN_CURVE && !lastTransition)
            {
                //ESP_LOGI(GetName().c_str(), "Alterando os leds para modo inCurve.");
                switch (TrackLen)
                {
                    case SHORT_CURVE:
                        LED->set_LED(LED_POSITION_FRONT, COLOR_RED, LED_EFFECT_SET, 0.05);
                        break;
                    case MEDIUM_CURVE:
                        LED->set_LED(LED_POSITION_FRONT, COLOR_YELLOW, LED_EFFECT_SET, 0.3);
                        break;
                    case LONG_CURVE:
                        LED->set_LED(LED_POSITION_FRONT, COLOR_PINK, LED_EFFECT_SET, 1);
                        break;
                    case ZIGZAG:
                        LED->set_LED(LED_POSITION_FRONT, COLOR_PURPLE, LED_EFFECT_SET, 1);
                        break;
                    case SPECIAL_TRACK:
                        LED->set_LED(LED_POSITION_FRONT, COLOR_PURPLE, LED_EFFECT_SET, 0.05);
                        break;
                    default:
                        LED->set_LED(LED_POSITION_FRONT, COLOR_WHITE, LED_EFFECT_SET, 1);
                        break;
                }
            }
            else if(lastTransition)
            {
                gpio_set_level((gpio_num_t)buzzer_pin,1);
                LED->set_LED(LED_POSITION_FRONT, COLOR_BLUE, LED_EFFECT_SET, 0.5);
            }
        }

        mediaEncActual = (get_Speed->EncRight->getData() + get_Speed->EncLeft->getData()) / 2; // calcula media dos encoders
        if(((!get_Status->robotIsMapping->getData() && !get_Status->encreading->getData() && !get_Status->TunningMode->getData()) || get_Status->ControlOff->getData()) && actualCarState != CAR_STOPPED)
        {// Muda o status do carro para parado, caso esteja no momento certo
            get_Status->robotIsMapping->setData(false);
            get_Status->encreading->setData(false);
            get_Status->TunningMode->setData(false);
            get_Status->robotState->setData(CAR_STOPPED);
            vTaskDelay(0);
            DataManager::getInstance()->saveAllParamDataChanged();
            LED->set_LED(LED_POSITION_FRONT, COLOR_BLACK, LED_EFFECT_SET, 1);
        }
        if (!get_Status->robotIsMapping->getData() && actualCarState != CAR_STOPPED && get_Status->encreading->getData() && firstmark && (!get_Status->TunningMode->getData() || !started_in_Tuning))
        {// Se estiver lendo o mapeamento, e já tiver passado pela linha de largada
            if ((mediaEncActual - initialmediaEnc) >= mediaEncFinal)
            {// 
                if(get_Status->TuningMapped->getData())
                {
                    get_Status->FirstMark->setData(false);
                    firstmark = false;
                    initialmediaEnc = 0;
                    get_Status->robotState->setData(CAR_IN_LINE);
                    get_Status->TrackStatus->setData(SHORT_LINE);
                    get_Status->RealTrackStatus->setData(SHORT_LINE);
                    get_latMarks->rightMarks->setData(0);
                }
                else
                {
                    //ESP_LOGI(GetName().c_str(), "Parando o robô");
                    get_Status->encreading->setData(false);

                    get_Status->robotState->setData(CAR_STOPPED);
                    DataManager::getInstance()->saveAllParamDataChanged();
                    LED->set_LED(LED_POSITION_FRONT, COLOR_BLACK, LED_EFFECT_SET, 1);
                }
            }
            if ((mediaEncActual - initialmediaEnc) < mediaEncFinal)
            {
                // define o status do carrinho
                int mark = 0;
                for (mark = 0; mark < numMarks - 1; mark++)
                {
                    // Verifica a contagem do encoder e atribui o estado ao robô
                    int32_t Manualmedia = get_latMarks->marks->getData(mark).MapEncMedia;        // Média dos encoders na chave mark
                    int32_t ManualmediaNxt = get_latMarks->marks->getData(mark + 1).MapEncMedia; // Média dos encoders na chave mark + 1
                    if ((mediaEncActual - initialmediaEnc) >= Manualmedia && (mediaEncActual - initialmediaEnc) <= ManualmediaNxt) // análise do valor das médias dos encoders
                    {
                        loadTrackMapped(mark+1, mark+1);
                        get_Status->RealTrackStatus->setData(trackLen);
                        bool transition = false;
                        if(mark_in_transition != mark){
                            in_transition = false;
                        }
                        int16_t offset = 0;
                        int16_t offsetnxt = 0;
                        // Verifica se o robô precisa reduzir a velocidade, entrando no modo curva
                        if((CarState)get_latMarks->marks->getData(mark).MapStatus == CAR_IN_CURVE && (CarState)get_latMarks->marks->getData(mark + 1).MapStatus == CAR_IN_LINE)
                        {
                            offset += pulsesAfterCurve;
                        }
                        if(offset > 0)
                        {
                            if((Manualmedia + offset) < ManualmediaNxt && (mediaEncActual - initialmediaEnc) < (Manualmedia + offset)) 
                            {
                                transition = true;
                                loadTrackMapped(mark+1, mark+1);
                            }
                            else if((Manualmedia + offset) >= ManualmediaNxt)
                            {
                                transition = true;
                                loadTrackMapped(mark+1, mark+1);
                            }
                        }
                        if(mark + 2 < numMarks)
                        {
                            if((CarState)get_latMarks->marks->getData(mark+1).MapStatus == CAR_IN_LINE && (CarState)get_latMarks->marks->getData(mark + 2).MapStatus == CAR_IN_CURVE){
                                /* 
                                offsetnxt = -calculate_offset(mark+1) - pulsesBeforeCurve;
                                */
                                offsetnxt = - pulsesBeforeCurve;
                            }
                            if(offsetnxt < 0)
                            {
                                if((ManualmediaNxt + offsetnxt) > Manualmedia && (mediaEncActual - initialmediaEnc) > (ManualmediaNxt + offsetnxt)) 
                                {
                                    transition = true;
                                    in_transition = true;
                                    mark_in_transition = mark;
                                    offset_transition = offsetnxt;
                                    loadTrackMapped(mark+2, mark+2);
                                }
                                else if((ManualmediaNxt + offsetnxt) <= Manualmedia) 
                                {
                                    transition = true;
                                    in_transition = true;
                                    mark_in_transition = mark;
                                    offset_transition = offsetnxt;
                                    loadTrackMapped(mark+2, mark+2);
                                }
                            }
                        }
                        // Atualiza estado do robô
                        get_Status->Transition->setData(transition);
                        get_Status->robotState->setData(trackType);
                        get_Status->TrackStatus->setData(trackLen);
                        get_Speed->vel_mapped->setData(trackSpeed);
                        break;
                    }
                }
            }
        
        }

        get_Status->stateMutex.unlock();

    }
}

void StatusService::mappingStatus(bool is_reading, bool is_mapping)
{// Muda as variáveis de estado ligadas ao mapeamento.
    get_Status->encreading->setData(is_reading);
    get_Status->robotIsMapping->setData(is_mapping);
}

void StatusService::loadTrackMapped(int section, int section_speed)
{// Carrega as variaveis de um trecho da pista, salvas no mapeamento.
    trackType = (CarState)get_latMarks->marks->getData(section).MapStatus;
    trackLen = (TrackState)get_latMarks->marks->getData(section).MapTrackStatus;
    trackSpeed = calculate_speed(section_speed);
    //ESP_LOGI(GetName().c_str(), "Para marcação %d Raio = %.2f e Velocidade = %.2f", section);
}

float StatusService::calculate_speed(int section){
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
        TrackState line_state = (TrackState)get_latMarks->marks->getData(section).MapTrackStatus;
        vel = get_Speed->Setpoint(line_state)->getData();
    }
    
    return vel;
}

int16_t StatusService::calculate_offset(int section){
    float actual_speed;
    float next_speed;
    
    actual_speed = (float)get_Speed->RPMCar_media->getData();
    next_speed = calculate_speed(section+1);

    actual_speed = convert_RPM_to_speed(actual_speed);
    next_speed = (next_speed*get_Spec->MaxRPM->getData())/100;
    next_speed = convert_RPM_to_speed(next_speed);
    
    float offset_mm = (std::pow(actual_speed, 2) - std::pow(next_speed, 2))/(2*get_Spec->Acceleration->getData());
    offset_mm = std::abs(offset_mm);
    float offset_enc = (offset_mm * (float)get_Spec->MPR->getData())/((float)get_Spec->WheelDiameter->getData() * M_PI);
    return (int16_t)offset_enc;
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