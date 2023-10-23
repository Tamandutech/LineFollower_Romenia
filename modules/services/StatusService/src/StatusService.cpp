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
StatusService::StatusService(std::string name, uint32_t stackDepth, UBaseType_t priority) : Thread(name, stackDepth, priority)
{
    // Atalhos para o RobotData:
    this->robot = Robot::getInstance();
    this->status = robot->getStatus();
    this->speed = robot->getMotorData();
    this->latMarks = robot->getSLatMarks();
    this->PID = robot->getPID();
    // Atalhos para servicos:
    mappingService = MappingService::getInstance();
    this->LED = LEDsService::getInstance();

    esp_log_level_set(name.c_str(), ESP_LOG_INFO);

    if(!status->TunningMode->getData()) // Se o robo nao estiver em modo de teste
    {
        latMarks->marks->loadData(); // carrega as marcações laterais salvas

        if (latMarks->marks->getSize() <= 0) // Se o robo não tem mapeamento salvo
        {
            // Muda o status para iniciar o mapeamento
            mappingStatus(false, true); // (bool is_reading, bool is_mapping)
        }
        else
        {// Se tem mapeamento salvo
            // Muda o status para ler o mapeamento
            mappingStatus(true, false); // (bool is_reading, bool is_mapping)
            numMarks = latMarks->marks->getSize();
            mediaEncFinal = latMarks->marks->getData(numMarks - 1).MapEncMedia;
        }
    }

    // Status inicial do robo (parado)
    status->robotState->setData(CAR_STOPPED);
    LEDposition[0] = LED_POSITION_FRONT;
    LEDposition[1] = LED_POSITION_NONE;

    stateChanged = true;
    lastMappingState = false;
    // Carregando o final da pista
    lastState = status->robotState->getData();
    lastTrack = (TrackState) status->TrackStatus->getData();

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
}

// Main do servico
void StatusService::Run()
{
    // Variavel necerraria para funcionalidade do vTaskDelayUtil, guarda a conGetName().c_str()em de pulsos da CPU
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // int iloop = 0;

    ESP_LOGI(GetName().c_str(), "Aguardando pressionamento do botão.");
    uint8_t num;
    do
    {// Aguarda o precionar do botao
        xQueueReceive(gpio_evt_queue, &num, portMAX_DELAY);
        //ESP_LOGI(GetName().c_str(), "Aguardando inicialização");
        if(status->robotState->getData() != CAR_STOPPED) break;
    } while (num != CAR_IN_LINE);

    // Acendendo a luz vermelha na LED da frente:
    LEDposition[0] = LED_POSITION_FRONT;
    LEDposition[1] = LED_POSITION_NONE;
    LED->config_LED(LEDposition, COLOR_RED, LED_EFFECT_SET, 1);


    vTaskDelay(1500 / portTICK_PERIOD_MS);
    // Deletar o mapeamento caso o botão de boot seja mantido pressionado e exista mapeamento na flash
    if(!gpio_get_level(GPIO_NUM_0) && latMarks->marks->getSize() > 0 && !status->TunningMode->getData() && status->HardDeleteMap->getData())
    {
        DataStorage::getInstance()->delete_data("Marcacoes_sLatMarks.marks");
        mappingStatus(false, true); // (bool is_reading, bool is_mapping)
        ESP_LOGI(GetName().c_str(), "Mapeamento Deletado");

        // Mudando a led frontal para amarelo:
        LED->config_LED(LEDposition, COLOR_YELLOW, LED_EFFECT_SET, 1);
    }
    //ESP_LOGI(GetName().c_str(), "Iniciando delay de 1500ms");
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    

    if(!status->TunningMode->getData())
    { // Se nao estiver em modo de teste
        if(status->robotIsMapping->getData())
        { // Se nao houver mapeamento salvo
            ESP_LOGI(GetName().c_str(), "Mapeamento inexistente, iniciando robô em modo mapemaneto.");
            
            // Mudando a led frontal para amarelo:
            LED->config_LED(LEDposition, COLOR_YELLOW, LED_EFFECT_SET, 1);

            vTaskDelay(1000 / portTICK_PERIOD_MS);
            // Começa mapeamento
            status->RealTrackStatus->setData(UNDEFINED);
            status->TrackStatus->setData(UNDEFINED);
            mappingService->startNewMapping();
        }
        else
        {// Se estiver mapeando, muda o status para linha curta (mais lento)
            status->TrackStatus->setData(MEDIUM_CURVE);
            status->RealTrackStatus->setData(MEDIUM_CURVE);
        }

        status->robotState->setData(CAR_IN_LINE); // Carro sai do estado parado
        started_in_Tuning = false;
    }
    else
    {// Se estiver em modo de teste
        started_in_Tuning = true;
        status->robotState->setData(CAR_TUNING);
        status->TrackStatus->setData(TUNNING);
        status->RealTrackStatus->setData(TUNNING);
        mappingStatus(false, false); // (bool is_reading, bool is_mapping)
        latMarks->marks->clearAllData();
        numMarks = 0;
        mediaEncFinal = 0;
        
        // Muda a LED frontal para branco com brilho 50%
        LED->config_LED(LEDposition, COLOR_WHITE, LED_EFFECT_SET, 0.5);
    }
    status->FirstMark->setData(false); // Indica que nao passou pela primeira marcaçao
    // Loop
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
        // Carregando status atual
        status->stateMutex.lock();
        TrackLen = (TrackState)status->TrackStatus->getData();
        pulsesBeforeCurve = latMarks->PulsesBeforeCurve->getData();
        pulsesAfterCurve = latMarks->PulsesAfterCurve->getData();
        actualCarState = (CarState) status->robotState->getData();

        if(started_in_Tuning && status->TunningMode->getData() && status->robotState->getData() != CAR_TUNING && status->robotState->getData() != CAR_STOPPED &&  !status->encreading->getData() && !status->robotIsMapping->getData()) 
        {// Se iniciar em modo de teste
            status->robotState->setData(CAR_TUNING);
            status->TrackStatus->setData(TUNNING);
            status->RealTrackStatus->setData(TUNNING);
            
            // LED frontal branca com brilho 50%
            LED->config_LED(LEDposition, COLOR_WHITE, LED_EFFECT_SET, 0.5);
        }
        if(latMarks->rightMarks->getData() >= 1 && !firstmark)
        {// Se passar pela linha de largada e nao tiver essa informacao salva
            firstmark = true;
            status->FirstMark->setData(true);
            initialmediaEnc = (speed->EncRight->getData() + speed->EncLeft->getData()) / 2;
        }
        if (lastMappingState != status->robotIsMapping->getData() && status->robotIsMapping->getData())
        {// Caso esteja mapeando
            lastMappingState = status->robotIsMapping->getData();

            //ESP_LOGI(GetName().c_str(), "Alterando velocidades para modo mapeamento.");
            // LED frontal branca com brilho 50%
            LED->config_LED(LEDposition, COLOR_YELLOW, LED_EFFECT_SET, 1);
        }
        else if ((lastState != status->robotState->getData() || lastTrack != (TrackState)status->TrackStatus->getData() || lastTransition != status->Transition->getData()) && !lastMappingState && status->robotState->getData() != CAR_STOPPED && status->robotState->getData() != CAR_TUNING)
        {// Caso tenha mudado o trecho em que o robô se encontra
            lastState = status->robotState->getData();
            lastTrack =  (TrackState)status->TrackStatus->getData();
            lastTransition = status->Transition->getData();
            if (lastState == CAR_IN_LINE && !lastTransition)
            {
                //ESP_LOGI(GetName().c_str(), "Alterando os leds para modo inLine.");
                switch (TrackLen)
                {
                    case SHORT_LINE:
                        LED->config_LED(LEDposition, COLOR_GREEN, LED_EFFECT_SET, 0.05);
                        break;
                    case MEDIUM_LINE:
                        LED->config_LED(LEDposition, COLOR_GREEN, LED_EFFECT_SET, 0.3);
                        break;
                    case LONG_LINE:
                        LED->config_LED(LEDposition, COLOR_GREEN, LED_EFFECT_SET, 1);
                        break;
                    case SPECIAL_TRACK:
                        LED->config_LED(LEDposition, COLOR_PURPLE, LED_EFFECT_SET, 0.05);
                        break;
                    default:
                        LED->config_LED(LEDposition, COLOR_WHITE, LED_EFFECT_SET, 1);
                        break;
                }
            }
            else if(lastState == CAR_IN_CURVE && !lastTransition)
            {
                //ESP_LOGI(GetName().c_str(), "Alterando os leds para modo inCurve.");
                switch (TrackLen)
                {
                    case SHORT_CURVE:
                        LED->config_LED(LEDposition, COLOR_RED, LED_EFFECT_SET, 0.05);
                        break;
                    case MEDIUM_CURVE:
                        LED->config_LED(LEDposition, COLOR_RED, LED_EFFECT_SET, 0.3);
                        break;
                    case LONG_CURVE:
                        LED->config_LED(LEDposition, COLOR_RED, LED_EFFECT_SET, 1);
                        break;
                    case ZIGZAG:
                        LED->config_LED(LEDposition, COLOR_PURPLE, LED_EFFECT_SET, 1);
                        break;
                    case SPECIAL_TRACK:
                        LED->config_LED(LEDposition, COLOR_PURPLE, LED_EFFECT_SET, 0.05);
                        break;
                    default:
                        LED->config_LED(LEDposition, COLOR_WHITE, LED_EFFECT_SET, 1);
                        break;
                }
            }
            else if(lastTransition)
            {
                LED->config_LED(LEDposition, COLOR_BLUE, LED_EFFECT_SET, 0.5);
            }
        }

        mediaEncActual = (speed->EncRight->getData() + speed->EncLeft->getData()) / 2; // calcula media dos encoders

        if(!status->robotIsMapping->getData() && !status->encreading->getData() && !status->TunningMode->getData() && actualCarState != CAR_STOPPED)
        {// Muda o status do carro para parado, caso esteja no momento certo
            status->robotState->setData(CAR_STOPPED);
            vTaskDelay(0);
            DataManager::getInstance()->saveAllParamDataChanged();
            LED->config_LED(LEDposition, COLOR_BLACK, LED_EFFECT_SET, 1);
        }
        if (!status->robotIsMapping->getData() && actualCarState != CAR_STOPPED && status->encreading->getData() /*&& firstmark*/ && (!status->TunningMode->getData() || !started_in_Tuning))
        {// Se estiver lendo o mapeamento, e já tiver passado pela linha de largada
            if ((mediaEncActual - initialmediaEnc) >= mediaEncFinal)
            {// 
                if(status->TuningMapped->getData())
                {
                    status->FirstMark->setData(false);
                    firstmark = false;
                    initialmediaEnc = 0;
                    status->robotState->setData(CAR_IN_LINE);
                    status->TrackStatus->setData(SHORT_LINE);
                    status->RealTrackStatus->setData(SHORT_LINE);
                    latMarks->rightMarks->setData(0);
                }
                else
                {
                    //ESP_LOGI(GetName().c_str(), "Parando o robô");
                    status->encreading->setData(false);
                    //vTaskDelay(100 / portTICK_PERIOD_MS);

                    // TODO: Encontrar forma bonita de suspender os outros serviços.
                    // vTaskSuspend(xTaskPID);
                    // vTaskSuspend(xTaskSensors);

                    status->robotState->setData(CAR_STOPPED);
                    DataManager::getInstance()->saveAllParamDataChanged();
                    LED->config_LED(LEDposition, COLOR_BLACK, LED_EFFECT_SET, 1);
                }
            }
            if ((mediaEncActual - initialmediaEnc) < mediaEncFinal)
            {
                // define o status do carrinho
                int mark = 0;
                for (mark = 0; mark < numMarks - 1; mark++)
                {
                    // Verifica a contagem do encoder e atribui o estado ao robô
                    int32_t Manualmedia = latMarks->marks->getData(mark).MapEncMedia;        // Média dos encoders na chave mark
                    int32_t ManualmediaNxt = latMarks->marks->getData(mark + 1).MapEncMedia; // Média dos encoders na chave mark + 1

                    if ((mediaEncActual - initialmediaEnc) >= Manualmedia && (mediaEncActual - initialmediaEnc) <= ManualmediaNxt) // análise do valor das médias dos encoders
                    {
                        loadTrackMapped(mark+1);
                        status->RealTrackStatus->setData(trackLen);
                        bool transition = false;

                        int16_t offset = latMarks->marks->getData(mark).MapOffset;
                        int16_t offsetnxt = latMarks->marks->getData(mark).MapOffset;
                        // Verifica se o robô precisa reduzir a velocidade, entrando no modo curva
                        if((CarState)latMarks->marks->getData(mark).MapStatus == CAR_IN_CURVE && (CarState)latMarks->marks->getData(mark + 1).MapStatus == CAR_IN_LINE && offset == 0)
                        {
                            offset = pulsesAfterCurve; 
                        }
                        if(offset > 0)
                        {
                            if((Manualmedia + offset) < ManualmediaNxt && (mediaEncActual - initialmediaEnc) < (Manualmedia + offset)) 
                            {
                                transition = true;
                                loadTrackMapped(mark);
                            }
                            else if((Manualmedia + offset) >= ManualmediaNxt) 
                            {
                                transition = true;
                                loadTrackMapped(mark);
                            }
                        }
                        if(mark + 2 < numMarks)
                        {
                            if((CarState)latMarks->marks->getData(mark+1).MapStatus == CAR_IN_LINE && (CarState)latMarks->marks->getData(mark + 2).MapStatus == CAR_IN_CURVE){
                                if(offsetnxt == 0) offsetnxt = -pulsesBeforeCurve;
                                else offsetnxt = -offsetnxt;
                            }
                            if(offsetnxt < 0)
                            {
                                if((ManualmediaNxt + offsetnxt) > Manualmedia && (mediaEncActual - initialmediaEnc) > (ManualmediaNxt + offsetnxt)) 
                                {
                                    transition = true;
                                    loadTrackMapped(mark+2);
                                }
                                else if((ManualmediaNxt + offsetnxt) <= Manualmedia) 
                                {
                                    transition = true;
                                    loadTrackMapped(mark+2);
                                }
                            }
                        }
                        // Atualiza estado do robô
                        status->Transition->setData(transition);
                        status->robotState->setData(trackType);
                        status->TrackStatus->setData(trackLen);
                        speed->vel_mapped->setData(trackSpeed);
                        break;
                    }
                }
            }
        }

        status->stateMutex.unlock();
    }
}

void StatusService::mappingStatus(bool is_reading, bool is_mapping)
{// Muda as variáveis de estado ligadas ao mapeamento.
    status->encreading->setData(is_reading);
    status->robotIsMapping->setData(is_mapping);
}

void StatusService::loadTrackMapped(int section)
{// Carrega as variaveis de um trecho da pista, salvas no mapeamento.
    trackType = (CarState)latMarks->marks->getData(section).MapStatus;
    trackLen = (TrackState)latMarks->marks->getData(section).MapTrackStatus;
    trackSpeed = latMarks->marks->getData(section).MapMaxSpeed;
}