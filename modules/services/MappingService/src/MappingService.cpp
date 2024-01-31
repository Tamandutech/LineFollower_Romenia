#include "MappingService.hpp"

MappingService::MappingService(std::string name, uint32_t stackDepth, UBaseType_t priority, BaseType_t coreid) : Thread(name, stackDepth, priority,  coreid)
{
    this->robot = Robot::getInstance();
    get_Spec = robot->getSpecification();

    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_0, GPIO_PULLUP_ONLY);

    get_Speed = robot->getMotorData();
    get_latMarks = robot->getSLatMarks();
    get_Status = robot->getStatus();
    esp_log_level_set(GetName().c_str(), ESP_LOG_ERROR);
};

esp_err_t MappingService::startNewMapping(uint16_t leftMarksToStop, int32_t mediaPulsesToStop, uint32_t timeToStop)
{
    //ESP_LOGI(GetName().c_str(), "Iniciando novo mapeamento.");

    get_Status->robotIsMapping->setData(true);

    this->leftMarksToStop = leftMarksToStop;
    //this->rightMarksToStop = get_latMarks->MarkstoStop->getData();
    this->mediaPulsesToStop = mediaPulsesToStop;
    this->ticksToStop = timeToStop / portTICK_PERIOD_MS;

    tempPreviousMark.MapEncLeft = 0;
    tempPreviousMark.MapEncRight = 0;
    tempPreviousMark.MapEncMedia = 0;
    tempPreviousMark.MapStatus = CAR_IN_CURVE;
    tempPreviousMark.MapTrackStatus = MEDIUM_CURVE;
    tempPreviousMark.MapTime = 0;
    tempPreviousMark.MapOffset = 0;


    get_latMarks->rightMarks->setData(0);
    get_latMarks->leftMarks->setData(0);

    tempActualMark = tempPreviousMark;

    get_latMarks->marks->clearAllData();

    this->Start();

    return ESP_OK;
}

esp_err_t MappingService::stopNewMapping()
{
    //ESP_LOGI(GetName().c_str(), "Parando novo mapeamento.");

    get_Status->stateMutex.lock();
    get_Status->robotState->setData(CAR_STOPPED);
    get_Status->robotIsMapping->setData(false);
    DataManager::getInstance()->saveAllParamDataChanged();
    get_Status->stateMutex.unlock();

    this->Cleanup();

    this->saveMapping();
    //ESP_LOGI(GetName().c_str(), "Parada do novo mapeamento finalizada");
    
    // Desligando a LED:
    LED->LedComandSend(LED_POSITION_FRONT, COLOR_BLACK, 1);
    return ESP_OK;
}

esp_err_t MappingService::loadMapping()
{
    //ESP_LOGI(GetName().c_str(), "Carregando mapeamento da memória.");

    get_latMarks->marks->loadData();

    return ESP_OK;
}

esp_err_t MappingService::saveMapping()
{
    //ESP_LOGI(GetName().c_str(), "Salvando mapeamento na memória.");

    get_latMarks->marks->saveData();

    return ESP_OK;
}

esp_err_t MappingService::createNewMark()
{
    if (get_Status->robotIsMapping->getData() && get_Status->robotState->getData() != CAR_STOPPED)
    {
        //ESP_LOGI(GetName().c_str(), "Criando nova marcação.");

        this->Resume();
        return ESP_OK;
    }
    return ESP_OK;
}

void MappingService::Run()
{
    
    //this->Suspend(); // Suspend necessário para mapeamento com marcações laterais

    initialLeftPulses = get_Speed->EncLeft->getData();
    initialRightPulses = get_Speed->EncRight->getData();
    initialMediaPulses = (initialLeftPulses + initialRightPulses) / 2;
    initialTicks = xTaskGetTickCount();

    get_latMarks->marks->newData(tempActualMark);

    //ESP_LOGI(GetName().c_str(), "Offset iniciais: initialLeftPulses: %d, initialRightPulses: %d, initialMediaPulses: %d, initialTicks: %d", initialLeftPulses, initialRightPulses, initialMediaPulses, initialTicks);
    //TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        if ((leftMarksToStop <= get_latMarks->leftMarks->getData()) || (get_latMarks->MarkstoStop->getData() <= get_latMarks->rightMarks->getData()) || (mediaPulsesToStop <= tempActualMark.MapEncMedia) || (!get_Status->robotIsMapping->getData()))
        {// Finalizando o mapeamento
            //ESP_LOGI(GetName().c_str(), "Mapeamento finalizado.");

            this->stopNewMapping();
            break;
        }
        MappingWithMarks();
    }
}

void MappingService::MappingWithMarks()
{
        tempPreviousMark = tempActualMark;

        vTaskDelay(0);
        this->Suspend();
        
        tempActualMark.MapOffset = 0;
        tempActualMark.MapEncLeft = get_Speed->EncLeft->getData() - initialLeftPulses;
        tempActualMark.MapEncRight = get_Speed->EncRight->getData() - initialRightPulses;
        tempActualMark.MapEncMedia = ((tempActualMark.MapEncLeft + tempActualMark.MapEncRight) / 2);
        tempActualMark.MapTime = ((xTaskGetTickCount() - initialTicks) * portTICK_PERIOD_MS);

        // variação de encoder em pulsos
        tempDeltaPulses = std::abs((tempActualMark.MapEncRight - tempPreviousMark.MapEncRight) - (tempActualMark.MapEncLeft - tempPreviousMark.MapEncLeft));
        // Quantidade de pulsos que o encoder precisa dar para avançar "x" milimetros
        tempMilimiterInPulses = (get_Spec->MPR->getData() * get_latMarks->thresholdToCurve->getData()) / (M_PI * get_Spec->WheelDiameter->getData());

        tempActualMark.MapStatus = (tempDeltaPulses > tempMilimiterInPulses) ? CAR_IN_CURVE : CAR_IN_LINE;
        tempDeltaDist = ((tempActualMark.MapEncMedia - tempPreviousMark.MapEncMedia) * (M_PI * get_Spec->WheelDiameter->getData())) / (get_Spec->MPR->getData()); // distância entre marcacões em mm
        if(tempActualMark.MapStatus == CAR_IN_LINE)
        {
            if(tempDeltaDist < get_latMarks->thresholdMediumLine->getData()) tempActualMark.MapTrackStatus = SHORT_LINE;
            else if(tempDeltaDist < get_latMarks->thresholdLongLine->getData()) tempActualMark.MapTrackStatus = MEDIUM_LINE;
            else tempActualMark.MapTrackStatus = LONG_LINE;
        }
        else if(tempActualMark.MapStatus == CAR_IN_CURVE)
        {
            if(tempDeltaDist < get_latMarks->thresholdMediumCurve->getData()) tempActualMark.MapTrackStatus = SHORT_CURVE;
            else if(tempDeltaDist < get_latMarks->thresholdLongCurve->getData()) tempActualMark.MapTrackStatus = MEDIUM_CURVE;
            else tempActualMark.MapTrackStatus = LONG_CURVE;
        }
        if(tempPreviousMark.MapStatus == CAR_IN_LINE && tempActualMark.MapStatus == CAR_IN_LINE) get_latMarks->marks->clearData(get_latMarks->marks->getSize() - 1);
        get_latMarks->marks->newData(tempActualMark);

        AtualizarLEDs();

        //ESP_LOGI(GetName().c_str(), "Marcação: MapEncLeft: %d, MapEncRight: %d, MapEncMedia: %d, MapTime: %d, MapStatus: %d", tempActualMark.MapEncLeft, tempActualMark.MapEncRight, tempActualMark.MapEncMedia, tempActualMark.MapTime, tempActualMark.MapStatus);

}

void MappingService::MappingWithoutMarks(TickType_t *xLastWakeTime)
{
        tempPreviousMark = tempActualMark;

        
        tempActualMark.MapOffset = 0;
        tempActualMark.MapEncLeft = get_Speed->EncLeft->getData() - initialLeftPulses;
        tempActualMark.MapEncRight = get_Speed->EncRight->getData() - initialRightPulses;
        tempActualMark.MapEncMedia = ((tempActualMark.MapEncLeft + tempActualMark.MapEncRight) / 2);
        tempActualMark.MapTime = ((xTaskGetTickCount() - initialTicks) * portTICK_PERIOD_MS);

        int32_t delta_right = (tempActualMark.MapEncRight - tempPreviousMark.MapEncRight);
        int32_t delta_left = (tempActualMark.MapEncLeft - tempPreviousMark.MapEncLeft);
        
        // variação de encoder em pulsos
        tempDeltaPulses = std::abs(delta_right - delta_left);
        // Quantidade de pulsos que o encoder precisa dar para avançar "x" milimetros
        tempMilimiterInPulses = (get_Spec->MPR->getData() * get_latMarks->thresholdToCurve->getData()) / (M_PI * get_Spec->WheelDiameter->getData());


        tempActualMark.MapStatus = (tempDeltaPulses > tempMilimiterInPulses) ? CAR_IN_CURVE : CAR_IN_LINE;
        tempDeltaDist = ((tempActualMark.MapEncMedia - tempPreviousMark.MapEncMedia) * (M_PI * get_Spec->WheelDiameter->getData())) / (get_Spec->MPR->getData()); // distância entre marcacões em mm
        if(tempActualMark.MapStatus == CAR_IN_LINE)
        {
            if(tempDeltaDist < get_latMarks->thresholdLongLine->getData()) tempActualMark.MapTrackStatus = MEDIUM_LINE;
            else tempActualMark.MapTrackStatus = LONG_LINE;
        }
        else if(tempActualMark.MapStatus == CAR_IN_CURVE)
        {
            tempActualMark.MapTrackStatus = MEDIUM_CURVE;
        }
        if(tempPreviousMark.MapStatus == CAR_IN_LINE && tempActualMark.MapStatus == CAR_IN_LINE) get_latMarks->marks->clearData(get_latMarks->marks->getSize() - 1);
        get_latMarks->marks->newData(tempActualMark);

        AtualizarLEDs();

        //ESP_LOGI(GetName().c_str(), "Marcação: MapEncLeft: %d, MapEncRight: %d, MapEncMedia: %d, MapTime: %d, MapStatus: %d", tempActualMark.MapEncLeft, tempActualMark.MapEncRight, tempActualMark.MapEncMedia, tempActualMark.MapTime, tempActualMark.MapStatus);
        vTaskDelayUntil(xLastWakeTime, get_latMarks->deltaT->getData() / portTICK_PERIOD_MS);
}

void MappingService::AtualizarLEDs(){
    // Mudando a cor das LEDs:
    if(tempActualMark.MapStatus == CAR_IN_CURVE) 
    {
        if(get_latMarks->latEsqPass->getData()){ 
            LED->LedComandSend(LED_POSITION_LEFT, COLOR_RED, 1);
        }
        else if(get_latMarks->latDirPass->getData()){ 
            LED->LedComandSend(LED_POSITION_RIGHT, COLOR_RED, 1);
        }
    }
    else if(tempActualMark.MapStatus == CAR_IN_LINE)
    {
        if(get_latMarks->latEsqPass->getData()){ 
            LED->LedComandSend(LED_POSITION_LEFT, COLOR_GREEN, 1);
        }
        else if(get_latMarks->latDirPass->getData()){ 
            LED->LedComandSend(LED_POSITION_RIGHT, COLOR_GREEN, 1);
        }
    }
}