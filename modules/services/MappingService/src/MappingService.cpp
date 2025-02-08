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
    esp_log_level_set(GetName().c_str(), ESP_LOG_INFO);
};

esp_err_t MappingService::startNewMapping(uint16_t leftMarksToStop, int32_t mediaPulsesToStop, uint32_t timeToStop)
{
    //ESP_LOGI(GetName().c_str(), "Iniciando novo mapeamento.");

    this->leftMarksToStop = leftMarksToStop;
    //this->rightMarksToStop = get_latMarks->MarkstoStop->getData();
    this->mediaPulsesToStop = mediaPulsesToStop;
    this->ticksToStop = timeToStop / portTICK_PERIOD_MS;

    EncLeft = 0;
    EncRight = 0;
    lastEncLeft = 0;
    lastEncRight = 0;
    lastEncMedia = 0;
    tempActualMark.MapEncMedia = 0;
    tempActualMark.MapTrackStatus = MEDIUM_LINE;
    tempActualMark.MapTime = 0;
    tempActualMark.MapOffset = 0;
    tempActualMark.MapAccelerationSpace = 0;
    tempActualMark.MapDecelerationSpace = 0;

    get_latMarks->rightMarks->setData(0);
    get_latMarks->leftMarks->setData(0);

    get_latMarks->marks->clearAllData();

    this->Start();

    return ESP_OK;
}

esp_err_t MappingService::stopNewMapping()
{
    //ESP_LOGI(GetName().c_str(), "Parando novo mapeamento.");

    get_Status->robotState->setData(CAR_STOPPED);
    DataManager::getInstance()->saveAllParamDataChanged();
    this->Cleanup();
    this->saveMapping();
    //ESP_LOGD(GetName().c_str(), "Parada do novo mapeamento finalizada");
    LED->LedComandSend(LED_POSITION_FRONT, COLOR_BLACK, 1);
    return ESP_OK;
}

esp_err_t MappingService::loadMapping()
{
    //ESP_LOGI(GetName().c_str(), "Carregando mapeamento da memÃ³ria.");

    get_latMarks->marks->loadData();

    return ESP_OK;
}

esp_err_t MappingService::saveMapping()
{
    //ESP_LOGI(GetName().c_str(), "Salvando mapeamento na memÃ³ria.");

    get_latMarks->marks->saveData();

    return ESP_OK;
}

esp_err_t MappingService::createNewMark()
{
    if (get_Status->robotState->getData() == CAR_MAPPING)
    {
        //ESP_LOGI(GetName().c_str(), "Criando nova marcaÃ§Ã£o.");

        this->Resume();
        return ESP_OK;
    }
    return ESP_OK;
}

void MappingService::Run()
{

    this->Suspend(); // Suspend necessÃ¡rio para mapeamento com marcaÃ§Ãµes laterais

    initialLeftPulses = get_Speed->EncLeft->getData();
    initialRightPulses = get_Speed->EncRight->getData();
    initialMediaPulses = (initialLeftPulses + initialRightPulses) / 2;
    initialTicks = xTaskGetTickCount();

    get_latMarks->marks->newData(tempActualMark);

    //ESP_LOGI(GetName().c_str(), "Offset iniciais: initialLeftPulses: %d, initialRightPulses: %d, initialMediaPulses: %d, initialTicks: %d", initialLeftPulses, initialRightPulses, initialMediaPulses, initialTicks);

    for (;;)
    {
        if (finishedMapping())
        {
            //ESP_LOGI(GetName().c_str(), "Mapeamento finalizado.");
            this->stopNewMapping();
            this->computeAccelerationParameters();
            break;
        }
        MappingWithMarks();
    }
}

bool MappingService::finishedMapping(){
    return (
           (leftMarksToStop <= get_latMarks->leftMarks->getData())
        || (get_latMarks->MarkstoStop->getData() <= get_latMarks->rightMarks->getData())
        || (mediaPulsesToStop <= tempActualMark.MapEncMedia)
        || (ticksToStop <= (tempActualMark.MapTime * portTICK_PERIOD_MS))
        || (get_Status->ControlOff->getData() == true)
        );
}

void MappingService::MappingWithMarks()
{
        lastEncLeft = EncLeft;
        lastEncRight = EncRight;
        lastEncMedia = tempActualMark.MapEncMedia;
        lastTrack = tempActualMark.MapTrackStatus;

        vTaskDelay(0);
        this->Suspend();

        tempActualMark.MapOffset = 0;
        EncLeft = get_Speed->EncLeft->getData() - initialLeftPulses;
        EncRight = get_Speed->EncRight->getData() - initialRightPulses;
        tempActualMark.MapEncMedia = ((EncLeft + EncRight) / 2);
        tempActualMark.MapTime = ((xTaskGetTickCount() - initialTicks) * portTICK_PERIOD_MS);

        // variaÃ§Ã£o de encoder em pulsos
        tempDeltaPulses = std::abs((EncRight - lastEncRight) - (EncLeft - lastEncLeft));
        // Quantidade de pulsos que o encoder precisa dar para avanÃ§ar "x" milimetros
        tempMilimiterInPulses = (get_Spec->MPR->getData() * get_latMarks->thresholdToCurve->getData()) / (M_PI * get_Spec->WheelDiameter->getData());

        tempDeltaDist = ((tempActualMark.MapEncMedia - lastEncMedia) * (M_PI * get_Spec->WheelDiameter->getData())) / (get_Spec->MPR->getData()); // distÃ¢ncia entre marcacÃµes em mm

        if(tempDeltaPulses <= tempMilimiterInPulses)
        {
            if(tempDeltaDist < get_latMarks->thresholdMediumLine->getData()) tempActualMark.MapTrackStatus = SHORT_LINE;
            else if(tempDeltaDist < get_latMarks->thresholdLongLine->getData()) tempActualMark.MapTrackStatus = MEDIUM_LINE;
            else tempActualMark.MapTrackStatus = LONG_LINE;
        }
        else
        {
            if(tempDeltaDist < get_latMarks->thresholdMediumCurve->getData()) tempActualMark.MapTrackStatus = SHORT_CURVE;
            else if(tempDeltaDist < get_latMarks->thresholdLongCurve->getData()) tempActualMark.MapTrackStatus = MEDIUM_CURVE;
            else tempActualMark.MapTrackStatus = LONG_CURVE;
        }

        get_latMarks->marks->newData(tempActualMark);
        //ESP_LOGI(GetName().c_str(), "Salvou uma marcaÃ§Ã£o");

        AtualizarLEDs();

        //ESP_LOGI(GetName().c_str(), "MarcaÃ§Ã£o: MapEncLeft: %d, MapEncRight: %d, MapEncMedia: %d, MapTime: %d, MapStatus: %d", tempActualMark.MapEncLeft, tempActualMark.MapEncRight, tempActualMark.MapEncMedia, tempActualMark.MapTime, tempActualMark.MapStatus);

}

void MappingService::MappingWithoutMarks(TickType_t *xLastWakeTime)
{
        lastEncLeft = EncLeft;
        lastEncRight = EncRight;
        lastEncMedia = tempActualMark.MapEncMedia;
        lastTrack = tempActualMark.MapTrackStatus;

        tempActualMark.MapOffset = 0;
        EncLeft = get_Speed->EncLeft->getData() - initialLeftPulses;
        EncRight = get_Speed->EncRight->getData() - initialRightPulses;
        tempActualMark.MapEncMedia = ((EncLeft + EncRight) / 2);
        tempActualMark.MapTime = ((xTaskGetTickCount() - initialTicks) * portTICK_PERIOD_MS);

        int32_t delta_right = (EncRight - lastEncRight);
        int32_t delta_left = (EncLeft - lastEncLeft);

        // variaÃ§Ã£o de encoder em pulsos
        tempDeltaPulses = std::abs(delta_right - delta_left);
        // Quantidade de pulsos que o encoder precisa dar para avanÃ§ar "x" milimetros
        tempMilimiterInPulses = (get_Spec->MPR->getData() * get_latMarks->thresholdToCurve->getData()) / (M_PI * get_Spec->WheelDiameter->getData());

        tempDeltaDist = ((tempActualMark.MapEncMedia - lastEncMedia) * (M_PI * get_Spec->WheelDiameter->getData())) / (get_Spec->MPR->getData()); // distÃ¢ncia entre marcacÃµes em mm
        if(tempDeltaPulses <= tempMilimiterInPulses)
        {
            if(tempDeltaDist < get_latMarks->thresholdLongLine->getData()) tempActualMark.MapTrackStatus = MEDIUM_LINE;
            else tempActualMark.MapTrackStatus = LONG_LINE;
        }
        else
        {
            tempActualMark.MapTrackStatus = MEDIUM_CURVE;
        }

        if(track_is_a_line(lastTrack) && track_is_a_line(tempActualMark.MapTrackStatus)){
            get_latMarks->marks->clearData(get_latMarks->marks->getSize() - 1);
            tempActualMark.MapTrackStatus = LONG_LINE;
        }

        get_latMarks->marks->newData(tempActualMark);

        AtualizarLEDs();

        //ESP_LOGI(GetName().c_str(), "MarcaÃ§Ã£o: MapEncLeft: %d, MapEncRight: %d, MapEncMedia: %d, MapTime: %d, MapStatus: %d", tempActualMark.MapEncLeft, tempActualMark.MapEncRight, tempActualMark.MapEncMedia, tempActualMark.MapTime, tempActualMark.MapStatus);
        vTaskDelayUntil(xLastWakeTime, get_latMarks->deltaT->getData() / portTICK_PERIOD_MS);
}

void MappingService::AtualizarLEDs(){
    // Mudando a cor das LEDs:
    if(track_is_a_curve(tempActualMark.MapTrackStatus))
    {
        if(get_latMarks->latEsqPass->getData()){
            LED->LedComandSend(LED_POSITION_LEFT, COLOR_RED, 1);
        }
        else if(get_latMarks->latDirPass->getData()){
            LED->LedComandSend(LED_POSITION_RIGHT, COLOR_RED, 1);
        }
    }
    else if(track_is_a_line(tempActualMark.MapTrackStatus))
    {
        if(get_latMarks->latEsqPass->getData()){
            LED->LedComandSend(LED_POSITION_LEFT, COLOR_GREEN, 1);
        }
        else if(get_latMarks->latDirPass->getData()){
            LED->LedComandSend(LED_POSITION_RIGHT, COLOR_GREEN, 1);
        }
    }
}

void MappingService::computeAccelerationParameters(){
    loadMapping();

    float accelerationSpaceMeter;
    float desaccelerationSpaceMeter;
    for (uint8_t i = 0; i < get_latMarks->marks->getSize(); i++)
    {
        MapData currentMapData = get_latMarks->marks->getData(i);
        if (!track_is_a_line(currentMapData.MapTrackStatus))
        {
            continue;
        }

        if (i < get_latMarks->marks->getSize() - 1)
        {
            if (i > 0)
            {
              accelerationSpaceMeter =
                  (pow(get_Spec->MaxSpeed->getData(), 2) -
                   pow(getSpeedForTrackStatusInMs(
                           get_latMarks->marks->getData(i - 1).MapTrackStatus),
                       2)) /
                  (2 * get_Spec->Acceleration->getData());
              desaccelerationSpaceMeter =
                  (pow(getSpeedForTrackStatusInMs(
                           get_latMarks->marks->getData(i + 1).MapTrackStatus),
                       2) -
                   pow(getSpeedForTrackStatusInMs(
                           get_latMarks->marks->getData(i + 1).MapTrackStatus),
                       2)) /
                  (2 * get_Spec->Acceleration->getData());
            }
            else
            {
                accelerationSpaceMeter = pow(get_Spec->MaxSpeed->getData(), 2) /
                    (2 * get_Spec->Acceleration->getData());
                desaccelerationSpaceMeter = -(
                    (
                        pow(getSpeedForTrackStatusInMs(get_latMarks->marks->getData(i-1).MapTrackStatus), 2) -
                        pow(get_Spec->MaxSpeed->getData(), 2)
                    ) / (2 * get_Spec->Acceleration->getData())
                );
            }
        }
        else
        {
            accelerationSpaceMeter = (
                pow(get_Spec->MaxSpeed->getData(), 2) -
                pow(getSpeedForTrackStatusInMs(get_latMarks->marks->getData(i-1).MapTrackStatus), 2)
            ) / (2 * get_Spec->Acceleration->getData());
            desaccelerationSpaceMeter = -(
                (pow(2, 2) - pow(get_Spec->MaxSpeed->getData(), 2)) / (2 * get_Spec->Acceleration->getData())
            );
        }
        currentMapData.MapAccelerationSpace = accelerationSpaceMeter/get_Spec->MetersPerPulse->getData();
        currentMapData.MapDecelerationSpace = desaccelerationSpaceMeter/get_Spec->MetersPerPulse->getData();
        get_latMarks->marks->setData(i, currentMapData);
    }
}

float MappingService::getSpeedForTrackStatusInMs(uint8_t trackStatus){
    // TODO: Implementar
    return 0.0;
}

bool MappingService::track_is_a_line(uint8_t track){
    if((track == XLONG_LINE) || (track == LONG_LINE) || (track == MEDIUM_LINE)|| (track == SHORT_LINE)){
        return true;
    }else{
        return false;
    }
}

bool MappingService::track_is_a_curve(uint8_t track){
    if((track == XLONG_CURVE) || (track == LONG_CURVE) || (track == MEDIUM_CURVE)|| (track == SHORT_CURVE)){
        return true;
    }else{
        return false;
    }
}