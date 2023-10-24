#include "MappingService.hpp"

MappingService::MappingService(std::string name, uint32_t stackDepth, UBaseType_t priority) : Thread(name, stackDepth, priority)
{
    this->robot = Robot::getInstance();
    get_Spec = robot->getSpecification();

#ifndef ESP32_QEMU
    gpio_pad_select_gpio(0);
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_0, GPIO_PULLUP_ONLY);
#endif

    speedMapping = robot->getMotorData();
    latMarks = robot->getSLatMarks();
    status = robot->getStatus();
    esp_log_level_set(GetName().c_str(), ESP_LOG_ERROR);
};

esp_err_t MappingService::startNewMapping(uint16_t leftMarksToStop, int32_t mediaPulsesToStop, uint32_t timeToStop)
{
    //ESP_LOGI(GetName().c_str(), "Iniciando novo mapeamento.");

    status->robotIsMapping->setData(true);

    this->leftMarksToStop = leftMarksToStop;
    //this->rightMarksToStop = latMarks->MarkstoStop->getData();
    this->mediaPulsesToStop = mediaPulsesToStop;
    this->ticksToStop = timeToStop / portTICK_PERIOD_MS;

    tempPreviousMark.MapEncLeft = 0;
    tempPreviousMark.MapEncRight = 0;
    tempPreviousMark.MapEncMedia = 0;
    tempPreviousMark.MapStatus = CAR_IN_CURVE;
    tempPreviousMark.MapTrackStatus = MEDIUM_CURVE;
    tempPreviousMark.MapTime = 0;
    tempPreviousMark.MapOffset = 0;


    latMarks->rightMarks->setData(0);
    latMarks->leftMarks->setData(0);

    tempActualMark = tempPreviousMark;

    latMarks->marks->clearAllData();

    this->Start();

    return ESP_OK;
}

esp_err_t MappingService::stopNewMapping()
{
    //ESP_LOGI(GetName().c_str(), "Parando novo mapeamento.");

    status->stateMutex.lock();
    status->robotState->setData(CAR_STOPPED);
    status->robotIsMapping->setData(false);
    DataManager::getInstance()->saveAllParamDataChanged();
    status->stateMutex.unlock();

    this->Cleanup();

    this->saveMapping();
    //ESP_LOGI(GetName().c_str(), "Parada do novo mapeamento finalizada");
    
    // Desligando aa LED:
    LEDposition[0] = LED_POSITION_FRONT;
    LEDposition[1] = LED_POSITION_NONE;
    LED->config_LED(LEDposition, COLOR_BLACK, LED_EFFECT_SET, 1);
    return ESP_OK;
}

esp_err_t MappingService::loadMapping()
{
    //ESP_LOGI(GetName().c_str(), "Carregando mapeamento da memória.");

    latMarks->marks->loadData();

    return ESP_OK;
}

esp_err_t MappingService::saveMapping()
{
    //ESP_LOGI(GetName().c_str(), "Salvando mapeamento na memória.");

    latMarks->marks->saveData();

    return ESP_OK;
}

esp_err_t MappingService::createNewMark()
{
    if (status->robotIsMapping->getData() && status->robotState->getData() != CAR_STOPPED)
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

    initialLeftPulses = speedMapping->EncLeft->getData();
    initialRightPulses = speedMapping->EncRight->getData();
    initialMediaPulses = (initialLeftPulses + initialRightPulses) / 2;
    initialTicks = xTaskGetTickCount();

    latMarks->marks->newData(tempActualMark);

    //ESP_LOGI(GetName().c_str(), "Offset iniciais: initialLeftPulses: %d, initialRightPulses: %d, initialMediaPulses: %d, initialTicks: %d", initialLeftPulses, initialRightPulses, initialMediaPulses, initialTicks);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        if ((leftMarksToStop <= latMarks->leftMarks->getData()) || (latMarks->MarkstoStop->getData() <= latMarks->rightMarks->getData()) || (mediaPulsesToStop <= tempActualMark.MapEncMedia) || (!status->robotIsMapping->getData()))
        {// Finalizando o mapeamento
            //ESP_LOGI(GetName().c_str(), "Mapeamento finalizado.");

            this->stopNewMapping();
            break;
        }
        MappingWithoutMarks(&xLastWakeTime);
    }
}

void MappingService::MappingWithMarks()
{
        tempPreviousMark = tempActualMark;

        vTaskDelay(0);
        this->Suspend();
        
        tempActualMark.MapOffset = 0;
        tempActualMark.MapEncLeft = speedMapping->EncLeft->getData() - initialLeftPulses;
        tempActualMark.MapEncRight = speedMapping->EncRight->getData() - initialRightPulses;
        tempActualMark.MapEncMedia = ((tempActualMark.MapEncLeft + tempActualMark.MapEncRight) / 2);
        tempActualMark.MapTime = ((xTaskGetTickCount() - initialTicks) * portTICK_PERIOD_MS);

        // variação de encoder em pulsos
        tempDeltaPulses = std::abs((tempActualMark.MapEncRight - tempPreviousMark.MapEncRight) - (tempActualMark.MapEncLeft - tempPreviousMark.MapEncLeft));
        // Quantidade de pulsos que o encoder precisa dar para avançar "x" milimetros
        tempMilimiterInPulses = (get_Spec->MPR->getData() * latMarks->thresholdToCurve->getData()) / (M_PI * get_Spec->WheelDiameter->getData());

        tempActualMark.MapStatus = (tempDeltaPulses > tempMilimiterInPulses) ? CAR_IN_CURVE : CAR_IN_LINE;
        tempDeltaDist = ((tempActualMark.MapEncMedia - tempPreviousMark.MapEncMedia) * (M_PI * get_Spec->WheelDiameter->getData())) / (get_Spec->MPR->getData()); // distância entre marcacões em mm
        if(tempActualMark.MapStatus == CAR_IN_LINE)
        {
            if(tempDeltaDist < latMarks->thresholdMediumLine->getData()) tempActualMark.MapTrackStatus = SHORT_LINE;
            else if(tempDeltaDist < latMarks->thresholdLongLine->getData()) tempActualMark.MapTrackStatus = MEDIUM_LINE;
            else tempActualMark.MapTrackStatus = LONG_LINE;
        }
        else if(tempActualMark.MapStatus == CAR_IN_CURVE)
        {
            if(tempDeltaDist < latMarks->thresholdMediumCurve->getData()) tempActualMark.MapTrackStatus = SHORT_CURVE;
            else if(tempDeltaDist < latMarks->thresholdLongCurve->getData()) tempActualMark.MapTrackStatus = MEDIUM_CURVE;
            else tempActualMark.MapTrackStatus = LONG_CURVE;
        }
        latMarks->marks->newData(tempActualMark);

        // Mudando a cor das LEDs:
        LEDposition[0] = LED_POSITION_NONE;
        LEDposition[1] = LED_POSITION_NONE;
        if(tempActualMark.MapStatus == CAR_IN_CURVE) 
        {
            if(latMarks->latEsqPass->getData()) LEDposition[0] = LED_POSITION_LEFT;
            else if(latMarks->latDirPass->getData()) LEDposition[0] = LED_POSITION_RIGHT;
            LED->config_LED(LEDposition, COLOR_RED, LED_EFFECT_SET, 1);
        }
        else if(tempActualMark.MapStatus == CAR_IN_LINE)
        {
            if(latMarks->latEsqPass->getData()) LEDposition[0] = LED_POSITION_LEFT;
            else if(latMarks->latDirPass->getData()) LEDposition[0] = LED_POSITION_RIGHT;
            LED->config_LED(LEDposition, COLOR_GREEN, LED_EFFECT_SET, 1);
        }
        //ESP_LOGI(GetName().c_str(), "Marcação: MapEncLeft: %d, MapEncRight: %d, MapEncMedia: %d, MapTime: %d, MapStatus: %d", tempActualMark.MapEncLeft, tempActualMark.MapEncRight, tempActualMark.MapEncMedia, tempActualMark.MapTime, tempActualMark.MapStatus);

}

void MappingService::MappingWithoutMarks(TickType_t *xLastWakeTime)
{
        tempPreviousMark = tempActualMark;

        
        tempActualMark.MapOffset = 0;
        tempActualMark.MapEncLeft = speedMapping->EncLeft->getData() - initialLeftPulses;
        tempActualMark.MapEncRight = speedMapping->EncRight->getData() - initialRightPulses;
        tempActualMark.MapEncMedia = ((tempActualMark.MapEncLeft + tempActualMark.MapEncRight) / 2);
        tempActualMark.MapTime = ((xTaskGetTickCount() - initialTicks) * portTICK_PERIOD_MS);

        int32_t delta_right = (tempActualMark.MapEncRight - tempPreviousMark.MapEncRight);
        int32_t delta_left = (tempActualMark.MapEncLeft - tempPreviousMark.MapEncLeft);

        /* if(delta_right != delta_left) tempActualMark.MapRadius = std::abs(((float)get_Spec->RobotDiameter->getData()/2)*((float)(delta_right+delta_left)/(float)(delta_right-delta_left)));
        else tempActualMark.MapRadius = 0;

        float speed = std::sqrt(tempActualMark.MapRadius * 9806.65 * get_Spec->Friction_Coef->getData()); // em mm/s
        speed = (speed*60)/((float)get_Spec->WheelDiameter->getData()*M_PI); // conversão RPM
        if(speed <= get_Spec->MaxRPM->getData()) tempActualMark.MapMaxSpeed = (speed/get_Spec->MaxRPM->getData())*100;
        else tempActualMark.MapMaxSpeed = 100;

        float offset_mm = (std::pow(tempActualMark.MapMaxSpeed, 2) - std::pow(tempPreviousMark.MapMaxSpeed, 2))/(2*get_Spec->Acceleration->getData());
        offset_mm = std::abs(offset_mm);
        float offset_enc = (offset_mm * (float)get_Spec->MPR->getData())/((float)get_Spec->WheelDiameter->getData() * M_PI);
        tempActualMark.MapOffset = (int16_t)offset_enc; */
        
        // variação de encoder em pulsos
        tempDeltaPulses = std::abs(delta_right - delta_left);
        // Quantidade de pulsos que o encoder precisa dar para avançar "x" milimetros
        tempMilimiterInPulses = (get_Spec->MPR->getData() * latMarks->thresholdToCurve->getData()) / (M_PI * get_Spec->WheelDiameter->getData());


        tempActualMark.MapStatus = (tempDeltaPulses > tempMilimiterInPulses) ? CAR_IN_CURVE : CAR_IN_LINE;
        tempDeltaDist = ((tempActualMark.MapEncMedia - tempPreviousMark.MapEncMedia) * (M_PI * get_Spec->WheelDiameter->getData())) / (get_Spec->MPR->getData()); // distância entre marcacões em mm
        if(tempActualMark.MapStatus == CAR_IN_LINE)
        {
            if(tempDeltaDist < latMarks->thresholdLongLine->getData()) tempActualMark.MapTrackStatus = MEDIUM_LINE;
            else tempActualMark.MapTrackStatus = LONG_LINE;
        }
        else if(tempActualMark.MapStatus == CAR_IN_CURVE)
        {
            tempActualMark.MapTrackStatus = MEDIUM_CURVE;
        }
        if(tempPreviousMark.MapStatus == CAR_IN_LINE && tempActualMark.MapStatus == CAR_IN_LINE) latMarks->marks->clearData(latMarks->marks->getSize() - 1);
        latMarks->marks->newData(tempActualMark);

        // Mudando a cor das LEDs:
        LEDposition[0] = LED_POSITION_NONE;
        LEDposition[1] = LED_POSITION_NONE;
        if(tempActualMark.MapStatus == CAR_IN_CURVE) 
        {
            if(latMarks->latEsqPass->getData()) LEDposition[0] = LED_POSITION_LEFT;
            else if(latMarks->latDirPass->getData()) LEDposition[0] = LED_POSITION_RIGHT;
            LED->config_LED(LEDposition, COLOR_RED, LED_EFFECT_SET, 1);
        }
        else if(tempActualMark.MapStatus == CAR_IN_LINE)
        {
            if(latMarks->latEsqPass->getData()) LEDposition[0] = LED_POSITION_LEFT;
            else if(latMarks->latDirPass->getData()) LEDposition[0] = LED_POSITION_RIGHT;
            LED->config_LED(LEDposition, COLOR_GREEN, LED_EFFECT_SET, 1);
        }
        //ESP_LOGI(GetName().c_str(), "Marcação: MapEncLeft: %d, MapEncRight: %d, MapEncMedia: %d, MapTime: %d, MapStatus: %d", tempActualMark.MapEncLeft, tempActualMark.MapEncRight, tempActualMark.MapEncMedia, tempActualMark.MapTime, tempActualMark.MapStatus);
        vTaskDelayUntil(xLastWakeTime, latMarks->deltaT->getData() / portTICK_PERIOD_MS);
}