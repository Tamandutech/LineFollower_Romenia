#include "dataStatus.h"

std::mutex dataStatus::stateMutex;

dataStatus::dataStatus(std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);

    robotState = new DataAbstract<uint8_t>("robotState", name, CAR_IN_LINE);
    TrackStatus = new DataAbstract<uint8_t>("TrackStatus", name, SHORT_LINE);
    robotIsMapping = new DataAbstract<bool>("robotIsMapping", name, 0);
    encreading = new DataAbstract<bool>("encreading", name, 0);
    LineColorBlack = new DataAbstract<bool>("LineColorBlack", name, WHITE);
    WithBrushless = new DataAbstract<bool>("WithBrushless", name, true);
    TunningMode = new DataAbstract<bool>("TunningMode", name, false);
    HardDeleteMap = new DataAbstract<bool>("HardDeleteMap", name, false);
    RealTrackStatus = new DataAbstract<uint8_t>("RealTrackStatus", name, 0);
    FirstMark = new DataAbstract<bool>("FirstMark", name, false);
    Transition = new DataAbstract<bool>("Transition", name, false);
    TuningMapped = new DataAbstract<bool>("TuningMapped", name, false);
}