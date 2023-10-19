#include "dataStatus.h"

std::mutex dataStatus::stateMutex;

dataStatus::dataStatus(CarState initialState, std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    //ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);
    dataManager = dataManager->getInstance();
    //variable = nex DataAbstract<variable_type>("variable_name", parentObjectName,value)

    // Obejetos do tipo uint8_t
    robotState = new DataAbstract<uint8_t>("robotState", name, initialState); // criado com o estado inicial
    TrackStatus = new DataAbstract<uint8_t>("TrackStatus", name, SHORT_LINE);
    RobotCenter = new DataAbstract<uint8_t>("RobotCenter", name, CAR_CENTERED);
    RealTrackStatus = new DataAbstract<uint8_t>("RealTrackStatus", name, 0);
    robotPaused = new DataAbstract<bool>("robotPaused", name, false);

    // Objetos do tipo bool
    robotIsMapping = new DataAbstract<bool>("robotIsMapping", name, 0);
    encreading = new DataAbstract<bool>("encreading", name, 0);
    LineColorBlack = new DataAbstract<bool>("LineColorBlack", name, WHITE);
    dataManager->registerParamData(LineColorBlack);
    WithBrushless = new DataAbstract<bool>("WithBrushless", name, true);
    TunningMode = new DataAbstract<bool>("TunningMode", name, false);
    dataManager->registerParamData(TunningMode);
    OpenLoopControl = new DataAbstract<bool>("OpenLoopControl", name, false);
    dataManager->registerParamData(OpenLoopControl);
    RPMControl = new DataAbstract<bool>("RPMControl", name, false);
    dataManager->registerParamData(RPMControl);
    HardDeleteMap = new DataAbstract<bool>("HardDeleteMap", name, false);
    FirstMark = new DataAbstract<bool>("FirstMark", name, false);
    Transition = new DataAbstract<bool>("Transition", name, false);
    TuningMapped = new DataAbstract<bool>("TuningMapped", name, false);
}