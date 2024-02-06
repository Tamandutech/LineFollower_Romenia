#include "dataStatus.h"

dataStatus::dataStatus(std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    //ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);
    dataManager = dataManager->getInstance();
    //variable = nex DataAbstract<variable_type>("variable_name", parentObjectName,value)

    // Obejetos do tipo uint8_t
    robotState = new DataAbstract<uint8_t>("robotState", name); // criado com o estado inicial
    ControlOff = new DataAbstract<bool>("Controloff", name, false);
    TrackStatus = new DataAbstract<uint8_t>("TrackStatus", name, SHORT_LINE);
    RobotCenter = new DataAbstract<uint8_t>("RobotCenter", name, CAR_CENTERED);
    RealTrackStatus = new DataAbstract<uint8_t>("RealTrackStatus", name, 0);
    robotPaused = new DataAbstract<bool>("robotPaused", name, false);

    // Objetos do tipo bool
    encreading = new DataAbstract<bool>("encreading", name, false);
    LineColorBlack = new DataAbstract<bool>("LineColorBlack", name, WHITE);
    dataManager->registerParamData(LineColorBlack);
    VelCalculated = new DataAbstract<bool>("Vel_Calculated", name, false);
    dataManager->registerParamData(VelCalculated);
    BrushlessON = new DataAbstract<bool>("Brushless_ON", name, false);
    dataManager->registerParamData(BrushlessON);
    LineInMaxSpeed = new DataAbstract<bool>("Use_Line_In_Max_Speed", name, false);
    dataManager->registerParamData(LineInMaxSpeed);
    WithBrushless = new DataAbstract<bool>("WithBrushless", name, true);
    TunningMode = new DataAbstract<bool>("TunningMode", name, false);
    dataManager->registerParamData(TunningMode);
    OpenLoopControl = new DataAbstract<bool>("OpenLoopControl", name, false);
    dataManager->registerParamData(OpenLoopControl);
    HardDeleteMap = new DataAbstract<bool>("HardDeleteMap", name, true);
    dataManager->registerParamData(HardDeleteMap);
    FirstMark = new DataAbstract<bool>("FirstMark", name, false);
    Transition = new DataAbstract<bool>("Transition", name, false);
    TuningMapped = new DataAbstract<bool>("TuningMapped", name, false);
    MappingTuningParam = new DataAbstract<bool>("Mapping_with_TuningParam", name, false);
    dataManager->registerParamData(MappingTuningParam);
}