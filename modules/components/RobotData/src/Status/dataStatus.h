#ifndef DATA_STATUS_HPP
#define DATA_STATUS_HPP

#include <stdint.h>
#include <stddef.h>
#include <string>

#include "DataAbstract.hpp"
#include "dataEnums.h"

#include "esp_log.h"

class dataStatus
{
public:
    dataStatus(std::string name = "dataStatus");

    DataAbstract<uint8_t> *robotState; // Armazena o estado geral do robô.
    DataAbstract<uint8_t> *TrackStatus; // status (velocidade) do trecho da pista em que o robô se encontra
    DataAbstract<int> *BatteryVoltage; // tensão da bateria

    DataAbstract<bool> *robotPaused;

    DataAbstract<bool> *encreading; //Atributo que indica se o robô está lendo o mapeamento (TRUE se sim)

    DataAbstract<bool> *LineColorBlack; // Se a linha é branca ou preta (TRUE/BLACK para preta)]
    DataAbstract<bool> *WithBrushless; // Se o brushless vai ser ligado ou não
    DataAbstract<bool> *VelCalculated; // Se vai ser utilizado a velocidade calculada (TRUE) ou a fixa de cada trecho (FALSE)
    DataAbstract<bool> *BrushlessON; // Se o brushless será ligado ou não
    DataAbstract<bool> *LineInMaxSpeed; // Velocidade máxima em qualquer reta
    DataAbstract<uint8_t> *RobotCenter; // Se o robô está centralizado e, se não, para que lado da linha o centro está

    DataAbstract<bool> *TunningMode; // Modo de teste
    DataAbstract<bool> *ControlOff; // Desliga o robô por controle remoto
    DataAbstract<bool> *OpenLoopControl; // Controle em malha aberta se o robô sair da linha
    DataAbstract<bool> *HardDeleteMap; // Se o mapa deve ser deletado
    DataAbstract<uint8_t> *RealTrackStatus; // status real do trecho da pista em que o robô se encontra
    DataAbstract<bool> *FirstMark; // Verifica se o robô já passou pela primeira marcação lateral
    DataAbstract<bool> *Transition; // Verifica se o robô está numa transição de curva para reta e vice-versa
    DataAbstract<bool> *TuningMapped; // Ativar tuningMode com o mapeamento da pista
    DataAbstract<bool> *MappingTuningParam; // Usa os parâmetros de Tunning mesmo quando está mapeado

private:
    std::string name;
    const char *tag = "RobotData";

    DataManager *dataManager;
};

#endif