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
    dataStatus(std::string name = "dataVel");

    DataAbstract<uint8_t> *robotState; // Armazena o estado geral do robô.
    DataAbstract<uint8_t> *TrackStatus; // status (velocidade) do trecho da pista em que o robô se encontra

    DataAbstract<bool> *robotIsMapping; // Atributo que indica se o robô está mapeando a pista (TRUE se sim)

    DataAbstract<bool> *encreading; //Atributo que indica se o robô está lendo o mapeamento (TRUE se sim)

    DataAbstract<bool> *LineColorBlack; // Se a linha é branca ou preta (TRUE/BLACK para preta)]
    DataAbstract<bool> *WithBrushless;

    DataAbstract<bool> *TunningMode;
    DataAbstract<bool> *HardDeleteMap;
    DataAbstract<uint8_t> *RealTrackStatus; // status real do trecho da pista em que o robô se encontra
    DataAbstract<bool> *FirstMark; // Verifica se o robô já passou pela primeira marcação lateral
    DataAbstract<bool> *Transition; // Verifica se o robô está numa transição de curva para reta e vice-versa
    DataAbstract<bool> *TuningMapped; // Ativar tuningMode com o mapeamento da pista

    static std::mutex stateMutex;

private:
    std::string name;
    const char *tag = "RobotData";

};

#endif