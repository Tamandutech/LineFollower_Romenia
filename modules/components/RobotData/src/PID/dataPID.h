#ifndef DATA_PID_HPP
#define DATA_PID_HPP

#include <stdint.h>
#include <stddef.h>
#include <string>

#include "DataAbstract.hpp"

#include "dataEnums.h"

#include "esp_log.h"

struct PID_Consts
{
    float Kp;
    float Kd;
};

class dataPID
{
public:
    dataPID(std::string name = "dataPID");// Contrutor do objeto

    DataAbstract<float> *setpoint;
    DataAbstract<float> *erro;
    DataAbstract<float> *erroquad;
    DataAbstract<float> *output;

    // Funções que retornam as constantes do PID definidas para um trecho da pista
    DataAbstract<float> *getKP(TrackSegment track);
    DataAbstract<float> *getKD(TrackSegment track);
    PID_Consts PD_values(TrackSegment track, CarState state);

private:
    std::string name;
    const char *tag = "RobotData";

    // Parâmetros do PID
    
    // Linha
    DataAbstract<float> *Kp_Long_Line;
    DataAbstract<float> *Kd_Long_Line; 

    DataAbstract<float> *Kp_Medium_Line; 
    DataAbstract<float> *Kd_Medium_Line; 

    DataAbstract<float> *Kp_Short_Line; 
    DataAbstract<float> *Kd_Short_Line;

    // Curva
    DataAbstract<float> *Kp_Long_Curve; 
    DataAbstract<float> *Kd_Long_Curve; 

    DataAbstract<float> *Kp_Medium_Curve; 
    DataAbstract<float> *Kd_Medium_Curve; 

    DataAbstract<float> *Kp_Short_Curve; 
    DataAbstract<float> *Kd_Short_Curve; 

    // Outros tipos de trecho
    DataAbstract<float> *Kp_Zigzag; 
    DataAbstract<float> *Kd_Zigzag; 

    DataAbstract<float> *Kp_Default; 
    DataAbstract<float> *Kd_Default; 
    
    DataAbstract<float> *Kp_Tunning; 
    DataAbstract<float> *Kd_Tunning; 

    DataManager *dataManager;
};

#endif