#ifndef DATA_PID_HPP
#define DATA_PID_HPP

#include <stdint.h>
#include <stddef.h>
#include <string>

#include "DataAbstract.hpp"

#include "dataEnums.h"

#include "esp_log.h"

class dataPID
{
public:
    dataPID(std::string name = "dataPID");// Contrutor do objeto

    DataAbstract<float> *setpoint;
    DataAbstract<float> *erro;
    DataAbstract<float> *erroquad;
    DataAbstract<float> *output;

    // PID igual para os dois Motores
    DataAbstract<float> *Kp_Linear;
    DataAbstract<float> *Kd_Linear;
    DataAbstract<float> *Ki_Linear;

    // PID diferente para os dois motores
    DataAbstract<float> *Kp_L_Linear;
    DataAbstract<float> *Kd_L_Linear;
    DataAbstract<float> *Ki_L_Linear;
    
    DataAbstract<float> *Kp_R_Linear;
    DataAbstract<float> *Kd_R_Linear;
    DataAbstract<float> *Ki_R_Linear;

    // Funções que retornam as constantes do PID definidas para um trecho da pista
    DataAbstract<float> *Kp(TrackState state);
    DataAbstract<float> *Kd(TrackState state);

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