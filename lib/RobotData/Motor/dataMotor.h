#ifndef DATA_SENSOR_HPP
#define DATA_SENSOR_HPP

#include <stdint.h>
#include <stddef.h>
#include <string>

#include "DataAbstract.hpp"

#include "esp_log.h"

class dataMotor
{
public:
    dataMotor(std::string name = "dataVel");

    // Contagem atual dos encoders
    DataAbstract<int32_t> *EncRight;
    DataAbstract<int32_t> *EncLeft;
    DataAbstract<int32_t> *EncMedia;
    
    // Velocidades atuais
    DataAbstract<int16_t> *RPMRight_inst;
    DataAbstract<int16_t> *RPMLeft_inst;
    DataAbstract<int16_t> *RPMCar_media;
    
    // // Vel. base para calibracao:
    DataAbstract<float> *vel_calibrate;

    //Setpoints para os tipos de trecho
    DataAbstract<float> *Long_Line;
    DataAbstract<float> *Medium_Line;
    DataAbstract<float> *Short_Line;
    DataAbstract<float> *Long_Curve;
    DataAbstract<float> *Medium_Curve;
    DataAbstract<float> *Short_Curve;
    DataAbstract<float> *ZIGZAG;
    DataAbstract<float> *Special_Track;
    DataAbstract<float> *Default_Track;
    DataAbstract<float> *Tunning;

private:
    std::string name;
    const char *tag = "RobotData";

};

#endif