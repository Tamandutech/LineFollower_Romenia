#ifndef DATA_MOTOR_HPP
#define DATA_MOTOR_HPP

#include <stdint.h>
#include <stddef.h>
#include <string>

#include "DataAbstract.hpp"
#include "dataEnums.h"

#include "esp_log.h"

class dataMotor
{
public:
    dataMotor(std::string name = "dataMotor");

    DataAbstract<float> *getSpeed(TrackSegment trackSegment, CarState state);

    // Contagem atual dos encoders
    DataAbstract<int32_t> *EncRight;
    DataAbstract<int32_t> *EncLeft;
    DataAbstract<int32_t> *EncMedia;
    
    // Velocidades atuais
    DataAbstract<int16_t> *RPMRight_inst;
    DataAbstract<int16_t> *RPMLeft_inst;
    DataAbstract<int16_t> *RPMCar_media;

    // PWM mandado ao motor
    DataAbstract<int16_t> *PWM_right;
    DataAbstract<int16_t> *PWM_left;

    DataAbstract<int16_t> *Brushless_TargetSpeed;

    // Restrições nos valores do PWM
    DataAbstract<int8_t> *max;
    DataAbstract<int8_t> *min;

    // Variável para guardar a velocidade do trecho
    DataAbstract<float> *vel_mapped;
    
    // Vel. base para calibracao:
    DataAbstract<float> *vel_calibrate;

private:
    std::string name;
    const char *tag = "RobotData";

    DataAbstract<float> *Track_Setpoint(TrackSegment state);

    //Setpoints para os tipos de trecho 
    DataAbstract<float> *Initial_Speed; // Velocidade inicial em rpm
    //privados pois a função Setpoint(TrackState state) que permite acesso a essas variáveis
    DataAbstract<float> *Long_Line;
    DataAbstract<float> *Medium_Line;
    DataAbstract<float> *Short_Line;
    DataAbstract<float> *XLong_Line;
    DataAbstract<float> *XLong_Curve; 
    DataAbstract<float> *Long_Curve;
    DataAbstract<float> *Medium_Curve;
    DataAbstract<float> *Short_Curve;
    DataAbstract<float> *Zig_zag;
    DataAbstract<float> *Special_Track;

    // Velocidade para o modo Tunning
    DataAbstract<float> *Tunning_speed;
    // Velocidade padrão do robô
    DataAbstract<float> *Default_speed;

    DataManager *dataManager;
};

#endif