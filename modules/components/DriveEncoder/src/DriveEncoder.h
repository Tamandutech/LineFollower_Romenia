#ifndef DRIVE_ENCODER_H
#define DRIVE_ENCODER_H

#include "RobotData.h"

#include "ESP32Encoder.h"
#include "math.h"

#include "esp_log.h"

class DriveEncoder
{
public:
    
    void ConfigEncoders();
    void ResetCount();
    void ReadBoth();

private:
    // Atalhos para facilitar a escrita do código
    Robot *robot;
    dataMotor *get_Vel;
    dataSpec *get_Spec;

    int32_t lastPulseRight = 0;
    int32_t lastPulseLeft = 0;

    ESP32Encoder enc_motEsq;
    ESP32Encoder enc_motDir;

    // Componente de gerenciamento dos encoders
    void SaveEncData(int32_t pulseRight, int32_t pulseLeft);
};

#endif