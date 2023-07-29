#ifndef SENSOR_SERVICE_HPP
#define SENSOR_SERVICE_HPP

// Bibliotecas para criacao do servico
#include "thread.hpp"
#include "esp_log.h"
#include "Injector/singleton.hpp"
// Bibliotecas de componentes/dados
#include "QTRSensors.h"
#include "QTRwithMUX.h" // biblioteca própria
#include "RobotData.h"
// Bibliotecas do C++
#include "bitset"// para transformar um número decimal para binário
#include "cmath" // para criar funcoes trigonometricas
// Bibliotecas de servicos
#include "MotorService.hpp"
#include "RPMService.hpp"



using namespace cpp_freertos;

class SensorService : public Thread, public Singleton<SensorService>
{
public:
    SensorService(std::string name, uint32_t stackDepth, UBaseType_t priority);

    void Run() override;
    void AngleError();

    float AngleArray[sQuantReading]; // array que salva as N ultimas leituras do sensor

private:

    // Atalhos para facilitar a escrita do código
    Robot *robot;
    dataMotor *get_Vel;
    dataSpec *get_Spec;
    dataStatus *get_Status;
    dataSLatMarks *get_Marks;
    dataSensor *get_latArray;
    dataSensor *get_centerArray;
    // Atalhos de servico
    MotorService *control_motor;
    RPMService *rpm;
    
    // Objetos usados no serviço:
    QTRSensors sArray[sQuant]; // sensores frontais
    QTRSensors sLat; // sensores laterais
    QTRSensors sCenter; // sensores no centro do corpo
    QTRwithMUX MUX; // objeto para acessar as funcoes do QTRwithMUX


    // Variaveis do servico
    uint16_t MPR_Mot = 0; // MPR atual do robo
    


    // Variaveis
    int tempo_cal = 7000; // tempo de calibracao, passar para RobotData
    int sumSensEsq = 0;
    int sumSensDir = 0;
    int MarksToMean = 0;
    int nLatReads = 0;

    void auto_calibrate();
    void SaveAngle(float new_angle);
    void ReadArray(QTRSensors *array, dataSensor *get_array);
    void processSLat();
    void processSCenter();
    void latState(bool rightPass, bool leftPass);
};

#endif