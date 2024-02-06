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
#include "LEDsService.hpp"
#include "MotorService.hpp"



using namespace cpp_freertos;

class SensorService : public Thread, public Singleton<SensorService>
{
public:
    SensorService(std::string name, uint32_t stackDepth, UBaseType_t priority, BaseType_t coreid);

    void Run() override;
    void AngleError();

    void Sensor_resume();
    void manual_calibrate(int mux);
    //void auto_calibrate(int mux);
    void processSLat();

    float AngleArray[sQuantReading]; // array que salva as N ultimas leituras do sensor

private:

    // Atalhos para o RobotData
    Robot *robot;
    dataMotor *get_Speed;
    dataSpec *get_Spec;
    dataStatus *get_Status;
    dataSLatMarks *get_Marks;
    dataUint16 *get_latMarks;
    dataUint16 *get_centerArray;
    dataFloat *get_frontArray;
    // Atalhos de servico
    LEDsService *LED;
    //MotorService *motors;
    
    // Objetos usados no serviço:
    QTRSensors sArray[sQuant]; // sensores frontais
    QTRSensors sBody[6]; // sensores laterais e do centro
    QTRwithMUX MUX; // objeto para acessar as funcoes do QTRwithMUX


    // Variaveis do servico
    uint16_t MPR_Mot = 0; // MPR atual do robo
    


    // Variaveis
    int tempo_cal = 7000; // tempo de calibracao, passar para RobotData
    int sumSensEsq = 0;
    int sumSensDir = 0;
    int MarksToMean = 0;
    int nLatReads = 0;

    void SaveAngle(float new_angle);
    int lower_value(uint16_t s_1, uint16_t s_2);
    void processSCenter();
    void latState(bool rightPass, bool leftPass);
    void processSLat_romenia();
};

#endif