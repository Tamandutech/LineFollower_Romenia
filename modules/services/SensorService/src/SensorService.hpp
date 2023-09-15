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
#include "LEDsService.hpp"



using namespace cpp_freertos;

class SensorService : public Thread, public Singleton<SensorService>
{
public:
    SensorService(std::string name, uint32_t stackDepth, UBaseType_t priority);

    void Run() override;
    void AngleError();

    void Sensor_resume();
    void manual_calibrate(int mux);
    void auto_calibrate(int mux);
    float AngleArray[sQuantReading]; // array que salva as N ultimas leituras do sensor

private:

    // Atalhos para o RobotData
    Robot *robot;
    dataMotor *get_Vel;
    dataSpec *get_Spec;
    dataStatus *get_Status;
    dataSLatMarks *get_Marks;
    dataUint16 *get_latArray;
    dataUint16 *get_centerArray;
    dataFloat *get_frontArray;
    // Atalhos de servico
    MotorService *control_motor;
    RPMService *rpm;
    LEDsService *LED;
    
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
    uint16_t lastPosition; // para teste
    uint16_t cont_calibracao = 0;
    uint16_t cont_leituras = 0;

    void SaveAngle(float new_angle);
    void SaveArray(uint16_t *array, int array_len, dataUint16 *get_array);
    int16_t readAll(QTRSensors *sArray, int quant, bool white_line); // para teste
    void processSLat();
    void processSCenter();
    void latState(bool rightPass, bool leftPass);

    led_position_t LEDposition[NUM_LEDS] = {LED_POSITION_NONE};
};

#endif