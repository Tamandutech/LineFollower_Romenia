#ifndef DRIVE_MOTORS_HPP
#define DRIVE_MOTORS_HPP

#include "esp_log.h"
// Bibliotecas de componentes/dados
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "ESP32MotorControl.h"
#include "RobotData.h"
#include "LEDsService.hpp"

using namespace cpp_freertos;

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define MIN_THROTTLE            205
#define MAX_THROTTLE            409
#define THROTTLE_SPEED          100

class DriveMotors
{
public:
    
    void ConfigMotors();
    void ControlMotors(float left, float right);
    void ControlBrushless(int speed);
    void WalkStraight(float vel, bool frente);
    bool StartBrushless(int speed);
    void StopBrushless();
    void StopMotors();


private:

    // Atalhos do RobotData:
    Robot *robot;
    dataMotor *get_Speed;
    // Atalhos para serviços:
    ESP32MotorControl control;
    LEDsService *LED;

    int Brushless_ActualPwm = 205;
    
    // Bibliotecas para controlar os motores:
    void AnalogWrite(ledc_channel_t channel, int pwm);
    void InitPWM(gpio_num_t pin, ledc_channel_t channel);
    void rampThrottle(int start, int stop, int step, int time);
    void calibrate();
};

#endif