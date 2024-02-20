#ifndef MOTOR_SERVICE_HPP
#define MOTOR_SERVICE_HPP

// Bibliotecas para criacao do servico
#include "thread.hpp"
#include "esp_log.h"
#include "Injector/singleton.hpp"
// Bibliotecas de componentes/dados
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "ESP32MotorControl.h"
#include "RobotData.h"
#include "LEDsService.hpp"
#include "DriveEncoder.h"

using namespace cpp_freertos;

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define MIN_THROTTLE            205
#define MAX_THROTTLE            409
#define THROTTLE_SPEED          100
#define PWM_BRUSHLESS_A         LEDC_CHANNEL_0
#define PWM_BRUSHLESS_B         LEDC_CHANNEL_1
#define BRUSHLESS_PWM_MODE      LEDC_HIGH_SPEED_MODE
#define BRUSHLESS_TIMER         LEDC_TIMER_0
#define BRUSHLESS_RESOLUTION    LEDC_TIMER_12_BIT
#define BRUSHLESS_FREQUENCY     50


class MotorService : public Thread, public Singleton<MotorService>
{
public:
    MotorService(std::string name, uint32_t stackDepth, UBaseType_t priority, BaseType_t coreid);
    
    void Run() override;
    void ControlMotors(float left, float right);
    void ControlBrushless(int speed);
    void ConfigBrushless();
    void TestMotors();
    void WalkStraight(float vel, bool frente);
    bool StartBrushless(int speed);
    void StopBrushless();
    void StopMotors();
    void stop_car();
    void read_both();
    void enc_reset_count();

private:

    std::string tag;

    // Atalhos do RobotData:
    Robot *robot;
    dataMotor *get_Speed;
    // Atalhos para serviços:
    ESP32MotorControl control;
    LEDsService *LED;
    DriveEncoder encs;

    int Brushless_ActualPwm = 205;
    
    // Bibliotecas para controlar os motores:
    void AnalogWrite(ledc_channel_t channel, int pwm);
    void InitBrushlessPWM(gpio_num_t pin, ledc_channel_t channel);
    void rampThrottle(int start, int stop, int step, int time);
    void calibrateBrushless();
};

#endif