#ifndef MOTOR_SERVICE_HPP
#define MOTOR_SERVICE_HPP

// Bibliotecas para criacao do servico
#include "thread.hpp"
#include "esp_log.h"
#include "Injector/singleton.hpp"
// Bibliotecas de componentes/dados
#include "DShotRMT.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "ESP32MotorControl.h"
#include "RobotData.h"
#include "LEDsService.hpp"

using namespace cpp_freertos;

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#define LEDC_TIMER              LEDC_TIMER_0 // Timer do LEDC utilizado
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE // Modo de velocidade do LEDC
#define PWM_A_PIN               LEDC_CHANNEL_0 // Canal do LEDC utilizado
#define PWM_B_PIN               LEDC_CHANNEL_1 // Canal do LEDC utilizado
#define LEDC_DUTY_RES           LEDC_TIMER_12_BIT // Resolução do PWM
#define LEDC_FREQUENCY          50 // Frequência em Hertz do sinal PWM
#define MIN_THROTTLE            205
#define MAX_THROTTLE            409
#define THROTTLE_SPEED          100
#define DSHOT_MODE              DSHOT600
#define USB_SERIAL_BAUD         115200
#define USB_Serial              Serial



class MotorService : public Thread, public Singleton<MotorService>
{
public:
    MotorService(std::string name, uint32_t stackDepth, UBaseType_t priority);
    
    void Run() override;
    void ControlMotors(float left, float right);
    void ControlBrushless();
    void WalkStraight(float vel, bool frente);
    bool StartBrushless();
    void StopBrushless();
    void StopMotors();


private:

    std::string tag;

    // Atalhos do RobotData:
    Robot *robot;
    dataStatus *status;

    ESP32MotorControl motors;
    LEDsService *LED;

    int Brushless_ActualPwm = 205;

    led_position_t LEDposition[NUM_LEDS] = {LED_POSITION_NONE};
    
    // Bibliotecas para controlar os motores:
    void AnalogWrite(ledc_channel_t channel, int pwm);
    void InitPWM(gpio_num_t pin, ledc_channel_t channel);
    void rampThrottle(int start, int stop, int step, int time);
    void calibrate();
};

#endif