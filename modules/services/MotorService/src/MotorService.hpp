#ifndef MOTOR_SERVICE_HPP
#define MOTOR_SERVICE_HPP

#include "thread.hpp"
#include "esp_log.h"
#include "Injector/singleton.hpp"
#include "QTRSensors.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "RobotData.h"

using namespace cpp_freertos;

#define LEDC_TIMER              LEDC_TIMER_0 // Timer do LEDC utilizado
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE // Modo de velocidade do LEDC
#define PWM_A_PIN               LEDC_CHANNEL_0 // Canal do LEDC utilizado
#define PWM_B_PIN               LEDC_CHANNEL_1 // Canal do LEDC utilizado
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Resolução do PWM
#define LEDC_FREQUENCY          5000 // Frequência em Hertz do sinal PWM

class MotorService : public Thread, public Singleton<MotorService>
{
public:
    MotorService(std::string name, uint32_t stackDepth, UBaseType_t priority);
    
    void Run() override;
    void ControlMotors(float left, float right);
    void WalkStraight(float vel, bool frente);
    void StopMotors();

private:

    std::string tag;
    
    // Bibliotecas para controlar os motores:
    void AnalogWrite(ledc_channel_t channel, int pwm);
    void InitPWM(gpio_num_t pin, ledc_channel_t channel);
};

#endif