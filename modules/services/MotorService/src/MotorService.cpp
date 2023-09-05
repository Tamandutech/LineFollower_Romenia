#include "MotorService.hpp"

MotorService::MotorService(std::string name, uint32_t stackDepth, UBaseType_t priority):Thread(name, stackDepth, priority)
{
    tag = name;
    // Atalhos para o RobotData:
    this->robot = Robot::getInstance();
    this->status = robot->getStatus();
    // Motores:
    esp_log_level_set(name.c_str(), ESP_LOG_INFO);

    //gpio_set_direction((gpio_num_t)in_dir1, GPIO_MODE_OUTPUT);
    //gpio_set_direction((gpio_num_t)in_dir2, GPIO_MODE_OUTPUT);
    //gpio_set_direction((gpio_num_t)in_esq1, GPIO_MODE_OUTPUT);
    //gpio_set_direction((gpio_num_t)in_esq2, GPIO_MODE_OUTPUT);
    //gpio_set_direction((gpio_num_t)stby, GPIO_MODE_OUTPUT);
    //gpio_set_level((gpio_num_t)stby, 1);
    //InitPWM((gpio_num_t)pwmA, PWM_A_PIN);
    //InitPWM((gpio_num_t)pwmB, PWM_B_PIN);

    motors.attachMotors(DRIVER_AIN1, DRIVER_AIN2, DRIVER_PWMA, DRIVER_BIN1, DRIVER_BIN2, DRIVER_PWMB);

    // Brushless:
    //brush_dir.begin(DSHOT_MODE);
    //brush_esq.begin(DSHOT_MODE);
}

void MotorService::Run()
{
    ESP_LOGI(GetName().c_str(), "Início MotorService");
    // Variavel necerraria para funcionalidade do vTaskDelayUtil, guarda a conGetName().c_str()em de pulsos da CPU
    // TickType_t xLastWakeTime = xTaskGetTickCount();

    // Loop
    for (;;)
    {
        vTaskDelay(0);
        this->Suspend();
    }
}

void MotorService::ControlMotors(float velesq, float veldir){
    velesq = constrain(velesq, -100, 100);
    veldir = constrain(veldir, -100, 100);
    motors.motorSpeed(0, velesq);
    motors.motorSpeed(1, veldir);
    
}

void MotorService::StopMotors()
{
    ESP_LOGI(GetName().c_str(), "Desligando motores");
    motors.motorsStop();
}

void MotorService::WalkStraight(float vel, bool frente){

    if(frente)
    {
        //ESP_LOGI(GetName().c_str(), "Andando para frente");
        motors.motorSpeed(0, vel);
        motors.motorSpeed(1, vel);
    }
    else{
        //ESP_LOGI(GetName().c_str(), "Andando para trás");
        motors.motorSpeed(0, (-vel));
        motors.motorSpeed(1, (-vel));
    }
    
}

void MotorService::StartBrushless()
{
    rampThrottle(brush_dir, MIN_THROTTLE, MAX_THROTTLE, THROTTLE_SPEED);
    rampThrottle(brush_esq, MIN_THROTTLE, MAX_THROTTLE, THROTTLE_SPEED);
}

void MotorService::StopBrushless()
{
    rampThrottle(brush_dir, MAX_THROTTLE, 0, -THROTTLE_SPEED);
    rampThrottle(brush_esq, MAX_THROTTLE, 0, -THROTTLE_SPEED);
}

void MotorService::rampThrottle(DShotRMT esc, int start, int stop, int step)
{
    if (step == 0)
        return;

    for (int i = start; step > 0 ? i < stop : i > stop; i += step)
    {
        esc.sendThrottleValue(i);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    esc.sendThrottleValue(stop);
}

void MotorService::AnalogWrite(ledc_channel_t channel, int pwm){
    ledc_set_duty_and_update(LEDC_MODE,channel,pwm,0); // Atribui um novo duty para o PWM
}

void MotorService::InitPWM(gpio_num_t pin, ledc_channel_t channel){
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode      = LEDC_MODE;
    ledc_timer.duty_resolution = LEDC_DUTY_RES;
    ledc_timer.timer_num       = LEDC_TIMER;
    ledc_timer.freq_hz         = LEDC_FREQUENCY; // Frequência de 5Khz
    ledc_timer.clk_cfg         = LEDC_AUTO_CLK; // Configuração da fonte de clock
    ledc_timer_config(&ledc_timer);

    // Prepara e aplica a configuração do canal do LEDC
    ledc_channel_config_t ledc_channel;
    ledc_channel.gpio_num       = pin;
    ledc_channel.speed_mode     = LEDC_MODE;
    ledc_channel.channel        = channel;
    ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel      = LEDC_TIMER;
    ledc_channel.duty           = 0; 
    ledc_channel.hpoint         = 0; // Ponto de início do duty cycle
    ledc_channel_config(&ledc_channel);

    ledc_fade_func_install(0);
}

