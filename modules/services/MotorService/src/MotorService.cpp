#include "MotorService.hpp"

MotorService::MotorService(std::string name, uint32_t stackDepth, UBaseType_t priority):Thread(name, stackDepth, priority)
{
    tag = name;
    // Atalhos para o RobotData:
    this->robot = Robot::getInstance();
    this->status = robot->getStatus();
    this->LED = LEDsService::getInstance();
    // Motores:
    esp_log_level_set(name.c_str(), ESP_LOG_INFO);

    motors.attachMotors(DRIVER_AIN1, DRIVER_AIN2, DRIVER_PWMA, DRIVER_BIN2, DRIVER_BIN1, DRIVER_PWMB);

    InitPWM((gpio_num_t)brushless_dir, PWM_A_PIN);
    InitPWM((gpio_num_t)brushless_esq, PWM_B_PIN);
    ESP_LOGI(GetName().c_str(), "Iniciando Calibracao");
    calibrate();
    ESP_LOGI(GetName().c_str(), "Fim Calibracao");
}



void MotorService::Run()
{
    //ESP_LOGI(GetName().c_str(), "Início MotorService");
    // Variavel necerraria para funcionalidade do vTaskDelayUtil, guarda a conGetName().c_str()em de pulsos da CPU
    // TickType_t xLastWakeTime = xTaskGetTickCount();

    // Loop
    for (;;)
    {
        vTaskDelay(0);
        this->Suspend();
    }
}

void MotorService::calibrate(){
    AnalogWrite(PWM_A_PIN, MAX_THROTTLE);
    AnalogWrite(PWM_B_PIN, MAX_THROTTLE);
    //ESP_LOGI(GetName().c_str(), "Brushless: %d", MAX_THROTTLE);
    LEDposition[0] = LED_POSITION_FRONT;
    LEDposition[1] = LED_POSITION_NONE;
    LED->config_LED(LEDposition, COLOR_BLUE, LED_EFFECT_SET, 1);
    vTaskDelay(5200 / portTICK_PERIOD_MS);
    
    AnalogWrite(PWM_A_PIN, MIN_THROTTLE);
    AnalogWrite(PWM_B_PIN, MIN_THROTTLE);
    LEDposition[0] = LED_POSITION_FRONT;
    LEDposition[1] = LED_POSITION_NONE;
    LED->config_LED(LEDposition, COLOR_PINK, LED_EFFECT_SET, 1);
    ESP_LOGI(GetName().c_str(), "Brushless: %d", MIN_THROTTLE);
    vTaskDelay(5200 / portTICK_PERIOD_MS);
    
    Brushless_ActualPwm = MIN_THROTTLE;
    //ESP_LOGI(GetName().c_str(), "Brushless: %d", 300);
}

void MotorService::ControlMotors(float velesq, float veldir){
    velesq = constrain(velesq, -100, 100);
    veldir = constrain(veldir, -100, 100);

    motors.motorSpeed(0, velesq);
    motors.motorSpeed(1, veldir);
}

void MotorService::ControlBrushless(){
    int speed = robot->getMotorData()->Brushless_TargetSpeed->getData();
    // speed = constrain(speed, MIN_THROTTLE, MAX_THROTTLE);
    if(Brushless_ActualPwm  <= speed)  rampThrottle(Brushless_ActualPwm, speed, THROTTLE_SPEED, 200);
    else rampThrottle(speed, Brushless_ActualPwm, -THROTTLE_SPEED, 200);
    Brushless_ActualPwm = speed;
}

void MotorService::StopMotors()
{
    //ESP_LOGI(GetName().c_str(), "Desligando motores");
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

bool MotorService::StartBrushless()
{
    int speed = robot->getMotorData()->Brushless_TargetSpeed->getData();
    rampThrottle(MIN_THROTTLE, speed, THROTTLE_SPEED, 200);
    Brushless_ActualPwm = speed;
    vTaskDelay(200 / portTICK_PERIOD_MS);
    return true;
}

void MotorService::StopBrushless()
{
    if(Brushless_ActualPwm  > MIN_THROTTLE) rampThrottle(Brushless_ActualPwm, MIN_THROTTLE, -THROTTLE_SPEED, 200);
    Brushless_ActualPwm = MIN_THROTTLE;
}

void MotorService::rampThrottle(int start, int stop, int step, int time)
{
    if (step == 0)
        return;

    for (int i = start; step > 0 ? i < stop : i > stop; i += step)
    {
        AnalogWrite(PWM_A_PIN, (i));
        AnalogWrite(PWM_B_PIN, (i));
        ESP_LOGI(GetName().c_str(), "Brushless: %d", i);
        vTaskDelay(time / portTICK_PERIOD_MS);
    }
    AnalogWrite(PWM_A_PIN, (stop));
    AnalogWrite(PWM_B_PIN, (stop));
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

