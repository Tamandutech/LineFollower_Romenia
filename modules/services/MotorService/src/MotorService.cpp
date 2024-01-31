#include "MotorService.hpp"

MotorService::MotorService(std::string name, uint32_t stackDepth, UBaseType_t priority, BaseType_t coreid):Thread(name, stackDepth, priority,  coreid)
{
    tag = name;
    // Atalhos para o RobotData:
    this->robot = Robot::getInstance();
    this->get_Speed = robot->getMotorData();

    LED = LEDsService::getInstance();

    esp_log_level_set(name.c_str(), ESP_LOG_INFO);

    control.attachMotors(DRIVER_AIN1, DRIVER_AIN2, DRIVER_PWMA, DRIVER_BIN2, DRIVER_BIN1, DRIVER_PWMB);

    ConfigBrushless();

    encs.ConfigEncoders();
}



void MotorService::Run()
{
    ESP_LOGI(GetName().c_str(), "Início MotorService");
    // Variavel necerraria para funcionalidade do vTaskDelayUtil, guarda a conGetName().c_str()em de pulsos da CPU
    // TickType_t xLastWakeTime = xTaskGetTickCount();
    //TestMotors();
    // Loop
    for (;;)
    {
        vTaskDelay(0);
        this->Suspend();
        
        int16_t right_wheel = get_Speed->PWM_right->getData();
        int16_t left_wheel = get_Speed->PWM_left->getData();

        ControlMotors(left_wheel, right_wheel);
    }
}

void MotorService::ConfigBrushless(){
    InitPWM((gpio_num_t)brushless_dir, PWM_A);
    InitPWM((gpio_num_t)brushless_esq, PWM_B);

    ESP_LOGI(GetName().c_str(), "Calibracao Brushless...");
    //calibrate();
    ESP_LOGI(GetName().c_str(), "Fim Calibracao Brushless");
}

void MotorService::TestMotors(){
    control.motorSpeed(0, 30.0);
    control.motorSpeed(1, 30.0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    control.motorsStop();
}

void MotorService::calibrate(){
    AnalogWrite(PWM_A, MAX_THROTTLE);
    AnalogWrite(PWM_B, MAX_THROTTLE);
    
    LED->LedComandSend(LED_POSITION_FRONT, COLOR_BLUE, 1);
    vTaskDelay(5200 / portTICK_PERIOD_MS);
    
    AnalogWrite(PWM_A, MIN_THROTTLE);
    AnalogWrite(PWM_B, MIN_THROTTLE);

    LED->LedComandSend(LED_POSITION_FRONT, COLOR_PINK, 1);
    
    vTaskDelay(5200 / portTICK_PERIOD_MS);
    
    Brushless_ActualPwm = MIN_THROTTLE;
    
}

void MotorService::ControlMotors(float velesq, float veldir){
    velesq = constrain(velesq, -100, 100);
    veldir = constrain(veldir, -100, 100);
    //ESP_LOGI(GetName().c_str(), "Velocidades: %.2f %.2f", velesq, veldir);
    control.motorSpeed(0, velesq);
    control.motorSpeed(1, veldir);
}

void MotorService::ControlBrushless(int speed){
    // speed = constrain(speed, MIN_THROTTLE, MAX_THROTTLE);
    if(Brushless_ActualPwm  <= speed)  rampThrottle(Brushless_ActualPwm, speed, THROTTLE_SPEED, 200);
    else rampThrottle(speed, Brushless_ActualPwm, -THROTTLE_SPEED, 200);
    Brushless_ActualPwm = speed;
}

void MotorService::StopMotors()
{
    //ESP_LOGI(GetName().c_str(), "Desligando motores");
    control.motorsStop();
}

void MotorService::WalkStraight(float vel, bool frente){
    //ESP_LOGI(GetName().c_str(), "Andando reto");
    if(frente)
    {
        //ESP_LOGI(GetName().c_str(), "Andando para frente");
        control.motorSpeed(0, vel);
        control.motorSpeed(1, vel);
    }
    else{
        //ESP_LOGI(GetName().c_str(), "Andando para trás");
        control.motorSpeed(0, (-vel));
        control.motorSpeed(1, (-vel));
    }
    
}

bool MotorService::StartBrushless(int speed)
{
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
        AnalogWrite(PWM_A, (i));
        AnalogWrite(PWM_B, (i));
        //ESP_LOGI(GetName().c_str(), "Brushless: %d", i);
        vTaskDelay(time / portTICK_PERIOD_MS);
    }
    AnalogWrite(PWM_A, (stop));
    AnalogWrite(PWM_B, (stop));
}

void MotorService::stop_car(){
    StopMotors();
    StopBrushless();
    encs.ResetCount();
}

void MotorService::enc_reset_count(){
    encs.ResetCount();
}

void MotorService::read_both(){
    encs.ReadBoth();
}

void MotorService::AnalogWrite(ledc_channel_t channel, int pwm){
    ledc_set_duty_and_update(LEDC_MODE,channel,pwm,0); // Atribui um novo duty para o PWM
}

void MotorService::InitPWM(gpio_num_t pin, ledc_channel_t channel){
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode      = LEDC_MODE;
    ledc_timer.duty_resolution = LEDC_DUTY_RES;
    ledc_timer.timer_num       = LEDC_TIMER_2;
    ledc_timer.freq_hz         = LEDC_FREQUENCY; // Frequência de 5Khz
    ledc_timer.clk_cfg         = LEDC_AUTO_CLK; // Configuração da fonte de clock
    ledc_timer_config(&ledc_timer);

    // Prepara e aplica a configuração do canal do LEDC
    ledc_channel_config_t ledc_channel;
    ledc_channel.gpio_num       = pin;
    ledc_channel.speed_mode     = LEDC_MODE;
    ledc_channel.channel        = channel;
    ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel      = LEDC_TIMER_2;
    ledc_channel.duty           = 0; 
    ledc_channel.hpoint         = 0; // Ponto de início do duty cycle
    ledc_channel_config(&ledc_channel);

    ledc_fade_func_install(0);
}
