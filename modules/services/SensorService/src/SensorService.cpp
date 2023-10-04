#include "SensorService.hpp"

SensorService::SensorService(std::string name, uint32_t stackDepth, UBaseType_t priority):Thread(name, stackDepth, priority)
{
    // Atalhos de dados:
    this->robot = Robot::getInstance();
    this->get_Vel = robot->getMotorData();
    this->get_Spec = robot->getSpecification();
    this->get_Status = robot->getStatus();
    this->get_Marks = robot->getSLatMarks();
    this->get_latArray = robot->getFotoSensors(SENSOR_SIDE);
    this->get_centerArray = robot->getFotoSensors(SENSOR_CENTER);
    this->get_frontArray = robot->getFrontSensors();
    // Atalhos de servicos:
    this->control_motor = MotorService::getInstance();
    this->rpm = RPMService::getInstance();
    this->LED = LEDsService::getInstance();
    
    esp_log_level_set(name.c_str(), ESP_LOG_INFO);

    // Definindo configs do ADC1 no GPIO35
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_11db);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_11db);

    for(int i=0; i< 4; i++){
        gpio_pad_select_gpio(sPins[i]);
        gpio_set_direction((gpio_num_t)sPins[i], GPIO_MODE_OUTPUT);
    }
    
    // Inicializacao dos sensores frontais
    // Todos sao controlados na mesma porta, porém cada objeto do vetor representa um sensor diferente
    // Para controlar cada sensor, é preciso mudar as portas digitais com o selectMuxPin()
    for(int i=0; i < sQuant; i++)
    {
        sArray[i].setTypeAnalogESP();
        sArray[i].setSensorPins((const adc1_channel_t[]){ADC1_CHANNEL_7}, 1);
        sArray[i].setSamplesPerSensor(5);
    }
    // Inicializacao dos sensores laterais
    for(int i=0; i < 6; i++)
    {
        sBody[i].setTypeAnalogESP();
        sBody[i].setSensorPins((const adc1_channel_t[]){ADC1_CHANNEL_5}, 1);
        sBody[i].setSamplesPerSensor(5);
    }
    
    // Setando todas as leituras anteriores como 0 (centralizado na reta)
    for(int i=0; i < sQuantReading; i++)
    {
        AngleArray[i] = 0;
    }
    std::vector<float> SChannelsAngle(AngleArray, AngleArray + sQuantReading);
    get_frontArray->setChannels(SChannelsAngle);

    //Calibracao
    LEDposition[0] = LED_POSITION_FRONT;
    LEDposition[1] = LED_POSITION_NONE;
    
    LED->config_LED(LEDposition, COLOR_GREEN, LED_EFFECT_SET, 1);

    ESP_LOGI(GetName().c_str(), "Início calibraçao frontal...");
    manual_calibrate(0);
    
    LED->config_LED(LEDposition, COLOR_YELLOW, LED_EFFECT_SET, 1);
    ESP_LOGI(GetName().c_str(), "Delay...");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    ESP_LOGI(GetName().c_str(), "Fim da calibração frontal...");
    LED->config_LED(LEDposition, COLOR_LIME, LED_EFFECT_SET, 1);
    manual_calibrate(1);
    
    ESP_LOGI(GetName().c_str(), "Fim da calibração.");
    LED->config_LED(LEDposition, COLOR_BLACK, LED_EFFECT_SET, 1);
}

void SensorService::Run()
{
    ESP_LOGI(GetName().c_str(), "Início SensorService");
    // Loop do servico
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Loop
    for (;;)
    {
        //vTaskDelay(0);
        //this->Suspend();

        vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);

        processSLat();
        //processSCenter();
    }
}

void SensorService::Sensor_resume()
{
    this->Resume();
}

void SensorService::manual_calibrate(int mux)
{
    for(int i=0; i<50; i++)
    {
        if(mux == 0){
            MUX.calibrate_all(sArray, sQuant); // Funcao que calibra os 16 sensores 1 vez cada
        }
        else{
            MUX.calibrate_all(sBody, 6); // Funcao que calibra os 16 sensores 1 vez cada
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void SensorService::auto_calibrate(int mux)
{// Calibracao automatica
    
    //ESP_LOGI(GetName().c_str(), "Início Calibração");
    
    MPR_Mot = get_Spec->MPR->getData();

    // Zera contagem dos encoders
    //rpm->ResetCount();
    uint16_t limit_enc;
    if(mux == 0){
        limit_enc = MPR_Mot/3;
    }else{
        limit_enc = MPR_Mot/4;
    }

    // Calibração dos sensores frontais -> mux == 0
    // Calibracao dos sensores laterais -> mux == 1
    for(int voltas=1; voltas <= 6; voltas++)
    {
        rpm->ResetCount(); // Reseta a contagem para comecar outra volta
        while((get_Vel->EncMedia->getData()) < limit_enc) // enquanto o robô não andou um giro da roda
        {
            rpm->ReadBoth(); // atualiza a leitura dos encoders
            /*
            int32_t enc = get_Vel->EncMedia->getData();
            int32_t enc_es = get_Vel->EncLeft->getData();
            int32_t enc_di = get_Vel->EncRight->getData();
            ESP_LOGI(GetName().c_str(), "EncEsq = %d, EncDir = %d, EncMedia = %d", enc_es, enc_di, enc);
            */

            // Anda para uma direcao por 1/4 da calibracao
            // Anda pra direcao contraria por 2/4
            // Volta a posicao original, andando o último 1/4 do tempo
            if(voltas%2 == 0)
            {
                // Chama a funcao do servico dos Motores para o robô andar reto
                control_motor->WalkStraight(get_Vel->vel_calibrate->getData(), 0);
            }
            else
            {
                // Chama a mesma funcao para o robô andar para o lado contraio
                control_motor->WalkStraight(get_Vel->vel_calibrate->getData(), 1);
            }

            // Escolhe qual sensor esta sendo calibrado:
            if(mux == 0){
                MUX.calibrate_all(sArray, sQuant); // Funcao que calibra os 16 sensores 1 vez cada
            }
            else{
                MUX.calibrate_all(sBody, 6); // Funcao que calibra os 16 sensores 1 vez cada
            }
            //vTaskDelay(10 / portTICK_PERIOD_MS); // Delay de 10 milisegundos
            rpm->ReadBoth();
        }
        control_motor->StopMotors();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    control_motor->StopMotors();
    rpm->ResetCount();
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void SensorService::SaveAngle(float new_angle)
{// Salva a leitura do sensor em um vetor
    ////ESP_LOGI(GetName().c_str(), "Salvando Array no RobotData...");
    for(int i=(sQuantReading-1); i > 0; i--){
        AngleArray[i] = AngleArray[i-1];
    }
    AngleArray[0] = new_angle;

    std::vector<float> SChannelsAngle(AngleArray, AngleArray + sQuantReading);
    get_frontArray->setChannels(SChannelsAngle);
    Sensor_resume();
    ////ESP_LOGI(GetName().c_str(), "Array salvo.");
}

void SensorService::SaveArray(uint16_t *array, int array_len, dataUint16 *get_array)
{
    std::vector<uint16_t> SChannelsVec(array, array + array_len); // construtor de vector(array) com os valores dos sensores laterais

    // armazenando da leitura bruta do sensor lateral no objeto Braia
    get_array->setChannels(SChannelsVec);
}

void SensorService::AngleError()
{
    // Funcao que retorna o erro em radianos, sendo esse erro o angulo em relacao ao centro de movimento
    
    // Carregando as variaveis do RobotData, para facilitar a leitura
    bool is_white = get_Spec->WhiteLine->getData();
    float max_angle = get_Spec->MaxAngle->getData(); // em graus
    uint16_t radius = get_Spec->RadiusSensor->getData();
    uint16_t dis_center = get_Spec->SensorToCenter->getData();
    //ESP_LOGI(GetName().c_str(), "MaxAngle = %d; Radius = %.2f; Ang centro = %.2f", max_angle, angle_radius, angle_with_center);
    max_angle = max_angle*M_PI/180; // converte graus em rad, sendo M_PI o valor de pi da biblioteca "cmath"

    //vTaskSuspendAll();// Pausando as outras tasks para evitar conflitos com o delay usado
    int16_t position = MUX.read_all(sArray, sQuant, is_white); // le os sensores e recebe uma posicao
    //xTaskResumeAll(); // Retoma o funcionamento normal das tasks

    // position vai de 0 ate (sQuant - 1)*1000, sendo sQuant a quantidade de sensores
    // Para 16 sensores, vai de 0 a 15000
    position = position - (sQuant - 1)*500; // subtrai a metade do valor máximo, posicao central fica em 0

    float angle_radius = position*max_angle/((sQuant - 1)*500); // converte a posicao para angulo, regra de 3s
    
    float angle_with_center = atan((sin(angle_radius))/(cos(angle_radius) -1 + (((float)(dis_center))/radius)));
    angle_radius = angle_radius*180/M_PI;
    angle_with_center = angle_with_center*180/M_PI;
    
    //ESP_LOGI(GetName().c_str(), "Leitura = %d; Ang arco = %.2f; Ang centro = %.2f", position, angle_radius, angle_with_center);
    
    // Salvando esse angulo na variavel do servico:
    SaveAngle(angle_with_center);
}

void SensorService::processSCenter()
{
    bool is_white = get_Spec->WhiteLine->getData();
    uint16_t center_values[2];
    MUX.read_from_body(center_values, sBody, 6, 7, is_white);
    SaveArray(center_values, 2, get_centerArray);

    uint16_t slesq = get_centerArray->getChannel(0);
    uint16_t sldir = get_centerArray->getChannel(1);

    if ((slesq < 300) && (sldir < 300)) 
    {
        get_Status->RobotCenter->setData(CAR_CENTERED);
    }
    else if ((slesq < 300) && (sldir > 600))
    {
        get_Status->RobotCenter->setData(CAR_TO_THE_LEFT);
    }
    else if ((slesq > 600) && (sldir < 300))
    {
        get_Status->RobotCenter->setData(CAR_TO_THE_RIGHT);
    }
    //ESP_LOGI(GetName().c_str(), "Estado RobotCenter = ", get_Status->RobotCenter->getData());
}

int SensorService::lower_value(uint16_t s_1, uint16_t s_2){
    if((s_1 < s_2)){
        return s_1;
    }
    else{
        return s_2;
    }
}

void SensorService::processSLat()
{
    bool is_white = get_Spec->WhiteLine->getData();
    uint16_t values[4];
    MUX.read_from_body(values, sBody, 0, 3, is_white);
    //SaveArray(values, 4, get_centerArray); 
    std::vector<uint16_t> SChannelsVec(values, values + 4);
    get_centerArray->setChannels(SChannelsVec);

    uint16_t slesq_1 = get_latArray->getChannel(0);
    uint16_t slesq_2 = get_latArray->getChannel(1);
    uint16_t sldir_1 = get_latArray->getChannel(2);
    uint16_t sldir_2 = get_latArray->getChannel(3);

    //ESP_LOGI("SensorService", "Esquerda: %d %d; Direita: %d %d", values[0], values[1], values[2], values[3]);
    //ESP_LOGI(GetName().c_str(), "Esquerda: %d %d; Direita: %d %d", slesq_1, slesq_2, sldir_1, sldir_2);
    
    nLatReads++; 
    sumSensEsq += lower_value(slesq_1, slesq_2);
    sumSensDir += lower_value(sldir_1, sldir_2);

    if(get_Status->robotIsMapping->getData())
    {
        MarksToMean = get_Marks->MarkstoMean->getData();
    }
    else
    {
        MarksToMean = 1;
    }

    if (nLatReads >= MarksToMean)  //MarksToMean definido na dashboard
    {
        int meanSensDir = (sumSensDir/nLatReads); // leitura média do sensor direito
        int meanSensEsq = (sumSensEsq/nLatReads); // leitura média do sensor esquerdo

        if (meanSensEsq < 300 || meanSensDir < 300)
        { // leitura de faixas brancas sensores laterais
            if ((meanSensEsq < 300) && (meanSensDir > 600)) 
            {// lendo sLat esq. branco e dir. preto
                if (!(get_Marks->latEsqPass->getData()))
                {
                    if(get_Status->robotState->getData() != CAR_STOPPED)
                    {
                        get_Marks->leftPassedInc();
                    }
                    latState(false, true);
                    ESP_LOGI(GetName().c_str(), "Marcação esquerda");
                    // LED esquerda vermelha e direita apagada
                    LEDposition[0] = LED_POSITION_LEFT;
                    LEDposition[1] = LED_POSITION_NONE;
                    LED->config_LED(LEDposition, COLOR_RED, LED_EFFECT_SET, 1);
                    LEDposition[0] = LED_POSITION_RIGHT;
                    LED->config_LED(LEDposition, COLOR_BLACK, LED_EFFECT_SET, 1);
                }
            }
            else if ((meanSensDir < 300) && (meanSensEsq > 600))
            { // lendo sldir. branco e sLat esq. preto
                if (!(get_Marks->latDirPass->getData()))
                {
                    if(get_Status->robotState->getData() != CAR_STOPPED)
                    {
                        get_Marks->rightPassedInc();

                    }
                    latState(true, false);
                    ESP_LOGI(GetName().c_str(), "Marcação direita");

                    // LED esquerda apagada e direita vermelha
                    LEDposition[0] = LED_POSITION_RIGHT;
                    LEDposition[1] = LED_POSITION_NONE;
                    LED->config_LED(LEDposition, COLOR_RED, LED_EFFECT_SET, 1);
                    LEDposition[0] = LED_POSITION_LEFT;
                    LED->config_LED(LEDposition, COLOR_BLACK, LED_EFFECT_SET, 1);
                }
            }

            else if ((meanSensEsq < 300) && (meanSensDir < 300)) 
            {// quando ler ambos brancos, contar nova marcação apenas se ambos os sensores lerem preto antes de lerem a nova marcação 
                if ((get_Marks->latDirPass->getData() && !get_Marks->latEsqPass->getData()) 
                    || (get_Marks->latEsqPass->getData() && !get_Marks->latDirPass->getData()))
                {
                    // Desligando as LEDs esquerda e direita
                    LEDposition[1] = LED_POSITION_RIGHT;
                    LEDposition[0] = LED_POSITION_LEFT;
                    LEDposition[2] = LED_POSITION_NONE;
                    LED->config_LED(LEDposition, COLOR_BLACK, LED_EFFECT_SET, 1);
                }
                latState(true, true);
                ESP_LOGI(GetName().c_str(), "Marcação esquerda e direita");
            }
        }
        else
        {
            if (get_Marks->latDirPass->getData() || get_Marks->latEsqPass->getData())
            {
                 // Desligando as LEDs esquerda e direita
                LEDposition[1] = LED_POSITION_RIGHT;
                LEDposition[0] = LED_POSITION_LEFT;
                LEDposition[2] = LED_POSITION_NONE;
                LED->config_LED(LEDposition, COLOR_BLACK, LED_EFFECT_SET, 1);
            }
            latState(false, false);
        }
        nLatReads = 0;
        sumSensDir = 0;
        sumSensEsq = 0;
    }
}

void SensorService::latState(bool rightPass, bool leftPass)
{
    get_Marks->latDirPass->setData(rightPass);
    get_Marks->latEsqPass->setData(leftPass);
}