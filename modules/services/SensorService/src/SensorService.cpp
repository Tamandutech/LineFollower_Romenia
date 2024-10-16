#include "SensorService.hpp"

SensorService::SensorService(std::string name, uint32_t stackDepth, UBaseType_t priority, BaseType_t coreid):Thread(name, stackDepth, priority,  coreid)
{
    // Atalhos de dados:
    this->robot = Robot::getInstance();
    this->get_Speed = robot->getMotorData();
    this->get_Spec = robot->getSpecification();
    this->get_Status = robot->getStatus();
    this->get_Marks = robot->getSLatMarks();
    this->get_latMarks = robot->getFotoSensors(SENSOR_SIDE);
    this->get_centerArray = robot->getFotoSensors(SENSOR_CENTER);
    this->get_frontArray = robot->getFrontSensors();
    // Atalhos de servicos:
    this->LED = LEDsService::getInstance();
    
    esp_log_level_set(name.c_str(), ESP_LOG_INFO);

    for(int i=0; i< 4; i++){
        gpio_set_direction((gpio_num_t)sPins[i], GPIO_MODE_OUTPUT);
    }
    
    // Inicializacao dos sensores frontais
    // Todos sao controlados na mesma porta, porém cada objeto do vetor representa um sensor diferente
    // Para controlar cada sensor, é preciso mudar as portas digitais com o selectMuxPin()

    adc_oneshot_unit_handle_t ADC_handle = robot->getADC_handle();

    for(int i=0; i < sQuant; i++)
    {
        sArray[i].setTypeAnalogESP(ADC_handle);
        sArray[i].setSensorPins((const adc_channel_t[]){ADC_CHANNEL_7}, 1);
        sArray[i].setSamplesPerSensor(4);
    }
    // Inicializacao dos sensores laterais
    for(int i=0; i < 6; i++)
    {
        sBody[i].setTypeAnalogESP(ADC_handle);
        sBody[i].setSensorPins((const adc_channel_t[]){ADC_CHANNEL_5}, 1);
        sBody[i].setSamplesPerSensor(4);
    }
    
    // Setando todas as leituras anteriores como 0 (centralizado na reta)
    for(int i=0; i < sQuantReading; i++)
    {
        AngleArray[i] = 0;
    }
    std::vector<float> SChannelsAngle(AngleArray, AngleArray + sQuantReading);
    get_frontArray->setChannels(SChannelsAngle);

    // Calculo do ângulo máximo para a distância atual
    float max_angle = (get_Spec->MaxAngle->getData())*M_PI/180; // de graus para rad
    uint16_t radius = get_Spec->RadiusSensor->getData();
    uint16_t dis_center = get_Spec->SensorToCenter->getData();
    float angle_with_center = atan((sin(max_angle))/(cos(max_angle) -1 + (((float)(dis_center))/radius)));
    angle_with_center = angle_with_center*180/M_PI; // de rad para graus
    get_Spec->MaxAngle_Center->setData(angle_with_center);

    update_voltage();

    //Calibracao
    
    LED->LedComandSend(LED_POSITION_FRONT, COLOR_GREEN, 1);
    //ESP_LOGI(GetName().c_str(), "Início calibraçao frontal...");
    manual_calibrate(0);
    //ESP_LOGI(GetName().c_str(), "Fim da calibração frontal...");
    LED->LedComandSend(LED_POSITION_FRONT, COLOR_RED, 1);
    manual_calibrate(1); 
    //ESP_LOGI(GetName().c_str(), "Fim da calibração.");
    LED->LedComandSend(LED_POSITION_FRONT, COLOR_BLACK, 1);

}

void SensorService::Run()
{
    //ESP_LOGI(GetName().c_str(), "Início SensorService");
    // Loop do servico
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Loop
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_PERIOD_MS);

        //processSLat();
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

void SensorService::SaveAngle(float new_angle)
{// Salva a leitura do sensor em um vetor
    ////ESP_LOGI(GetName().c_str(), "Salvando Array no RobotData...");
    for(int i=(sQuantReading-1); i > 0; i--){
        AngleArray[i] = AngleArray[i-1];
    }
    AngleArray[0] = new_angle;

    std::vector<float> SChannelsAngle(AngleArray, AngleArray + sQuantReading);
    get_frontArray->setChannels(SChannelsAngle);
    //Sensor_resume();
    ////ESP_LOGI(GetName().c_str(), "Array salvo.");
}

void SensorService::AngleError()
{
    // Funcao que retorna o erro em radianos, sendo esse erro o angulo em relacao ao centro de movimento
    
    // Carregando as variaveis do RobotData, para facilitar a leitura
    bool is_white = get_Status->LineColorBlack->getData();
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
    get_Speed->PositionError->setData(position);

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
    bool is_white = get_Status->LineColorBlack->getData();
    uint16_t center_values[2];
    MUX.read_from_body(center_values, sBody, 4, 5, is_white);
    std::vector<uint16_t> SChannelsVec(center_values, center_values + 2);
    get_centerArray->setChannels(SChannelsVec);

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
    //uint16_t status = get_Status->RobotCenter->getData();
    //ESP_LOGI(GetName().c_str(), "Estado RobotCenter = %d", status);
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
    bool is_white = get_Status->LineColorBlack->getData();
    uint16_t values[4];
    MUX.read_from_body(values, sBody, 0, 3, is_white);
    std::vector<uint16_t> SChannelsVec(values, values + 4);
    get_latMarks->setChannels(SChannelsVec);

    uint16_t slesq_1 = get_latMarks->getChannel(0);
    uint16_t slesq_2 = get_latMarks->getChannel(1);
    uint16_t sldir_1 = get_latMarks->getChannel(2);
    uint16_t sldir_2 = get_latMarks->getChannel(3);

    //ESP_LOGI("SensorService", "Esquerda: %d %d; Direita: %d %d", values[0], values[1], values[2], values[3]);
    //ESP_LOGI(GetName().c_str(), "Esquerda: %d %d; Direita: %d %d", slesq_1, slesq_2, sldir_1, sldir_2);
    
    nLatReads++; 
    sumSensEsq += lower_value(slesq_1, slesq_2);
    sumSensDir += lower_value(sldir_1, sldir_2);

    if(get_Status->robotState->getData() == CAR_MAPPING)
    {
        MarksToMean = get_Marks->MarkstoMean->getData();
    }
    else
    {
        MarksToMean = 1;
    }

    if (nLatReads >= MarksToMean)  //MarksToMean definido na dashboard
    {
        uint16_t meanSensDir = (sumSensDir/nLatReads);
        uint16_t meanSensEsq = (sumSensEsq/nLatReads);
        
        // Salvando Parâmetros
        get_Marks->MeanSensorRight->setData(meanSensDir);
        get_Marks->MeanSensorLeft->setData(meanSensEsq);
        
        //ESP_LOGI(GetName().c_str(), "Esquerdo: %d, Direito: %d", meanSensEsq, meanSensDir);

        
        
        if (meanSensEsq < 500 || meanSensDir < 500)
        { // leitura de faixas nos sensores laterais
            //ESP_LOGI(GetName().c_str(), "Esquerdo: %d, Direito: %d", meanSensEsq, meanSensDir);
            if ((meanSensEsq < 500) && (meanSensDir < 500)) 
            {// quando ler ambos, contar nova marcação apenas se ambos os sensores lerem preto antes de lerem a nova marcação 
                if ((get_Marks->latDirPass->getData() && !get_Marks->latEsqPass->getData()) 
                    || (get_Marks->latEsqPass->getData() && !get_Marks->latDirPass->getData()))
                {
                    // Desligando as LEDs esquerda e direita
                    if(get_Status->robotState->getData() == CAR_MAPPING || get_Status->robotState->getData() == CAR_STOPPED)
                    {
                        LED->LedComandSend(LED_POSITION_RIGHT, COLOR_BLACK, 1);
                        LED->LedComandSend(LED_POSITION_LEFT, COLOR_BLACK, 1);
                    }
                }
                latState(true, true);
                //ESP_LOGI(GetName().c_str(), "Marcação esquerda e direita");
            }
            else if ((meanSensEsq < 500))
            {// lendo sLat esq. branco e dir. preto
                if (!(get_Marks->latEsqPass->getData()))
                {
                    if(get_Status->robotState->getData() == CAR_MAPPING || get_Status->robotState->getData() == CAR_STOPPED)
                    {
                        LED->LedComandSend(LED_POSITION_LEFT, COLOR_RED, 1);
                        LED->LedComandSend(LED_POSITION_RIGHT, COLOR_BLACK, 1);
                    }
                    
                    if(get_Status->robotState->getData() != CAR_STOPPED)
                    {
                        get_Marks->leftPassedInc();
                        //ESP_LOGI(GetName().c_str(), "Marcação esquerda");
                    }
                    latState(false, true);
                    //ESP_LOGI(GetName().c_str(), "Marcação esquerda");
                    
                }
            }
            else if ((meanSensDir < 500))
            {
                // lendo sldir. branco e sLat esq. preto
                if (!(get_Marks->latDirPass->getData()))
                {
                    // LED esquerda apagada e direita vermelha
                    if(get_Status->robotState->getData() == CAR_MAPPING || get_Status->robotState->getData() == CAR_STOPPED)
                    {
                        LED->LedComandSend(LED_POSITION_RIGHT, COLOR_RED, 1);
                        LED->LedComandSend(LED_POSITION_LEFT, COLOR_BLACK, 1);
                    }

                    if(get_Status->robotState->getData() != CAR_STOPPED)
                    {
                        get_Marks->rightPassedInc();

                    }
                    latState(true, false);
                    //ESP_LOGI(GetName().c_str(), "Marcação direita");

                }
            }
        }
        else
        {
            if (get_Marks->latDirPass->getData() || get_Marks->latEsqPass->getData())
            {
                // Desligando as LEDs esquerda e direita
                if(get_Status->robotState->getData() == CAR_MAPPING || get_Status->robotState->getData() == CAR_STOPPED)
                {
                    LED->LedComandSend(LED_POSITION_RIGHT, COLOR_BLACK, 1);
                    LED->LedComandSend(LED_POSITION_LEFT, COLOR_BLACK, 1);
                }
            }
            latState(false, false);
        }
        nLatReads = 0;
        sumSensDir = 0;
        sumSensEsq = 0;
    }
}

void SensorService::processSLat_romenia()
{
    bool is_white = get_Status->LineColorBlack->getData();
    uint16_t values[4];
    MUX.read_from_body(values, sBody, 0, 3, is_white);
    std::vector<uint16_t> SChannelsVec(values, values + 4);
    get_latMarks->setChannels(SChannelsVec);

    uint16_t slesq_1 = get_latMarks->getChannel(0);
    uint16_t slesq_2 = get_latMarks->getChannel(1);
    uint16_t sldir_1 = get_latMarks->getChannel(2);
    uint16_t sldir_2 = get_latMarks->getChannel(3);

    //ESP_LOGI("SensorService", "Esquerda: %d %d; Direita: %d %d", values[0], values[1], values[2], values[3]);
    //ESP_LOGI(GetName().c_str(), "Esquerda: %d %d; Direita: %d %d", slesq_1, slesq_2, sldir_1, sldir_2);
    
    nLatReads++; 
    sumSensEsq += lower_value(slesq_1, slesq_2);
    sumSensDir += lower_value(sldir_1, sldir_2);

    if(get_Status->robotState->getData() == CAR_MAPPING)
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

        if (meanSensEsq < 950 || meanSensDir < 950)
        { // leitura de faixas nos sensores laterais
            if (!(get_Marks->latDirPass->getData()))
            {
                if(get_Status->robotState->getData() != CAR_STOPPED)
                {
                    get_Marks->rightPassedInc();
                }
                latState(true, false);
                //ESP_LOGI(GetName().c_str(), "Marcação direita");

                LED->LedComandSend(LED_POSITION_LEFT, COLOR_RED, 1);
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_RED, 1);
            }
        }
        else
        {
            if (get_Marks->latDirPass->getData() || get_Marks->latEsqPass->getData())
            {
                // Desligando as LEDs esquerda e direita
                LED->LedComandSend(LED_POSITION_RIGHT, COLOR_BLACK, 1);
                LED->LedComandSend(LED_POSITION_LEFT, COLOR_BLACK, 1);
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

/* void SensorService::auto_calibrate(int mux)
{// Calibracao automatica
    
    //ESP_LOGI(GetName().c_str(), "Início Calibração");
    
    MPR_Mot = get_Spec->MPR->getData();

    // Zera contagem dos encoders
    //encs.ResetCount();
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
        motors->enc_reset_count(); // Reseta a contagem para comecar outra volta
        while((get_Speed->EncMedia->getData()) < limit_enc) // enquanto o robô não andou um giro da roda
        {
            motors->enc_reset_count(); // atualiza a leitura dos encoders
            
            int32_t enc = get_Speed->EncMedia->getData();
            int32_t enc_es = get_Speed->EncLeft->getData();
            int32_t enc_di = get_Speed->EncRight->getData();
            ESP_LOGI(GetName().c_str(), "EncEsq = %d, EncDir = %d, EncMedia = %d", enc_es, enc_di, enc);
            

            // Anda para uma direcao por 1/4 da calibracao
            // Anda pra direcao contraria por 2/4
            // Volta a posicao original, andando o último 1/4 do tempo
            if(voltas%2 == 0)
            {
                // Chama a funcao do servico dos Motores para o robô andar reto
                motors->WalkStraight(get_Speed->vel_calibrate->getData(), 0);
            }
            else
            {
                // Chama a mesma funcao para o robô andar para o lado contraio
                motors->WalkStraight(get_Speed->vel_calibrate->getData(), 1);
            }

            // Escolhe qual sensor esta sendo calibrado:
            if(mux == 0){
                MUX.calibrate_all(sArray, sQuant); // Funcao que calibra os 16 sensores 1 vez cada
            }
            else{
                MUX.calibrate_all(sBody, 6); // Funcao que calibra os 16 sensores 1 vez cada
            }
            //vTaskDelay(10 / portTICK_PERIOD_MS); // Delay de 10 milisegundos
            motors->read_both();
        }
        motors->StopMotors();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    motors->stop_car();
    vTaskDelay(10 / portTICK_PERIOD_MS);
} */