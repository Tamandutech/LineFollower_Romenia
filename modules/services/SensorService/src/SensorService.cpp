#include "SensorService.hpp"

SensorService::SensorService(std::string name, uint32_t stackDepth, UBaseType_t priority):Thread(name, stackDepth, priority)
{
    // Atalhos de dados:
    this->robot = Robot::getInstance();
    this->get_Vel = robot->getMotorData();
    this->get_Spec = robot->getSpecification();
    // Atalhos de servicos:
    this->control_motor = MotorService::getInstance();
    this->rpm = RPMService::getInstance();
    
    esp_log_level_set(name.c_str(), ESP_LOG_INFO);

    // Inicializacao dos sensores frontais
    // Todos sao controlados na mesma porta, porém cada objeto do vetor representa um sensor diferente
    // Para controlar cada sensor, é preciso mudar as portas digitais com o selectMuxPin()
    for(int i=0; i < sQuant; i++)
    {
        sArray[i].setTypeAnalogESP();
        sArray[i].setSensorPins((const adc1_channel_t[]){(adc1_channel_t)sInput}, 1);
        sArray[i].setSamplesPerSensor(5);
    }
    // Inicializacao dos sensores laterais
    sLat.setTypeAnalogESP();
    sLat.setSensorPins((const adc1_channel_t[]){(adc1_channel_t)s_lat_esq, (adc1_channel_t)s_lat_dir}, 2);
    sLat.setSamplesPerSensor(5);

    //Inicializacao dos sensores do corpo
    sLat.setTypeAnalogESP();
    sLat.setSensorPins((const adc1_channel_t[]){(adc1_channel_t)s_c_esq, (adc1_channel_t)s_c_dir}, 2);
    sLat.setSamplesPerSensor(5);

    // Setando todas as leituras anteriores como 0 (centralizado na reta)
    for(int i=0; i < sQuantReading; i++)
    {
        AngleArray[i] = 0;
    }

    // Calibracao
    auto_calibrate();
}

void SensorService::Run()
{
    ESP_LOGE("Sensor", "Inicio.");
    // Loop do servico
    
}

void SensorService::auto_calibrate()
{
    
    MPR_Mot = get_Spec->MPR->getData();
    // Calibracao automatica, em desenvolvimento
    

    // Zera contagem dos encoders
    rpm->ResetCount();

    // Calibração dos sensores frontais
    for(int mux=0; mux < 2; mux++)
    {
        for(int voltas=1; voltas <= 4; voltas++)
        {
            while((get_Vel->EncMedia->getData()) < MPR_Mot) // enquanto o robô não andou um giro da roda
            {
                // Anda para uma direcao por 1/4 da calibracao
                // Anda pra direcao contraria por 2/4
                // Volta a posicao original, andando o último 1/4 do tempo
                if((voltas<2)||((voltas > 3)))
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
                }else{
                    sLat.calibrate(); // calibracao normal dos sensores laterais
                }
                vTaskDelay(100 / portTICK_PERIOD_MS); // Delay de 10 milisegundos
                rpm->ReadBoth(); // atualiza a leitura dos encoders
            }
            rpm->ResetCount(); // Reseta a contagem para comecar outra volta
        }
    }
}

void SensorService::SaveAngle(float new_angle)
{
    for(int i=(sQuantReading-1); i > 0; i++){
        AngleArray[i] = AngleArray[i-1];
    }
    AngleArray[0] = new_angle;
}

void SensorService::AngleError()
{
    // Funcao que retorna o erro em radianos, sendo esse erro o angulo em relacao ao centro de movimento
    
    // Carregando as variaveis do RobotData, para facilitar a leitura
    bool is_white = get_Spec->WhiteLine->getData();
    uint16_t max_angle = get_Spec->MaxAngle->getData(); // em graus
    uint16_t radius = get_Spec->RadiusSensor->getData();
    uint16_t dis_center = get_Spec->SensorToCenter->getData();
    
    max_angle = max_angle*M_PI/180; // converte graus em rad, sendo M_PI o valor de pi da biblioteca "cmath"

    int16_t position = MUX.read_all(sArray, sQuant, is_white); // le os sensores e recebe uma posicao
    // position vai de 0 ate (sQuant - 1)*1000, sendo sQuant a quantidade de sensores
    // Para 16 sensores, vai de 0 a 15000
    position = position - (sQuant - 1)*500; // subtrai a metade do valor máximo, posicao central fica em 0

    
    float angle_radius = position * max_angle/((sQuant - 1)*500); // converte a posicao para angulo, regra de 3s
    
    float angle_with_center = atan((sin(angle_radius))/(cos(angle_radius) -1 +(dis_center/radius)));
    
    // Salvando esse angulo na variavel do servico:
    SaveAngle(angle_with_center);
}
