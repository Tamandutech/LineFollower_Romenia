#include "SensorService.hpp"

SensorService::SensorService(std::string name, uint32_t stackDepth, UBaseType_t priority):Thread(name, stackDepth, priority)
{
    // Atalhos:
    this->robot = Robot::getInstance();
    this->get_Vel = robot->getMotorData();
    this->get_Spec = robot->getSpecification();
    this->motor = MotorService::getInstance();
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

    rev = get_Spec->Revolution->getData();
    gear = get_Spec->GearRatio->getData();
    
    get_Spec->MPR->setData((rev*gear));
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
    for(int voltas=1; voltas <= 4; voltas++)
    {
        while((get_Vel->EncMedia->getData()) < MPR_Mot) // enquanto o robô não andou um giro da roda
        {
            // Anda para uma direcao por 1/4 do tempo de calibracao
            // Anda pra direcao contraria por 2/4 do tempo
            // Volta a posicao original, andando o último 1/4 do tempo de volta
            if((voltas<2)||((voltas > 3)))
            {
                // Chama a funcao do servico dos Motores para o robô andar reto
                motor->WalkStraight(get_Vel->vel_calibrate->getData(), 0);
            }
            else
            {
                // Chama a mesma funcao para o robô andar para o lado contraio
                motor->WalkStraight(get_Vel->vel_calibrate->getData(), 1);
            }
            
            MUX.calibrate_all(sArray, sQuant); // Funcao que calibra os 16 sensores 1 vez cada
            
            vTaskDelay(100 / portTICK_PERIOD_MS); // Delay de 10 milisegundos
            rpm->ReadBoth(); // atualiza a leitura dos encoders
        }
        rpm->ResetCount(); // Reseta a contagem para comecar outra volta
    }

    // Calibração dos sensores laterais
    for(int voltas=1; voltas <= 4; voltas++)
    {
        while((get_Vel->EncMedia->getData()) < MPR_Mot)
        {
            if((voltas<2)||((voltas > 3)))
            {
                motor->WalkStraight(get_Vel->vel_calibrate->getData(), 0);
            }
            else
            {
                motor->WalkStraight(get_Vel->vel_calibrate->getData(), 1);
            }
            
           sLat.calibrate();
            
            vTaskDelay(100 / portTICK_PERIOD_MS); // Delay de 10 milisegundos
            rpm->ReadBoth(); // atualiza a leitura dos encoders
        }
        rpm->ResetCount(); // Reseta a contagem para comecar outra volta
    }
}
