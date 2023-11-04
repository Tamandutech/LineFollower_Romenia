#include "QTRwithMUX.h"


void QTRwithMUX::calibrate_all(QTRSensors *sArray, int quant)
{ 
    
    for(int i=0; i < quant; i++)
    {
        selectMuxPin(std::bitset<4>(i)); // std::bitset<4> transforma i de decimal para binário
        //ESP_LOGI("QTRSensorMUX", "MUX %d selecionado.", i);
        sArray[i].calibrate();
        //ESP_LOGI("QTRSensorMUX", "Sensor %d calibrado.", i);
        
        vTaskSuspendAll();// Pausando as outras tasks para evitar conflitos com o delay usado
        ets_delay_us(1); // função que pausa o código por N microsegundos
        xTaskResumeAll(); // Retoma o funcionamento normal das tasks
        //vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_PERIOD_MS);
    }
}

int16_t QTRwithMUX::read_all(QTRSensors *sArray, int quant, bool white_line)
{
    // Funcao baseada na funcao .readLineWhite da biblioteca QTRSensors
    // A funcao original nao pode ser usada pelas leituras não serem simultaneas
    // white_line = TRUE -> leitura de linha branca

    bool on_Line = false; // se falso até o final, os sensores estão todos fora da linha
    uint32_t avg = 0; // soma ponderada das leituras
    uint16_t sum = 0; // soma das leituras
    //uint16_t teste_value[sQuant];
    
    
    for(int i=0; i < quant; i++)
    {
        uint16_t sensor_value;
        selectMuxPin(std::bitset<4>(i)); // Passa o valor i em formato 4 bits para selecionar o sensor
        //vTaskSuspendAll();// Pausando as outras tasks para evitar conflitos com o delay usado
        //ets_delay_us(1); // função que pausa o código por N microsegundos
        //xTaskResumeAll(); // Retoma o funcionamento normal das tasks
        sArray[i].readCalibrated(&sensor_value); // lê cada sensor como se fosse um array e salva em sensor_value
        if (white_line == WHITE) { sensor_value = 1000 - sensor_value; } // inverte as leituras
        //teste_value[i] = sensor_value;
        if (sensor_value > 200) { on_Line = true; } // verifica se tem um sensor na linha
        if (sensor_value > 50) // valores menores que 50 são considerados ruídos
        {
            avg += (uint32_t)sensor_value*(i*1000); // soma ponderada de cada leitura
            sum += sensor_value; // soma total das leituras
        }
        //vTaskSuspendAll();// Pausando as outras tasks para evitar conflitos com o delay usado
        //ets_delay_us(1); // função que pausa o código por N microsegundos
        //xTaskResumeAll(); // Retoma o funcionamento normal das tasks
    }
     
    /* ESP_LOGI("SensorService", "Leituras = %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", 
                                teste_value[0], teste_value[1], teste_value[2], teste_value[3], teste_value[4], 
                                teste_value[5], teste_value[6], teste_value[7], teste_value[8], teste_value[9], 
                                teste_value[10], teste_value[11], teste_value[12], teste_value[13], teste_value[14], 
                                teste_value[15]); */
    
    if (!on_Line) // Se o robo esta fora da linha, retorna a direcao da ultima leitura
    {
     // Se a ultima posicao foi a direita do centro, returna 0.
     if (lastPosition < (sQuant - 1) * 1000 / 2)
     {
       //ESP_LOGI("SensorService", "Ultima leitura no lado direito");
       return 0;
     }
     // Se a ultima posicao foi a esquerda do centro, retorna o valor maximo.
     else
     {
       //ESP_LOGI("SensorService", "Ultima leitura no lado direito");
       return ((sQuant - 1)*1000);
     }
    }
    else
    {
        // Dividindo avg por sum, se obtém a posicao relativa do robô na linha
        // Com 16 sensores, o centro se encontra no valor 7.500
        lastPosition = uint16_t(avg/sum);
        //cont_leituras++;
        //ESP_LOGI("SensorService", "Posição = %d, Leitura = %d, Lido %d vezes", lastPosition, sum, cont_leituras);
        return lastPosition;
    }
}

void QTRwithMUX::read_from_body(uint16_t *values, QTRSensors *sArray, int pin_init, int pin_end, bool white_line)
{// pin_init é o primeiro pino do mux em que os sensores estão conectado (de 0 a 15)
 // pin_end é o último (também de 0 a 15)
    int position = 0;
    for(int i = pin_init; i <= pin_end; i++)
    {
        uint16_t sensor_value;
        
        selectMuxPin(std::bitset<4>(i)); // Passa o valor i em formato 4 bits para selecionar o sensor
        
        vTaskSuspendAll();// Pausando as outras tasks para evitar conflitos com o delay usado
        ets_delay_us(1); // função que pausa o código por N microsegundos
        xTaskResumeAll(); // Retoma o funcionamento normal das tasks
        
        sArray[i].readCalibrated(&sensor_value); // lê cada sensor como se fosse um array e salva em sensor_value
        values[position] = sensor_value;
        if (white_line != WHITE) { values[position] = 1000 - values[position]; }

        vTaskSuspendAll();// Pausando as outras tasks para evitar conflitos com o delay usado
        ets_delay_us(1); // função que pausa o código por N microsegundos
        xTaskResumeAll(); // Retoma o funcionamento normal das tasks
        position++;
    }
    //ESP_LOGI("SensorService", "Esquerda: %d %d; Direita: %d %d", values[0], values[1], values[2], values[3]);
    
}

void QTRwithMUX::selectMuxPin(std::bitset<4> pin)
{
    // Função que controla qual sensor está conectado ao ESP32
    // Recebe um vetor com a sequencia de 4 bits de 0000 a 1111 e ajusta o nível lógico das portas digitais de acordo
    // 0010
    for(int i=0; i<pin.size(); i++)
    {
        if(pin[i]) { gpio_set_level((gpio_num_t)sPins[i], 1); } // se bit=1, nível lógico alto
        else { gpio_set_level((gpio_num_t)sPins[i], 0); } // se bit=0, nível lógico baixo
    }
}