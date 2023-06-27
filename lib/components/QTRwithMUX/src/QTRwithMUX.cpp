#include "QTRwithMUX.h"


void QTRwithMUX::calibrate_all(QTRSensors *sArray, int quant)
{
    // Função que calibra os sensores, um por vez
    for(int i=0; i < quant; i++)
    {
        selectMuxPin(std::bitset<4>(i)); // std::bitset<4> transforma i de decimal para binário
        sArray[i].calibrate();
    }
}

int16_t QTRwithMUX::read_all(QTRSensors *sArray, int quant, bool white_line)
{
    // Funcao baseada na funcao .readLineWhite da biblioteca QTRSensors
    // A funcao original nao pode ser usada pelas leituras não serem simultaneas

    bool on_Line = false; // se falso até o final, os sensores estão todos fora da linha
    uint32_t avg = 0;
    uint16_t sum = 0;

    for(int i=0; i < quant; i++)
    {
        uint16_t sensor_value;
        selectMuxPin(std::bitset<4>(i)); // Passa o valor i em formato 4 bits para selecionar o sensor
        sArray[i].readCalibrated(&sensor_value); // lê cada sensor como se fosse um array e salva em sensor_value
        
        if (white_line) { sensor_value = 1000 - sensor_value; } // inverte as leituras
        if (sensor_value > 200) { on_Line = true; } // verifica se tem um sensor na linha
        if (sensor_value > 50) // valores menores que 50 são considerados ruídos
        {
            avg += (uint32_t)sensor_value*(i*1000); // soma ponderada de cada leitura
            sum += sensor_value; // soma total das leituras
        }
    }
    
    if (!on_Line) // Se o robo esta fora da linha, retorna a direcao da ultima leitura
    {
     // Se a ultima posicao foi a esquerda do centro, returna 0.
     if (lastPosition < (sQuant - 1) * 1000 / 2)
     {
       return 0;
     }
     // Se a ultima posicao foi a direita do centro, retorna o valor maximo.
     else
     {
       return (sQuant - 1) * 1000;
     }
    }
    // Dividindo avg por sum, se obtém a posicao relativa do robô na linha
    // Com 16 sensores, o centro se encontra no valor 7.500
    lastPosition = avg/sum;
    return lastPosition;
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