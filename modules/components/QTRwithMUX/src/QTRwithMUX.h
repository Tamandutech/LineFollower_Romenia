#include "esp_log.h"
#include "QTRSensors.h"
#include "RobotData.h"
#include "bitset"// biblioteca que transforma um número decimal para binário
#include <rom/ets_sys.h> // para a função ets_delay_us


class QTRwithMUX
{
public:
    
    void calibrate_all(QTRSensors *sArray, int quant);
    int16_t read_all(QTRSensors *sArray, int quant, bool white_line);
    void selectMuxPin(std::bitset<4> pin); // recebe um vetor binário

private:

    uint16_t lastPosition;

};