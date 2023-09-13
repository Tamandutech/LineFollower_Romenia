#include "esp_log.h"
#include "QTRSensors.h"
#include "RobotData.h"
#include "bitset"// biblioteca que transforma um número decimal para binário
#include <rom/ets_sys.h> // para a função ets_delay_us


class QTRwithMUX
{
public:
    
    void calibrate_all(QTRSensors *sArray, int quant);
    void calibrate_body(QTRSensors *sArray, int quant);
    int16_t read_all(QTRSensors *sArray, int quant, bool white_line);
    void read_from_body(uint16_t *values, QTRSensors *sArray, int pin_init, int pin_end, bool white_line);
    void selectMuxPin(std::bitset<4> pin); // recebe um vetor binário

private:

    SemaphoreHandle_t xSemaphore;
    bool start_mux = false;
    uint16_t lastPosition;

};