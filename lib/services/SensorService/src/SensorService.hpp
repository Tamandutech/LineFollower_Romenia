#ifndef SENSOR_SERVICE_HPP
#define SENSOR_SERVICE_HPP

#include "thread.hpp"
#include "esp_log.h"
#include "Injector/singleton.hpp"
#include "QTRSensors.h"
#include "IOs.hpp"
#include "bitset"// biblioteca que transforma um número decimal para binário
#include "QTRwithMUX.h"


using namespace cpp_freertos;

class SensorService : public Thread, public Singleton<SensorService>
{
public:
    SensorService(std::string name, uint32_t stackDepth, UBaseType_t priority);

    void Run() override;

private:

    QTRSensors sArray[sQuant];

    void auto_calibrate();
};

#endif