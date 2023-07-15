#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

#include <stdint.h>
#include <stddef.h>
#include <string>
#include <queue>
#include <atomic>
#include <mutex>

#include "dataEnums.h"
#include "IOs.hpp"
#include "dataMotor.h"
#include "dataSpec.h"
#include "dataPID.h"
#include "dataStatus.h"
#include "SLatMarks/dataSLatMarks.h"

#include "esp_log.h"

class Robot
{
public:
    static Robot *getInstance(std::string name = "RobotData")
    {
        Robot *sin = instance.load(std::memory_order_acquire);
        if (!sin)
        {
            std::lock_guard<std::mutex> myLock(instanceMutex);
            sin = instance.load(std::memory_order_relaxed);
            if (!sin)
            {
                sin = new Robot(name);
                instance.store(sin, std::memory_order_release);
            }
        }

        return sin;
    };

    dataMotor *getMotorData();
    dataSpec *getSpecification();
    dataPID *getPID();
    dataStatus *getStatus();
    dataSLatMarks *getSLatMarks();

    std::string GetName();

private:
    std::string name;

    static std::atomic<Robot *> instance;
    static std::mutex instanceMutex;

    dataMotor *MotorVel;
    dataSpec *RobotSpec;
    dataPID *PID;
    dataStatus *RobotStatus;

    Robot(std::string name);
};

#endif