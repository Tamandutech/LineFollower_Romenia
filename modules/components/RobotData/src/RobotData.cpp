#include "RobotData.h"

std::atomic<Robot *> Robot::instance;
std::mutex Robot::instanceMutex;

Robot::Robot(std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    //ESP_LOGD(name.c_str(), "Criando objeto: %s (%p)", name.c_str(), this);


    // Instânciando objetos componentes do Robô.
    //ESP_LOGD(name.c_str(), "Criando sub-objetos para o %s", "Robô");

    this->MotorVel = new dataMotor("Velocidade Motor");
    //ESP_LOGD(name.c_str(), "velocidade (%p)", this->MotorVel);
    this->RobotSpec = new dataSpec("Especificacoes Robo");
    //ESP_LOGD(name.c_str(), "velocidade (%p)", this->RobotSpec);
    this->PID = new dataPID("Valores PID");
    //ESP_LOGD(name.c_str(), "velocidade (%p)", this->PID);
    this->RobotStatus = new dataStatus(CAR_IN_LINE, "Estados RobotStatus"); // inicialmente se encontra numa linha
    //ESP_LOGD(name.c_str(), "RobotStatus (%p)", this->RobotStatus);
    this->sLatMarks = new dataSLatMarks("Marcacoes sLatMarks");
    //ESP_LOGD(name.c_str(), "sLatMarks (%p)", this->sLatMarks);
}

std::string Robot::GetName()
{
    return this->name;
}

dataMotor *Robot::getMotorData()
{
    return this->MotorVel;
}

dataSpec *Robot::getSpecification()
{
    return this->RobotSpec;
}

dataPID *Robot::getPID()
{
    return this->PID;
}

dataStatus *Robot::getStatus()
{
    return this->RobotStatus;
}

dataSLatMarks *Robot::getSLatMarks()
{
    return this->sLatMarks;
}

dataFloat *Robot::getFrontSensors()
{
    return this->FrontSensors;
}

dataUint16 *Robot::getFotoSensors(CarSensor which_sensor)
{
    if(which_sensor == SENSOR_CENTER)
    {
        return this->CenterSensors;
    }
    else
    {
        return this->LatSensors;
    }
}

dataInt32 *Robot::getFromIMU(CarIMU acc_or_gyr)
{
    if(acc_or_gyr == ACCELERATION)
    {
        return this->IMUacc;
    }
    else
    {
        return this->IMUgyr;
    }
}