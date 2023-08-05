#ifndef DATA_ENUMS_HPP
#define DATA_ENUMS_HPP

enum CarState
{
    CAR_IN_CURVE,
    CAR_IN_LINE,
    CAR_STOPPED,
    CAR_TUNING,
};

enum CarSensor
{
    SENSOR_CENTER,
    SENSOR_SIDE,
    SENSOR_FRONT,
};

enum CarIMU
{
    ACCELERATION,
    GYROSCOPE,
};

enum CarMassCenter
{
    CAR_CENTERED,
    CAR_TO_THE_RIGHT,
    CAR_TO_THE_LEFT,
};

enum DataFunction
{
    RETORNO_OK,
    RETORNO_ERRO,
    RETORNO_ERRO_GENERICO,
    RETORNO_ARGUMENTO_INVALIDO,
    RETORNO_VARIAVEL_OCUPADA
};

enum TrackState
{
    LONG_LINE,
    MEDIUM_LINE,
    SHORT_LINE,
    LONG_CURVE,
    MEDIUM_CURVE,
    SHORT_CURVE,
    ZIGZAG,
    SPECIAL_TRACK,
    DEFAULT_TRACK,
    UNDEFINED, 
    TUNNING,
};

enum LineColor
{
    WHITE,
    BLACK,
};

#endif