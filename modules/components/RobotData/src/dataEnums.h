#ifndef DATA_ENUMS_HPP
#define DATA_ENUMS_HPP

enum CarState
{
    CAR_MAPPING,
    CAR_ENC_READING,
    CAR_ENC_READING_BEFORE_FIRSTMARK,
    CAR_TUNING,
    CAR_STOPPED,
};

enum TrackSegment
{
    SHORT_LINE = 2,
    MEDIUM_LINE = 1,
    LONG_LINE = 0,
    XLONG_LINE = 11,

    SHORT_CURVE = 5,
    MEDIUM_CURVE = 4,
    LONG_CURVE = 3,
    XLONG_CURVE = 12,

    ZIGZAG_TRACK = 6,
    SPECIAL_TRACK = 7,
    DEFAULT_TRACK = 13
};

enum CarSensor
{
    SENSOR_CENTER,
    SENSOR_SIDE,
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

enum LineColor
{
    WHITE,
    BLACK,
};

#endif