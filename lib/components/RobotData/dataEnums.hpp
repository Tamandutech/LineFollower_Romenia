#ifndef DATA_ENUMS_HPP
#define DATA_ENUMS_HPP

enum CarSensor
{
    CAR_SENSOR_LEFT,
    CAR_SENSOR_RIGHT,
    CAR_SENSOR_FRONT,
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
    TUNING,
};

#endif