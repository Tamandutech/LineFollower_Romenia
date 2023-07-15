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
    TUNNING,
};

enum LineColor
{
    WHITE,
    BLACK,
};

#endif