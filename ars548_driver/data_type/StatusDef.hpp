#pragma once

namespace Driver
{

enum ARS548RadarDetectionFlags
{
    VALID_ALL = 0x00,
    INVALID_DISTANCE = 0x01,
    INVALID_DISTANCE_STD = 0x02,
    INVALID_AZIMUTH = 0x04,
    INVALID_AZIMUTH_STD = 0x08,
    INVALID_ELEVATION = 0x10,
    INVALID_ELEVATION_STD = 0x20,
    INVALID_RANGE_RATE = 0x40,
    INVALID_RANGE_RATE_STD = 0x80
};
enum ARS548RadarDetectionClasses
{
    NO_CLASSIFICATION = 0,
    CLASS_NOISE = 1,
    CLASS_GROUND = 2,
    CLASS_TRAVERSABLE_UNDER = 3,
    CLASS_OBSTACLE = 4,
    CLASS_INVALID = 255
};

enum ARS548SyncStatus
{
    SYNC_OK = 1,
    SYNC_NEVER_SYNC = 2,
    SYNC_LOST = 3,
};

enum ARS548AlignmentStatus
{
    ALIGNMENT_INIT = 0,
    ALIGNMENT_OK = 1,
    ALIGNMENT_NOTOK = 2,
};

enum ARS548ObjectStatus
{
    OBJECT_MEASURED = 0,
    OBJECT_NEW = 1,
    OBJECT_PREDICTED = 2,
    OBJECT_MEASUREMENT_INVALID = 255,
};

enum ARS548MovementStatus
{
    OBJECT_MOVED = 0,
    OBJECT_STATIONARY = 1,
    OBJECT_MOVEMENT_INVALID = 255,
};

enum ARS548PositionReference
{
    REFERENCE_CORNER_FRONT_LEFT = 0,
    REFERENCE_MIDDLE_FRONT = 1,
    REFERENCE_CORNER_FRONT_RIGHT = 2,
    REFERENCE_MIDDLE_SIDE_RIGHT = 3,
    REFERENCE_CORNER_REAR_RIGHT = 4,
    REFERENCE_MIDDLE_REAR = 5,
    REFERENCE_CORNER_REAR_LEFT = 6,
    REFERENCE_MIDDLE_SIDE_LEFT = 7,
    REFERENCE_SIGNAL_UNFILLED = 255,
};

enum ARS548ShapeStatus
{
    SHAPE_COMPLETELY_VISIBLE = 0,
    SHAPE_PARTIALLY_OCCLUDED = 1,
    SHAPE_COMPLETELY_OCCLUDED = 2,
    SHAPE_INVALID = 255,
};

enum ARS548PlugOrientation
{
    PLUG_RIGHT = 0,
    PLUG_LEFT = 1,
};

enum ARS548FrequencySlot
{
    // 76.23 GHz
    LOW = 0,
    // 76.48 GHz
    MID = 1,
    //  76.73 GHz
    HIGH = 2,
};

enum ARS548CountryCode
{
    WORLDWIDE = 1,
    JAPAN = 2,
};

enum ARS548PowerSaveStandstillMode
{
    OFF = 0,
    ON = 1,
};

enum ARS548VDYStatus
{
    VDY_OK = 0,
    VDY_NOTOK = 1,
};

enum ARS548SensorStatus
{
    SENSOR_STATE_INIT = 0,
    SENSOR_STATE_OK = 1,
    SENSOR_STATE_INVALID = 2,
};

enum ARS548ParameterConfiguration
{
    IGNORE_PARAMETERS = 0,
    USE_PARAMETERS = 1,
};

enum ARS548VehicleDirection
{
    DIRECTION_VOID = 0,
    DIRECTION_FORWARD = 1,
    DIRECTION_BACKWARDS = 2,
};

}  // namespace Driver