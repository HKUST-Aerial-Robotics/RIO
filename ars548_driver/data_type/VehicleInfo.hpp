#pragma once

#include <cstdint>
namespace Driver
{
struct VehicleAccLateral
{
    // unused
    float accelerationLateralErrAmp;
    // unused
    uint8_t accelerationLateralErrAmpInvalidFlag;
    // unused
    uint8_t qualifierAccelerationLateral;
    // -65 TO 65 m/s^2
    float accelerationLateral;
    // unused
    uint8_t accelerationLateralInvalidFlag;
    // unused
    uint8_t accelerationLateralEventDataQualifier;
};

struct VehicleAccLongitudinal
{
    // unused
    float accelerationLongitudinalErrAmp;
    // unused
    uint8_t accelerationLongitudinalErrAmpInvalidFlag;
    // unused
    uint8_t qualifierAccelerationLongitudinal;
    // -65 TO 65 m/s^2
    float accelerationLongitudinal;
    // unused
    uint8_t accelerationLongitudinalInvalidFlag;
    // unused
    uint8_t accelerationLongitudinalEventDataQualifier;
};

struct VehicleCharacteristicSpeed
{
    // unused
    uint8_t characteristicSpeedErrAmp;
    // unused
    uint8_t qualifierCharacteristicSpeed;
    // 0 to 255, km/h
    uint8_t characteristicSpeed;
};

struct VehicleDrivingDirection
{
    // unused
    uint8_t drivingDirectionUnconfirmed;
    // ARS548VehicleDirection
    uint8_t drivingDirectionConfirmed;
};

struct VehicleSteeringAngle
{
    // unused
    uint8_t qualifierSteeringAngleFrontAxle;
    // unused
    float steeringAngleFrontAxleErrAmp;
    // unused
    uint8_t steeringAngleFrontAxleErrAmpInvalidFlag;
    // -90 to 90, degree
    float steeringAngleFrontAxle;
    // unused
    uint8_t steeringAngleFrontAxleInvalidFlag;
    // unused
    uint8_t steeringAngleFrontAxleEventDataQualifier;
};

struct VehicleVelocity
{
    // unused
    uint8_t statusVelocityNearStandstill;
    // unused
    uint8_t qualifierVelocityVehicle;
    // unused
    uint8_t velocityVehicleEventDataQualifier;
    // 0 to 350 km/h
    float velocityVehicle;
    // unused
    uint8_t velocityVehicleInvalidFlag;
};

struct VehicleYawRate
{
    // unused
    float yawRateErrAmp;
    // unused
    uint8_t yawRateErrAmpInvalidFlag;
    // unused
    uint8_t qualifierYawRate;
    // -163.84 to 163.83 deg/s
    float yawRate;
    // unused
    uint8_t yawRateInvalidFlag;
    // unused
    uint8_t yawRateEventDataQualifier;
};
}  // namespace Driver