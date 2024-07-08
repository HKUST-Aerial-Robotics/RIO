#pragma once
#include <cstdint>
namespace Driver
{
struct ARS548RadarStatus
{
    // -100 to 100 m
    float longitudinal;
    // -100 to 100 m
    float lateral;
    // -10 to 10 m
    float vertical;
    // -3.14159 to 3.14159 rad
    float yaw;
    // -1.5707 to -1.5707 rad
    float pitch;
    // 0 to 255, ARS548PlugOrientation
    uint8_t plugOrientation;
    // 0.01 to 100, this is vehicle setup
    float length;
    // 0.01 to 100, this is vehicle setup
    float width;
    // 0.01 to 100, this is vehicle setup
    float height;
    // 0.01 to 100, this is vehicle setup
    float wheelbase;
    // 93 to 1514 meter
    uint16_t maximumDistance;
    // 0 to 255, ARS548FrequencySlot
    uint8_t frequencySlot;
    // 0 to 255 represents for 50 to 100
    uint8_t cycleTime;
    // 0 to 255 represents for 10 to 99
    uint8_t timeSlot;
    // 0 to 255, country code, ARS548CountryCode
    uint8_t hcc;
    // 0 to 255, ARS548PowerSaveStandstillMode
    uint8_t powerSaveStandstill;
    // (0-255).(0-255).(0-255).(0-255)
    uint32_t sensorIPAddress0;
    // Reserved
    uint32_t sensorIPAddress1;
    // increase after configuration accepted
    uint8_t configurationCounter;
    // 0 to 255, ARS548VDYStatus
    uint8_t statusLongitudinalVelocity;
    // 0 to 255, ARS548VDYStatus
    uint8_t statusLongitudinalAcceleration;
    // 0 to 255, ARS548VDYStatus
    uint8_t statusLateralAcceleration;
    // 0 to 255, ARS548VDYStatus
    uint8_t statusYawRate;
    // 0 to 255, ARS548VDYStatus
    uint8_t statusSteeringAngle;
    // 0 to 255, ARS548VDYStatus
    uint8_t statusDrivingDirection;
    // 0 to 255, ARS548VDYStatus
    uint8_t statusCharacteristicSpeed;
    // 0 to 255, ARS548SensorStatus
    uint8_t statusRadar;
};
}  // namespace Driver