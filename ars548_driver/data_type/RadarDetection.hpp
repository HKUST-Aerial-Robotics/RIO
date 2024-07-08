#pragma once
#include <cstdint>
namespace Driver
{

struct ARS548RadarDetection
{
    // -3.14 to 3.14 in rad
    float azimuthAngle;
    // 0 to 1
    float azimuthSTD;
    // 0 to 255, ARS548RadarDetectionFlags
    uint8_t invalidFlags;
    // -3.14 to 3.14 in rad
    float elevationAngle;
    // 0 to 1
    float elevationSTD;
    // 0 to 1500 in meter
    float range;
    // 0 to 1
    float rangeSTD;
    // -100 to 100 in meter/seconds
    float rangeRate;
    // 0 to 1
    float rangeRateSTD;
    // radar cross-section, -128 to 127 in dBm^2
    int8_t rcs;
    // 0 to 65535
    uint16_t measurementID;
    // 0 to 100, %
    uint8_t positivePredictiveValue;
    // 0 to 255, ARS548RadarDetectionClasses
    uint8_t classification;
    // 0 to 100, &
    uint8_t multiTargetProbability;
    // 0 to 65535
    uint16_t objectID;
    // 0 to 100, %, Probability for resolved velocity ambiguity
    uint8_t ambiguityFlag;
    // TBD
    uint16_t sortIndex;
};

struct ARS548RadarDetectionList
{
    // header
    uint16_t serviceID;

    uint16_t methodID;

    uint32_t dataLength;

    uint16_t clientID;

    uint16_t sessionID;

    uint8_t protocolVersion;

    uint8_t interfaceVersion;

    uint8_t messageType;

    uint8_t returnCode;
    
    // body
    uint64_t crc;

    uint32_t length;
    // Sequence counter incremented with each message.
    uint32_t sqc;

    uint32_t dataID;
    // 0 to 999999999
    uint32_t timestampNs;
    // 0 to 4294967295
    uint32_t timestampS;
    // 0 to 255, ARS548SyncState, unused
    uint8_t timestampSyncStatus;
    // 0 to 4294967295, unused
    uint32_t eventDataQualifier;
    // 0 to 255, unused
    uint8_t extendedQualifier;
    // 0 to 65535, unused
    uint16_t originInvalidFlags;
    // -10 to 10, meter, x position of radar
    float originX;
    // TBD, unused
    float originXSTD;
    // -10 to 10, meter, y position of radar
    float originY;
    // TBD, unused
    float originYSTD;
    // -10 to 10, meter, z position of radar
    float originZ;
    // TBD, unused
    float originZSTD;
    // -3.14 to 3.14 rad, unused
    float originRoll;
    // 0 to 0.1, unused
    float originRollSTD;
    // -3.14 to 3.14 rad
    float originPitch;
    // 0 to 0.1
    float originPitchSTD;
    // -3.14 to 3.14 rad
    float originYaw;
    // 0 to 0.1
    float originYawSTD;
    // 0 to 255, unused
    uint8_t listInvalidFlags;

    ARS548RadarDetection detectionArray[800];
    // -100 to 100, m/s , min ambiguity free Doppler velocity range
    float listRadVelDomainMin;
    // -100 to 100, m/s , max ambiguity free Doppler velocity range
    float listRadVelDomainMax;
    // 0 to 2047
    uint32_t listNumOfDetections;
    // -3.14 to 3.14, rad
    float alnAzimuthCorrection;
    // -3.14 to 3.14, rad
    float alnElevationCorrection;
    // alignment status, ARS548AlignmentStatus
    uint8_t alnStatus;
};

}  // namespace Driver