#pragma once

#include <cstdint>
namespace Driver
{
struct ARS548RadarObject
{
    // 0 to 65535, TBD
    uint16_t statusSensor;
    // 0 to 4294967295
    uint32_t id;
    // 0 to 65535
    uint16_t age;
    // 0 to 255, ARS548ObjectStatus
    uint8_t statusMeasurement;
    // 0 to 255, ARS548MovementStatus
    uint8_t statusMovement;
    // 0 to 65535, TBD
    uint16_t positionInvalidFlags;
    // 0 to 255, ARS548PositionReference
    uint8_t positionReference;
    // meter
    float positionX;
    // meter
    float positionXSTD;
    // meter
    float positionY;
    // meter
    float positionYSTD;
    // meter
    float positionZ;
    // meter
    float positionZSTD;
    // covariance m^2
    float positionCovarianceXY;
    // rad
    float positionOrientation;
    // rad
    float positionOrientationSTD;
    // 0 to 255, unused
    uint8_t existenceInvalidFlags;
    // 0 to 100, %
    float existenceProbability;
    // 0 to 100, %, unused
    float existencePPV;
    // 0 to 100, %
    uint8_t classificationCar;
    // 0 to 100, %
    uint8_t classificationTruck;
    // 0 to 100, %
    uint8_t classificationMotorcycle;
    // 0 to 100, %
    uint8_t classificationBicycle;
    // 0 to 100, %
    uint8_t classificationPedestrian;
    // 0 to 100, %
    uint8_t classificationAnimal;
    // 0 to 100, %
    uint8_t classificationHazard;
    // 0 to 100, %
    uint8_t classificationUnknown;
    // 0 to 100, %, unused
    uint8_t classificationOverdrivable;
    // 0 to 100, %, unused
    uint8_t classificationUnderdrivable;
    // 0 to 255, unused
    uint8_t dynamicsAbsVelInvalidFlags;
    // m/s
    float dynamicsAbsVelX;
    // m/s
    float dynamicsAbsVelXSTD;
    // m/s
    float dynamicsAbsVelY;
    // m/s
    float dynamicsAbsVelYSTD;
    // (m/s)^2
    float dynamicsAbsVelCovarianceXY;
    // 0 to 255, unused
    uint8_t dynamicsRelVelInvalidFlags;
    // m/s
    float dynamicsRelVelX;
    // m/s
    float dynamicsRelVelXSTD;
    // m/s
    float dynamicsRelVelY;
    // m/s
    float dynamicsRelVelYSTD;
    // (m/s)^2
    float dynamicsRelVelCovarianceXY;
    // 0 to 255, unused
    uint8_t dynamicsAbsAccelInvalidFlags;
    // m/s^2
    float dynamicsAbsAccelX;
    // m/s^2
    float dynamicsAbsAccelXSTD;
    // m/s^2
    float dynamicsAbsAccelY;
    // m/s^2
    float dynamicsAbsAccelYSTD;
    // (m/s^2)^2
    float dynamicsAbsAccelCovarianceXY;
    // 0 to 255, unused
    uint8_t dynamicsRelAccelInvalidFlags;
    // m/s^2
    float dynamicsRelAccelX;
    // m/s^2
    float dynamicsRelAccelXSTD;
    // m/s^2
    float dynamicsRelAccelY;
    // m/s^2
    float dynamicsRelAccelYSTD;
    // (m/s^2)^2
    float dynamicsRelAccelCovarianceXY;
    // 0 to 255, unused
    uint8_t dynamicsOrientationInvalidFlags;
    // rad/s
    float dynamicsOrientationRateMean;
    // rad/s
    float dynamicsOrientationRateSTD;
    // 0 to 4294967295, ARS548ShapeStatus, unused
    uint32_t shapeLengthStatus;
    // 0 to 255, unused
    uint8_t shapeLengthEdgeInvalidFlags;
    // meter
    float shapeLengthEdgeMean;
    // meter unused
    float shapeLengthEdgeSTD;
    // 0 to 4294967295, ARS548ShapeStatus, unused
    uint32_t shapeWidthStatus;
    // 0 to 255, unused
    uint8_t shapeWidthEdgeInvalidFlags;
    // meter
    float shapeWidthEdgeMean;
    // meter unused
    float shapeWidthEdgeSTD;
};

struct ARS548RadarObjectList
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
    int64_t crc;

    uint32_t length;
    // Sequence counter incremented with each message.
    uint32_t sqc;

    uint32_t dataID;
    // 0 to 999999999, ns
    uint32_t timestampNs;
    // 0 to 4294967295
    uint32_t timestampS;
    // 0 to 255, ARS548SyncState, unused
    uint8_t timestampSyncStatus;
    // 0 to 4294967295, unused
    uint32_t eventDataQualifier;
    // 0 to 255, unused
    uint8_t extendedQualifier;

    uint8_t objectListNumOfObjects;

    ARS548RadarObject objectArray[50];
};

}  // namespace Driver