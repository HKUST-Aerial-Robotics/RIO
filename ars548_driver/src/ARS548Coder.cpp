#include <cstddef>
#include <cstdint>

#include "ARS548Coder.hpp"

namespace Driver
{
ARS548RadarObjectList ARS548Coder::decodeObjectListMessage(
    const uint8_t *data) const
{
    ARS548RadarObjectList result{};

    // decode header
    result.serviceID = byteArrayToUint16(data);
    // methodID should be 329
    result.methodID = byteArrayToUint16(data + 2);
    // dataLength should be 9401 - 8
    result.dataLength = byteArrayToUint32(data + 4);
    result.clientID = byteArrayToUint16(data + 8);
    result.sessionID = byteArrayToUint16(data + 10);
    result.protocolVersion = byteArrayToUint8(data + 12);
    result.interfaceVersion = byteArrayToUint8(data + 13);
    result.messageType = byteArrayToUint8(data + 14);
    result.returnCode = byteArrayToUint8(data + 15);

    // decoder body
    result.crc = byteArrayToInt64(data + 16);
    result.length = byteArrayToUint32(data + 24);
    result.sqc = byteArrayToUint32(data + 28);
    result.dataID = byteArrayToUint32(data + 32);
    result.timestampNs = byteArrayToUint32(data + 36);
    result.timestampS = byteArrayToUint32(data + 40);
    result.timestampSyncStatus = byteArrayToUint8(data + 44);
    result.eventDataQualifier = byteArrayToUint32(data + 45);
    result.extendedQualifier = byteArrayToUint8(data + 49);
    // number should less than 51
    result.objectListNumOfObjects = byteArrayToUint8(data + 50);

    // object list
    uint8_t *base = nullptr;
    for (int i = 0; i < result.objectListNumOfObjects; i++)
        result.objectArray[i] =
            decodeObjectMessage(data + static_cast<ptrdiff_t>(51 + i * 187));
    return result;
}

ARS548RadarObject ARS548Coder::decodeObjectMessage(const uint8_t *data) const
{
    ARS548RadarObject result{};
    result.statusSensor = byteArrayToUint16(data);
    result.id = byteArrayToUint32(data + 2);
    result.age = byteArrayToUint16(data + 6);
    result.statusMeasurement = byteArrayToUint8(data + 8);
    result.statusMovement = byteArrayToUint8(data + 9);
    result.positionInvalidFlags = byteArrayToUint16(data + 10);
    result.positionReference = byteArrayToUint8(data + 12);
    result.positionX = byteArrayToFloat(data + 13);
    result.positionXSTD = byteArrayToFloat(data + 17);
    result.positionY = byteArrayToFloat(data + 21);
    result.positionYSTD = byteArrayToFloat(data + 25);
    result.positionZ = byteArrayToFloat(data + 29);
    result.positionZSTD = byteArrayToFloat(data + 33);
    result.positionCovarianceXY = byteArrayToFloat(data + 37);
    result.positionOrientation = byteArrayToFloat(data + 41);
    result.positionOrientationSTD = byteArrayToFloat(data + 45);
    result.existenceInvalidFlags = byteArrayToUint8(data + 49);
    result.existenceProbability = byteArrayToFloat(data + 50);
    result.existencePPV = byteArrayToFloat(data + 54);
    result.classificationCar = byteArrayToUint8(data + 58);
    result.classificationTruck = byteArrayToUint8(data + 59);
    result.classificationMotorcycle = byteArrayToUint8(data + 60);
    result.classificationBicycle = byteArrayToUint8(data + 61);
    result.classificationPedestrian = byteArrayToUint8(data + 62);
    result.classificationAnimal = byteArrayToUint8(data + 63);
    result.classificationHazard = byteArrayToUint8(data + 64);
    result.classificationUnknown = byteArrayToUint8(data + 65);
    result.classificationOverdrivable = byteArrayToUint8(data + 66);
    result.classificationUnderdrivable = byteArrayToUint8(data + 67);
    result.dynamicsAbsVelInvalidFlags = byteArrayToUint8(data + 68);
    result.dynamicsAbsVelX = byteArrayToFloat(data + 69);
    result.dynamicsAbsVelXSTD = byteArrayToFloat(data + 73);
    result.dynamicsAbsVelY = byteArrayToFloat(data + 77);
    result.dynamicsAbsVelYSTD = byteArrayToFloat(data + 81);
    result.dynamicsAbsVelCovarianceXY = byteArrayToFloat(data + 85);
    result.dynamicsRelVelInvalidFlags = byteArrayToUint8(data + 89);
    result.dynamicsRelVelX = byteArrayToFloat(data + 90);
    result.dynamicsRelVelXSTD = byteArrayToFloat(data + 94);
    result.dynamicsRelVelY = byteArrayToFloat(data + 98);
    result.dynamicsRelVelYSTD = byteArrayToFloat(data + 102);
    result.dynamicsRelVelCovarianceXY = byteArrayToFloat(data + 106);
    result.dynamicsAbsAccelInvalidFlags = byteArrayToUint8(data + 110);
    result.dynamicsAbsAccelX = byteArrayToFloat(data + 111);
    result.dynamicsAbsAccelXSTD = byteArrayToFloat(data + 115);
    result.dynamicsAbsAccelY = byteArrayToFloat(data + 119);
    result.dynamicsAbsAccelYSTD = byteArrayToFloat(data + 123);
    result.dynamicsAbsAccelCovarianceXY = byteArrayToFloat(data + 127);
    result.dynamicsRelAccelInvalidFlags = byteArrayToUint8(data + 131);
    result.dynamicsRelAccelX = byteArrayToFloat(data + 132);
    result.dynamicsRelAccelXSTD = byteArrayToFloat(data + 136);
    result.dynamicsRelAccelY = byteArrayToFloat(data + 140);
    result.dynamicsRelAccelYSTD = byteArrayToFloat(data + 144);
    result.dynamicsRelAccelCovarianceXY = byteArrayToFloat(data + 148);
    result.dynamicsOrientationInvalidFlags = byteArrayToUint8(data + 152);
    result.dynamicsOrientationRateMean = byteArrayToFloat(data + 153);
    result.dynamicsOrientationRateSTD = byteArrayToFloat(data + 157);
    result.shapeLengthStatus = byteArrayToUint32(data + 161);
    result.shapeLengthEdgeInvalidFlags = byteArrayToUint8(data + 165);
    result.shapeLengthEdgeMean = byteArrayToFloat(data + 166);
    result.shapeLengthEdgeSTD = byteArrayToFloat(data + 170);
    result.shapeWidthStatus = byteArrayToUint32(data + 174);
    result.shapeWidthEdgeInvalidFlags = byteArrayToUint8(data + 178);
    result.shapeWidthEdgeMean = byteArrayToFloat(data + 179);
    result.shapeWidthEdgeSTD = byteArrayToFloat(data + 183);
    return result;
}

ARS548RadarDetectionList ARS548Coder::decodeDetectionListMessage(
    const uint8_t *data) const
{
    ARS548RadarDetectionList result{};
    // header
    result.serviceID = byteArrayToUint16(data);
    // method ID should be 336
    result.methodID = byteArrayToUint16(data + 2);
    // dataLength should be 35336 - 8
    result.dataLength = byteArrayToUint32(data + 4);
    result.clientID = byteArrayToUint16(data + 8);
    result.sessionID = byteArrayToUint16(data + 10);
    result.protocolVersion = byteArrayToUint8(data + 12);
    result.interfaceVersion = byteArrayToUint8(data + 13);
    result.messageType = byteArrayToUint8(data + 14);
    result.returnCode = byteArrayToUint8(data + 15);

    // body
    result.crc = byteArrayToUint64(data + 16);
    result.length = byteArrayToUint32(data + 24);
    result.sqc = byteArrayToUint32(data + 28);
    result.dataID = byteArrayToUint32(data + 32);
    result.timestampNs = byteArrayToUint32(data + 36);
    result.timestampS = byteArrayToUint32(data + 40);
    result.timestampSyncStatus = byteArrayToUint8(data + 44);
    result.eventDataQualifier = byteArrayToUint32(data + 45);
    result.extendedQualifier = byteArrayToUint8(data + 49);
    result.originInvalidFlags = byteArrayToUint32(data + 50);
    result.originX = byteArrayToFloat(data + 52);
    result.originXSTD = byteArrayToFloat(data + 56);
    result.originY = byteArrayToFloat(data + 60);
    result.originYSTD = byteArrayToFloat(data + 64);
    result.originZ = byteArrayToFloat(data + 68);
    result.originZSTD = byteArrayToFloat(data + 72);
    result.originRoll = byteArrayToFloat(data + 76);
    result.originRollSTD = byteArrayToFloat(data + 80);
    result.originPitch = byteArrayToFloat(data + 84);
    result.originPitchSTD = byteArrayToFloat(data + 88);
    result.originYaw = byteArrayToFloat(data + 92);
    result.originYawSTD = byteArrayToFloat(data + 96);
    result.listInvalidFlags = byteArrayToUint8(data + 100);
    result.listRadVelDomainMin = byteArrayToFloat(data + 35301);
    result.listRadVelDomainMax = byteArrayToFloat(data + 35305);
    result.listNumOfDetections = byteArrayToUint32(data + 35309);
    result.alnAzimuthCorrection = byteArrayToFloat(data + 35313);
    result.alnElevationCorrection = byteArrayToFloat(data + 35317);
    result.alnStatus = byteArrayToUint8(data + 35321);

    // detection list
    for (int i = 0; i < result.listNumOfDetections; i++)
        result.detectionArray[i] =
            decodeDetectionMessage(data + static_cast<ptrdiff_t>(101 + i * 44));

    return result;
}

ARS548RadarDetection ARS548Coder::decodeDetectionMessage(
    const uint8_t *data) const
{
    // x = range * cos(elevationAngle) * cos(azimuthAngle);
    // y = range * cos(elevationAngle) * sin(azimuthAngle);
    // z = range * sin(elevationAngle);

    ARS548RadarDetection result{};
    result.azimuthAngle = byteArrayToFloat(data);
    result.azimuthSTD = byteArrayToFloat(data + 4);
    result.invalidFlags = byteArrayToUint8(data + 8);
    result.elevationAngle = byteArrayToFloat(data + 9);
    result.elevationSTD = byteArrayToFloat(data + 13);
    result.range = byteArrayToFloat(data + 17);
    result.rangeSTD = byteArrayToFloat(data + 21);
    result.rangeRate = byteArrayToFloat(data + 25);
    result.rangeRateSTD = byteArrayToFloat(data + 29);
    result.rcs = byteArrayToInt8(data + 33);
    result.measurementID = byteArrayToUint16(data + 34);
    result.positivePredictiveValue = byteArrayToUint8(data + 36);
    result.classification = byteArrayToUint8(data + 37);
    result.multiTargetProbability = byteArrayToUint8(data + 38);
    result.objectID = byteArrayToUint32(data + 39);
    result.ambiguityFlag = byteArrayToUint8(data + 41);
    result.sortIndex = byteArrayToUint32(data + 42);
    return result;
}

ARS548RadarStatus ARS548Coder::decodeBasicStatusMessage(
    const uint8_t *data) const
{
    ARS548RadarStatus result{};
    uint16_t serviceID = 0;
    uint16_t methodID = 0;
    uint32_t length = 0;
    serviceID = byteArrayToUint16(data);
    // methodID should be 380
    methodID = byteArrayToUint16(data + 2);
    // length should be 69 - 8
    length = byteArrayToUint32(data + 4);

    result.longitudinal = byteArrayToFloat(data + 8);
    result.lateral = byteArrayToFloat(data + 12);
    result.vertical = byteArrayToFloat(data + 16);
    result.yaw = byteArrayToFloat(data + 20);
    result.pitch = byteArrayToFloat(data + 24);
    result.plugOrientation = byteArrayToUint8(data +28);
    result.length = byteArrayToFloat(data + 29);
    result.width = byteArrayToFloat(data + 33);
    result.height = byteArrayToFloat(data + 37);
    result.wheelbase = byteArrayToFloat(data + 41);
    result.maximumDistance = byteArrayToUint16(data + 45);
    result.frequencySlot = byteArrayToUint8(data +47);
    result.cycleTime = byteArrayToUint8(data +48);
    result.timeSlot = byteArrayToUint8(data +49);
    result.hcc = byteArrayToUint8(data +50);
    result.powerSaveStandstill = byteArrayToUint8(data +51);
    result.sensorIPAddress0 = byteArrayToUint32(data + 52);
    result.sensorIPAddress1 = byteArrayToUint32(data + 56);
    result.configurationCounter = byteArrayToUint8(data +60);
    result.statusLongitudinalVelocity = byteArrayToUint8(data +61);
    result.statusLongitudinalAcceleration = byteArrayToUint8(data +62);
    result.statusLateralAcceleration = byteArrayToUint8(data +63);
    result.statusYawRate = byteArrayToUint8(data +64);
    result.statusSteeringAngle = byteArrayToUint8(data +65);
    result.statusDrivingDirection = byteArrayToUint8(data +66);
    result.statusCharacteristicSpeed = byteArrayToUint8(data +67);
    result.statusRadar = byteArrayToUint8(data +68);

    return result;
}

}  // namespace Driver