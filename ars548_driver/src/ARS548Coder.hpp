#pragma once
#include <cstdint>
#include <iostream>
#include "../data_type/RadarDetection.hpp"
#include "../data_type/RadarObject.hpp"
#include "../data_type/RadarStatus.hpp"
#include "../data_type/StatusDef.hpp"
#include "../data_type/TypeConverter.hpp"

namespace Driver
{
class ARS548Coder
{
   public:
    ARS548Coder() = default;
    ~ARS548Coder() = default;
    ARS548Coder(ARS548Coder &another) = delete;
    ARS548Coder(ARS548Coder &&another) = delete;
    ARS548Coder &operator=(const ARS548Coder &another) = delete;
    ARS548Coder &operator=(ARS548Coder &&another) = delete;

    ARS548RadarObjectList decodeObjectListMessage(const uint8_t *data) const;
    ARS548RadarObject decodeObjectMessage(const uint8_t *data) const;
    ARS548RadarDetectionList decodeDetectionListMessage(const uint8_t *data) const;
    ARS548RadarDetection decodeDetectionMessage(const uint8_t *data) const;
    ARS548RadarStatus decodeBasicStatusMessage(const uint8_t *data) const;
};
}  // namespace Driver