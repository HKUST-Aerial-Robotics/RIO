#pragma once

#include <netinet/in.h>

#include <cstdint>

inline uint8_t byteArrayToUint8(const uint8_t *array) { return *array; }

inline int8_t byteArrayToInt8(const uint8_t *array)
{
    return *reinterpret_cast<const int8_t *>(array);
}

inline uint16_t byteArrayToUint16(const uint8_t *array)
{
    return ntohs(*reinterpret_cast<const uint16_t *>(array));
}

inline int16_t byteArrayToInt16(const uint8_t *array)
{
    uint16_t temp = ntohs(*reinterpret_cast<const uint16_t *>(array));
    return *reinterpret_cast<int16_t *>(&temp);
}

inline uint32_t byteArrayToUint32(const uint8_t *array)
{
    return ntohl(*reinterpret_cast<const uint32_t *>(array));
}

inline int32_t byteArrayToInt32(const uint8_t *array)
{
    uint32_t temp = ntohl(*reinterpret_cast<const uint32_t *>(array));
    return *reinterpret_cast<int32_t *>(&temp);
}

inline uint64_t byteArrayToUint64(const uint8_t *array)
{
    return static_cast<uint64_t>(*array) << 56 |
           static_cast<uint64_t>(*(array + 1)) << 48 |
           static_cast<uint64_t>(*(array + 2)) << 40 |
           static_cast<uint64_t>(*(array + 3)) << 32 |
           static_cast<uint64_t>(*(array + 4)) << 24 |
           static_cast<uint64_t>(*(array + 5)) << 16 |
           static_cast<uint64_t>(*(array + 6)) << 8 |
           static_cast<uint64_t>(*(array + 7));
}

inline int64_t byteArrayToInt64(const uint8_t *array)
{
    uint64_t temp = static_cast<uint64_t>(*array) << 56 |
                    static_cast<uint64_t>(*(array + 1)) << 48 |
                    static_cast<uint64_t>(*(array + 2)) << 40 |
                    static_cast<uint64_t>(*(array + 3)) << 32 |
                    static_cast<uint64_t>(*(array + 4)) << 24 |
                    static_cast<uint64_t>(*(array + 5)) << 16 |
                    static_cast<uint64_t>(*(array + 6)) << 8 |
                    static_cast<uint64_t>(*(array + 7));
    return *reinterpret_cast<int64_t *>(&temp);
}

inline float byteArrayToFloat(const uint8_t *array)
{
    uint32_t temp = ntohl(*reinterpret_cast<const uint32_t *>(array));
    return *reinterpret_cast<float *>(&temp);
}

inline double byteArrayToDouble(const uint8_t *array)
{
    uint64_t temp = static_cast<uint64_t>(*array) << 56 |
                    static_cast<uint64_t>(*(array + 1)) << 48 |
                    static_cast<uint64_t>(*(array + 2)) << 40 |
                    static_cast<uint64_t>(*(array + 3)) << 32 |
                    static_cast<uint64_t>(*(array + 4)) << 24 |
                    static_cast<uint64_t>(*(array + 5)) << 16 |
                    static_cast<uint64_t>(*(array + 6)) << 8 |
                    static_cast<uint64_t>(*(array + 7));
    return *reinterpret_cast<double *>(&temp);
}

// inline void uint8ToByteArray(uint8_t value, uint8_t *array) { *array = value;
// }

// inline void int8ToByteArray(int8_t value, uint8_t *array)
// {
//     *reinterpret_cast<int8_t *>(array) = value;
// }

// inline void uint16ToByteArray(uint16_t value, uint8_t *array)
// {
//     *reinterpret_cast<uint16_t *>(array) = value;
// }

// inline void int16ToByteArray(int16_t value, uint8_t *array)
// {
//     *reinterpret_cast<int16_t *>(array) = value;
// }

// inline void uint32ToByteArray(uint32_t value, uint8_t *array)
// {
//     *reinterpret_cast<uint32_t *>(array) = value;
// }

// inline void int32ToByteArray(int32_t value, uint8_t *array)
// {
//     *reinterpret_cast<int32_t *>(array) = value;
// }

// inline void floatToByteArray(float value, uint8_t *array)
// {
//     *reinterpret_cast<float *>(array) = value;
// }