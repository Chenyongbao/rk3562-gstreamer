#ifndef CALIB_DATA_H
#define CALIB_DATA_H

// NOTE: This header is shared by both C and C++ compilation units.
#include <stdint.h>
#include <stddef.h>

// Lens type definitions (stored as uint8_t in CalibData::lensType)
enum LensType {
    LENS_TYPE_PINHOLE = 1,
    LENS_TYPE_FISHEYE = 2,
};

// Calibration data structure (74 bytes)
#pragma pack(push, 1)
typedef struct CalibData {
    uint8_t lensType;       // 1 byte: lens type
    double fx;              // 8 bytes: focal length x
    double fy;              // 8 bytes: focal length y
    double cx;              // 8 bytes: principal point x
    double cy;              // 8 bytes: principal point y
    double distCoeffs[5];   // 40 bytes: distortion coefficients (4 for fisheye + 1 reserved)
    uint8_t checksum;       // 1 byte: XOR checksum (over all previous bytes)
} CalibData;
#pragma pack(pop)

// -------- Shared checksum helpers (C/C++) --------
static inline uint8_t calib_xor_checksum(const uint8_t* data, size_t len)
{
    if (!data || len == 0) return 0;
    uint8_t ck = 0;
    for (size_t i = 0; i < len; ++i) {
        ck ^= data[i];
    }
    return ck;
}

static inline uint8_t calibdata_calc_checksum(const CalibData* data)
{
    if (!data) return 0;
    // checksum is the last byte in the packed struct
    return calib_xor_checksum((const uint8_t*)data, sizeof(CalibData) - 1);
}

#endif // CALIB_DATA_H
