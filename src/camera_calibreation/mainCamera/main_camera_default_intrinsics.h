#ifndef MAIN_CAMERA_DEFAULT_INTRINSICS_H
#define MAIN_CAMERA_DEFAULT_INTRINSICS_H

#include "../../calib/camToolKit/calibData.h"

// 提供主相机鱼眼标定的默认内参，用于 EEPROM 参数异常时兜底。
inline CalibData makeDefaultMainCameraCalibData(int imageWidth, int imageHeight)
{
    (void)imageWidth;
    (void)imageHeight;

    CalibData calibData{};
    calibData.lensType = static_cast<uint8_t>(LENS_TYPE_FISHEYE);
    calibData.fx = 1729.74;
    calibData.fy = 1730.21;
    calibData.cx = 2165.11;
    calibData.cy = 1569.28;
    calibData.distCoeffs[0] = 0.45109;    // k1
    calibData.distCoeffs[1] = -0.0601574;  // k2
    calibData.distCoeffs[2] = -0.0811439;   // k3
    calibData.distCoeffs[3] =  0.0680583;   // k4
    calibData.distCoeffs[4] = 0.0;
    calibData.checksum = calibdata_calc_checksum(&calibData);
    return calibData;
}

#endif // MAIN_CAMERA_DEFAULT_INTRINSICS_H
