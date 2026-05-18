#ifndef MAIN_CAMERA_DEFAULT_INTRINSICS_H
#define MAIN_CAMERA_DEFAULT_INTRINSICS_H

#include "../../calib/camToolKit/calibData.h"

inline CalibData makeDefaultMainCameraCalibData(int imageWidth, int imageHeight)
{
    (void)imageWidth;
    (void)imageHeight;

    CalibData calibData{};
    calibData.lensType = static_cast<uint8_t>(LENS_TYPE_FISHEYE);
    calibData.fx = 1731.201934327078;
    calibData.fy = 1731.67756088414;
    calibData.cx = 2154.73748314423;
    calibData.cy = 1551.947636766554;
    calibData.distCoeffs[0] = 0.4454115956484164;
    calibData.distCoeffs[1] = -0.02440432103448638;
    calibData.distCoeffs[2] = -0.1788392320878971;
    calibData.distCoeffs[3] = 0.1563695143314557;
    calibData.distCoeffs[4] = 0.0;
    calibData.checksum = calibdata_calc_checksum(&calibData);
    return calibData;
}

#endif // MAIN_CAMERA_DEFAULT_INTRINSICS_H
