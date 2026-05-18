#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <opencv2/opencv.hpp>
#include <cstdint>

#include "../calib/camToolKit/calibData.h"

using namespace cv;
using namespace std;

// Initialize camera calibration parameters
void cameraInit();

// Apply both intrinsics (CalibData) and BEV extrinsic together, and recreate maps
void applyMainCamCalibWithBevExtrinsic(const CalibData& data, const cv::Vec3d& rvec, const cv::Vec3d& tvec);

// Get overhead view image using OpenCV remap
Mat getOverHeadViewImage(const Mat& srcImg);

// Get big undistorted image using OpenCV remap  
Mat getBigUndistortImage(const Mat& srcImg);

// Get map data for GLES remap
void getOverHeadMaps(Mat& mapX, Mat& mapY);

void cameraSetOverheadXYOffset(double dxMm, double dyMm);

void cameraReloadConfAndRebuildMaps();

// Overhead map version counter (increments when maps are recreated)
uint64_t getOverHeadMapVersion();

// 厚度补偿相关函数
bool setMaterialThickness(double thicknessMm);

// 获取当前设置的厚�?
double getCurrentThickness();

// 检查厚度补偿是否启�?
bool isThicknessCompensationEnabled();

// 保存当前标定为厚度补偿基�?
bool saveAsThicknessBaseline(const CalibData& calib, const cv::Vec3d& rvec, const cv::Vec3d& tvec, double mmPerWorld);


// 厚度补偿测试接口（C接口�?
// int test_set_material_thickness(double thickness_mm);
// double test_get_current_thickness(void);
// int test_is_thickness_compensation_enabled(void);
// void test_thickness_compensation_status(void);


#endif // _CAMERA_H_
