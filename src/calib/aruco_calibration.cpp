#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "charuco_detector.h"
#include "saddleFit.h"

using namespace std;
using namespace cv;

// 处理单个文件夹中的所有图像进行相机标定
static bool calibrateCamera(const std::string& folderPath, const std::string& outputPrefix)
{
    std::cout << "\n=== Processing folder: " << folderPath << " ===" << std::endl;
    
    // 存储所有有效图像的角点
    std::vector<std::vector<cv::Point2f>> allImagePoints;
    std::vector<std::vector<cv::Point3f>> allWorldPoints;
    std::vector<std::string> validImages;
    
    // 查找文件夹中的所有JPG文件
    std::vector<cv::String> imageFiles;
    cv::glob(folderPath + "/*.jpg", imageFiles);
    
    if (imageFiles.empty()) {
        std::cerr << "No JPG files found in " << folderPath << std::endl;
        return false;
    }
    
    cv::Mat firstImage;
    cv::Size imageSize;
    
    for (const auto& imagePath : imageFiles) {
        std::cout << "\nProcessing: " << imagePath << std::endl;
        cv::Mat image = cv::imread(imagePath, 0);
        if (image.empty()) {
            std::cerr << "Failed to load: " << imagePath << std::endl;
            continue;
        }
        
        if (firstImage.empty()) {
            firstImage = image.clone();
            imageSize = image.size();
        }
        
        std::vector<cv::Point2f> imagePoints;
        std::vector<cv::Point3f> objectPoints;
        
        // 使用独立的Charuco检测接口
        int detected = CharucoMultiBoardDetector::detect(image, imagePoints, objectPoints);
        std::cout << "  Detected " << detected << " charuco corners" << std::endl;
        
        if (detected > 10) {
            ReallinkSaddlePointFit(image, imagePoints, 10);
            allImagePoints.push_back(imagePoints);
            allWorldPoints.push_back(objectPoints);
            validImages.push_back(imagePath);
        }
    }
    
    if (allImagePoints.size() < 3) {
        std::cerr << "Not enough valid images for calibration (need at least 3, got " 
                  << allImagePoints.size() << ")" << std::endl;
        return false;
    }
    
    std::cout << "\nCalibrating camera with " << allImagePoints.size() << " images..." << std::endl;
    
    // 相机标定
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    
    double rms = cv::calibrateCamera(allWorldPoints, allImagePoints, imageSize,
                                   cameraMatrix, distCoeffs, rvecs, tvecs);
    
    std::cout << "Calibration RMS error: " << rms << std::endl;
    std::cout << "Camera matrix:\n" << cameraMatrix << std::endl;
    std::cout << "Distortion coefficients: " << distCoeffs.t() << std::endl;
    
    // 保存标定结果
    cv::FileStorage fs(outputPrefix + "_calibration.yml", cv::FileStorage::WRITE);
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "rms_error" << rms;
    fs << "valid_images" << validImages;
    fs.release();
    
    std::cout << "Calibration saved to: " << outputPrefix << "_calibration.yml" << std::endl;
    
    // 测试去畸变效果
    if (!firstImage.empty()) {
        cv::Mat undistorted;
        cv::undistort(firstImage, undistorted, cameraMatrix, distCoeffs);
        cv::imwrite(outputPrefix + "_undistorted_sample.jpg", undistorted);
        std::cout << "Sample undistorted image saved to: " << outputPrefix << "_undistorted_sample.jpg" << std::endl;
    }
    
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "ArUco Charuco Camera Calibration Tool" << std::endl;
    std::cout << "Based on 6-board Charuco pattern (8x8, 80px squares, 48px markers)" << std::endl;
    
    // 要处理的6个文件夹
    std::vector<std::string> folders = {
        //"5M/leftTop", "5M/rightTop", "5M/leftCenter",
        //"5M/rightCenter", "5M/leftBottom", "5M/rightBottom"
        "5M/Total"
    };
    
    std::vector<std::string> prefixes = {
        //"calib_leftTop", "calib_rightTop", "calib_leftCenter",
        //"calib_rightCenter", "calib_leftBottom", "calib_rightBottom"
        "calib_Total"
    };
    
    bool anySuccess = false;
    
    for (size_t i = 0; i < folders.size(); ++i) {
        if (calibrateCamera(folders[i], prefixes[i])) {
            anySuccess = true;
        }
    }
    
    if (anySuccess) {
        std::cout << "\n=== Calibration Summary ===" << std::endl;
        std::cout << "Calibration files generated with prefix 'calib_*'" << std::endl;
        std::cout << "Each .yml file contains camera matrix and distortion coefficients" << std::endl;
    } else {
        std::cout << "\nNo successful calibrations performed." << std::endl;
    }
    
    return 0;
}
