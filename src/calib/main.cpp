

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <filesystem>
#include "charuco_detector.h"
#include "saddleFit.h"
#include "calibinit.h"
#include "ChessboardDetector.h"
using namespace std;
using namespace cv;
static void fisheyeInitMap(InputArray _rvec, InputArray _tvec, InputArray _K, InputArray _D, const cv::Size& size, int m1type, OutputArray map1, OutputArray map2)
{
	Vec3d om = _rvec.depth() == CV_32F ? (Vec3d)*_rvec.getMat().ptr<Vec3f>() : *_rvec.getMat().ptr<Vec3d>();
	Vec3d T = _tvec.depth() == CV_32F ? (Vec3d)*_tvec.getMat().ptr<Vec3f>() : *_tvec.getMat().ptr<Vec3d>();
	map1.create(size, m1type <= 0 ? CV_16SC2 : m1type);
	map2.create(size, map1.type() == CV_16SC2 ? CV_16UC1 : CV_32F);
	cv::Vec2d f, c;
	if (_K.depth() == CV_32F)
	{
		Matx33f K = _K.getMat();
		f = Vec2f(K(0, 0), K(1, 1));
		c = Vec2f(K(0, 2), K(1, 2));
	}
	else
	{
		Matx33d K = _K.getMat();
		f = Vec2d(K(0, 0), K(1, 1));
		c = Vec2d(K(0, 2), K(1, 2));
	}

	Vec4d k = _D.depth() == CV_32F ? (Vec4d)*_D.getMat().ptr<Vec4f>() : *_D.getMat().ptr<Vec4d>();
	Affine3d aff(om, T);
	for (int i = 0; i < size.height; ++i)
	{
		float* m1f = map1.getMat().ptr<float>(i);
		float* m2f = map2.getMat().ptr<float>(i);
		short* m1 = (short*)m1f;
		ushort* m2 = (ushort*)m2f;
		for (int j = 0; j < size.width; ++j) {
			Vec3d Xi(j, i, 0);
			Vec3d Y = aff * Xi;

			Vec2d x(Y[0] / Y[2], Y[1] / Y[2]);

			double r2 = x.dot(x);
			double r = std::sqrt(r2);

			// Angle of the incoming ray:
			double theta = atan(r);

			double theta2 = theta * theta, theta3 = theta2 * theta, theta4 = theta2 * theta2, theta5 = theta4 * theta,
				theta6 = theta3 * theta3, theta7 = theta6 * theta, theta8 = theta4 * theta4, theta9 = theta8 * theta;

			double theta_d = theta + k[0] * theta3 + k[1] * theta5 + k[2] * theta7 + k[3] * theta9;

			double inv_r = r > 1e-8 ? 1.0 / r : 1;
			double cdist = r > 1e-8 ? theta_d * inv_r : 1;

			Vec2d xd1 = x * cdist;
			Vec2d xd3(xd1[0], xd1[1]);

			double u = xd3[0] * f[0] + c[0];
			double v = xd3[1] * f[1] + c[1];
			if (m1type == CV_16SC2)
			{
				int iu = cv::saturate_cast<int>(u * cv::INTER_TAB_SIZE);
				int iv = cv::saturate_cast<int>(v * cv::INTER_TAB_SIZE);
				m1[j * 2 + 0] = (short)(iu >> cv::INTER_BITS);
				m1[j * 2 + 1] = (short)(iv >> cv::INTER_BITS);
				m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE - 1)) * cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE - 1)));
			}
			else if (m1type == CV_32FC1)
			{
				m1f[j] = (float)u;
				m2f[j] = (float)v;
			}
		}
	}
}

static void showDetectedBoards(const string& windowPrefix,
	const Mat& grayImg,
	const vector<BoardDetectionResult>& boards,
	const Size& patternSize)
{
	if (boards.empty())
	{
		cout << windowPrefix << ": no boards to visualize" << endl;
		return;
	}
	Mat vis;
	if (grayImg.channels() == 1)
	{
		cvtColor(grayImg, vis, COLOR_GRAY2BGR);
	}
	else
	{
		vis = grayImg.clone();
	}
	const vector<Scalar> palette = {
		Scalar(0, 255, 0),
		Scalar(0, 0, 255),
		Scalar(255, 0, 0),
		Scalar(0, 255, 255),
		Scalar(255, 0, 255),
		Scalar(255, 255, 0)
	};
	for (size_t i = 0; i < boards.size(); ++i)
	{
		const auto& board = boards[i];
		Scalar color = palette[i % palette.size()];
		if (board.roi.area() > 0)
		{
			rectangle(vis, board.roi, color, 2);
			string label = "B" + to_string(i) + " (" + to_string(board.rows) + "x" +
				to_string(board.cols) + "/" + to_string(board.pointCount) + ")";
			Point textOrg(board.roi.x, std::max(board.roi.y - 5, 15));
			putText(vis, label, textOrg, FONT_HERSHEY_SIMPLEX, 2, color, 8);
		}
		bool isFullPattern = board.patternMatched &&
			board.corners.size() == static_cast<size_t>(patternSize.area());
		for (size_t cIdx = 0; cIdx < board.corners.size(); ++cIdx)
		{
			const Point2f& pt = board.corners[cIdx];
			circle(vis, pt, isFullPattern ? 5 : 4, color, FILLED);
			/*if (isFullPattern && cIdx % patternSize.width == 0)
			{
				string cornerIdx = to_string(static_cast<int>(cIdx));
				putText(vis, cornerIdx, pt + Point2f(3.f, -3.f), FONT_HERSHEY_PLAIN, 1.0, color, 1);
			}*/
		}
	}
	string windowName = windowPrefix + "_boards";
	namedWindow(windowName, WINDOW_NORMAL);
	imshow(windowName, vis);
	waitKey(0);
}

/**
 * @brief 显示图像坐标和世界坐标（两个独立窗口，每个角点都标记索引）
 * @param windowPrefix 窗口前缀
 * @param grayImg 灰度图像
 * @param boards 检测到的板子列表
 * @param patternSize 棋盘格模式尺寸
 */
static void visualizeBoardCorners(const string& windowPrefix,
	const Mat& grayImg,
	const vector<BoardDetectionResult>& boards,
	const Size& patternSize)
{
	if (boards.empty())
	{
		cout << windowPrefix << ": no boards to visualize" << endl;
		return;
	}

	// 颜色调色板
	const vector<Scalar> palette = {
		Scalar(0, 180, 0),    // 绿色
		Scalar(0, 0, 200),    // 红色
		Scalar(200, 0, 0),    // 蓝色
		Scalar(0, 180, 180),  // 黄色
		Scalar(180, 0, 180),  // 洋红
		Scalar(180, 180, 0)   // 青色
	};

	// ==================== 窗口1：图像坐标可视化 ====================
	Mat imgVis;
	if (grayImg.channels() == 1)
	{
		cvtColor(grayImg, imgVis, COLOR_GRAY2BGR);
	}
	else
	{
		imgVis = grayImg.clone();
	}

	for (size_t boardIdx = 0; boardIdx < boards.size(); ++boardIdx)
	{
		const auto& board = boards[boardIdx];
		Scalar color = palette[boardIdx % palette.size()];

		// 绘制 ROI 边框
		if (board.roi.area() > 0)
		{
			rectangle(imgVis, board.roi, color, 3);

			// 板子标签
			char boardLabel[64];
			snprintf(boardLabel, sizeof(boardLabel), "Board %zu (%d pts)", boardIdx, board.pointCount);
			putText(imgVis, boardLabel, Point(board.roi.x, board.roi.y - 10),
				FONT_HERSHEY_SIMPLEX, 1.2, color, 3);
		}

		// 绘制所有角点并标记索引
		for (size_t cIdx = 0; cIdx < board.corners.size(); ++cIdx)
		{
			const Point2f& pt = board.corners[cIdx];
			if (cIdx / 7)
				continue;
			// 绘制角点圆圈
			circle(imgVis, pt, 10, Scalar(255, 255, 255), FILLED);
			circle(imgVis, pt, 10, color, 2);

			// 标注索引号
			char idxStr[16];
			snprintf(idxStr, sizeof(idxStr), "%zu", cIdx);

			int baseline = 0;
			Size textSize = getTextSize(idxStr, FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
			Point textOrg(static_cast<int>(pt.x - textSize.width / 2),
				static_cast<int>(pt.y + textSize.height / 2));
			putText(imgVis, idxStr, textOrg, FONT_HERSHEY_SIMPLEX, 2, color, 4);
		}
	}

	// 添加标题
	putText(imgVis, "Image Coordinates (corners) - Index shown at each point", Point(20, 50),
		FONT_HERSHEY_SIMPLEX, 1.2, Scalar(0, 0, 0), 3);

	// ==================== 计算世界坐标范围 ====================
	float globalMinX = std::numeric_limits<float>::max();
	float globalMinY = std::numeric_limits<float>::max();
	float globalMaxX = -std::numeric_limits<float>::max();
	float globalMaxY = -std::numeric_limits<float>::max();

	for (const auto& board : boards)
	{
		for (const auto& pt : board.objectPoints)
		{
			globalMinX = std::min(globalMinX, pt.x);
			globalMinY = std::min(globalMinY, pt.y);
			globalMaxX = std::max(globalMaxX, pt.x);
			globalMaxY = std::max(globalMaxY, pt.y);
		}
	}

	// 添加边距
	float margin = 30.0f;
	globalMinX -= margin;
	globalMinY -= margin;
	globalMaxX += margin;
	globalMaxY += margin;

	float worldWidth = globalMaxX - globalMinX;
	float worldHeight = globalMaxY - globalMinY;

	// ==================== 窗口2：世界坐标可视化（白色背景）====================
	int worldVisWidth = 1600;
	int worldVisHeight = 1400;

	// 计算缩放比例（保持宽高比）
	float scaleX = (worldVisWidth - 150) / worldWidth;
	float scaleY = (worldVisHeight - 150) / worldHeight;
	float scale = std::min(scaleX, scaleY);

	// 偏移量（居中显示）
	float offsetX = (worldVisWidth - worldWidth * scale) / 2.0f;
	float offsetY = (worldVisHeight - worldHeight * scale) / 2.0f;

	// 世界坐标到像素坐标的转换
	auto worldToPixel = [&](float wx, float wy) -> Point {
		int px = static_cast<int>((wx - globalMinX) * scale + offsetX);
		int py = static_cast<int>((wy - globalMinY) * scale + offsetY);
		return Point(px, py);
	};

	Mat worldVis(worldVisHeight, worldVisWidth, CV_8UC3, Scalar(255, 255, 255));

	// 绘制网格线（辅助查看）
	Scalar gridColor(230, 230, 230);
	float gridStep = 50.0f;  // 每 50mm 一条网格线
	for (float x = 0; x <= globalMaxX; x += gridStep)
	{
		Point p1 = worldToPixel(x, globalMinY);
		Point p2 = worldToPixel(x, globalMaxY);
		line(worldVis, p1, p2, gridColor, 1);
	}
	for (float y = 0; y <= globalMaxY; y += gridStep)
	{
		Point p1 = worldToPixel(globalMinX, y);
		Point p2 = worldToPixel(globalMaxX, y);
		line(worldVis, p1, p2, gridColor, 1);
	}

	// 绘制坐标轴
	Point origin = worldToPixel(0, 0);
	Point xEnd = worldToPixel(globalMaxX - margin + 10, 0);
	Point yEnd = worldToPixel(0, globalMaxY - margin + 10);

	// X 轴（红色）
	arrowedLine(worldVis, origin, xEnd, Scalar(0, 0, 200), 3, LINE_AA, 0, 0.015);
	putText(worldVis, "X (mm)", xEnd + Point(15, 5), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 200), 2);

	// Y 轴（绿色）
	arrowedLine(worldVis, origin, yEnd, Scalar(0, 200, 0), 3, LINE_AA, 0, 0.015);
	putText(worldVis, "Y (mm)", yEnd + Point(5, 25), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 200, 0), 2);

	// 原点标记
	circle(worldVis, origin, 8, Scalar(0, 0, 0), FILLED);
	putText(worldVis, "O(0,0)", origin + Point(-50, -15), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 0), 2);

	// ==================== 绘制每个板子的世界坐标 ====================
	for (size_t boardIdx = 0; boardIdx < boards.size(); ++boardIdx)
	{
		const auto& board = boards[boardIdx];
		Scalar color = palette[boardIdx % palette.size()];

		bool isFullPattern = board.patternMatched &&
			board.corners.size() == static_cast<size_t>(patternSize.area()) &&
			board.corners.size() == board.objectPoints.size();

		// 绘制板子边框（连接四个角点）
		if (isFullPattern && board.objectPoints.size() >= static_cast<size_t>(patternSize.area()))
		{
			size_t tlIdx = 0;
			size_t trIdx = patternSize.width - 1;
			size_t blIdx = (patternSize.height - 1) * patternSize.width;
			size_t brIdx = patternSize.height * patternSize.width - 1;

			Point tl = worldToPixel(board.objectPoints[tlIdx].x, board.objectPoints[tlIdx].y);
			Point tr = worldToPixel(board.objectPoints[trIdx].x, board.objectPoints[trIdx].y);
			Point bl = worldToPixel(board.objectPoints[blIdx].x, board.objectPoints[blIdx].y);
			Point br = worldToPixel(board.objectPoints[brIdx].x, board.objectPoints[brIdx].y);

			// 填充半透明背景
			vector<Point> boardContour = { tl, tr, br, bl };
			Mat overlay = worldVis.clone();
			fillPoly(overlay, vector<vector<Point>>{boardContour}, Scalar(color[0] * 0.3 + 180, color[1] * 0.3 + 180, color[2] * 0.3 + 180));
			addWeighted(overlay, 0.3, worldVis, 0.7, 0, worldVis);

			// 绘制边框
			line(worldVis, tl, tr, color, 2);
			line(worldVis, tr, br, color, 2);
			line(worldVis, br, bl, color, 2);
			line(worldVis, bl, tl, color, 2);
		}

		// 绘制所有角点并标记索引和坐标
		for (size_t cIdx = 0; cIdx < board.objectPoints.size(); ++cIdx)
		{
			const auto& objPt = board.objectPoints[cIdx];
			Point pt = worldToPixel(objPt.x, objPt.y);

			// 绘制角点圆圈
			circle(worldVis, pt, 12, Scalar(255, 255, 255), FILLED);
			circle(worldVis, pt, 12, color, 2);

			// 标注索引号（在圆圈内）
			char idxStr[16];
			snprintf(idxStr, sizeof(idxStr), "%zu", cIdx);

			int baseline = 0;
			Size textSize = getTextSize(idxStr, FONT_HERSHEY_SIMPLEX, 0.45, 1, &baseline);
			Point textOrg(pt.x - textSize.width / 2, pt.y + textSize.height / 2);
			putText(worldVis, idxStr, textOrg, FONT_HERSHEY_SIMPLEX, 0.45, color, 1);

			// 在四个角点旁边显示世界坐标
			if (isFullPattern)
			{
				int row = static_cast<int>(cIdx) / patternSize.width;
				int col = static_cast<int>(cIdx) % patternSize.width;

				if ((row == 0 || row == patternSize.height - 1) &&
					(col == 0 || col == patternSize.width - 1))
				{
					char coordStr[32];
					snprintf(coordStr, sizeof(coordStr), "(%.1f,%.1f)", objPt.x, objPt.y);

					// 计算文本偏移
					Point2f offset(15, -5);
					if (col == patternSize.width - 1)
					{
						offset.x = -90;
					}
					if (row == patternSize.height - 1)
					{
						offset.y = 20;
					}

					Point coordOrg = pt + Point(static_cast<int>(offset.x), static_cast<int>(offset.y));

					// 绘制坐标文本背景
					Size coordSize = getTextSize(coordStr, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
					rectangle(worldVis,
						coordOrg + Point(-3, baseline + 3),
						coordOrg + Point(coordSize.width + 3, -coordSize.height - 3),
						Scalar(255, 255, 255), FILLED);
					rectangle(worldVis,
						coordOrg + Point(-3, baseline + 3),
						coordOrg + Point(coordSize.width + 3, -coordSize.height - 3),
						color, 1);
					putText(worldVis, coordStr, coordOrg, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
				}
			}
		}

		// 在板子中心显示板子索引
		if (!board.objectPoints.empty())
		{
			float centerX = 0, centerY = 0;
			for (const auto& pt : board.objectPoints)
			{
				centerX += pt.x;
				centerY += pt.y;
			}
			centerX /= board.objectPoints.size();
			centerY /= board.objectPoints.size();

			Point center = worldToPixel(centerX, centerY);
			char boardLabel[32];
			snprintf(boardLabel, sizeof(boardLabel), "Board %zu", boardIdx);

			int baseline = 0;
			Size textSize = getTextSize(boardLabel, FONT_HERSHEY_SIMPLEX, 0.9, 2, &baseline);
			Point textOrg(center.x - textSize.width / 2, center.y + textSize.height / 2);

			rectangle(worldVis,
				textOrg + Point(-8, baseline + 8),
				textOrg + Point(textSize.width + 8, -textSize.height - 8),
				Scalar(255, 255, 255), FILLED);
			rectangle(worldVis,
				textOrg + Point(-8, baseline + 8),
				textOrg + Point(textSize.width + 8, -textSize.height - 8),
				color, 2);
			putText(worldVis, boardLabel, textOrg, FONT_HERSHEY_SIMPLEX, 0.9, color, 2);
		}
	}

	// 添加标题
	putText(worldVis, "World Coordinates (objectPoints) - Index shown at each point", Point(20, 40),
		FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 0), 2);

	// 添加图例
	int legendY = worldVisHeight - 60;
	putText(worldVis, "Grid: 50mm | Circle number = corner index | Corner coords shown at 4 corners",
		Point(20, legendY), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(100, 100, 100), 2);

	// ==================== 显示两个独立窗口 ====================
	// 窗口1: 图像坐标
	string imgWinName = windowPrefix + "_ImageCoords";
	namedWindow(imgWinName, WINDOW_NORMAL);
	resizeWindow(imgWinName, 1400, 1000);
	imshow(imgWinName, imgVis);

	// 窗口2: 世界坐标
	string worldWinName = windowPrefix + "_WorldCoords";
	namedWindow(worldWinName, WINDOW_NORMAL);
	resizeWindow(worldWinName, 1400, 1200);
	imshow(worldWinName, worldVis);

	cout << "\n按任意键继续..." << endl;
	waitKey(0);

	destroyWindow(imgWinName);
	destroyWindow(worldWinName);
}

static bool loadNV12Bin(const string& filePath, int width, int height, cv::Mat& grayOut)
{
	std::ifstream fin(filePath, std::ios::binary);
	if (!fin.is_open()) {
		std::cout << "Failed to open NV12 bin: " << filePath << std::endl;
		return false;
	}
	size_t ySize = static_cast<size_t>(width) * static_cast<size_t>(height);
	Mat gray(height, width, CV_8UC1);
	fin.read(reinterpret_cast<char*>(gray.data), static_cast<std::streamsize>(ySize));
	if (fin.gcount() != static_cast<std::streamsize>(ySize)) {
		std::cout << "NV12 size mismatch for: " << filePath << std::endl;
		return false;
	}
	grayOut = gray;
	return true;
}

// 获取指定目录下所有.nv12文件
vector<string> getNV12Files(const string& directory) {
	vector<string> files;
	vector<string> detected;
	// 使用OpenCV glob收集指定目录下的.nv12文件
	string pattern = directory;
	if (!pattern.empty() && pattern.back() != '/' && pattern.back() != '\\') {
		pattern += "/";
	}
	pattern += "*.nv12";
	glob(pattern, detected, false);
	for (auto& path : detected) {
		files.push_back(path);
	}
	return files;
}

void fisheyeCamCalibOnePlane()
{
	const int imageWidth = 4224;
	const int imageHeight = 3136;
	const float birdMmPerPixel = 1.0f;
	const float birdMarginMm = 30.0f;
	double f = 1723.2143;
	Matx33d mCameraK = Matx33d{ f, 0, 2112.0,
		0, f, 1568.0,
		0, 0, 1 };
	Vec4d mCameraDist(0.45, -0.055, -0.1, 0.08);
	vector<string> nv12Files = getNV12Files("13M");
	if (nv12Files.empty())
	{
		cout << "fisheyeCalib: no NV12 files found" << endl;
		return;
	}
	//std::filesystem::create_directories("birdview");
	for (const auto& filePath : nv12Files)
	{
		Mat gray;
		if (!loadNV12Bin(filePath, imageWidth, imageHeight, gray))
		{
			cout << "fisheyeCalib: failed to load " << filePath << endl;
			continue;
		}
		vector<Point2f> imagePoints;
		vector<Point3f> objectPoints;
		// 使用 ChessboardDetector 检测棋盘格
		const Size patternSize(7, 7);  // 8x8 方格的 7x7 内角点
		const float squareSize = 1280.0*80.0/3300.0; // 每格物理尺寸（毫米）
		ChessboardDetector detector(patternSize, squareSize, 3);
		vector<BoardDetectionResult> boards = detector.detect(gray);

		// 可视化角点和世界坐标对比（左：图像坐标，右：世界坐标）
		//visualizeBoardCorners(filePath, gray, boards, patternSize);

		for (const auto& board : boards)
		{
			if (board.patternMatched && board.pointCount > 20)
			{
				imagePoints.insert(imagePoints.end(), board.corners.begin(), board.corners.end());
				objectPoints.insert(objectPoints.end(), board.objectPoints.begin(), board.objectPoints.end());
			}
		}

		// 调试输出：验证 squareSize 对 objectPoints 的影响
		if (!objectPoints.empty())
		{
			cout << "DEBUG: squareSize = " << squareSize << endl;
			cout << "DEBUG: objectPoints[0] = (" << objectPoints[0].x << ", " << objectPoints[0].y << ")" << endl;
			cout << "DEBUG: objectPoints[1] = (" << objectPoints[1].x << ", " << objectPoints[1].y << ")" << endl;
			if (objectPoints.size() > 49)
			{
				cout << "DEBUG: objectPoints[49] = (" << objectPoints[49].x << ", " << objectPoints[49].y << ")" << endl;
			}
		}
		ReallinkSaddlePointFit(gray, imagePoints, 10);
		vector<vector<Point3f>> objectPointsList{ objectPoints };
		vector<vector<Point2f>> imagePointsList{ imagePoints };
		Mat cameraMatrix = Mat(3, 3, CV_64F, (void*)mCameraK.val).clone();
		Mat distCoeffs = (Mat_<double>(4, 1) << mCameraDist[0], mCameraDist[1], mCameraDist[2], mCameraDist[3]);
		vector<Vec3d> rvecs, tvecs;
		try
		{
			double rms = -1.0;
#if 0
			Vec3d Rvec, Tvec;
			fisheye::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, Rvec, Tvec);
			rvecs.push_back(Rvec);
			tvecs.push_back(Tvec);
#endif
			rms = cv::fisheye::calibrate(
				objectPointsList,
				imagePointsList,
				Size(imageWidth, imageHeight),
				cameraMatrix,
				distCoeffs,
				rvecs,
				tvecs,
				cv::fisheye::CALIB_USE_INTRINSIC_GUESS | cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC /* | cv::fisheye::CALIB_FIX_FOCAL_LENGTH*/ |
				cv::fisheye::CALIB_FIX_SKEW,
				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-6));
			if (rvecs.empty() || tvecs.empty())
			{
				cout << "fisheyeCalib: calibration failed for " << filePath << endl;
				continue;
			}
			cout << "fisheyeCalib: " << filePath << " RMS=" << rms << endl;
			cout << "rvec: " << rvecs[0] << endl;
			cout << "tvec: " << tvecs[0] << endl;
			cout << "cameraMatrix: " << cameraMatrix << endl;
			cout << "distCoeffs: " << distCoeffs << endl;
			Mat map1, map2;
			fisheyeInitMap(rvecs[0], tvecs[0], cameraMatrix, distCoeffs, Size(1280,1280), CV_32FC1, map1, map2);
			Mat birdView;
			remap(gray, birdView, map1, map2, INTER_LINEAR, BORDER_CONSTANT, Scalar(0));
			namedWindow("birdView", WINDOW_NORMAL);
			imshow("birdView", birdView);
			waitKey();
			string fileName = filePath;
			size_t pos = fileName.find_last_of("/\\");
			if (pos != string::npos)
			{
				fileName = fileName.substr(pos + 1);
			}
			size_t dot = fileName.find_last_of('.');
			if (dot != string::npos)
			imwrite("birdView/" + fileName + ".jpg", birdView);
		}
		catch (const cv::Exception& e)
		{
			cout << "fisheyeCalib: exception for " << filePath << " -> " << e.what() << endl;
		}
	}
}

int main(int argc, char** argv)
{
	fisheyeCamCalibOnePlane();
	return 0;
}


