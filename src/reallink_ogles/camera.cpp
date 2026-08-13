#include "camera.h"
#include <iostream>
#include <mutex>
#include <cstdio>
#include <unistd.h>
#include <limits.h>
#include <filesystem>
#include "../calib/camToolKit/usbcam_toolkit.h"
#include "file_utils.h"
#include "../calib/camToolKit/calibData.h"
#include "../calib/camToolKit/calib_eeprom.h"

// 主摄运行时状态（统一收敛，便于维护和线程安全管理）
struct CameraRuntimeState {
    Matx33d camera_k = Matx33d::eye();
    Mat camera_dist;
    Mat big_map_x, big_map_y;
    Mat overhead_map_x, overhead_map_y;
    bool is_inited = false;

    // XY偏移量（从配置文件读取，用于俯视图对齐）
    double dx_px = 0.0;
    double dy_px = 0.0;
    double thickness_high = 0.0;

    // 标定输出的俯视图外参（rvec/tvec），优先级高于默认外参
    bool has_bev_extrinsic = false;
    cv::Vec3d bev_rvec = cv::Vec3d(0.0, 0.0, 0.0);
    cv::Vec3d bev_tvec = cv::Vec3d(0.0, 0.0, 0.0);

    // overhead map 版本号：每次重建俯视图 map 递增，用于 GLES 侧热更新
    uint64_t overhead_map_version = 0;

    // 保护标定参数和映射表
    std::mutex param_mutex;
};

static CameraRuntimeState g_state;

static const cv::Vec3d kBevRvecDefault(-0.521888, 0.52143, 1.51301);
static const cv::Vec3d kBevTvecDefault(593.682, -793.101, 1396.25);

// 前向声明
static void createPitchMap();
static bool createPitchMapDirectPose();

namespace {

static std::string getReallinkConfPath()
{
    return std::string("/home/linaro/reallinkCV.conf");
}

static bool readFisheyeDistCoeffs(const Mat& cameraDist, Vec4d& outDist)
{
    if (cameraDist.empty() || cameraDist.total() < 4) {
        return false;
    }

    Mat flattenedDist = cameraDist.reshape(1, 1);
    Mat flattenedDist64;
    flattenedDist.convertTo(flattenedDist64, CV_64F);
    const double* coeffs = flattenedDist64.ptr<double>(0);
    outDist = Vec4d(coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
    return true;
}

static std::string formatIntrinsicsSummary(const Matx33d& cameraK, const Mat& cameraDist)
{
    std::ostringstream oss;
    oss << "fx=" << cameraK(0, 0)
        << " fy=" << cameraK(1, 1)
        << " cx=" << cameraK(0, 2)
        << " cy=" << cameraK(1, 2);
    Vec4d distCoeffs;
    if (readFisheyeDistCoeffs(cameraDist, distCoeffs)) {
        oss << " k1=" << distCoeffs[0]
            << " k2=" << distCoeffs[1]
            << " k3=" << distCoeffs[2]
            << " k4=" << distCoeffs[3];
    } else {
        oss << " k1=? k2=? k3=? k4=?";
    }
    return oss.str();
}

// 高度补偿开关与参数
static constexpr bool kEnableThicknessComp = true;
static constexpr double kMmPerWorld = 320.0 / 1280.0;
static constexpr double kCompSign = +1.0;  // 方向反了可改为 -1.0

// 使用 Rodrigues + 平面偏移计算补偿后的 tvec
static cv::Vec3d computeCompensatedTvec(const cv::Vec3d& rvec,
                                        const cv::Vec3d& tvec,
                                        double dHmm,
                                        double* out_plane_z = nullptr)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    cv::Vec3d zAxisInCam(R.at<double>(0, 2), R.at<double>(1, 2), R.at<double>(2, 2));
    cv::Vec3d xAxisInCam(R.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(2, 0));
    cv::Vec3d yAxisInCam(R.at<double>(0, 1), R.at<double>(1, 1), R.at<double>(2, 1));
    const double planeZ = kCompSign * (-dHmm /kMmPerWorld );
    if (out_plane_z) {
        *out_plane_z = planeZ;
    }

    return tvec + zAxisInCam * planeZ + xAxisInCam * g_state.dx_px + yAxisInCam * g_state.dy_px;
}

// 核心函数：根据输入参数生成鱼眼映射表（支持显式传入 XY 偏移，避免依赖全局状态）
static void fisheyeInitMapWithOffset(InputArray _rvec, InputArray _tvec, InputArray _K, InputArray _D,
    const cv::Size& size, int m1type, OutputArray map1, OutputArray map2)
{
    // 1. 解析旋转向量和平移向量（外参）
    Vec3d om = _rvec.depth() == CV_32F ? (Vec3d)*_rvec.getMat().ptr<Vec3f>() : *_rvec.getMat().ptr<Vec3d>(); // 旋转向量
    Vec3d T = _tvec.depth() == CV_32F ? (Vec3d)*_tvec.getMat().ptr<Vec3f>() : *_tvec.getMat().ptr<Vec3d>(); // 平移向量

    // 2. 创建输出映射表
    map1.create(size, m1type <= 0 ? CV_16SC2 : m1type); // map1 类型：CV_16SC2 或传入类型
    map2.create(size, map1.type() == CV_16SC2 ? CV_16UC1 : CV_32F); // map2 类型：CV_16UC1 或 CV_32F

    // 3. 提取相机内参中的焦距与主点
    cv::Vec2d f, c; // f: 焦距，c: 主点(x,y)
    if (_K.depth() == CV_32F)
    {
        Matx33f K = _K.getMat(); // 32F 内参矩阵
        f = Vec2f(K(0, 0), K(1, 1)); // fx, fy
        c = Vec2f(K(0, 2), K(1, 2)); // cx, cy
    }
    else
    {
        Matx33d K = _K.getMat();    // 64F 内参矩阵
        f = Vec2d(K(0, 0), K(1, 1));
        c = Vec2d(K(0, 2), K(1, 2));
    }

    // 4. 读取畸变参数
    Vec4d k = _D.depth() == CV_32F ? (Vec4d)*_D.getMat().ptr<Vec4f>() : *_D.getMat().ptr<Vec4d>(); // [k1,k2,k3,k4]

    // 5. 构建旋转+平移的仿射变换
    Affine3d aff(om, T); // 外参组合成仿射变换对象

    // 6. 遍历输出俯视图每个像素，计算映射关系
    for (int i = 0; i < size.height; ++i)
    {
        float* m1f = map1.getMat().ptr<float>(i); // 仅 CV_32FC1 类型下使用
        float* m2f = map2.getMat().ptr<float>(i);
        short* m1 = (short*)m1f; // CV_16SC2 情况下的实际映射指针
        ushort* m2 = (ushort*)m2f;

        for (int j = 0; j < size.width; ++j) {
            // 8. 计算每个 BEV 像素在世界坐标中的位置
            //Vec3d Xi(j + dxPx, i + dyPx, 0); // 世界坐标，z=0（地面点：(x,y,0)

            Vec3d Xi(j, i, 0); // 世界坐标，z=0（地面点：(x,y,0)）
            // 9. 世界坐标转相机坐标（仿射变换）
            Vec3d Y = aff * Xi;                                 // 外参变换：Xc = R * Xw + T

            // 10. 透视归一化，得到像平面坐标 x = (X/Z, Y/Z)
            Vec2d x(Y[0] / Y[2], Y[1] / Y[2]);                  // 归一化平面坐标公式：x = (X/Z, Y/Z)

            // 11. 计算径向距离 r
            double r2 = x.dot(x);               // r^2 = x^2 + y^2
            double r = std::sqrt(r2);           // r

            // 12. 计算投影角度
            double theta = atan(r);                             // 投影角度公式：theta = atan(r)
            // 13. 依次计算 theta 的高次项（鱼眼畸变）
            double theta2 = theta * theta, theta3 = theta2 * theta, theta4 = theta2 * theta2, theta5 = theta4 * theta,
                   theta6 = theta3 * theta3, theta7 = theta6 * theta, theta8 = theta4 * theta4, theta9 = theta8 * theta;

            // 14. 鱼眼模型下的畸变校正
            double theta_d = theta + k[0] * theta3 + k[1] * theta5 + k[2] * theta7 + k[3] * theta9;

            double inv_r = r > 1e-8 ? 1.0 / r : 1;            // 防止除 0
            double cdist = r > 1e-8 ? theta_d * inv_r : 1;

            // 15. 畸变后的坐标（回到像平面）
            Vec2d xd1 = x * cdist;
            Vec2d xd3(xd1[0], xd1[1]);

            // 16. 转换到图像像素坐标系：u = x * f[0] + c[0], v = y * f[1] + c[1]
            double u = xd3[0] * f[0] + c[0];
            double v = xd3[1] * f[1] + c[1];

            // 17. 按映射表类型写入结果
            if (m1type == CV_16SC2)
            {
                // 16 位有符号定点格式
                int iu = cv::saturate_cast<int>(u * cv::INTER_TAB_SIZE);
                int iv = cv::saturate_cast<int>(v * cv::INTER_TAB_SIZE);
                m1[j * 2 + 0] = (short)(iu >> cv::INTER_BITS); // u 整数部分
                m1[j * 2 + 1] = (short)(iv >> cv::INTER_BITS); // v 整数部分
                m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE - 1)) * cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE - 1))); // 小数部分（低位）
            }
            else if (m1type == CV_32FC1)
            {
                // 32 位浮点格式
                m1f[j] = (float)u;
                m2f[j] = (float)v;
            }
        }
    }
}

static void fisheyeInitMap(InputArray _rvec, InputArray _tvec, InputArray _K, InputArray _D,
    const cv::Size& size, int m1type, OutputArray map1, OutputArray map2)
{
    // 使用 g_state.dx_px / g_state.dy_px 作为全局偏移
    fisheyeInitMapWithOffset(_rvec, _tvec, _K, _D, size, m1type, map1, map2);
}

} // namespace

// 创建俯视图映射，优先使用已标定外参，否则使用默认外参
static bool createPitchMapDirectPose()
{
    const double dHmm = kEnableThicknessComp ? g_state.thickness_high : 0.0;

    // 优先使用已标定的 BEV 外参（has_bev_extrinsic = true）
    if (g_state.has_bev_extrinsic) {
        cv::Vec3d tvecForMap = g_state.bev_tvec;
        double planeZ = 0.0;
        if (kEnableThicknessComp) {
            tvecForMap = computeCompensatedTvec(g_state.bev_rvec, g_state.bev_tvec, dHmm, &planeZ);
        }

        Size targetSize(1280, 1280);
        try {
            // 使用（可选厚度补偿后的）外参初始化鱼眼映射
            fisheyeInitMap(g_state.bev_rvec, tvecForMap, g_state.camera_k, g_state.camera_dist, targetSize, CV_32FC1, g_state.overhead_map_x, g_state.overhead_map_y);
        } catch (const cv::Exception& e) {
            // 异常处理：输出错误信息
            std::cout << "[camera] Direct BEV: calibrated extrinsic fisheyeInitMap exception: " << e.what() << std::endl;
            return false;
        }
        // 打印调试信息：已标定外参的 rvec 和 tvec
        std::cout << "[camera] Direct BEV: using calibrated extrinsic. rvec=" << g_state.bev_rvec
                  << " tvec(base)=" << g_state.bev_tvec
                  << " tvec(used)=" << tvecForMap
                  << " thicknesshigh(dHmm)=" << dHmm
                  << " planeZ=" << planeZ << std::endl;
        std::cout << "[camera] BEV_runtime_intrinsics "
                  << formatIntrinsicsSummary(g_state.camera_k, g_state.camera_dist) << std::endl;
        return true;
    }

    // 使用默认 BEV 外参
    cv::Vec3d rvec = kBevRvecDefault;
    cv::Vec3d tvec = kBevTvecDefault;
    cv::Vec3d tvecForMap = tvec;
    double planeZ = 0.0;
    if (kEnableThicknessComp) {
        tvecForMap = computeCompensatedTvec(rvec, tvec, dHmm, &planeZ);
    }

    Size targetSize(1280, 1280);
    try {
        // 使用（可选厚度补偿后的）默认外参初始化鱼眼映射
        fisheyeInitMap(rvec, tvecForMap, g_state.camera_k, g_state.camera_dist, targetSize, CV_32FC1, g_state.overhead_map_x, g_state.overhead_map_y);
    } catch (const cv::Exception& e) {
        // 异常处理：输出错误信息
        std::cout << "[camera] Direct BEV: fixed extrinsic fisheyeInitMap exception: " << e.what() << std::endl;
        return false;
    }

    // 打印调试信息：默认外参的 rvec 和 tvec
    std::cout << "[camera] Direct BEV: using fixed extrinsic. rvec=" << rvec
              << " tvec(base)=" << tvec
              << " tvec(used)=" << tvecForMap
              << " thicknesshigh(dHmm)=" << dHmm
              << " planeZ=" << planeZ << std::endl;
    std::cout << "[camera] BEV_runtime_intrinsics "
              << formatIntrinsicsSummary(g_state.camera_k, g_state.camera_dist) << std::endl;
    return true;
}

// 创建俯视图映射表（pitch map）
// 优先使用已标定外参，若失败则不做额外处理
static void createPitchMap()
{
    if (createPitchMapDirectPose()) {
        ++g_state.overhead_map_version; // 每次重建映射表，版本号递增
        return;
    }
}

// 初始化：内参优先从 EEPROM 读取；失败则回退默认硬编码内参；外参从 reallinkCV.conf 读取
void cameraInit()
{
    std::lock_guard<std::mutex> lock(g_state.param_mutex);
    bool intrinsic_loaded = false;
    bool extrinsic_loaded = false;
    std::string intrinsic_source = "default";
    g_state.thickness_high = 0.0;  // 厚度补偿仅运行时生效，启动默认 0

    // 1) 优先从 EEPROM 读取 .bin 内容
    {
        CalibEEPROM eeprom;
        CalibData calibData{};
        if (eeprom.read(calibData)) {
            g_state.camera_k(0, 0) = calibData.fx;
            g_state.camera_k(1, 1) = calibData.fy;
            g_state.camera_k(0, 2) = calibData.cx;
            g_state.camera_k(1, 2) = calibData.cy;
            g_state.camera_dist = (Mat_<double>(1, 4)
                << calibData.distCoeffs[0],
                   calibData.distCoeffs[1],
                   calibData.distCoeffs[2],
                   calibData.distCoeffs[3]);
            intrinsic_loaded = true;
            intrinsic_source = "EEPROM";
            std::cout << "[camera] Intrinsics loaded from EEPROM" << std::endl;
            std::cout << "[camera]   fx=" << calibData.fx << " fy=" << calibData.fy
                      << " cx=" << calibData.cx << " cy=" << calibData.cy << std::endl;
            std::cout << "[camera] EEPROM_full_intrinsics "
                      << formatIntrinsicsSummary(g_state.camera_k, g_state.camera_dist) << std::endl;
        } else {
            std::cout << "[camera] EEPROM read failed, fallback to .bin file" << std::endl;
        }
    }

    // 3) 从 reallinkCV.conf 读取外参和 XY 偏移
    {
        const std::string conf_path = getReallinkConfPath();
        ReallinkCVConfig config;
        if (readReallinkCVConf(conf_path, config)) {
            g_state.bev_rvec = cv::Vec3d(config.cam0.rvec[0], config.cam0.rvec[1], config.cam0.rvec[2]);
            g_state.bev_tvec = cv::Vec3d(config.cam0.tvec[0], config.cam0.tvec[1], config.cam0.tvec[2]);
            g_state.dx_px = config.cam0.dxPx;
            g_state.dy_px = config.cam0.dyPx;
            g_state.has_bev_extrinsic = true;
            extrinsic_loaded = true;
            std::cout << "[camera] Extrinsics loaded from " << conf_path << std::endl;
            std::cout << "[camera]   BEV: rvec=" << g_state.bev_rvec << " tvec=" << g_state.bev_tvec << std::endl;
            std::cout << "[camera]   XY offset: dxPx=" << g_state.dx_px << " dyPx=" << g_state.dy_px << std::endl;
            std::cout << "[camera] measurement_context rvec=" << g_state.bev_rvec
                      << " tvec=" << g_state.bev_tvec
                      << " dxPx=" << g_state.dx_px
                      << " dyPx=" << g_state.dy_px << std::endl;
        } else {
            std::cout << "[camera] INFO: " << conf_path << " not found" << std::endl;
        }
    }

    // 4) 内参未加载时，回退到默认硬编码内参
    if (!intrinsic_loaded) {
        g_state.camera_k(0, 0) = 1724.2514057999579;
        g_state.camera_k(0, 2) = 2164.8117597277233;
        g_state.camera_k(1, 1) = 1723.8088995436899;
        g_state.camera_k(1, 2) = 1566.0508318237344;
        g_state.camera_dist = (Mat_<double>(1, 4)
            << 0.46047995579449141,
               -0.08916995943023287,
               -0.044134920702018653,
                0.051721062476795435);
        std::cout << "[camera] Using default intrinsics (no fisheye_calibration.bin)" << std::endl;
    }

    // 5) 外参未加载时，回退到默认硬编码外参
    if (!extrinsic_loaded) {
        g_state.bev_rvec = kBevRvecDefault;
        g_state.bev_tvec = kBevTvecDefault;
        g_state.has_bev_extrinsic = true;
        std::cout << "[camera] Using default extrinsics" << std::endl;
    }

    // 6) 重建映射表（此时内参和外参一定有效）
    Matx33d K = g_state.camera_k;
    K(0, 2) += 74*2;
    K(1, 2) += 6*2;
    fisheye::initUndistortRectifyMap(g_state.camera_k, g_state.camera_dist, Matx33d::eye(), K,
                                     Size(2276*2, 1582*2), CV_32FC1, g_state.big_map_x, g_state.big_map_y);
    createPitchMap();
    g_state.is_inited = true;

    std::cout << "[camera] Initialized (intrinsic=" << intrinsic_source
              << ", extrinsic=" << (extrinsic_loaded ? "conf" : "default") << ")" << std::endl;
}

// 获取映射表（x 方向、y 方向）
void getOverHeadMaps(Mat& mapX, Mat& mapY)
{
    if (!g_state.is_inited)
        cameraInit();
    
    // 读取映射表时加锁
    std::lock_guard<std::mutex> lock(g_state.param_mutex);
    mapX = g_state.overhead_map_x.clone();
    mapY = g_state.overhead_map_y.clone();
}


// 获取 overhead map 版本号
// 返回值：当前版本号
uint64_t getOverHeadMapVersion()
{
    if (!g_state.is_inited)  // 如果尚未初始化，则执行初始化
        cameraInit();
    std::lock_guard<std::mutex> lock(g_state.param_mutex);  // 加锁保护标定参数和映射表
    return g_state.overhead_map_version;  // 返回当前版本号
}

// 作用：重设俯视图 XY 偏移
// 参数：dxPx（x 方向偏移，单位像素），dyPx（y 方向偏移，单位像素）
void cameraSetOverheadXYOffset(double dxPx, double dyPx)
{
    if (!g_state.is_inited)  // 如果尚未初始化，则执行初始化
        cameraInit();
 
    std::lock_guard<std::mutex> lock(g_state.param_mutex);  // 加锁保护标定参数和映射表
    // 将生成的俯视图整体做偏移
    g_state.dx_px = dxPx;
    g_state.dy_px = dyPx;
    createPitchMap();  // 重新生成俯视图映射表
}

// 热更新配置并重建映射表
void cameraReloadConfAndRebuildMaps()
{
    if (!g_state.is_inited) {
        cameraInit();
    }
 
    // 线程安全
    std::lock_guard<std::mutex> lock(g_state.param_mutex);
 
    // 读取配置文件
    const std::string conf_path = getReallinkConfPath();
    ReallinkCVConfig config;
    if (readReallinkCVConf(conf_path, config)) {
        g_state.bev_rvec = cv::Vec3d(config.cam0.rvec[0], config.cam0.rvec[1], config.cam0.rvec[2]);
        g_state.bev_tvec = cv::Vec3d(config.cam0.tvec[0], config.cam0.tvec[1], config.cam0.tvec[2]);
        g_state.dx_px = config.cam0.dxPx;
        g_state.dy_px = config.cam0.dyPx;
        g_state.has_bev_extrinsic = true;
        std::cout << "[camera] Reloaded extrinsic + XY offset from " << conf_path << std::endl;
        std::cout << "[camera]   BEV: rvec=" << g_state.bev_rvec << " tvec=" << g_state.bev_tvec << std::endl;
        std::cout << "[camera]   XY offset: dxPx=" << g_state.dx_px << " dyPx=" << g_state.dy_px << std::endl;
        std::cout << "[camera] measurement_context rvec=" << g_state.bev_rvec
                  << " tvec=" << g_state.bev_tvec
                  << " dxPx=" << g_state.dx_px
                  << " dyPx=" << g_state.dy_px << std::endl;
    } else {
        std::cout << "[camera] WARNING: Failed to reload " << conf_path << ", keep current params" << std::endl;
    }
 
    createPitchMap();
}

bool setMaterialThickness(double thicknessMm)
{
    if (!g_state.is_inited) {
        cameraInit();
    }

    std::lock_guard<std::mutex> lock(g_state.param_mutex);
    g_state.thickness_high = thicknessMm;
    createPitchMap();
    return true;
}

double getCurrentThickness()
{
    if (!g_state.is_inited) {
        cameraInit();
    }

    std::lock_guard<std::mutex> lock(g_state.param_mutex);
    return g_state.thickness_high;
}

bool isThicknessCompensationEnabled()
{
    return kEnableThicknessComp;
}

