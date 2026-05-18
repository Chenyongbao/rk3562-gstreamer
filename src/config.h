#ifndef CONFIG_H
#define CONFIG_H

#define REALLINKCV_VERSION "1.1"

#define V4L2_DEVICE       "/dev/video11"
#define INPUT_WIDTH       4224
#define INPUT_HEIGHT      3136
#define V4L2_BUFFER_COUNT 6


#define ORIGINAL_WIDTH    1280
#define ORIGINAL_HEIGHT   960


#define BEV_INPUT_WIDTH   INPUT_WIDTH
#define BEV_INPUT_HEIGHT  INPUT_HEIGHT


#define BEV_OUTPUT_WIDTH  1280
#define BEV_OUTPUT_HEIGHT 1280


#define TARGET_FPS        30
#define ENCODE_BPP        0.20f
#define ENCODE_GOP        30
#define ENCODE_MAX_PENDING 2

#define RTSP_SERVER_PORT     "8554"
#define RTSP_PATH_ORIGINAL   "/stream_original"
#define RTSP_PATH_BEV        "/stream_bev"
#define RTSP_BIND_ADDRESS    "auto"


#define APPSRC_MAX_BYTES_ORIGINAL  (ORIGINAL_WIDTH * ORIGINAL_HEIGHT * 3 / 2 * 4)
#define APPSRC_MAX_BYTES_BEV       (BEV_OUTPUT_WIDTH * BEV_OUTPUT_HEIGHT * 3 / 2 * 4)


#define APPSRC_MAX_TIME_MS  80


#define YOLO_SOCKET_PORT    9999
#define YOLO_IMAGE_SAVE_DIR "/home/linaro/build/img"
#define YOLO_SAVE_IMAGES    1
#define YOLO_CONF_THRESHOLD 0.1f
#define YOLO_NMS_THRESHOLD  0.45f


#define SERVER_BIND_ADDRESS           "192.168.50.1"  // 所有服务绑定到此IP

// ----------------------------------------------------------------------------
// Klipper / Moonraker defaults
// ----------------------------------------------------------------------------
#define KLIPPER_MOONRAKER_DEFAULT_HOST "192.168.50.1"
#define KLIPPER_MOONRAKER_PORT        7125
#define KLIPPER_MOONRAKER_API_KEY     ""  // legacy: used by thickness module, not used by KlipperManager path
#define KLIPPER_HOMING_TIMEOUT_MS     300000
#define REALLINK_CV_CONF_PATH         "/home/linaro/reallinkCV.conf"

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
#define PERF_STATS_INTERVAL  (TARGET_FPS * 10)

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// camera calibration
// ----------------------------------------------------------------------------
#define CALIB_CAPTURE_DIR            "calibration/captures"
#define CALIB_RESULT_DIR             "calibration/results"
#define CALIB_TEMP_DIR               "calibration/temp"
#define CALIB_BIN_NAME               "reallinkCV.bin"
#define CALIB_CAMERA_DEVICE          "/dev/video18"  // 副摄设备（标定用）
#define CALIB_MAIN_CAMERA_DEVICE     "/dev/video11"  // 主摄设备
#define CALIB_V4L2_CTL_PATH         "v4l2-ctl"
#define CALIB_FRAME_WIDTH           1920
#define CALIB_FRAME_HEIGHT          1080
#define CALIB_PIXEL_FORMAT          "MJPG"
#define CALIB_DEFAULT_Z_HEIGHT      0.0     // 默认Z轴高度(mm)
#define CALIB_DEFAULT_MOVES         20       // 默认拍照次数
#define CALIB_X_MIN                 0
#define CALIB_X_MAX                 30
#define CALIB_Y_MIN                 75
#define CALIB_Y_MAX                 90
#define CALIB_FEEDRATE              2500.0

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
#define NV12_SIZE(w, h)  ((w) * (h) * 3 / 2)

#define ORIGINAL_NV12_SIZE  NV12_SIZE(ORIGINAL_WIDTH, ORIGINAL_HEIGHT)
#define BEV_INPUT_NV12_SIZE NV12_SIZE(BEV_INPUT_WIDTH, BEV_INPUT_HEIGHT)
#define BEV_OUTPUT_NV12_SIZE NV12_SIZE(BEV_OUTPUT_WIDTH, BEV_OUTPUT_HEIGHT)

#endif // CONFIG_H

