#ifndef DETECT_COMMAND_HANDLER_H
#define DETECT_COMMAND_HANDLER_H

#include "protocol/CommandHandler.h"
#include "detection/thickness.h"
#include "yolo/yolo_common.h"

#include <cstdint>
#include <string>
#include <vector>

class YOLOModel;

// DETECT 指令处理器：
// 1) 从 BEV 共享缓冲读取首帧并执行 YOLO
// 2) 基于首帧结果执行厚度补偿（空目标时走 160,160 fallback）
// 3) 补偿后移动到固定位姿
// 4) 复用首帧图像与检测结果，组织 JSON + JPEG 二进制响应返回给客户端
class DetectCommandHandler : public ICommandHandler {
private:
    struct DetectResponsePayload {
        bool success = false;
        int width = 0;
        int height = 0;
        int count = 0;
        bool focal_ok = false;
        double focal_long = 0.0;
        double focal_short = 0.0;
        bool total_high_ok = false;
        double total_high = 0.0;
        bool thickness_triggered = false;
        bool thickness_ok = false;
        ThicknessResult thickness_result{};
        std::string thickness_error;
        std::string error_code;
        std::string error_message;
        bool bev_stream_connected = false;
        bool bev_stream_ready = false;
        bool bev_refresh_requested = false;
        bool bev_refresh_forced = false;
        uint32_t bev_pending_refresh_requests = 0;
        uint64_t first_frame_id = 0;
        uint64_t latest_bev_frame_id = 0;
        std::string yolo_json;
    };

    YOLOModel* yolo_model_;

    static std::string escapeJsonString(const std::string& input);
    static std::string buildObjectsJson(const std::vector<SegmentationResult>& results,
                                        int width,
                                        int height);
    void updateYoloJson(int width,
                        int height,
                        const std::vector<SegmentationResult>& results,
                        DetectResponsePayload& payload) const;
    bool detectNV12(const uint8_t* nv12_data,
                    int width,
                    int height,
                    std::vector<SegmentationResult>& out_results,
                    double* out_total_ms = nullptr) const;
    static std::string extractJsonObjectMembers(const std::string& json_object);
    std::string buildDetectJson(const DetectResponsePayload& payload) const;
    bool sendDetectResponse(CommandContext& ctx,
                            const DetectResponsePayload& payload,
                            const uint8_t* nv12_data,
                            const std::vector<SegmentationResult>* draw_results = nullptr);
    bool encodeNV12ToJPEG(const uint8_t* nv12_data,
                          int width,
                          int height,
                          std::vector<uint8_t>& jpeg_out,
                          const std::vector<SegmentationResult>* draw_results = nullptr,
                          int quality = 90);

public:
    explicit DetectCommandHandler(YOLOModel* model);

    std::string getName() const override;
    std::string getDescription() const override;
    CommandResult execute(CommandContext& ctx) override;
};

#endif // DETECT_COMMAND_HANDLER_H
