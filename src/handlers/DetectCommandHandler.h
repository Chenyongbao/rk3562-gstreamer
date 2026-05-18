#ifndef DETECT_COMMAND_HANDLER_H
#define DETECT_COMMAND_HANDLER_H

#include "../protocol/CommandHandler.h"
#include "../yolodetect/thickness.h"
#include "../yolodetect/yolo_wrapper.h"

#include <cstdint>
#include <string>
#include <vector>

// DETECT 指令处理器：
// 1) 从 BEV 共享缓冲读取首帧并执行 YOLO
// 2) 基于首帧结果执行厚度补偿（空目标时走 160,160 fallback）
// 3) 补偿后移动到固定位姿，等待第二帧并再次 YOLO
// 4) 组织 JSON + JPEG 二进制响应返回给客户端
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

    YOLOHandle yolo_handle_;

    static std::string escapeJsonString(const std::string& input);
    bool updateYoloJson(int width,
                        int height,
                        const YOLOFrameResult& frame_result,
                        DetectResponsePayload& payload) const;
    static std::string extractJsonObjectMembers(const std::string& json_object);
    std::string buildDetectJson(const DetectResponsePayload& payload) const;
    bool sendDetectResponse(CommandContext& ctx,
                            const DetectResponsePayload& payload,
                            const uint8_t* nv12_data);
    bool encodeNV12ToJPEG(const uint8_t* nv12_data,
                          int width,
                          int height,
                          std::vector<uint8_t>& jpeg_out,
                          int quality = 90);

public:
    explicit DetectCommandHandler(YOLOHandle handle);

    std::string getName() const override;
    std::string getDescription() const override;
    bool isLongRunning() const override;
    CommandResult execute(CommandContext& ctx) override;
};

#endif // DETECT_COMMAND_HANDLER_H
