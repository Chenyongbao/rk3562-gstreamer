#include "klipper_flow.h"

#include <spdlog/spdlog.h>

// 唤醒：发送 G4 等待命令（g4 p20）。Klipper 空闲后快速响应。
void sendG4Wait(KlipperManager* klipper) {
    if (!klipper) {
        return;
    }
    std::string wait_work;
    if (klipper->sendGcode("g4 p20\n", nullptr, 5L, &wait_work)) {
        spdlog::info("[Unified Server] G4 wait command response: {}",
                     wait_work.empty() ? "(empty)" : wait_work);
    } else {
        spdlog::warn("[Unified Server] WARNING: Failed to send G4 wait command");
    }
}

// 归位：调用 forceHome（G28）并记录结果。用于标定流程退出前的归位。
void runFinalHoming(KlipperManager* klipper) {
    if (!klipper) {
        return;
    }
    std::string homing_error;
    spdlog::info("[Unified Server] Final homing (G28) before exit...");
    if (!klipper->forceHome(&homing_error)) {
        spdlog::warn("[Unified Server] Final homing failed: {}",
                     homing_error.empty() ? "(no error detail)" : homing_error);
    } else {
        spdlog::info("[Unified Server] Final homing completed");
    }
}
