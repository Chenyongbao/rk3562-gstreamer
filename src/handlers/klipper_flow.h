#ifndef KLIPPER_FLOW_H
#define KLIPPER_FLOW_H

#include <spdlog/spdlog.h>

#include "klipper/klipper_manager.h"

// ============================================================================
// Klipper 通用控制辅助
// ============================================================================
//
// 这些片段原先在 CalibCommandHandler.cpp / DetectCommandHandler.cpp 中各写一份，
// 现抽取为共享辅助，避免 DETECT 与标定流程对 Klipper 的控制逻辑漂移。
// 依赖方向：handler -> KlipperManager（设备客户端），直接依赖具体实现。

// 唤醒：发送 G4 等待命令（g4 p20）。Klipper 空闲后快速响应。
void sendG4Wait(KlipperManager* klipper);

// 归位：调用 forceHome（G28）并记录结果。用于标定流程退出前的归位。
void runFinalHoming(KlipperManager* klipper);

// RAII：作用域结束时执行 G90/G28/M400 归位，确保 DETECT 无论成功/失败/提前返回都会恢复。
// 与 runFinalHoming 区别：走显式 Gcode（G90/G28/M400），且通过 armed 控制是否生效。
struct FinalHomeGuard {
    KlipperManager* klipper_;
    bool armed{false};

    explicit FinalHomeGuard(KlipperManager* klipper) : klipper_(klipper) {}

    ~FinalHomeGuard() {
        if (!armed) {
            return;
        }

        std::string final_home_error;
        const std::string final_home_script = "G90\nG28\nM400\n";
        if (!klipper_->sendGcode(final_home_script, nullptr, 60L, &final_home_error)) {
            spdlog::warn("[Unified Server] G28 failed during DETECT cleanup: {}",
                         final_home_error.empty() ? "sendGcode failed" : final_home_error);
        } else {
            spdlog::info("[Unified Server] DETECT cleanup homed by explicit G28");
        }
    }
};

// 蜂鸣器提示：提示开始检测/标定等动作。
inline void buzzDeep(KlipperManager* klipper) {
    if (klipper) {
        klipper->deep();
    }
}

#endif // KLIPPER_FLOW_H
