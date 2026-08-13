#ifndef FOCALABFOCAL_H
#define FOCALABFOCAL_H

#include <string>
#include <vector>

struct CaptureLoopState;

enum class FocalMachineType {
    Unknown = 0,
    NewProjector,
    OldProjector,
};

struct FocalScanPhase {
    std::string name;
    double step_mm{1.0};
    double window_mm{0.0};
    double start_mm{0.0};
    bool has_start{false};
};

struct FocalAutoFocusConfig {
    double focus_x_mm{200.0};
    double focus_y_mm{120.0};
    double z_feedrate{600.0};
    double xy_feedrate{3000.0};
    double sensor_offset_mm{36.5};   // 焦距公式：totalHigh - final_z - sensor_offset_mm
    double z_max_mm{35.0};
    int score_frames{5};
    int sensor_retry{10};
    int outlier_retry{2};
    int flush_frames{30};
    int settle_ms{1000};
    int stepper_switch_wait_ms{10000};
    int stepper_flush_ms{3000};
    const char* conf_path{nullptr};
    std::vector<FocalScanPhase> phases;

    FocalAutoFocusConfig();
};

// Result for one lens mode.
struct FocalFocusResult {
    bool success{false};
    std::string label;
    double best_z_mm{0.0};
    double interpolated_z_mm{0.0};
    bool has_interpolated_z{false};
    double best_score{0.0};
    double sensor_distance_mm{0.0};
    double focal_distance_mm{0.0};
    std::string error;
};

// Full autofocus result for both lens modes.
struct FocalAutoFocusResult {
    bool success{false};
    FocalMachineType machine_type{FocalMachineType::Unknown};
    std::string initial_focus_mode;
    FocalFocusResult long_focus;
    FocalFocusResult short_focus;
    double focal_long{0.0};
    double focal_short{0.0};
    std::string error;
};

// Autofocus service.
class Focalabfocal {
public:
    explicit Focalabfocal(CaptureLoopState* capture_state = nullptr);
    Focalabfocal(const FocalAutoFocusConfig& config,
                 CaptureLoopState* capture_state = nullptr);
    ~Focalabfocal() = default;

    bool runAutoFocus(FocalAutoFocusResult& result);

    bool runAutoFocusAndSave(double& focal_long,
                             double& focal_short,
                             std::string& error_msg);

    bool measureLongFocus(FocalFocusResult& result);

    bool measureShortFocus(FocalFocusResult& result);

    bool saveResultToConfig(const FocalAutoFocusResult& result, std::string& error_msg);

    void cleanupHardware();

private:
    FocalAutoFocusConfig config_;
    CaptureLoopState* capture_state_ = nullptr;
};

#endif // FOCALABFOCAL_H
