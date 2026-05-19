#define _USE_MATH_DEFINES

#include "focalabfocal.h"
#include "focal_range_normalizer.h"

#include "../config.h"
#include "../reallink_ogles/bev_api.h"

#include "../app/capture_state.h"
#include "../reallink_ogles/file_utils.h"
#include "../video/LatestNv12FrameBuffer.h"
#include "klipper/klipper_manager.h"

#include <algorithm>
#include <chrono>
#include <curl/curl.h>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

struct SpotResult {
    bool found = false;
    cv::Point center;
    double pixel_area = DBL_MAX;
};

struct FocusResult {
    bool success = false;
    double best_z = 0.0;
    double para_z = 0.0;
    bool has_para_z = false;
    double best_score = DBL_MAX;
    cv::Point best_center;
    bool has_best_center = false;
    std::vector<double> heights;
    std::vector<double> scores;
};

struct FocusScanMetricsLog {
    std::string path;
    FILE* file = nullptr;

    ~FocusScanMetricsLog() {
        if (file) {
            std::fclose(file);
        }
    }

    bool open(const char* tag) {
        if (file) {
            std::fclose(file);
            file = nullptr;
        }
        path.clear();

        const long long timestamp_ms = static_cast<long long>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count());
        char filename[256];
        std::snprintf(filename,
                      sizeof(filename),
                      "/tmp/focal_scan_metrics_%s_%lld.txt",
                      tag && *tag ? tag : "scan",
                      timestamp_ms);

        file = std::fopen(filename, "w");
        if (!file) {
            return false;
        }

        path = filename;
        std::fprintf(file, "# Focal autofocus scan metrics\n");
        std::fprintf(file, "# columns: scan_label phase z_mm area\n");
        std::fflush(file);
        std::fprintf(stderr, "[FocalAutoFocus] writing scan metrics to %s\n", path.c_str());
        return true;
    }

    void appendSample(const char* scan_label,
                      const std::string& phase_name,
                      double z_mm,
                      const SpotResult& spot) {
        if (!file) {
            return;
        }

        std::fprintf(file,
                     "%s\t%s\t%.3f\t",
                     scan_label && *scan_label ? scan_label : "scan",
                     phase_name.c_str(),
                     z_mm);
        if (spot.found) {
            std::fprintf(file, "%.6f\n", spot.pixel_area);
        } else {
            std::fprintf(file, "no_spot\n");
        }
        std::fflush(file);
    }
};

static constexpr double kMinAcceptedFocusSpotArea = 100.0;
static constexpr int kBlueSpotMinValue = 45;
static constexpr const char* kFocusState1Label = "state1";
static constexpr const char* kFocusState2Label = "state2";

static void setError(std::string& out, const std::string& msg) {
    if (out.empty()) {
        out = msg;
    }
}

static const char* machineTypeName(FocalMachineType type) {
    switch (type) {
    case FocalMachineType::NewProjector:
        return "new";
    case FocalMachineType::OldProjector:
        return "old";
    default:
        return "unknown";
    }
}

static bool isFrameCorrupted(const cv::Mat& bgr) {
    if (bgr.empty()) {
        return true;
    }

    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::Mat gray32;
    gray.convertTo(gray32, CV_32F);

    cv::Mat row_diff;
    cv::absdiff(gray32.rowRange(1, gray32.rows), gray32.rowRange(0, gray32.rows - 1), row_diff);
    const cv::Scalar mean_diff_scalar = cv::mean(row_diff);
    const double mean_diff = mean_diff_scalar[0];

    int high_rows = 0;
    for (int r = 0; r < row_diff.rows; ++r) {
        if (cv::mean(row_diff.row(r))[0] > 50.0) {
            ++high_rows;
        }
    }
    const double high_row_frac =
        row_diff.rows > 0 ? high_rows / static_cast<double>(row_diff.rows) : 0.0;
    if (mean_diff > 40.0 || high_row_frac > 0.3) {
        return true;
    }

    int black_rows = 0;
    for (int r = 0; r < gray.rows; ++r) {
        if (cv::mean(gray.row(r))[0] < 5.0) {
            ++black_rows;
        }
    }
    const double black_row_frac =
        gray.rows > 0 ? black_rows / static_cast<double>(gray.rows) : 0.0;
    return black_row_frac > 0.10;
}

static bool sendRequiredGcode(const std::string& script, long timeout_sec, std::string& error);

struct HttpResponse {
    std::string data;
    long status = 0;
    CURLcode code = CURLE_OK;
};

static size_t writeHttpCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    const size_t total = size * nmemb;
    auto* resp = static_cast<HttpResponse*>(userp);
    resp->data.append(static_cast<char*>(contents), total);
    return total;
}

static bool parseJsonString(const std::string& body, const char* key, std::string& out) {
    const std::string pattern = std::string("\"") + key + "\"";
    size_t pos = body.find(pattern);
    if (pos == std::string::npos) {
        return false;
    }

    pos = body.find(':', pos + pattern.size());
    if (pos == std::string::npos) {
        return false;
    }

    const size_t quote_begin = body.find('"', pos + 1);
    if (quote_begin == std::string::npos) {
        return false;
    }

    const size_t quote_end = body.find('"', quote_begin + 1);
    if (quote_end == std::string::npos) {
        return false;
    }

    out = body.substr(quote_begin + 1, quote_end - quote_begin - 1);
    return true;
}

static bool queryFocusMode(std::string& out_mode, std::string& error) {
    out_mode.clear();
    CURL* curl = curl_easy_init();
    if (!curl) {
        error = "curl_easy_init failed";
        return false;
    }

    HttpResponse resp;
    curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:7125/printer/objects/query?laser_head_state");
    curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeHttpCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resp);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 3L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3L);

    resp.code = curl_easy_perform(curl);
    if (resp.code != CURLE_OK) {
        error = std::string("CURL error: ") + curl_easy_strerror(resp.code);
        curl_easy_cleanup(curl);
        return false;
    }

    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &resp.status);
    curl_easy_cleanup(curl);

    if (resp.status < 200 || resp.status >= 300) {
        std::ostringstream oss;
        oss << "laser_head_state query returned status " << resp.status;
        if (!resp.data.empty()) {
            oss << ": " << resp.data;
        }
        error = oss.str();
        return false;
    }

    if (!parseJsonString(resp.data, "focus_mode", out_mode)) {
        error = "Failed to parse focus_mode from laser_head_state";
        return false;
    }

    return true;
}

static bool waitForFocusMode(const char* gcode,
                             const char* expected_mode,
                             long timeout_sec,
                             int timeout_ms,
                             std::string& error) {
    if (!expected_mode || !*expected_mode) {
        error = "Expected focus_mode is empty";
        return false;
    }

    const int retry_interval_ms = 10000;
    const int max_retry_count = 3;
    const int requested_timeout_ms = timeout_ms > 0 ? timeout_ms : retry_interval_ms;
    const int effective_timeout_ms =
        std::max(requested_timeout_ms, retry_interval_ms * (max_retry_count + 1));
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(effective_timeout_ms);
    auto next_retry =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(retry_interval_ms);
    std::string last_mode;
    std::string last_error;
    int retry_count = 0;

    while (std::chrono::steady_clock::now() <= deadline) {
        const auto now = std::chrono::steady_clock::now();
        if (gcode && *gcode && now >= next_retry && retry_count < max_retry_count) {
            ++retry_count;
            std::fprintf(stderr,
                         "[FocalAutoFocus] retrying focus switch, re-sending switch command '%s' for expected=%s attempt=%d/%d\n",
                         gcode,
                         expected_mode,
                         retry_count,
                         max_retry_count);
            std::string resend_error;
            if (!sendRequiredGcode(gcode, timeout_sec, resend_error)) {
                last_error = resend_error;
                std::fprintf(stderr,
                             "[FocalAutoFocus] re-sending switch command failed for %s: %s\n",
                             expected_mode,
                             resend_error.c_str());
            }
            next_retry = now + std::chrono::milliseconds(retry_interval_ms);
        }

        std::string current_mode;
        std::string query_error;
        if (queryFocusMode(current_mode, query_error)) {
            std::fprintf(stderr,
                         "[FocalAutoFocus] focus_mode=%s expected=%s\n",
                         current_mode.c_str(),
                         expected_mode);
            if (current_mode == expected_mode) {
                std::fprintf(stderr,
                             "[FocalAutoFocus] current state: \"focus_mode\":\"%s\"\n",
                             current_mode.c_str());
                return true;
            }
            last_mode = current_mode;
        } else {
            last_error = query_error;
            std::fprintf(stderr,
                         "[FocalAutoFocus] focus_mode query failed while waiting for %s: %s\n",
                         expected_mode,
                         query_error.c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if (!last_mode.empty()) {
        error = std::string("Timed out waiting for focus_mode=") + expected_mode +
                ", last=" + last_mode + ", retries=" + std::to_string(retry_count);
    } else if (!last_error.empty()) {
        error = last_error;
    } else {
        error = std::string("Timed out waiting for focus_mode=") + expected_mode +
                ", retries=" + std::to_string(retry_count);
    }
    return false;
}

static bool switchFocusMode(const char* gcode,
                            const char* expected_mode,
                            long timeout_sec,
                            int wait_timeout_ms,
                            std::string& error) {
    std::fprintf(stderr,
                 "[FocalAutoFocus] switching focus with '%s', waiting for %s\n",
                 gcode ? gcode : "",
                 expected_mode ? expected_mode : "");
    if (!sendRequiredGcode(gcode ? gcode : "", timeout_sec, error)) {
        return false;
    }
    return waitForFocusMode(gcode, expected_mode, timeout_sec, wait_timeout_ms, error);
}

static void normalizeFinalFocalOrdering(FocalAutoFocusResult& result) {
    const double focal_long = std::max(result.focal_long, result.focal_short);
    const double focal_short = std::min(result.focal_long, result.focal_short);
    result.focal_long = focal_long;
    result.focal_short = focal_short;
}

static std::string formatMoveScript(double x, double y, double feedrate) {
    std::ostringstream script;
    script << std::fixed << std::setprecision(3);
    script << "G90\n";
    script << "G1 X" << x << " Y" << y << " F" << feedrate << "\n";
    script << "M400\n";
    return script.str();
}

static std::string formatMoveZScript(double z_mm, double feedrate) {
    std::ostringstream script;
    script << std::fixed << std::setprecision(3);
    script << "G90\n";
    script << "G1 Z" << z_mm << " F" << feedrate << "\n";
    script << "M400\n";
    return script.str();
}

static bool sendRequiredGcode(const std::string& script, long timeout_sec, std::string& error) {
    std::string local_error;
    if (!KlipperManager::instance().sendGcode(script, nullptr, timeout_sec, &local_error)) {
        error = local_error.empty() ? "sendGcode failed" : local_error;
        return false;
    }
    return true;
}

static bool parabolaRefine(const std::vector<double>& heights,
                           const std::vector<double>& scores,
                           int best_idx,
                           double& out_z) {
    const int n = static_cast<int>(scores.size());
    if (best_idx <= 0 || best_idx >= n - 1 || heights.size() != scores.size()) {
        return false;
    }

    const int lo = std::max(best_idx - 2, 0);
    const int hi = std::min(best_idx + 2, n - 1);
    if (hi - lo < 2) {
        return false;
    }
    for (int i = lo; i <= hi; ++i) {
        if (scores[static_cast<size_t>(i)] >= DBL_MAX) {
            return false;
        }
    }

    cv::Mat A(hi - lo + 1, 3, CV_64F);
    cv::Mat y(hi - lo + 1, 1, CV_64F);
    for (int i = lo; i <= hi; ++i) {
        const int row = i - lo;
        const double h = heights[static_cast<size_t>(i)];
        A.at<double>(row, 0) = h * h;
        A.at<double>(row, 1) = h;
        A.at<double>(row, 2) = 1.0;
        y.at<double>(row, 0) = scores[static_cast<size_t>(i)];
    }

    cv::Mat coeffs;
    if (!cv::solve(A, y, coeffs, cv::DECOMP_SVD)) {
        return false;
    }

    const double a = coeffs.at<double>(0, 0);
    const double b = coeffs.at<double>(1, 0);
    if (a <= 1e-12) {
        return false;
    }

    out_z = std::max(heights[static_cast<size_t>(lo)],
                     std::min(heights[static_cast<size_t>(hi)], -b / (2.0 * a)));
    return std::isfinite(out_z);
}

// Adaptive blue-spot area via B-max(G,R) "blueness" with hole-fill.
// Tight 30x30 crop → blueness image → threshold at 20% of peak →
// fill interior holes (white core) via contour → count filled pixels.
// Smaller count = tighter spot = better focus.
static SpotResult detectSpot(const cv::Mat& image) {
    SpotResult result;
    if (image.empty() || image.channels() != 3) {
        return result;
    }

    // Tight 30x30 crop centered on laser spot (~794, 480).
    static constexpr int CROP_X1 = 779, CROP_X2 = 809;
    static constexpr int CROP_Y1 = 465, CROP_Y2 = 495;
    static constexpr float kBluenessRatio = 0.2f;

    const cv::Rect crop_rect(CROP_X1, CROP_Y1,
                             CROP_X2 - CROP_X1, CROP_Y2 - CROP_Y1);
    const cv::Rect use_roi = crop_rect & cv::Rect(0, 0, image.cols, image.rows);
    if (use_roi.empty()) {
        return result;
    }

    cv::Mat ch[3];
    cv::split(image(use_roi), ch);
    const int rows = use_roi.height, cols = use_roi.width;

    // Blueness = max(0, B - max(G, R)).
    cv::Mat blueness(rows, cols, CV_8UC1);
    uchar peakBn = 0;
    int peakX = 0, peakY = 0;
    for (int r = 0; r < rows; ++r) {
        const uchar* bP = ch[0].ptr<uchar>(r);
        const uchar* gP = ch[1].ptr<uchar>(r);
        const uchar* rP = ch[2].ptr<uchar>(r);
        uchar* oP = blueness.ptr<uchar>(r);
        for (int c = 0; c < cols; ++c) {
            const int v = static_cast<int>(bP[c]) - std::max(gP[c], rP[c]);
            oP[c] = (v > 0) ? static_cast<uchar>(v) : 0;
            if (oP[c] > peakBn) { peakBn = oP[c]; peakX = c; peakY = r; }
        }
    }

    if (peakBn < kBlueSpotMinValue) {
        return result;
    }

    // Adaptive threshold + fill holes (white core).
    cv::Mat mask;
    cv::threshold(blueness, mask, static_cast<int>(kBluenessRatio * peakBn), 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (!contours.empty()) {
        cv::drawContours(mask, contours, -1, cv::Scalar(255), cv::FILLED);
    }

    const int count = cv::countNonZero(mask);
    if (count < 1) {
        return result;
    }

    cv::Moments mom = cv::moments(mask, true);
    result.found = true;
    result.center = cv::Point(
        static_cast<int>(mom.m10 / mom.m00) + use_roi.x,
        static_cast<int>(mom.m01 / mom.m00) + use_roi.y);
    result.pixel_area = static_cast<double>(count);
    return result;
}

class FocalFrameSource {
public:
    explicit FocalFrameSource(CaptureLoopState* capture_state = nullptr)
        : capture_state_(capture_state) {}

    ~FocalFrameSource() {
        if (bev_handle_) {
            bev_cleanup(bev_handle_);
        }
    }

    bool init(std::string& error) {
        error.clear();
        main_nv12_.resize(static_cast<size_t>(BEV_INPUT_WIDTH) * BEV_INPUT_HEIGHT * 3 / 2);
        bev_nv12_.resize(static_cast<size_t>(BEV_OUTPUT_WIDTH) * BEV_OUTPUT_HEIGHT * 3 / 2);
        bev_handle_ = bev_init(BEV_INPUT_WIDTH, BEV_INPUT_HEIGHT, BEV_OUTPUT_WIDTH, BEV_OUTPUT_HEIGHT);
        if (!bev_handle_) {
            error = "Failed to initialize GPU BEV processor";
            return false;
        }
        if (!bev_bind_context_to_thread(bev_handle_)) {
            error = "Failed to bind GPU BEV context to focal autofocus thread";
            bev_cleanup(bev_handle_);
            bev_handle_ = nullptr;
            return false;
        }
        return true;
    }

    bool grabBgrFrame(cv::Mat& out_bgr, std::string& error, uint64_t* out_frame_id = nullptr) {
        size_t filled = 0;
        uint64_t frame_id = 0;
        const bool copied = capture_state_
            ? main_camera_frame_buffer_request_fresh_copy(capture_state_,
                                                          main_nv12_.data(),
                                                          main_nv12_.size(),
                                                          &filled,
                                                          &frame_id,
                                                          5000,
                                                          20)
            : main_camera_frame_buffer_copy(main_nv12_.data(), main_nv12_.size(), &filled, &frame_id);
        if (!copied) {
            error = "Failed to copy latest main camera frame";
            return false;
        }
        if (filled < main_nv12_.size()) {
            error = "Main camera frame is incomplete";
            return false;
        }

        if (!bev_process_frame(bev_handle_,
                               main_nv12_.data(),
                               main_nv12_.size(),
                               bev_nv12_.data(),
                               bev_nv12_.size())) {
            error = "Failed to process main camera frame with GPU BEV";
            return false;
        }

        cv::Mat y_plane(BEV_OUTPUT_HEIGHT, BEV_OUTPUT_WIDTH, CV_8UC1, bev_nv12_.data());
        cv::Mat uv_plane(BEV_OUTPUT_HEIGHT / 2,
                         BEV_OUTPUT_WIDTH / 2,
                         CV_8UC2,
                         bev_nv12_.data() + static_cast<size_t>(BEV_OUTPUT_WIDTH) * BEV_OUTPUT_HEIGHT);
        cv::cvtColorTwoPlane(y_plane, uv_plane, out_bgr, cv::COLOR_YUV2BGR_NV12);
        if (out_bgr.empty()) {
            error = "Failed to convert GPU BEV NV12 to BGR";
            return false;
        }

        if (out_frame_id) {
            *out_frame_id = frame_id;
        }
        return !out_bgr.empty();
    }

    void flushByFrames(int count) {
        cv::Mat frame;
        std::string ignored;
        for (int i = 0; i < count; ++i) {
            grabBgrFrame(frame, ignored, nullptr);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    void flushForMs(int ms) {
        const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
        cv::Mat frame;
        std::string ignored;
        while (std::chrono::steady_clock::now() < end) {
            grabBgrFrame(frame, ignored, nullptr);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

private:
    std::vector<uint8_t> main_nv12_;
    std::vector<uint8_t> bev_nv12_;
    BevHandle bev_handle_ = nullptr;
    CaptureLoopState* capture_state_ = nullptr;
};

static bool grabScoredFrame(FocalFrameSource& frames,
                            int score_frames,
                            cv::Mat& out_frame,
                            std::string& error) {
    std::vector<std::pair<double, cv::Mat>> scored;
    cv::Mat last_frame;
    const int frame_count = score_frames > 0 ? score_frames : 1;
    for (int i = 0; i < frame_count; ++i) {
        cv::Mat frame;
        std::string local_error;
        if (!frames.grabBgrFrame(frame, local_error, nullptr)) {
            error = local_error;
            continue;
        }
        last_frame = frame;
        if (!isFrameCorrupted(frame)) {
            const SpotResult spot = detectSpot(frame);
            scored.emplace_back(spot.found ? spot.pixel_area : DBL_MAX, frame);
        }
    }

    if (scored.empty()) {
        if (!last_frame.empty()) {
            out_frame = last_frame;
            return true;
        }
        setError(error, "No camera frames available for focal scoring");
        return false;
    }

    std::sort(scored.begin(), scored.end(),
              [](const std::pair<double, cv::Mat>& a, const std::pair<double, cv::Mat>& b) {
                  return a.first < b.first;
              });
    out_frame = scored[scored.size() / 2].second;
    return true;
}

static bool moveToZ(double z_mm, double feedrate, std::string& error) {
    return sendRequiredGcode(formatMoveZScript(z_mm, feedrate), 40L, error);
}
static void logFocusStepperStatus() {
    std::string response;
    std::string error;
    if (!KlipperManager::instance().sendGcode("stepper_status motor=1", &response, 20L, &error)) {
        std::fprintf(stderr, "[FocalAutoFocus] stepper_status motor=1 failed: %s\n", error.c_str());
        return;
    }
    if (!response.empty()) {
        std::fprintf(stderr, "[FocalAutoFocus] stepper_status motor=1 response: %s\n", response.c_str());
    }
}

static bool initFocusStepperAndDetectMode(FocalMachineType& out_machine_type,
                                          std::string& out_initial_mode,
                                          const FocalAutoFocusConfig& config,
                                          std::string& error) {
    out_machine_type = FocalMachineType::Unknown;
    out_initial_mode.clear();

    std::fprintf(stderr, "[FocalAutoFocus] initializing focus stepper with stepper_init\n");
    if (!sendRequiredGcode("stepper_init", 20L, error)) {
        return false;
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(config.stepper_switch_wait_ms);
    while (std::chrono::steady_clock::now() <= deadline) {
        std::string current_mode;
        std::string query_error;
        if (queryFocusMode(current_mode, query_error)) {
            if (current_mode == "short" || current_mode == "long") {
                out_initial_mode = current_mode;
                break;
            }
        } else {
            error = query_error;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if (out_initial_mode == "short") {
        out_machine_type = FocalMachineType::NewProjector;
    } else if (out_initial_mode == "long") {
        out_machine_type = FocalMachineType::OldProjector;
    } else {
        if (error.empty()) {
            error = std::string("Unexpected focus_mode after stepper_init: ") +
                    (out_initial_mode.empty() ? "empty" : out_initial_mode);
        }
        return false;
    }

    logFocusStepperStatus();
    std::fprintf(stderr,
                 "[FocalAutoFocus] stepper_init initial focus_mode=%s machine_type=%s\n",
                 out_initial_mode.c_str(),
                 machineTypeName(out_machine_type));
    return true;
}

static bool grabAtZ(FocalFrameSource& frames,
                    const FocalAutoFocusConfig& config,
                    double z_mm,
                    double& last_z,
                    cv::Mat& out_frame,
                    std::string& error) {
    if (!std::isfinite(last_z) || std::abs(last_z - z_mm) > 0.001) {
        if (!moveToZ(z_mm, config.z_feedrate, error)) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(std::max(config.settle_ms, 2000)));
        last_z = z_mm;
        frames.flushByFrames(config.flush_frames);
    }
    return grabScoredFrame(frames, config.score_frames, out_frame, error);
}

static FocusResult runScanPhase(FocalFrameSource& frames,
                                const FocalAutoFocusConfig& config,
                                const char* scan_label,
                                const FocalScanPhase& phase,
                                double z_lo,
                                double z_hi,
                                double& last_z,
                                FocusScanMetricsLog* metrics_log,
                                std::string& error) {
    FocusResult result;
    FocusResult fallback_result;
    bool has_fallback_result = false;

    std::vector<double> prev_scores;
    const int count = std::max(1, static_cast<int>(std::round((z_hi - z_lo) / phase.step_mm))) + 1;
    for (int i = 0; i < count; ++i) {
        const double z = z_lo + i * phase.step_mm;
        if (z > z_hi + 1e-6) {
            break;
        }

        cv::Mat frame;
        std::string grab_error;
        if (!grabAtZ(frames, config, z, last_z, frame, grab_error)) {
            setError(error, grab_error);
            continue;
        }

        SpotResult spot = detectSpot(frame);

        if (spot.found && prev_scores.size() >= 2) {
            const int win = std::min<int>(3, static_cast<int>(prev_scores.size()));
            double sum = 0.0;
            for (int j = static_cast<int>(prev_scores.size()) - win; j < static_cast<int>(prev_scores.size()); ++j) {
                sum += prev_scores[static_cast<size_t>(j)];
            }
            const double avg = sum / win;
            if (avg > 0.0 && spot.pixel_area > avg * 1.6) {
                std::fprintf(stderr,
                             "[FocalAutoFocus] %s/%s Z=%.3f area=%.2f > avg %.2f * 1.6, retry\n",
                             scan_label,
                             phase.name.c_str(),
                             z,
                             spot.pixel_area,
                             avg);
                for (int retry = 0; retry < config.outlier_retry; ++retry) {
                    cv::Mat retry_frame;
                    std::string retry_error;
                    if (!grabScoredFrame(frames, config.score_frames, retry_frame, retry_error)) {
                        std::fprintf(stderr,
                                     "[FocalAutoFocus] %s/%s Z=%.3f large-outlier retry %d grab failed: %s\n",
                                     scan_label,
                                     phase.name.c_str(),
                                     z,
                                     retry + 1,
                                     retry_error.c_str());
                        continue;
                    }
                    SpotResult retry_spot = detectSpot(retry_frame);
                    std::fprintf(stderr,
                                 "[FocalAutoFocus] %s/%s Z=%.3f large-outlier retry %d area=%s\n",
                                 scan_label,
                                 phase.name.c_str(),
                                 z,
                                 retry + 1,
                                 retry_spot.found ? std::to_string(retry_spot.pixel_area).c_str() : "no_spot");
                    if (retry_spot.found) {
                        spot = retry_spot;
                    }
                    if (!spot.found || spot.pixel_area <= avg * 1.6) {
                        break;
                    }
                }
            }
        }

        const double score = spot.found ? spot.pixel_area : DBL_MAX;
        if (metrics_log) {
            metrics_log->appendSample(scan_label, phase.name, z, spot);
        }
        result.heights.push_back(z);
        result.scores.push_back(score);
        const bool acceptedScore = score >= kMinAcceptedFocusSpotArea;

        if (spot.found) {
            if (acceptedScore) {
                prev_scores.push_back(spot.pixel_area);
            }
            std::fprintf(stderr,
                         "[FocalAutoFocus] %s/%s Z=%.3f area=%.2f center=(%d,%d)\n",
                         scan_label,
                         phase.name.c_str(),
                         z,
                         spot.pixel_area,
                         spot.center.x,
                         spot.center.y);

            if ((!has_fallback_result || score < fallback_result.best_score) &&
                !acceptedScore) {
                fallback_result.best_z = z;
                fallback_result.best_score = score;
                fallback_result.best_center = spot.center;
                fallback_result.has_best_center = true;
                fallback_result.success = true;
                fallback_result.heights = result.heights;
                fallback_result.scores = result.scores;
                has_fallback_result = true;
                std::fprintf(stderr,
                             "[FocalAutoFocus] %s/%s skip small-area candidate Z=%.3f area=%.2f (< %.2f), continue\n",
                             scan_label,
                             phase.name.c_str(),
                             z,
                             score,
                             kMinAcceptedFocusSpotArea);
            }

            if (acceptedScore && score < result.best_score) {
                result.best_z = z;
                result.best_score = score;
                result.best_center = spot.center;
                result.has_best_center = true;
                result.success = true;
                std::fprintf(stderr,
                             "[FocalAutoFocus] %s/%s new best Z=%.3f area=%.2f center=(%d,%d)\n",
                             scan_label,
                             phase.name.c_str(),
                             result.best_z,
                             result.best_score,
                             result.best_center.x,
                             result.best_center.y);
            }
        } else {
            std::fprintf(stderr,
                         "[FocalAutoFocus] %s/%s Z=%.3f no spot\n",
                         scan_label,
                         phase.name.c_str(),
                         z);
        }
    }

    if (!result.success && has_fallback_result) {
        result = fallback_result;
        std::fprintf(stderr,
                     "[FocalAutoFocus] %s/%s no area >= %.2f, use best skipped candidate Z=%.3f area=%.2f\n",
                     scan_label,
                     phase.name.c_str(),
                     kMinAcceptedFocusSpotArea,
                     result.best_z,
                     result.best_score);
    }

    if (!result.success) {
        setError(error, "No laser spot detected in focal scan phase");
    } else {
        int best_idx = -1;
        for (size_t i = 0; i < result.heights.size(); ++i) {
            if (std::abs(result.heights[i] - result.best_z) <= 1e-6) {
                best_idx = static_cast<int>(i);
                break;
            }
        }
        if (best_idx >= 0) {
            result.has_para_z = parabolaRefine(result.heights, result.scores, best_idx, result.para_z);
        }
        std::fprintf(stderr,
                     "[FocalAutoFocus] %s/%s phase best Z=%.3f area=%.2f center=(%d,%d)\n",
                     scan_label,
                     phase.name.c_str(),
                     result.best_z,
                     result.best_score,
                     result.best_center.x,
                     result.best_center.y);
    }
    return result;
}

static FocusResult runSingleScan(FocalFrameSource& frames,
                                 const FocalAutoFocusConfig& config,
                                 const char* label,
                                 FocusScanMetricsLog* metrics_log,
                                 std::string& error) {
    FocusResult global;
    double last_z = std::numeric_limits<double>::quiet_NaN();

    FocalScanPhase global_phase{"global", 1.0, 0.0, 25.0, true};
    if (!config.phases.empty()) {
        global_phase = config.phases.front();
        global_phase.name = "global";
    }

    double z_lo = global_phase.has_start ? global_phase.start_mm : 0.0;
    double z_hi = config.z_max_mm;
    std::fprintf(stderr,
                 "[FocalAutoFocus] %s phase %s range %.3f -> %.3f step %.3f\n",
                 label,
                 global_phase.name.c_str(),
                 z_lo,
                 z_hi,
                 global_phase.step_mm);

    std::string global_error;
    global = runScanPhase(frames,
                          config,
                          label,
                          global_phase,
                          z_lo,
                          z_hi,
                          last_z,
                          metrics_log,
                          global_error);
    if (!global.success) {
        std::fprintf(stderr,
                     "[FocalAutoFocus] %s phase %s failed: %s\n",
                     label,
                     global_phase.name.c_str(),
                     global_error.c_str());
        setError(error, global_error);
        return global;
    }

    std::fprintf(stderr,
                 "[FocalAutoFocus] %s phase %s result Z=%.3f area=%.2f center=(%d,%d)\n",
                 label,
                 global_phase.name.c_str(),
                 global.best_z,
                 global.best_score,
                 global.best_center.x,
                 global.best_center.y);

    if (!global.success) {
        setError(error, std::string(label) + " focal scan failed");
    } else {
        std::fprintf(stderr,
                     "[FocalAutoFocus] %s final scan best Z=%.3f area=%.2f center=(%d,%d)\n",
                     label,
                     global.best_z,
                     global.best_score,
                     global.best_center.x,
                     global.best_center.y);
    }
    return global;
}

static bool measureFocusDistance(FocalFrameSource& frames,
                                 const FocalAutoFocusConfig& config,
                                 const char* label,
                                 FocalFocusResult& out,
                                 FocusScanMetricsLog* metrics_log,
                                 std::string& error) {
    out = FocalFocusResult{};
    out.label = label ? label : "";

    FocusResult result = runSingleScan(frames, config, label, metrics_log, error);
    if (!result.success) {
        out.error = error;
        return false;
    }

    const double final_z = result.has_para_z ? result.para_z : result.best_z;
    std::string move_error;
    if (!moveToZ(final_z, config.z_feedrate, move_error)) {
        std::fprintf(stderr, "[FocalAutoFocus] %s move to best Z failed: %s\n",
                     label, move_error.c_str());
    }

    out.success = true;
    out.best_z_mm = result.best_z;
    out.interpolated_z_mm = result.para_z;
    out.has_interpolated_z = result.has_para_z;
    out.best_score = result.best_score;
    std::fprintf(stderr,
                 "[FocalAutoFocus] %s accepted best Z=%.3f interp=%s area=%.2f\n",
                 label,
                 out.best_z_mm,
                 out.has_interpolated_z ? std::to_string(out.interpolated_z_mm).c_str() : "none",
                 out.best_score);

    const char* conf_path = config.conf_path ? config.conf_path : REALLINK_CV_CONF_PATH;
    ReallinkCVConfig file_config;
    if (!readReallinkCVConf(conf_path, file_config)) {
        error = std::string("Failed to read config file for totalHigh: ") + conf_path;
        out.error = error;
        return false;
    }
    if (!std::isfinite(file_config.totalHigh) || file_config.totalHigh <= 0.0) {
        error = "Invalid totalHigh in config";
        out.error = error;
        return false;
    }

    out.sensor_distance_mm = file_config.totalHigh;
    out.focal_distance_mm = file_config.totalHigh - final_z - config.sensor_offset_mm;
    std::fprintf(stderr, "[FocalAutoFocus] %s totalHigh=%.3f final_z=%.3f offset=%.3f focal=%.3f\n",
                 label, file_config.totalHigh, final_z,
                 config.sensor_offset_mm, out.focal_distance_mm);
    return true;
}

FocalAutoFocusConfig::FocalAutoFocusConfig() {
    conf_path = REALLINK_CV_CONF_PATH;
    phases = {
        {"global", 1.0, 0.0, 25.0, true},
    };
}

Focalabfocal::Focalabfocal(CaptureLoopState* capture_state)
    : config_(),
      capture_state_(capture_state) {}

Focalabfocal::Focalabfocal(const FocalAutoFocusConfig& config,
                           CaptureLoopState* capture_state)
    : config_(config),
      capture_state_(capture_state) {
    if (!config_.conf_path) {
        config_.conf_path = REALLINK_CV_CONF_PATH;
    }
    if (config_.phases.empty()) {
        config_.phases = FocalAutoFocusConfig().phases;
    }
}

bool Focalabfocal::runAutoFocus(FocalAutoFocusResult& result) {
    result = FocalAutoFocusResult{};

    FocalFrameSource frames(capture_state_);
    if (!frames.init(result.error)) {
        return false;
    }

    std::string local_error;
    if (!KlipperManager::instance().forceHome(&local_error)) {
        result.error = local_error.empty() ? "Focal homing failed" : local_error;
        return false;
    }

    sendRequiredGcode("SET_IDLE_TIMEOUT TIMEOUT=1800", 20L, local_error);
    if (!initFocusStepperAndDetectMode(result.machine_type,
                                       result.initial_focus_mode,
                                       config_,
                                       result.error)) {
        return false;
    }
    if (!sendRequiredGcode(formatMoveScript(config_.focus_x_mm,
                                            config_.focus_y_mm,
                                            config_.xy_feedrate),
                           40L,result.error)) {
        return false;
    }
    if (!KlipperManager::instance().setFillLight(0, &result.error)) {
        return false;
    }
    if (!KlipperManager::instance().sendGcode("M3 S1", nullptr, 20L, &result.error)) {
        return false;
    }

    const bool initial_mode_is_short = result.initial_focus_mode == "short";
    const char* state1_focus_mode = initial_mode_is_short ? "short" : "long";
    const char* state2_focus_mode = initial_mode_is_short ? "long" : "short";
    const char* state2_switch_gcode =
        initial_mode_is_short ? "stepper_limit_seek motor=1" : "stepper_home motor=1";

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    frames.flushForMs(config_.stepper_flush_ms);
    cv::Mat test_frame;
    if (!grabScoredFrame(frames, config_.score_frames, test_frame, result.error)) {
        return false;
    }

    // Staged descent to coarse start Z
    for (double z : {10.0, 20.0, 25.0}) {
        if (!moveToZ(z, config_.z_feedrate, result.error)) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    const char* conf_path = config_.conf_path ? config_.conf_path : REALLINK_CV_CONF_PATH;
    ReallinkCVConfig file_config;
    if (!readReallinkCVConf(conf_path, file_config)) {
        result.error = std::string("Failed to read config file: ") + conf_path;
        return false;
    }

    FocusScanMetricsLog metrics_log;
    metrics_log.open("autofocus");

    FocalFocusResult state1_result;
    std::string state1_error;
    if (!measureFocusDistance(frames,
                              config_,
                              kFocusState1Label,
                              state1_result,
                              &metrics_log,
                              state1_error)) {
        state1_result.error = state1_error;
        result.error = state1_error;
        return false;
    }

    std::fprintf(stderr, "[FocalAutoFocus] Switching to focus state2\n");
    if (!switchFocusMode(state2_switch_gcode,
                         state2_focus_mode,
                         20L,
                         config_.stepper_switch_wait_ms,
                         result.error)) {
        return false;
    }
    logFocusStepperStatus();
    frames.flushForMs(config_.stepper_flush_ms);

    FocalFocusResult state2_result;
    std::string state2_error;
    if (!measureFocusDistance(frames,
                              config_,
                              kFocusState2Label,
                              state2_result,
                              &metrics_log,
                              state2_error)) {
        state2_result.error = state2_error;
        result.error = state2_error;
        return false;
    }

    if (std::string(state1_focus_mode) == "short") {
        result.short_focus = state1_result;
        result.long_focus = state2_result;
    } else {
        result.long_focus = state1_result;
        result.short_focus = state2_result;
    }

    result.focal_short = result.short_focus.focal_distance_mm;
    result.focal_long = result.long_focus.focal_distance_mm;
    normalizeFocalResultForNewProjectorRanges(result);
    result.success = true;
    return true;
}

bool Focalabfocal::runAutoFocusAndSave(double& focal_long,
                                       double& focal_short,
                                       std::string& error_msg) {
    focal_long = 0.0;
    focal_short = 0.0;
    error_msg.clear();

    FocalAutoFocusResult result;
    const bool ok = runAutoFocus(result);
    cleanupHardware();
    if (ok) {
        normalizeFinalFocalOrdering(result);
    }
    if (!ok || !saveResultToConfig(result, error_msg)) {
        if (error_msg.empty()) {
            error_msg = result.error;
        }
        if (error_msg.empty()) {
            error_msg = "Focal autofocus failed";
        }
        std::fprintf(stderr, "[FocalAutoFocus] failed: %s\n", error_msg.c_str());
        return false;
    }

    focal_long = result.focal_long;
    focal_short = result.focal_short;
    std::fprintf(stderr,
                 "[FocalAutoFocus] saved focal_long=%.2f focal_short=%.2f to %s\n",
                 focal_long,
                 focal_short,
                 config_.conf_path ? config_.conf_path : REALLINK_CV_CONF_PATH);
    return true;
}

bool Focalabfocal::measureLongFocus(FocalFocusResult& result) {
    FocalFrameSource frames(capture_state_);
    std::string error;
    if (!frames.init(error)) {
        result = FocalFocusResult{};
        result.label = "long";
        result.error = error;
        return false;
    }
    FocusScanMetricsLog metrics_log;
    metrics_log.open("long");
    return measureFocusDistance(frames, config_, "long", result, &metrics_log, error);
}

bool Focalabfocal::measureShortFocus(FocalFocusResult& result) {
    FocalFrameSource frames(capture_state_);
    std::string error;
    if (!frames.init(error)) {
        result = FocalFocusResult{};
        result.label = "short";
        result.error = error;
        return false;
    }
    FocusScanMetricsLog metrics_log;
    metrics_log.open("short");
    return measureFocusDistance(frames, config_, "short", result, &metrics_log, error);
}

bool Focalabfocal::saveResultToConfig(const FocalAutoFocusResult& result, std::string& error_msg) {
    if (!result.success) {
        error_msg = result.error.empty() ? "Cannot save failed focal result" : result.error;
        return false;
    }

    const char* conf_path = config_.conf_path ? config_.conf_path : REALLINK_CV_CONF_PATH;
    ReallinkCVConfig file_config;
    if (!readReallinkCVConf(conf_path, file_config)) {
        error_msg = std::string("Failed to read config file: ") + conf_path;
        return false;
    }
    file_config.focal_long = result.focal_long;
    file_config.focal_short = result.focal_short;
    if (!writeReallinkCVConf(conf_path, file_config)) {
        error_msg = std::string("Failed to write config file: ") + conf_path;
        return false;
    }
    return true;
}

void Focalabfocal::cleanupHardware() {
    std::string ignored;
    KlipperManager::instance().laserOff(&ignored);
    KlipperManager::instance().setFillLight(128, &ignored);
    KlipperManager::instance().forceHome(&ignored);
}
