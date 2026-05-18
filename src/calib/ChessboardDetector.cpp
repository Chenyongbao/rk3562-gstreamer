#include "ChessboardDetector.h"
#include "calibration_board.h"
#include "cbdetect/findCorners.h"
#include "cbdetect/boards.h"
#include "calibinit.h"
#include "charuco_detector.h"
#include <iostream>
#include <algorithm>
#include <opencv2/core/types_c.h>

using namespace cv;
using namespace std;
using namespace CalibBoard;

namespace {

// Detection parameters
constexpr float kBottomScale = 0.3f;
constexpr float kTopFallbackScale = 0.5f;
constexpr float kMergeIoU = 0.3f;
constexpr float kUniqueIoU = 0.45f;
constexpr float kRoiExpand = 0.6f;

// Board index mapping: sorted by x-coordinate in image
// Bottom half (sees top of board): indices 3, 5, 1
// Top half (sees bottom of board): indices 2, 4, 0
constexpr int kBottomIndices[3] = {3, 5, 1};
constexpr int kTopIndices[3] = {2, 4, 0};

// Standardize corner order to top-left first, row-major
void sortCorners(vector<Point2f>& corners, int rows, int cols) {
    int area = rows * cols;
    if (corners.size() != static_cast<size_t>(area) || area <= 0) return;
    
    Point2f temp;
    int row0E = cols - 1;
    int lastRowB = area - cols;
    int lastRowE = area - 1;
    
    if (corners[lastRowE].y < corners[0].y) {
        if (corners[lastRowB].y < corners[row0E].y) {
            if (corners[0].x > corners[row0E].x) {
                for (int r = 0; r < rows; r++) {
                    for (int c = 0; c < cols / 2; c++) {
                        int a = r * cols + c;
                        CV_SWAP(corners[a], corners[2 * r * cols + row0E - a], temp);
                    }
                }
            }
        } else {
            vector<Point2f> dst(area);
            bool flipX = corners[0].x > corners[lastRowB].x;
            for (int r = 0; r < rows; r++) {
                for (int c = 0; c < cols; c++) {
                    dst[r * cols + c] = flipX ? corners[(row0E - c) * rows + r] : corners[c * rows + r];
                }
            }
            corners = std::move(dst);
        }
    } else {
        if (corners[row0E].y < corners[lastRowB].y) {
            if (corners[lastRowE].x > corners[lastRowB].x) {
                for (int r = 0; r < rows / 2; r++) {
                    for (int c = 0; c < cols; c++) {
                        CV_SWAP(corners[r * cols + c], corners[(rows - 1 - r) * cols + c], temp);
                    }
                }
            } else {
                for (int i = 0; i < area / 2; i++) {
                    CV_SWAP(corners[i], corners[lastRowE - i], temp);
                }
            }
        } else {
            vector<Point2f> dst(area);
            bool flipX = corners[row0E].x > corners[lastRowE].x;
            for (int r = 0; r < rows; r++) {
                for (int c = 0; c < cols; c++) {
                    dst[r * cols + c] = flipX ? corners[(row0E - c) * rows + rows - 1 - r] 
                                              : corners[c * rows + rows - 1 - r];
                }
            }
            corners = std::move(dst);
        }
    }
}

// Calculate world coordinates for a board given its global index
void calcObjectPoints(BoardDetectionResult& board, int boardIdx, float squareSize) {
    if (boardIdx < 0 || boardIdx >= kNumBoards || board.corners.empty()) return;
    
    float scale = squareSize / static_cast<float>(kSquarePx);
    float ox = kBoardOrigins[boardIdx].x * scale;
    float oy = kBoardOrigins[boardIdx].y * scale;
    int cols = board.cols > 0 ? board.cols : 7;
    
    board.objectPoints.clear();
    board.objectPoints.reserve(board.corners.size());
    
    // Mapping: image row -> world column (right to left), image col -> world row (bottom to top)
    for (size_t i = 0; i < board.corners.size(); ++i) {
        int col = static_cast<int>(i) % cols;
        int row = static_cast<int>(i) / cols;
        float x = ox + static_cast<float>(cols - row) * squareSize;
        float y = oy + static_cast<float>(cols - col) * squareSize;
        board.objectPoints.emplace_back(x, y, 0.0f);
    }
}

float getRoiCenterX(const BoardDetectionResult& b) {
    if (b.roi.width > 0) return b.roi.x + b.roi.width * 0.5f;
    if (!b.corners.empty()) {
        float sum = 0;
        for (const auto& p : b.corners) sum += p.x;
        return sum / b.corners.size();
    }
    return 0;
}

int getGlobalIndex(size_t pos, bool isBottom) {
    return pos < 3 ? (isBottom ? kBottomIndices[pos] : kTopIndices[pos]) : -1;
}

float computeIoU(const Rect& a, const Rect& b) {
    if (a.empty() || b.empty()) return 0;
    Rect inter = a & b;
    if (inter.empty()) return 0;
    float iArea = static_cast<float>(inter.area());
    float uArea = static_cast<float>(a.area() + b.area() - inter.area());
    return uArea > 0 ? iArea / uArea : 0;
}

Point2f roiCenter(const Rect& r) {
    return Point2f(r.x + r.width * 0.5f, r.y + r.height * 0.5f);
}

Rect expandRect(const Rect& r, float ratio, const Size& imgSize) {
    if (r.empty()) return r & Rect(0, 0, imgSize.width, imgSize.height);
    int px = max(1, static_cast<int>(r.width * ratio));
    int py = max(1, static_cast<int>(r.height * ratio));
    Rect exp(r.x - px, r.y - py, r.width + 2 * px, r.height + 2 * py);
    exp &= Rect(0, 0, imgSize.width, imgSize.height);
    return exp.empty() ? (r & Rect(0, 0, imgSize.width, imgSize.height)) : exp;
}

Rect unionRect(const Rect& a, const Rect& b, const Size& sz) {
    Rect bounds(0, 0, sz.width, sz.height);
    if (a.empty()) return b & bounds;
    if (b.empty()) return a & bounds;
    int x1 = max(0, min(a.x, b.x));
    int y1 = max(0, min(a.y, b.y));
    int x2 = min(sz.width, max(a.x + a.width, b.x + b.width));
    int y2 = min(sz.height, max(a.y + a.height, b.y + b.height));
    return Rect(x1, y1, max(0, x2 - x1), max(0, y2 - y1));
}

bool isBetter(const BoardDetectionResult& a, const BoardDetectionResult& b, const Size& pat) {
    bool aFull = a.patternMatched && a.pointCount == pat.area();
    bool bFull = b.patternMatched && b.pointCount == pat.area();
    if (aFull != bFull) return aFull;
    if (a.pointCount != b.pointCount) return a.pointCount > b.pointCount;
    return a.roi.area() > b.roi.area();
}

bool matchesPattern(int rows, int cols, int count, const Size& pat) {
    bool match = (rows == pat.height && cols == pat.width) || (rows == pat.width && cols == pat.height);
    return match && count == pat.area();
}

} // namespace

ChessboardDetector::ChessboardDetector(const Size& patternSize, float squareSize, int expectedBoardsPerHalf)
    : patternSize_(patternSize), squareSize_(squareSize), expectedBoardsPerHalf_(expectedBoardsPerHalf) {}

vector<BoardDetectionResult> ChessboardDetector::detect(const Mat& gray) const {
    if (gray.empty()) {
        cerr << "ChessboardDetector: empty input image" << endl;
        return {};
    }

    int halfH = gray.rows / 2;
    Rect topRect(0, 0, gray.cols, halfH);
    int startBottomY = gray.rows / 3;
    Rect bottomRect(0, startBottomY, gray.cols, gray.rows - startBottomY);

    // Detect in both regions
    auto bottom = detectInRegion(gray, bottomRect, kBottomScale);

    if (static_cast<int>(bottom.size()) < expectedBoardsPerHalf_) {
        vector<Point2f> arucoPts;
        vector<Point3f> arucoObj;
        int detected = CharucoMultiBoardDetector::detect(gray, arucoPts, arucoObj, squareSize_);
        if (detected >= 20) {
            BoardDetectionResult det;
            det.corners = std::move(arucoPts);
            det.objectPoints = std::move(arucoObj);
            det.roi = det.corners.empty() ? Rect() : boundingRect(det.corners);
            det.rows = 0;
            det.cols = 0;
            det.pointCount = static_cast<int>(det.corners.size());
            det.patternMatched = true;
            return { std::move(det) };
        }
    }

    auto top = detectInRegion(gray, topRect, 1.0f);
    
    // Fallback with scale if needed
    if (static_cast<int>(top.size()) < expectedBoardsPerHalf_) {
        auto scaled = detectInRegion(gray, topRect, kTopFallbackScale);
        for (const auto& b : scaled) {
            bool dup = false;
            for (const auto& e : top) {
                if (computeIoU(e.roi, b.roi) >= kMergeIoU) { dup = true; break; }
            }
            if (!dup) top.push_back(b);
        }
    }

    // Merge and filter
    vector<BoardDetectionResult> all = std::move(bottom);
    all.insert(all.end(), make_move_iterator(top.begin()), make_move_iterator(top.end()));
    all = mergeAndFilter(std::move(all), kUniqueIoU, patternSize_, gray.size());

    // Refine incomplete boards
    vector<BoardDetectionResult> refined;
    for (auto& b : all) {
        if (!b.patternMatched || b.pointCount != patternSize_.area()) {
            if (!refineBoardROI(gray, patternSize_, b)) continue;
        }
        if (b.patternMatched && b.pointCount == patternSize_.area()) {
            refined.push_back(std::move(b));
        }
    }
    refined = mergeAndFilter(std::move(refined), kUniqueIoU, patternSize_, gray.size());

    // Sort corners
    for (auto& b : refined) {
        if (b.corners.size() == static_cast<size_t>(patternSize_.area())) {
            sortCorners(b.corners, b.rows, b.cols);
        }
    }

    // Assign global indices and calculate world coordinates
    float halfY = gray.rows * 0.5f;
    vector<BoardDetectionResult*> bottomRef, topRef;
    for (auto& b : refined) {
        float cy = b.roi.y + b.roi.height * 0.5f;
        (cy >= halfY ? bottomRef : topRef).push_back(&b);
    }

    auto sortByX = [](const BoardDetectionResult* a, const BoardDetectionResult* b) {
        return getRoiCenterX(*a) < getRoiCenterX(*b);
    };
    
    sort(bottomRef.begin(), bottomRef.end(), sortByX);
    for (size_t i = 0; i < bottomRef.size(); ++i) {
        int idx = getGlobalIndex(i, true);
        if (idx >= 0) calcObjectPoints(*bottomRef[i], idx, squareSize_);
    }

    sort(topRef.begin(), topRef.end(), sortByX);
    for (size_t i = 0; i < topRef.size(); ++i) {
        int idx = getGlobalIndex(i, false);
        if (idx >= 0) calcObjectPoints(*topRef[i], idx, squareSize_);
    }

    return refined;
}

vector<BoardDetectionResult> ChessboardDetector::detectInRegion(const Mat& img, const Rect& roi, float scale) const {
    vector<BoardDetectionResult> results;
    
    Rect safeRoi = roi & Rect(0, 0, img.cols, img.rows);
    if (safeRoi.empty()) return results;

    Mat region = img(safeRoi).clone();
    float invScale = 1.f / max(scale, 1e-3f);
    Mat scaled;
    
    if (abs(scale - 1.f) > 1e-3f) {
        Size sz(max(1, static_cast<int>(region.cols * scale)), max(1, static_cast<int>(region.rows * scale)));
        resize(region, scaled, sz, 0, 0, scale < 1.f ? INTER_AREA : INTER_LINEAR);
    } else {
        scaled = region;
    }

    ReallinkCB::Corner corners;
    vector<ReallinkCB::Board> boards;
    ReallinkCB::Params params;
    ReallinkCB::find_corners(scaled, corners, params);
    ReallinkCB::boards_from_corners(scaled, corners, boards, params);

    for (const auto& board : boards) {
        if (board.idx.empty() || board.idx[0].size() < 2) continue;
        
        int rows = static_cast<int>(board.idx[0].size()) - 2;
        int cols = static_cast<int>(board.idx.size()) - 2;
        if (rows <= 0 || cols <= 0) continue;

        vector<Point2f> pts;
        pts.reserve(rows * cols);
        for (int j = 1; j < static_cast<int>(board.idx[0].size()) - 1; ++j) {
            for (int i = 1; i < static_cast<int>(board.idx.size()) - 1; ++i) {
                int idx = board.idx[i][j];
                if (idx < 0 || idx >= static_cast<int>(corners.p.size())) continue;
                pts.emplace_back(static_cast<float>(corners.p[idx].x * invScale) + safeRoi.x,
                                 static_cast<float>(corners.p[idx].y * invScale) + safeRoi.y);
            }
        }
        if (pts.empty()) continue;

        BoardDetectionResult det;
        det.corners = std::move(pts);
        det.roi = boundingRect(det.corners);
        det.rows = rows;
        det.cols = cols;
        det.pointCount = board.num;
        det.patternMatched = matchesPattern(rows, cols, board.num, patternSize_);
        
        // Placeholder objectPoints (recalculated later)
        det.objectPoints.reserve(det.corners.size());
        for (size_t k = 0; k < det.corners.size(); ++k) {
            int c = static_cast<int>(k) % cols;
            int r = static_cast<int>(k) / cols;
            det.objectPoints.emplace_back(static_cast<float>(cols - r) * squareSize_,
                                          static_cast<float>(cols - c) * squareSize_, 0);
        }
        results.push_back(std::move(det));
    }
    return results;
}

bool ChessboardDetector::refineBoardROI(const Mat& img, const Size& pattern, BoardDetectionResult& board) {
    if (board.corners.empty()) return false;
    
    Rect base = board.roi.area() > 0 ? board.roi : boundingRect(board.corners);
    if (base.empty()) return false;
    
    Rect exp = expandRect(base, kRoiExpand, img.size());
    if (exp.empty()) return false;

    Mat roiImg = img(exp).clone();
    vector<Point2f> corners;
    if (ReallinkFindChessboardCorners(roiImg, pattern, corners, CALIB_CB_ADAPTIVE_THRESH) <= 0 ||
        corners.size() != static_cast<size_t>(pattern.area())) {
        return false;
    }

    board.corners.clear();
    board.objectPoints.clear();
    int cols = pattern.width, rows = pattern.height;
    
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const auto& pt = corners[r * cols + c];
            board.corners.emplace_back(pt.x + exp.x, pt.y + exp.y);
            board.objectPoints.emplace_back(static_cast<float>(cols - r), static_cast<float>(cols - c), 0);
        }
    }

    board.roi = exp;
    board.rows = rows;
    board.cols = cols;
    board.pointCount = static_cast<int>(board.corners.size());
    board.patternMatched = true;
    return true;
}

vector<BoardDetectionResult> ChessboardDetector::mergeAndFilter(vector<BoardDetectionResult> boards,
                                                                  float iouThresh, const Size& pattern, const Size& imgSize) {
    if (boards.empty()) return boards;
    
    sort(boards.begin(), boards.end(), [&](const BoardDetectionResult& a, const BoardDetectionResult& b) {
        return isBetter(a, b, pattern);
    });

    float distThresh = min(imgSize.width, imgSize.height) * 0.02f;
    vector<BoardDetectionResult> unique;
    
    for (const auto& b : boards) {
        bool overlap = false;
        for (auto& k : unique) {
            float iou = computeIoU(k.roi, b.roi);
            float dist = static_cast<float>(norm(roiCenter(k.roi) - roiCenter(b.roi)));
            Rect inter = k.roi & b.roi;
            float c1 = k.roi.area() > 0 ? static_cast<float>(inter.area()) / k.roi.area() : 0;
            float c2 = b.roi.area() > 0 ? static_cast<float>(inter.area()) / b.roi.area() : 0;
            
            if (iou >= iouThresh || dist <= distThresh || c1 >= 0.7f || c2 >= 0.7f) {
                overlap = true;
                Rect merged = unionRect(k.roi, b.roi, imgSize);
                if (isBetter(b, k, pattern)) {
                    k = b;
                    k.roi = merged;
                } else {
                    k.roi = merged;
                }
                break;
            }
        }
        if (!overlap) unique.push_back(b);
    }
    return unique;
}
