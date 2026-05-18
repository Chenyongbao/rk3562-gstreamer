#include "charuco_detector.h"
#include "calibration_board.h"
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

using namespace cv;
using namespace std;
using namespace CalibBoard;

namespace {

constexpr float kMarkerRatio = 0.6f;
constexpr int kMarkersPerBoard = 41;  // 250/6

// Create dictionary subsets for each board
vector<Ptr<aruco::Dictionary>> createDictSubsets() {
    Ptr<aruco::Dictionary> base = aruco::getPredefinedDictionary(aruco::DICT_5X5_250);
    vector<Ptr<aruco::Dictionary>> dicts;

    for (int i = 0; i < kNumBoards; ++i) {
        Ptr<aruco::Dictionary> d = makePtr<aruco::Dictionary>();
        d->markerSize = base->markerSize;
        d->maxCorrectionBits = base->maxCorrectionBits;
        int start = i * kMarkersPerBoard;
        int end = min(start + kMarkersPerBoard, base->bytesList.rows);
        d->bytesList = base->bytesList.rowRange(start, end).clone();
        dicts.push_back(d);
    }
    return dicts;
}

// Generate world coordinates for all board corners
vector<vector<Point3f>> generateObjectPoints(float squareSize) {
    vector<vector<Point3f>> allPoints(kNumBoards);
    float scale = squareSize / static_cast<float>(kSquarePx);

    for (int b = 0; b < kNumBoards; ++b) {
        float ox = kBoardOrigins[b].x * scale;
        float oy = kBoardOrigins[b].y * scale;

        for (int row = 0; row < kSquaresY - 1; ++row) {
            for (int col = 0; col < kSquaresX - 1; ++col) {
                float x = ox + (col + 1) * squareSize;
                float y = oy + (row + 1) * squareSize;
                allPoints[b].emplace_back(x, y, 0);
            }
        }
    }
    return allPoints;
}

} // namespace

int CharucoMultiBoardDetector::detectPerBoard(const Mat& image,
                                              vector<CharucoBoardDetection>& boards,
                                              float squareSize) {
    boards.assign(kNumBoards, CharucoBoardDetection{});

    Mat gray;
    if (image.channels() == 3) {
        cvtColor(image, gray, COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    // Initialize once
    static vector<Ptr<aruco::Dictionary>> dicts = createDictSubsets();
    static float cachedSize = 0;
    static vector<vector<Point3f>> objPoints;
    if (cachedSize != squareSize) {
        objPoints = generateObjectPoints(squareSize);
        cachedSize = squareSize;
    }

    // Detect all markers at once
    Ptr<aruco::Dictionary> fullDict = aruco::getPredefinedDictionary(aruco::DICT_5X5_250);
    Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();
    params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;

    vector<int> markerIds;
    vector<vector<Point2f>> markerCorners;
    aruco::detectMarkers(gray, fullDict, markerCorners, markerIds, params);

    if (markerIds.empty()) return 0;

    int total = 0;
    float markerPx = kSquarePx * kMarkerRatio;

    for (int b = 0; b < kNumBoards; ++b) {
        int idStart = b * kMarkersPerBoard;
        int idEnd = idStart + kMarkersPerBoard;

        vector<int> localIds;
        vector<vector<Point2f>> localCorners;
        localIds.reserve(markerIds.size());
        localCorners.reserve(markerCorners.size());

        for (size_t m = 0; m < markerIds.size(); ++m) {
            int id = markerIds[m];
            if (id >= idStart && id < idEnd) {
                localIds.push_back(id - idStart);
                localCorners.push_back(markerCorners[m]);
            }
        }
        if (localIds.empty()) continue;

        Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(
            kSquaresX, kSquaresY, static_cast<float>(kSquarePx), markerPx, dicts[b]);

        vector<Point2f> charucoCorners;
        vector<int> charucoIds;
        aruco::interpolateCornersCharuco(localCorners, localIds, gray, board, charucoCorners, charucoIds);

        if (charucoIds.empty()) continue;

        CharucoBoardDetection det;
        det.imagePoints.reserve(charucoIds.size());
        det.objectPoints.reserve(charucoIds.size());
        det.ids.reserve(charucoIds.size());

        for (size_t j = 0; j < charucoIds.size(); ++j) {
            int cid = charucoIds[j];
            if (cid >= 0 && cid < static_cast<int>(objPoints[b].size())) {
                det.imagePoints.push_back(charucoCorners[j]);
                det.objectPoints.push_back(objPoints[b][cid]);
                det.ids.push_back(cid);
            }
        }
        if (!det.imagePoints.empty()) {
            det.roi = boundingRect(det.imagePoints);
            boards[b] = std::move(det);
            total += static_cast<int>(boards[b].imagePoints.size());
        }
    }

    return total;
}

int CharucoMultiBoardDetector::detect(const Mat& image,
                                       vector<Point2f>& imagePoints,
                                       vector<Point3f>& objectPoints,
                                       float squareSize) {
    imagePoints.clear();
    objectPoints.clear();

    vector<CharucoBoardDetection> perBoard;
    int total = detectPerBoard(image, perBoard, squareSize);
    if (total <= 0) return 0;

    for (const auto& b : perBoard) {
        imagePoints.insert(imagePoints.end(), b.imagePoints.begin(), b.imagePoints.end());
        objectPoints.insert(objectPoints.end(), b.objectPoints.begin(), b.objectPoints.end());
    }
    return static_cast<int>(imagePoints.size());
}
