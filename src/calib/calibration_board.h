#pragma once

#include <opencv2/core.hpp>

// Calibration board layout constants (shared by ChessboardDetector and CharucoDetector)
namespace CalibBoard {

// Canvas size (pixels)
constexpr int kCanvasW = 3300;
constexpr int kCanvasH = 3020;

// Board configuration
constexpr int kSquaresX = 8;           // Squares per row
constexpr int kSquaresY = 8;           // Squares per column
constexpr int kSquarePx = 80;          // Square size in pixels
constexpr int kReservedEdge = 10;      // Edge margin in pixels
constexpr int kNumBoards = 6;          // Total number of boards

// Derived constants
constexpr int kBoardW = kSquaresX * kSquarePx;
constexpr int kBoardH = kSquaresY * kSquarePx;
constexpr int kMidY = (kCanvasH - kBoardH) / 2;
constexpr int kCornersPerBoard = (kSquaresX - 1) * (kSquaresY - 1);  // 7x7 = 49

// Board origins in pixel coordinates
// Layout: 0=TopLeft, 1=TopRight, 2=BottomLeft, 3=BottomRight, 4=MiddleLeft, 5=MiddleRight
static const cv::Point kBoardOrigins[kNumBoards] = {
    {kReservedEdge, kReservedEdge},                                    // 0: Top-left
    {kCanvasW - kBoardW - kReservedEdge, kReservedEdge},              // 1: Top-right
    {kReservedEdge, kCanvasH - kBoardH - kReservedEdge},              // 2: Bottom-left
    {kCanvasW - kBoardW - kReservedEdge, kCanvasH - kBoardH - kReservedEdge}, // 3: Bottom-right
    {kReservedEdge, kMidY},                                            // 4: Middle-left
    {kCanvasW - kBoardW - kReservedEdge, kMidY}                       // 5: Middle-right
};

// Convert pixel coordinate to physical coordinate (mm)
static float pixelToMM(int pixelVal, float squareSize) {
    return static_cast<float>(pixelVal) * squareSize / static_cast<float>(kSquarePx);
}

} // namespace CalibBoard
