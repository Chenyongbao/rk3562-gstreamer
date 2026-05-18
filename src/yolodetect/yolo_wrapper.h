#ifndef YOLO_WRAPPER_H
#define YOLO_WRAPPER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <string>

typedef struct {
    int class_id;
    char class_name[32];
    float confidence;
    int x, y, w, h;
    int polygon_count;
    // 动态点集：长度为 polygon_count，元素为 [x, y]
    int (*polygon)[2];
} YOLODetection;

typedef struct {
    uint64_t frame_id;
    uint64_t timestamp;
    int detection_count;
    YOLODetection detections[32];
    float inference_time_ms;
} YOLOFrameResult;

typedef void* YOLOHandle;

// 初始化内嵌模型（不再依赖外部模型路径）
YOLOHandle yolo_init(void);

void yolo_set_confidence(YOLOHandle handle, float threshold);

void yolo_set_nms(YOLOHandle handle, float threshold);

bool yolo_detect_nv12(YOLOHandle handle, 
                      const uint8_t* nv12_data, 
                      int width, 
                      int height,
                      uint64_t frame_id,
                      YOLOFrameResult* result);

// 直接复用 YOLOSegmentor::ResultsToJson，返回完整 JSON 对象文本
bool yolo_get_results_json(YOLOHandle handle,
                           int width,
                           int height,
                           std::string& out_results_json);

// 返回最近一次成功检测对应的 objects JSON 数组文本（例如 "[]" / "[{...}]"）
bool yolo_get_last_objects_json(YOLOHandle handle,
                                int width,
                                int height,
                                uint64_t expected_frame_id,
                                std::string& out_objects_json);

// 释放 YOLOFrameResult 内部动态分配的 polygon 内存，并清零对应字段
void yolo_free_frame_result(YOLOFrameResult* result);

void yolo_cleanup(YOLOHandle handle);


#endif // YOLO_WRAPPER_H

