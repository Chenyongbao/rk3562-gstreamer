#ifndef PLATFORM_H
#define PLATFORM_H

#include "gles_context.h"
#include <opencv2/opencv.hpp>
#include <cstdint>


bool platformInit(GLContext& ctx, int output_width, int output_height, 
                  int input_width, int input_height);


void platformCleanup(GLContext& ctx);



const uint8_t* platformGetInputDmaBufferPtr(GLContext& ctx);


size_t platformGetInputDmaStride(GLContext& ctx);


int platformGetInputDmaWidth(GLContext& ctx);
int platformGetInputDmaHeight(GLContext& ctx);


void platformNotifyDmaReady(GLContext& ctx);

#endif // PLATFORM_H


