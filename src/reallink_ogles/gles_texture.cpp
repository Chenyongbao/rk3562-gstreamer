#include "gles_context.h"
#include "gles_texture.h"
#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <iomanip>

using namespace std;
using namespace cv;


void checkGLError(const char* operation) {
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        cerr << "OpenGL error in " << operation << ": 0x" << hex << error << dec << endl;
    }
}


GLuint createTexture(GLenum format, int width, int height, const void* data) {
    GLuint texture = 0;
    glGenTextures(1, &texture);
    if (texture == 0) {
        cerr << "Failed to generate texture" << endl;
        return 0;
    }
    glBindTexture(GL_TEXTURE_2D, texture);
    checkGLError("glBindTexture");
    


    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    if (format == GL_LUMINANCE || format == GL_LUMINANCE_ALPHA) {
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    }
    
    if (data) {
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
    } else {
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, nullptr);
    }
    checkGLError("glTexImage2D");
    
    if (format == GL_LUMINANCE || format == GL_LUMINANCE_ALPHA) {
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    }
    
    glBindTexture(GL_TEXTURE_2D, 0);
    return texture;
}


void updateTexture(GLuint texture, GLenum format, int width, int height, const void* data) {

    glBindTexture(GL_TEXTURE_2D, texture);
    

    static GLint current_alignment = 4;
    if (format == GL_LUMINANCE || format == GL_LUMINANCE_ALPHA) {
        if (current_alignment != 1) {
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            current_alignment = 1;
        }
    } else {
        if (current_alignment != 4) {
            glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
            current_alignment = 4;
        }
    }
    
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, format, GL_UNSIGNED_BYTE, data);
    checkGLError("glTexSubImage2D");
    

    // glBindTexture(GL_TEXTURE_2D, 0);
}


bool loadMapTextures(GLContext& ctx, const Mat& mapX, const Mat& mapY) {
    auto start = chrono::high_resolution_clock::now();
    
    cout << "Loading map textures: " << mapX.cols << "x" << mapX.rows << endl;
    
    // If maps are reloaded (e.g. after calibration), delete old texture to avoid leaks
    if (ctx.map_texture_xy != 0) {
        glDeleteTextures(1, &ctx.map_texture_xy);
        ctx.map_texture_xy = 0;
    }
    

    Mat normX, normY;
    
    float max_x = 4224.0f - 1.0f;
    float max_y = 3136.0f - 1.0f;
    

    normX = mapX / max_x;
    normY = mapY / max_y;
    normX = cv::min(normX, 1.0f);
    normY = cv::min(normY, 1.0f);
    normX = cv::max(normX, 0.0f);
    normY = cv::max(normY, 0.0f);
    


    Mat xScaled, yScaled;
    normX.convertTo(xScaled, CV_16UC1, 65535.0f, 0.5f);
    normY.convertTo(yScaled, CV_16UC1, 65535.0f, 0.5f);
    
    Mat xHigh, xLow, yHigh, yLow;
    



    int totalBytes = xScaled.rows * xScaled.cols * sizeof(uint16_t);
    Mat xBytes(xScaled.rows, xScaled.cols * 2, CV_8UC1, xScaled.data, xScaled.step);
    Mat yBytes(yScaled.rows, yScaled.cols * 2, CV_8UC1, yScaled.data, yScaled.step);
    

    Mat xScaled2ch = xBytes.reshape(2, xScaled.rows);
    Mat yScaled2ch = yBytes.reshape(2, yScaled.rows);
    

    vector<Mat> xChannels, yChannels;
    split(xScaled2ch, xChannels);
    split(yScaled2ch, yChannels);
    

    xLow = xChannels[0];
    xHigh = xChannels[1];
    yLow = yChannels[0];
    yHigh = yChannels[1];
    

    // R = X high, G = X low, B = Y high, A = Y low
    vector<Mat> channels = {xHigh, xLow, yHigh, yLow};
    Mat mapXY;
    merge(channels, mapXY);
    

    ctx.map_texture_xy = createTexture(GL_RGBA, mapX.cols, mapX.rows, mapXY.data);
    

    if (ctx.map_texture_xy != 0) {
        glBindTexture(GL_TEXTURE_2D, ctx.map_texture_xy);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }
    
    if (ctx.map_texture_xy == 0) {
        cerr << "Failed to create map texture" << endl;
        return false;
    }
    
    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
    ctx.map_load_time = duration.count() / 1000.0;
    
    return true;
}

