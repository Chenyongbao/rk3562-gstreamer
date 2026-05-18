#ifndef GLES_TEXTURE_H
#define GLES_TEXTURE_H

#include <GLES2/gl2.h>


GLuint createTexture(GLenum format, int width, int height, const void* data);


void updateTexture(GLuint texture, GLenum format, int width, int height, const void* data);


void checkGLError(const char* operation);

#endif // GLES_TEXTURE_H




