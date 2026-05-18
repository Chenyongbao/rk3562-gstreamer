#include "gles_context.h"
#include <iostream>
#include <vector>

using namespace std;


static GLuint compileShader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    
    GLint compiled;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (!compiled) {
        GLint info_len = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &info_len);
        if (info_len > 1) {
            vector<char> info_log(info_len);
            glGetShaderInfoLog(shader, info_len, nullptr, info_log.data());
            cerr << "Shader compilation error: " << info_log.data() << endl;
        }
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}


bool createShaderProgram(GLContext& ctx) {
    const char* vertex_shader_source = R"(
        attribute vec2 a_position;
        attribute vec2 a_texcoord;
        varying vec2 v_texcoord;
        void main() {
            gl_Position = vec4(a_position, 0.0, 1.0);
            v_texcoord = a_texcoord;
        }
    )";
    

    const char* fragment_shader_y_source = R"(
        precision highp float;
        uniform sampler2D u_input_y;
        uniform sampler2D u_map_xy;
        varying vec2 v_texcoord;
        
        void main() {

            vec4 mapXY = texture2D(u_map_xy, v_texcoord);
            


            const float hScale = 0.99610895;//255*256/65535;
            const float lScale = 0.00389105;//255/65535;
            

            float map_x_norm = dot(mapXY.rg, vec2(hScale, lScale));
            float map_y_norm = dot(mapXY.ba, vec2(hScale, lScale));
            
            //vec2 input_coord = clamp(vec2(map_x_norm, map_y_norm), 0.0, 1.0);
            vec2 input_coord = vec2(map_x_norm, map_y_norm);
            

            float y = texture2D(u_input_y, input_coord).r;
            

            gl_FragColor = vec4(y, 0.0, 0.0, 0.0);
        }
    )";
    

    const char* fragment_shader_uv_source = R"(
        precision highp float;
        uniform sampler2D u_input_uv;
        uniform sampler2D u_map_xy;
        varying vec2 v_texcoord;
        
        void main() {

            vec4 mapXY = texture2D(u_map_xy, v_texcoord);
            const float hScale = 0.99610895;//255*256/65535;
            const float lScale = 0.00389105;//255/65535;
            float map_x_norm = dot(mapXY.rg, vec2(hScale, lScale));
            float map_y_norm = dot(mapXY.ba, vec2(hScale, lScale));
            vec2 input_coord = vec2(map_x_norm, map_y_norm);
            




            vec2 uv_sample = texture2D(u_input_uv, input_coord).rg;
            

            gl_FragColor = vec4(uv_sample, 0.0, 0.0);
        }
    )";
    

    GLuint vertex_shader_y = compileShader(GL_VERTEX_SHADER, vertex_shader_source);
    GLuint fragment_shader_y = compileShader(GL_FRAGMENT_SHADER, fragment_shader_y_source);
    
    if (!vertex_shader_y || !fragment_shader_y) {
        cerr << "  Error: Failed to compile Y plane shaders" << endl;
        return false;
    }
    
    ctx.shader_program_y = glCreateProgram();
    if (ctx.shader_program_y == 0) {
        cerr << "  Error: glCreateProgram failed for Y plane" << endl;
        glDeleteShader(vertex_shader_y);
        glDeleteShader(fragment_shader_y);
        return false;
    }
    glAttachShader(ctx.shader_program_y, vertex_shader_y);
    glAttachShader(ctx.shader_program_y, fragment_shader_y);
    glLinkProgram(ctx.shader_program_y);
    
    GLint linked;
    glGetProgramiv(ctx.shader_program_y, GL_LINK_STATUS, &linked);
    if (!linked) {
        GLint info_len = 0;
        glGetProgramiv(ctx.shader_program_y, GL_INFO_LOG_LENGTH, &info_len);
        if (info_len > 1) {
            vector<char> info_log(info_len);
            glGetProgramInfoLog(ctx.shader_program_y, info_len, nullptr, info_log.data());
            cerr << "  Y shader program linking error: " << info_log.data() << endl;
        }
        glDeleteShader(vertex_shader_y);
        glDeleteShader(fragment_shader_y);
        glDeleteProgram(ctx.shader_program_y);
        ctx.shader_program_y = 0;
        return false;
    }
    

    GLuint vertex_shader_uv = compileShader(GL_VERTEX_SHADER, vertex_shader_source);
    GLuint fragment_shader_uv = compileShader(GL_FRAGMENT_SHADER, fragment_shader_uv_source);
    
    if (!vertex_shader_uv || !fragment_shader_uv) {
        cerr << "  Error: Failed to compile UV plane shaders" << endl;
        glDeleteShader(vertex_shader_y);
        glDeleteShader(fragment_shader_y);
        glDeleteProgram(ctx.shader_program_y);
        ctx.shader_program_y = 0;
        return false;
    }
    
    ctx.shader_program_uv = glCreateProgram();
    if (ctx.shader_program_uv == 0) {
        cerr << "  Error: glCreateProgram failed for UV plane" << endl;
        glDeleteShader(vertex_shader_y);
        glDeleteShader(fragment_shader_y);
        glDeleteShader(vertex_shader_uv);
        glDeleteShader(fragment_shader_uv);
        glDeleteProgram(ctx.shader_program_y);
        ctx.shader_program_y = 0;
        return false;
    }
    glAttachShader(ctx.shader_program_uv, vertex_shader_uv);
    glAttachShader(ctx.shader_program_uv, fragment_shader_uv);
    glLinkProgram(ctx.shader_program_uv);
    
    glGetProgramiv(ctx.shader_program_uv, GL_LINK_STATUS, &linked);
    if (!linked) {
        GLint info_len = 0;
        glGetProgramiv(ctx.shader_program_uv, GL_INFO_LOG_LENGTH, &info_len);
        if (info_len > 1) {
            vector<char> info_log(info_len);
            glGetProgramInfoLog(ctx.shader_program_uv, info_len, nullptr, info_log.data());
            cerr << "  UV shader program linking error: " << info_log.data() << endl;
        }
        glDeleteShader(vertex_shader_y);
        glDeleteShader(fragment_shader_y);
        glDeleteShader(vertex_shader_uv);
        glDeleteShader(fragment_shader_uv);
        glDeleteProgram(ctx.shader_program_y);
        glDeleteProgram(ctx.shader_program_uv);
        ctx.shader_program_y = 0;
        ctx.shader_program_uv = 0;
        return false;
    }
    
    glDeleteShader(vertex_shader_y);
    glDeleteShader(fragment_shader_y);
    glDeleteShader(vertex_shader_uv);
    glDeleteShader(fragment_shader_uv);
    

    ctx.attr_position = glGetAttribLocation(ctx.shader_program_y, "a_position");
    ctx.attr_texcoord = glGetAttribLocation(ctx.shader_program_y, "a_texcoord");
    ctx.uniform_input_y = glGetUniformLocation(ctx.shader_program_y, "u_input_y");
    ctx.uniform_input_uv = glGetUniformLocation(ctx.shader_program_uv, "u_input_uv");
    ctx.uniform_map_xy = glGetUniformLocation(ctx.shader_program_y, "u_map_xy");
    ctx.uniform_map_xy_uv = glGetUniformLocation(ctx.shader_program_uv, "u_map_xy");
    
    if (ctx.uniform_map_xy_uv < 0) {
        cerr << "Warning: UV shader missing u_map_xy uniform" << endl;
    }
    

    if (ctx.shader_program_y == 0 || ctx.shader_program_uv == 0) {
        cerr << "Error: Shader programs not created (Y=" << ctx.shader_program_y 
             << ", UV=" << ctx.shader_program_uv << ")" << endl;
        return false;
    }
    
    return true;
}




