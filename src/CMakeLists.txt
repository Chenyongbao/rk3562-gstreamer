cmake_minimum_required(VERSION 3.16)

# 自动加载交叉编译工具链（免去命令行 -DCMAKE_TOOLCHAIN_FILE=...）
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain_rk3562.cmake")
endif()

project(reallinkCVserver
    VERSION 0.9
    DESCRIPTION "reallinkCV server for RK3562"
    LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# spdlog — 统一日志（header-only，避免为交叉编译目标额外编译库）
# ---------------------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.14.1
)
FetchContent_MakeAvailable(spdlog)

# ---------------------------------------------------------------------------
# SDK paths — defaults match the Makefile, override with -D or env
# ---------------------------------------------------------------------------
set(RK3562_SDK_ROOT "/home/yuheng.song/rk3562"
    CACHE PATH "RK3562 SDK root")
set(GST_RTSP_SERVER_ROOT "/home/yuheng.song/gst-rtsp-server-1.22.10/gst-rtsp-server-1.22.10/install"
    CACHE PATH "GStreamer RTSP server install prefix")

if(NOT CMAKE_SYSROOT)
    set(CMAKE_SYSROOT "${RK3562_SDK_ROOT}/debian/binary"
        CACHE PATH "Sysroot directory for cross-compilation")
endif()

# ---------------------------------------------------------------------------
# Sysroot-relative directories (derived)
# ---------------------------------------------------------------------------
if(CMAKE_SYSROOT)
    set(SYSROOT_LIB_DIR  "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu")
    set(SYSROOT_INC_DIR  "${CMAKE_SYSROOT}/usr/include")
else()
    set(SYSROOT_LIB_DIR  "")
    set(SYSROOT_INC_DIR  "")
endif()

if(RK3562_SDK_ROOT)
    set(RKNN_ROOT   "${RK3562_SDK_ROOT}/external/rknpu2/runtime/Linux/librknn_api")
    set(RKNN_INC    "${RKNN_ROOT}/include")
    set(RKNN_LIB    "${RKNN_ROOT}/aarch64")
    set(CURL_INC    "${RK3562_SDK_ROOT}/buildroot/output/rockchip_rk3562_recovery/host/aarch64-buildroot-linux-gnu/sysroot/usr/include")
else()
    set(RKNN_INC    "")
    set(RKNN_LIB    "")
    set(CURL_INC    "")
endif()

if(GST_RTSP_SERVER_ROOT)
    set(GST_RTSP_SERVER_INC "${GST_RTSP_SERVER_ROOT}/usr/include/gstreamer-1.0")
    set(GST_RTSP_SERVER_LIB "${GST_RTSP_SERVER_ROOT}/usr/lib")
else()
    set(GST_RTSP_SERVER_INC "")
    set(GST_RTSP_SERVER_LIB "")
endif()

# ---------------------------------------------------------------------------
# Collect sources
# ---------------------------------------------------------------------------
set(C_SOURCES
    calib/camToolKit/SFData.c
    calib/camToolKit/XUOptDev.c
    calib/camToolKit/SonixCamera.c
    calib/camToolKit/usbcam_toolkit.c
    calib/camToolKit/ROMData.c
)

set(CXX_SOURCES
    main.cpp
    app/app_runtime.cpp

    core/dmabuf_utils.cpp

    pipeline/1_source/gst_v4l2_source.cpp
    pipeline/common/frame_provider.cpp
    pipeline/2_workers/stream_workers.cpp
    pipeline/3_sink/rtsp/rtsp_streamer.cpp
    pipeline/3_sink/rtsp/dual_rtsp_server.cpp

    opencv_vendor/fisheye/fisheye.cpp

    reallink_ogles/bev_api.cpp
    reallink_ogles/camera.cpp
    reallink_ogles/file_utils.cpp
    reallink_ogles/gles_init.cpp
    reallink_ogles/gles_render.cpp
    reallink_ogles/gles_shader.cpp
    reallink_ogles/gles_texture.cpp
    reallink_ogles/gles_dma_texture.cpp
    reallink_ogles/platform.cpp
    detection/thickness.cpp

    yolo/embedded_model.cpp
    yolo/yolo_model.cpp
    yolo/yolo_preprocess.cpp
    yolo/yolo_postprocess.cpp

    camera_calibration/viceCamera/vice_camera_service.cpp
    camera_calibration/viceCamera/vice_camera_capture.cpp
    camera_calibration/viceCamera/vice_camera_calibration.cpp
    camera_calibration/viceCamera/vice_camera_utils.cpp
    camera_calibration/viceCamera/vice_camera_extrinsics.cpp
    camera_calibration/mainCamera/main_camera_fisheye_calib.cpp
    camera_calibration/mainCamera/main_camera_default_intrinsics_refinement.cpp
    camera_calibration/XYoffset.cpp
    camera_calibration/totalHigh.cpp
    camera_calibration/focalabfocal.cpp
    camera_calibration/focal_range_normalizer.cpp

    klipper/klipper_manager.cpp

    calib/charuco_detector.cpp
    calib/ChessboardDetector.cpp
    calib/calibinit.cpp
    calib/saddleFit.cpp
    calib/camToolKit/calib_eeprom.cpp
    calib/cbdetect/findCorners.cpp
    calib/cbdetect/boards.cpp
    calib/cbdetect/growBoard.cpp
    calib/cbdetect/imgPatch.cpp
    # 注：calib/aruco_calibration.cpp 是离线标定工具（自带 main），不能编进服务端（重复 main）。
    #     calib/cbdetect/plot_corners.cpp / plot_boards.cpp 是可视化遗留，全工程无调用，不再编译。

    handlers/CalibCommandHandler.cpp
    handlers/DetectCommandHandler.cpp
    handlers/klipper_flow.cpp

    protocol/CommandHandler.cpp
    protocol/CommandRouter.cpp
    protocol/UnifiedSocketServer.cpp
)

# ---------------------------------------------------------------------------
# Target
# ---------------------------------------------------------------------------
add_executable(reallinkCVserver ${C_SOURCES} ${CXX_SOURCES})

target_include_directories(reallinkCVserver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# -- sysroot includes (only when CMAKE_SYSROOT is set) --
if(CMAKE_SYSROOT)
    target_include_directories(reallinkCVserver PRIVATE
        ${SYSROOT_INC_DIR}
        ${SYSROOT_INC_DIR}/aarch64-linux-gnu
        ${SYSROOT_INC_DIR}/rga
        ${SYSROOT_INC_DIR}/rockchip
        ${SYSROOT_INC_DIR}/gstreamer-1.0
        ${SYSROOT_INC_DIR}/gstreamer-1.0/gst
        ${SYSROOT_INC_DIR}/gstreamer-1.0/gst/app
        ${SYSROOT_INC_DIR}/glib-2.0
        ${SYSROOT_LIB_DIR}/glib-2.0/include
        ${SYSROOT_INC_DIR}/opencv4
    )
endif()

# -- RKNN include --
if(RKNN_INC)
    target_include_directories(reallinkCVserver PRIVATE ${RKNN_INC})
endif()

# -- GStreamer RTSP server include --
if(GST_RTSP_SERVER_INC)
    target_include_directories(reallinkCVserver PRIVATE ${GST_RTSP_SERVER_INC})
endif()

# -- cURL include (idirafter: lower priority) --
if(CURL_INC)
    target_include_directories(reallinkCVserver PRIVATE ${CURL_INC})
endif()

# ---------------------------------------------------------------------------
# Compile definitions
# ---------------------------------------------------------------------------
target_compile_definitions(reallinkCVserver PRIVATE
    $<$<CONFIG:Release>:NDEBUG>
    ENABLE_PERFORMANCE_MONITORING
)

# ---------------------------------------------------------------------------
# Compile options
# ---------------------------------------------------------------------------
target_compile_options(reallinkCVserver PRIVATE
    -Wall -Wextra -O2 -g
)

# ---------------------------------------------------------------------------
# Link options (matching Makefile's LDFLAGS)
# ---------------------------------------------------------------------------
if(CMAKE_SYSROOT)
    # rpath-link for shared library dependency resolution at link time
    target_link_options(reallinkCVserver PRIVATE
        -Wl,-rpath-link,${SYSROOT_LIB_DIR}
        -Wl,-rpath-link,${SYSROOT_LIB_DIR}/../gcc/aarch64-linux-gnu/12
        -Wl,--as-needed
        -B${SYSROOT_LIB_DIR}
    )
    target_link_directories(reallinkCVserver PRIVATE
        ${SYSROOT_LIB_DIR}
        ${SYSROOT_LIB_DIR}/blas
        ${SYSROOT_LIB_DIR}/lapack
        ${SYSROOT_LIB_DIR}/../gcc/aarch64-linux-gnu/12
    )
endif()

if(GST_RTSP_SERVER_LIB)
    target_link_options(reallinkCVserver PRIVATE
        -Wl,-rpath-link,${GST_RTSP_SERVER_LIB}
    )
    target_link_directories(reallinkCVserver PRIVATE ${GST_RTSP_SERVER_LIB})

    find_library(GST_RTSP_SERVER_LIBRARY
        NAMES gstrtspserver-1.0
        PATHS ${GST_RTSP_SERVER_LIB}
        NO_DEFAULT_PATH
    )
endif()

if(RKNN_LIB)
    target_link_options(reallinkCVserver PRIVATE
        -Wl,-rpath-link,${RKNN_LIB}
    )
    target_link_directories(reallinkCVserver PRIVATE ${RKNN_LIB})
endif()

# ---------------------------------------------------------------------------
# Libraries
# ---------------------------------------------------------------------------
target_link_libraries(reallinkCVserver PRIVATE
    spdlog::spdlog_header_only
    gstapp-1.0
    gstbase-1.0
    gstreamer-1.0
    gstallocators-1.0
    gstvideo-1.0
    gobject-2.0
    glib-2.0
    rockchip_mpp
    rga
    EGL
    GLESv2
    gbm
    drm
    opencv_calib3d
    opencv_features2d
    opencv_imgcodecs
    opencv_imgproc
    opencv_core
    opencv_objdetect
    opencv_aruco
    opencv_highgui
    jpeg
    rknnrt
    pthread
    stdc++
    m
    gfortran
    dl
)

# GStreamer RTSP server — prefer found full path, fallback to -l
if(GST_RTSP_SERVER_LIBRARY)
    target_link_libraries(reallinkCVserver PRIVATE "${GST_RTSP_SERVER_LIBRARY}")
else()
    target_link_libraries(reallinkCVserver PRIVATE gstrtspserver-1.0)
endif()

# BLAS / LAPACK as full paths (order matters — after OpenCV)
if(SYSROOT_LIB_DIR)
    target_link_libraries(reallinkCVserver PRIVATE
        "${SYSROOT_LIB_DIR}/lapack/liblapack.so.3"
        "${SYSROOT_LIB_DIR}/blas/libblas.so.3"
        "${SYSROOT_LIB_DIR}/libcurl.so.4"
    )
endif()

# ---------------------------------------------------------------------------
# Output in bin/ as the Makefile does
# ---------------------------------------------------------------------------
set_target_properties(reallinkCVserver PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)

# ---------------------------------------------------------------------------
# Post-link: strip & print binary size
# ---------------------------------------------------------------------------
add_custom_command(TARGET reallinkCVserver POST_BUILD
    COMMAND ${CMAKE_STRIP} "$<TARGET_FILE:reallinkCVserver>"
    COMMAND ${CMAKE_COMMAND} -E echo "Build complete: $<TARGET_FILE:reallinkCVserver>"
    COMMAND ${CMAKE_COMMAND} -E echo "Binary size:"
    COMMAND ls -lh "$<TARGET_FILE:reallinkCVserver>"
    COMMENT "Stripping and verifying binary"
)

# ---------------------------------------------------------------------------
# Convenience targets
# ---------------------------------------------------------------------------
add_custom_target(clean-all
    COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_SOURCE_DIR}/bin"
    COMMENT "Remove bin/ directory"
)

add_custom_target(help
    COMMAND ${CMAKE_COMMAND} -E echo "reallinkCVserver — CMake build"
    COMMAND ${CMAKE_COMMAND} -E echo ""
    COMMAND ${CMAKE_COMMAND} -E echo "Targets:"
    COMMAND ${CMAKE_COMMAND} -E echo "  reallinkCVserver  - Build the server binary"
    COMMAND ${CMAKE_COMMAND} -E echo "  clean             - Remove CMake build artifacts"
    COMMAND ${CMAKE_COMMAND} -E echo "  clean-all         - Also remove bin/ directory"
)
