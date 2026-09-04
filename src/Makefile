# Build reallinkCVserver for RK3562
 
 # Toolchain
 TOOLCHAIN_ROOT ?= /home/yuheng.song/rk3562/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu
 CROSS_COMPILE ?= $(TOOLCHAIN_ROOT)/bin/aarch64-none-linux-gnu-
 CC = $(CROSS_COMPILE)gcc
 CXX = $(CROSS_COMPILE)g++
 STRIP = $(CROSS_COMPILE)strip
 PKG_CONFIG = $(CROSS_COMPILE)pkg-config
 
 # SDK paths
 RK3562_SDK_ROOT ?= /home/yuheng.song/rk3562
 SYSROOT ?= $(RK3562_SDK_ROOT)/debian/binary
 
 # External packages
 GST_RTSP_SERVER_ROOT ?= /home/yuheng.song/gst-rtsp-server-1.22.10/gst-rtsp-server-1.22.10/install
 
 # BEV GLES sources
 BIRDEYE_DIR ?= $(SRC_DIR)/infrastructure/media/bev_gles
 
 # RKNN
 RKNN_ROOT = $(RK3562_SDK_ROOT)/external/rknpu2/runtime/Linux/librknn_api
 RKNN_INC = $(RKNN_ROOT)/include
 RKNN_LIB = $(RKNN_ROOT)/aarch64
 
 # System include and library roots
 LIB_DIR = $(SYSROOT)/usr/lib/aarch64-linux-gnu
 INCLUDE_DIR = $(SYSROOT)/usr/include
 
 # MPP
 MPP_INC = $(INCLUDE_DIR)/rockchip
 MPP_LIB = $(LIB_DIR)
 
 # GStreamer
 GSTREAMER_INC = $(INCLUDE_DIR)/gstreamer-1.0
 GSTREAMER_LIB = $(LIB_DIR)
 GSTREAMER_PKG_CONFIG = $(LIB_DIR)/pkgconfig
 
 # GStreamer RTSP server
 GST_RTSP_SERVER_INC = $(GST_RTSP_SERVER_ROOT)/usr/include/gstreamer-1.0
 GST_RTSP_SERVER_LIB = $(GST_RTSP_SERVER_ROOT)/usr/lib
 GST_RTSP_SERVER_PKG_CONFIG = $(GST_RTSP_SERVER_ROOT)/usr/lib/pkgconfig
 GST_RTSP_SERVER_SO = $(firstword $(wildcard $(GST_RTSP_SERVER_LIB)/libgstrtspserver-1.0.so $(GST_RTSP_SERVER_LIB)/libgstrtspserver-1.0.so.*))
 
 # GLib
 GLIB_INC = $(INCLUDE_DIR)/glib-2.0
 GLIB_LIB = $(LIB_DIR)
 GLIB_LIB_INTERNAL = $(LIB_DIR)/glib-2.0/include
 
 # Project layout
 SRC_DIR = src
 
 # Compile flags
 CFLAGS = -Wall -Wextra -O2 -g
 CFLAGS += -fPIC
 CFLAGS += -DENABLE_PERFORMANCE_MONITORING
 
 # Sysroot
 CFLAGS += --sysroot=$(SYSROOT)
 LDFLAGS = --sysroot=$(SYSROOT)
 
 # Common include paths
 CFLAGS += -I$(SRC_DIR)
 CFLAGS += -I$(RKNN_INC)
 CFLAGS += -I$(INCLUDE_DIR)/aarch64-linux-gnu
 CFLAGS += -I$(INCLUDE_DIR)/rga
 CFLAGS += -I$(MPP_INC)
 CFLAGS += -I$(GSTREAMER_INC)
 CFLAGS += -I$(GSTREAMER_INC)/gst
 CFLAGS += -I$(GSTREAMER_INC)/gst/app
 CFLAGS += -I$(GST_RTSP_SERVER_INC)
 CFLAGS += -I$(GLIB_INC)
 CFLAGS += -I$(GLIB_LIB_INTERNAL)
 CFLAGS += -I$(BIRDEYE_DIR)
 CFLAGS += -I$(SRC_DIR)/app
 CFLAGS += -I$(SRC_DIR)/board_surface
 CFLAGS += -I$(SRC_DIR)/camera_calibration
 CFLAGS += -I$(SRC_DIR)/camera_calibration/viceCamera
 CFLAGS += -I$(SRC_DIR)/camera_calibration/mainCamera
 CFLAGS += -I$(SRC_DIR)/calibration_toolkit
 CFLAGS += -I$(SRC_DIR)/calibration_toolkit/camToolKit
 CFLAGS += -I$(SRC_DIR)/common
 CFLAGS += -I$(SRC_DIR)/infrastructure
 CFLAGS += -I$(SRC_DIR)/infrastructure/media
 CFLAGS += -I$(SRC_DIR)/infrastructure/media/core
 CFLAGS += -I$(SRC_DIR)/infrastructure/media/rtsp
 CFLAGS += -I$(SRC_DIR)/infrastructure/media/video
 CFLAGS += -I$(SRC_DIR)/infrastructure/media/video/consumers
 CFLAGS += -I$(SRC_DIR)/infrastructure/media/bev_gles
 CFLAGS += -I$(SRC_DIR)/application
 CFLAGS += -I$(SRC_DIR)/application/workflows
 CFLAGS += -I$(SRC_DIR)/interfaces
 CFLAGS += -I$(SRC_DIR)/interfaces/command_server
 CFLAGS += -I$(SRC_DIR)/runtime
 CFLAGS += -I$(SRC_DIR)/detection
 CFLAGS += -I$(SRC_DIR)/detection/core
 # Keep curl after standard headers.
 CURL_INC = $(RK3562_SDK_ROOT)/buildroot/output/rockchip_rk3562_recovery/host/aarch64-buildroot-linux-gnu/sysroot/usr/include
 CFLAGS += -idirafter $(CURL_INC)
 
 # C++ flags
 CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -g
 CXXFLAGS += -fPIC
 CXXFLAGS += --sysroot=$(SYSROOT)
 CXXFLAGS += -DENABLE_PERFORMANCE_MONITORING
 CXXFLAGS += -I$(SRC_DIR)
 CXXFLAGS += -I$(BIRDEYE_DIR)
 CXXFLAGS += -I$(MPP_INC)
 CXXFLAGS += -I$(GSTREAMER_INC)
 CXXFLAGS += -I$(GSTREAMER_INC)/gst
 CXXFLAGS += -I$(GSTREAMER_INC)/gst/app
 CXXFLAGS += -I$(GST_RTSP_SERVER_INC)
 CXXFLAGS += -I$(GLIB_INC)
 CXXFLAGS += -I$(GLIB_LIB_INTERNAL)
 CXXFLAGS += -I$(SRC_DIR)/app
 CXXFLAGS += -I$(SRC_DIR)/board_surface
 CXXFLAGS += -I$(SRC_DIR)/camera_calibration
 CXXFLAGS += -I$(SRC_DIR)/camera_calibration/viceCamera
 CXXFLAGS += -I$(SRC_DIR)/camera_calibration/mainCamera
 CXXFLAGS += -I$(SRC_DIR)/calibration_toolkit
 CXXFLAGS += -I$(SRC_DIR)/calibration_toolkit/camToolKit
 CXXFLAGS += -I$(SRC_DIR)/common
 CXXFLAGS += -I$(SRC_DIR)/infrastructure
 CXXFLAGS += -I$(SRC_DIR)/infrastructure/media
 CXXFLAGS += -I$(SRC_DIR)/infrastructure/media/core
 CXXFLAGS += -I$(SRC_DIR)/infrastructure/media/rtsp
 CXXFLAGS += -I$(SRC_DIR)/infrastructure/media/video
 CXXFLAGS += -I$(SRC_DIR)/infrastructure/media/video/consumers
 CXXFLAGS += -I$(SRC_DIR)/infrastructure/media/bev_gles
 CXXFLAGS += -I$(SRC_DIR)/application
 CXXFLAGS += -I$(SRC_DIR)/application/workflows
 CXXFLAGS += -I$(SRC_DIR)/interfaces
 CXXFLAGS += -I$(SRC_DIR)/interfaces/command_server
 CXXFLAGS += -I$(SRC_DIR)/runtime
 CXXFLAGS += -I$(SRC_DIR)/detection
 CXXFLAGS += -I$(SRC_DIR)/detection/core
 CXXFLAGS += -I$(RKNN_INC)
 CXXFLAGS += -I$(INCLUDE_DIR)/aarch64-linux-gnu
 CXXFLAGS += -I$(INCLUDE_DIR)
 CXXFLAGS += -I$(INCLUDE_DIR)/rga
 CXXFLAGS += -I$(INCLUDE_DIR)/opencv4
 # Keep curl after standard headers.
 CXXFLAGS += -idirafter $(CURL_INC)
 
 # Link flags
 LDFLAGS += -L$(LIB_DIR)
 LDFLAGS += -L$(LIB_DIR)/blas
 LDFLAGS += -L$(LIB_DIR)/lapack
 LDFLAGS += -L$(MPP_LIB)
 LDFLAGS += -L$(GSTREAMER_LIB)
 LDFLAGS += -L$(GST_RTSP_SERVER_LIB)
 LDFLAGS += -L$(RKNN_LIB)
 LDFLAGS += -Wl,-rpath-link,$(RKNN_LIB)
 LDFLAGS += -L$(GLIB_LIB)
 
 # Startfiles
 LDFLAGS += -B$(LIB_DIR)
 
 # Runtime search paths
 LDFLAGS += -Wl,-rpath-link,$(LIB_DIR)
 LDFLAGS += -Wl,-rpath-link,$(GSTREAMER_LIB)
 LDFLAGS += -Wl,-rpath-link,$(GST_RTSP_SERVER_LIB)
 
 # Link only required libraries.
 LDFLAGS += -Wl,--as-needed
 
 # Use libstdc++ from the sysroot.
 LDFLAGS += -Wl,-rpath-link,$(SYSROOT)/usr/lib/gcc/aarch64-linux-gnu/12
 LDFLAGS += -L$(SYSROOT)/usr/lib/gcc/aarch64-linux-gnu/12
 
 # Libraries
 LIBS = 
 LIBS += -lgstapp-1.0 -lgstbase-1.0 -lgstreamer-1.0 -lgstallocators-1.0 -lgstvideo-1.0
 LIBS += $(if $(GST_RTSP_SERVER_SO),$(GST_RTSP_SERVER_SO),-lgstrtspserver-1.0)
 LIBS += -lgobject-2.0 -lglib-2.0
 LIBS += -lrockchip_mpp
 LIBS += -lrga
 LIBS += -lEGL -lGLESv2 -lgbm -ldrm
 LIBS += -lopencv_calib3d -lopencv_features2d -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lopencv_objdetect -lopencv_aruco -lopencv_highgui
 LIBS += -ljpeg
 # Keep BLAS and LAPACK after OpenCV.
 LIBS += $(LIB_DIR)/lapack/liblapack.so.3 $(LIB_DIR)/blas/libblas.so.3
 LIBS += -lrknnrt
 LIBS += -lpthread -lstdc++ -lm -lgfortran -ldl
 # Use the full libcurl path because libcurl.so may be missing.
 LIBS += $(LIB_DIR)/libcurl.so.4
 
 # Target
 TARGET = bin/reallinkCVserver
 
 # Source files
 CPP_SRCS = $(SRC_DIR)/app/main.cpp
 C_SRCS = $(SRC_DIR)/calibration_toolkit/camToolKit/SFData.c \
        $(SRC_DIR)/calibration_toolkit/camToolKit/XUOptDev.c \
        $(SRC_DIR)/calibration_toolkit/camToolKit/SonixCamera.c \
        $(SRC_DIR)/calibration_toolkit/camToolKit/usbcam_toolkit.c \
        $(SRC_DIR)/calibration_toolkit/camToolKit/ROMData.c
 
 # Infrastructure and media.
 INFRA_CXX_SRCS = $(SRC_DIR)/infrastructure/media/video/LatestNv12FrameBuffer.cpp \
                  $(SRC_DIR)/infrastructure/media/video/consumer_buffer_pool.cpp \
                  $(SRC_DIR)/infrastructure/media/video/video_router.cpp \
                  $(SRC_DIR)/infrastructure/media/video/v4l2_capture.cpp \
                  $(SRC_DIR)/infrastructure/media/video/consumers/bev_rtsp_consumer.cpp \
                  $(SRC_DIR)/infrastructure/media/video/consumers/latest_frame_consumer.cpp \
                  $(SRC_DIR)/infrastructure/media/video/consumers/original_rtsp_consumer.cpp \
                  $(SRC_DIR)/infrastructure/media/rtsp/rga_processor.cpp \
                  $(SRC_DIR)/infrastructure/media/rtsp/rtsp_streamer.cpp \
                  $(SRC_DIR)/infrastructure/media/rtsp/dual_rtsp_server.cpp \
                  $(SRC_DIR)/infrastructure/media/core/dmabuf_utils.cpp \
                  $(SRC_DIR)/infrastructure/media/core/frame_queue.cpp \
                  $(SRC_DIR)/opencv_vendor/fisheye/fisheye.cpp \
                  $(SRC_DIR)/infrastructure/media/bev_gles/bev_api.cpp \
                  $(SRC_DIR)/infrastructure/media/bev_gles/dma_buffer.cpp \
                  $(SRC_DIR)/infrastructure/media/bev_gles/gles_init.cpp \
                  $(SRC_DIR)/infrastructure/media/bev_gles/gles_render.cpp \
                  $(SRC_DIR)/infrastructure/media/bev_gles/gles_shader.cpp \
                  $(SRC_DIR)/infrastructure/media/bev_gles/gles_texture.cpp \
                  $(SRC_DIR)/infrastructure/media/bev_gles/gles_dma_texture.cpp \
                  $(SRC_DIR)/infrastructure/media/bev_gles/platform.cpp

 # Domain logic and calibration.
 DOMAIN_CXX_SRCS = $(SRC_DIR)/runtime/camera.cpp \
                   $(SRC_DIR)/runtime/reallink_cv_config.cpp \
                   $(SRC_DIR)/board_surface/board_surface_scan.cpp \
                   $(SRC_DIR)/board_surface/surface_quadric.cpp \
                   $(SRC_DIR)/detection/z_probe_common.cpp \
                   $(SRC_DIR)/detection/thickness.cpp \
                   $(SRC_DIR)/detection/core/YOLOSegmentor.cpp \
                   $(SRC_DIR)/detection/core/embedded_model.cpp \
                   $(SRC_DIR)/camera_calibration/viceCamera/vice_camera_service.cpp \
                   $(SRC_DIR)/camera_calibration/viceCamera/vice_camera_capture.cpp \
                   $(SRC_DIR)/camera_calibration/viceCamera/vice_camera_calibration.cpp \
                   $(SRC_DIR)/camera_calibration/viceCamera/vice_camera_utils.cpp \
                   $(SRC_DIR)/camera_calibration/viceCamera/vice_camera_extrinsics.cpp \
                   $(SRC_DIR)/camera_calibration/klipper/klipper_manager.cpp \
                   $(SRC_DIR)/camera_calibration/mainCamera/main_camera_fisheye_calib.cpp \
                   $(SRC_DIR)/camera_calibration/mainCamera/main_camera_default_intrinsics_refinement.cpp \
                   $(SRC_DIR)/camera_calibration/XYoffset.cpp \
                   $(SRC_DIR)/camera_calibration/totalHigh.cpp \
                   $(SRC_DIR)/camera_calibration/focalabfocal.cpp \
                   $(SRC_DIR)/camera_calibration/focal_range_normalizer.cpp \
                   $(SRC_DIR)/calibration_toolkit/charuco_detector.cpp \
                   $(SRC_DIR)/calibration_toolkit/ChessboardDetector.cpp \
                   $(SRC_DIR)/calibration_toolkit/calibinit.cpp \
                   $(SRC_DIR)/calibration_toolkit/camToolKit/calib_eeprom.cpp \
                   $(SRC_DIR)/calibration_toolkit/cbdetect/findCorners.cpp \
                   $(SRC_DIR)/calibration_toolkit/cbdetect/boards.cpp \
                   $(SRC_DIR)/calibration_toolkit/cbdetect/growBoard.cpp \
                   $(SRC_DIR)/calibration_toolkit/cbdetect/imgPatch.cpp \
                   $(SRC_DIR)/calibration_toolkit/cbdetect/plot_corners.cpp \
                   $(SRC_DIR)/calibration_toolkit/cbdetect/plot_boards.cpp \
                   $(SRC_DIR)/calibration_toolkit/saddleFit.cpp

 # App runtime.
 APP_CXX_SRCS = $(SRC_DIR)/app/stream_workers.cpp \
                $(SRC_DIR)/app/capture_loop.cpp \
                $(SRC_DIR)/app/app_runtime.cpp

 # External command interfaces.
 INTERFACE_CXX_SRCS = $(SRC_DIR)/interfaces/command_server/handlers/CalibCommandHandler.cpp \
                      $(SRC_DIR)/interfaces/command_server/handlers/DetectCommandHandler.cpp \
                      $(SRC_DIR)/interfaces/command_server/handlers/MetricsCommandHandler.cpp \
                      $(SRC_DIR)/interfaces/command_server/handlers/PatrolDetectCommandHandler.cpp \
                      $(SRC_DIR)/interfaces/command_server/protocol/UnifiedSocketServer.cpp

 # C++ source files
 CXX_SRCS = $(INFRA_CXX_SRCS) \
            $(DOMAIN_CXX_SRCS) \
            $(APP_CXX_SRCS) \
            $(INTERFACE_CXX_SRCS)

 # Object files
 OBJS = $(C_SRCS:.c=.o) $(CPP_SRCS:.cpp=.o)
 CXX_OBJS = $(CXX_SRCS:.cpp=.o)
 
 # Build rules
 
 .PHONY: all clean help

all: $(TARGET)

$(TARGET): $(OBJS) $(CXX_OBJS)
	@mkdir -p bin
	@echo "Linking $@..."
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)
	@echo "Build complete: $@"
	@ls -lh $@
	@echo "Removing object files..."
	@rm -f $(OBJS) $(CXX_OBJS)

%.o: %.c
	@echo "Compiling $<..."
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.cpp
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	@echo "Cleaning..."
	rm -f $(OBJS) $(CXX_OBJS) $(TARGET)
	rm -rf bin
	@echo "Clean complete"

help:
	@echo "Dual Stream RTSP Server - Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  all     - Build the dual stream server"
	@echo "  clean   - Remove build artifacts"
	@echo "  help    - Show this help message"
