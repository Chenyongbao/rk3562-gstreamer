# Toolchain file for RK3562 cross-compilation
#
# Usage:
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain_rk3562.cmake \
#                  -DRK3562_SDK_ROOT=/path/to/rk3562          \
#                  -DSYSROOT=/path/to/sysroot                  \
#                  -DGST_RTSP_SERVER_ROOT=/path/to/gst-rtsp-server/install
#
# Or set the corresponding environment variables before running cmake.

# Target architecture
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Toolchain — override TOOLCHAIN_ROOT env / cache variable
set(TOOLCHAIN_ROOT "$ENV{TOOLCHAIN_ROOT}"
    CACHE PATH "Cross-compiler toolchain root")
if(NOT TOOLCHAIN_ROOT)
    set(TOOLCHAIN_ROOT "/home/yuheng.song/rk3562/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu"
        CACHE PATH "Cross-compiler toolchain root (default)")
endif()

set(CROSS_COMPILE "${TOOLCHAIN_ROOT}/bin/aarch64-none-linux-gnu-")

set(CMAKE_C_COMPILER   "${CROSS_COMPILE}gcc")
set(CMAKE_CXX_COMPILER "${CROSS_COMPILE}g++")
set(CMAKE_STRIP        "${CROSS_COMPILE}strip")
set(CMAKE_AR           "${CROSS_COMPILE}ar")
set(CMAKE_RANLIB       "${CROSS_COMPILE}ranlib")

# Sysroot
if(NOT CMAKE_SYSROOT)
    set(CMAKE_SYSROOT "$ENV{SYSROOT}"
        CACHE PATH "Sysroot directory for cross-compilation")
    if(NOT CMAKE_SYSROOT)
        set(CMAKE_SYSROOT "${RK3562_SDK_ROOT}/debian/binary"
            CACHE PATH "Sysroot directory for cross-compilation")
    endif()
endif()

# pkg-config: tell it to look inside the sysroot
set(PKG_CONFIG_EXECUTABLE "${CROSS_COMPILE}pkg-config" CACHE FILEPATH "pkg-config for target")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

add_compile_options(--sysroot=${CMAKE_SYSROOT})
add_link_options(--sysroot=${CMAKE_SYSROOT})

# Linker search path within the sysroot
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -B${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -B${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu")
