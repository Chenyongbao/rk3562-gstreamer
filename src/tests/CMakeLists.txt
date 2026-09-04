cmake_minimum_required(VERSION 3.16)
project(reallinkCVserver_unit_tests LANGUAGES CXX)

# 单元测试在宿主机上构建/运行（不使用主工程的 ARM 交叉工具链），
# 只依赖纯 C++ 组件（FrameProvider / capture_state），不涉及 GStreamer/OpenCV/硬件。
#
# 用法（在项目根目录 src/ 下执行）：
#   cmake -S tests -B build-tests
#   cmake --build build-tests
#   ctest --test-dir build-tests --output-on-failure

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

find_package(Threads REQUIRED)

add_executable(unit_tests
    frame_provider_test.cpp
    capture_state_test.cpp
    ../pipeline/common/frame_provider.cpp
)

target_include_directories(unit_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(unit_tests PRIVATE
    gtest
    gtest_main
    Threads::Threads
)

# MinGW 下静态链接 C/C++ 运行时，避免 gtest_discover_tests 在构建期运行
# 测试可执行文件时因缺少 libstdc++/libgcc DLL 而失败（0xc0000139）。
if(MINGW)
    target_link_options(unit_tests PRIVATE -static-libgcc -static-libstdc++)
endif()

include(GoogleTest)
gtest_discover_tests(unit_tests)
