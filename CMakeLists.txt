cmake_minimum_required(VERSION 3.8)
project(argus_vpi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" "${CMAKE_MODULE_PATH}")

# include_directories("/usr/include/opencv4")
# link_directories("/usr/lib/aarch64-linux-gnu")
set(OpenCV_DIR /usr/lib/aarch64-linux-gnu/cmake/opencv4)

find_package(Argus REQUIRED)
find_package(OpenGLES REQUIRED)
find_package(EGL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vpi 3.2 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBDRM REQUIRED libdrm)

find_library(V4L2_LIB v4l2 REQUIRED)

message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dir: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")


add_subdirectory(utils)

pkg_check_modules(LIBDRM REQUIRED libdrm)
include_directories(${LIBDRM_INCLUDE_DIRS})

include_directories(
    include
    ${ARGUS_INCLUDE_DIR}
    ${EGL_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/utils
    include/libjpeg-8b
)

add_library(${PROJECT_NAME}_lib SHARED
	#src/argus_consumer.cpp
	#src/argus_camera_node.cpp

    src/NvApplicationProfiler.cpp
    src/NvBuffer.cpp
    src/NvBufSurface.cpp
    src/NvDrmRenderer.cpp
    src/NvEglRenderer.cpp
    src/NvElement.cpp
    src/NvElementProfiler.cpp
    src/NvJpegDecoder.cpp
    src/NvJpegEncoder.cpp
    src/NvLogging.cpp
    src/NvUtils.cpp
    src/NvV4l2Element.cpp
    src/NvV4l2ElementPlane.cpp
    # src/NvVideoEncoder.cpp
    # src/NvVideoDecoder.cpp
)

add_executable(${PROJECT_NAME}
   src/main.cpp
)

set(NVV4L2_LIB_PATH /usr/lib/aarch64-linux-gnu/libv4l2.so)
set(NVTRANS_LIB_PATH /usr/lib/aarch64-linux-gnu/nvidia/libnvbufsurftransform.so)
target_link_libraries(${PROJECT_NAME}
   ${ARGUS_LIBRARIES}
   ${OpenCV_LIBS}
   argussampleutils
   ${PROJECT_NAME}_lib
   ${LIBDRM_LIBRARIES} 
   vpi
    ${NVTRANS_LIB_PATH}
    ${NVV4L2_LIB_PATH}
   )



# Compiling: main.cpp
# make[1]: Entering directory '/usr/src/jetson_multimedia_api/samples/common/classes'
# Compiling: NvApplicationProfiler.cpp
# Compiling: NvBuffer.cpp
# Compiling: NvBufSurface.cpp
# Compiling: NvDrmRenderer.cpp
# Compiling: NvEglRenderer.cpp
# Compiling: NvElement.cpp
# Compiling: NvElementProfiler.cpp
# Compiling: NvJpegDecoder.cpp
# Compiling: NvJpegEncoder.cpp
# Compiling: NvLogging.cpp
# Compiling: NvUtils.cpp
# Compiling: NvV4l2Element.cpp
# Compiling: NvV4l2ElementPlane.cpp
# Compiling: NvVideoDecoder.cpp
# Compiling: NvVideoEncoder.cpp
# make[1]: Leaving directory '/usr/src/jetson_multimedia_api/samples/common/classes'
# Compiling: /usr/src/jetson_multimedia_api/argus/samples/utils/Thread.cpp
# Compiling: /usr/src/jetson_multimedia_api/argus/samples/utils/NativeBuffer.cpp
# Compiling: /usr/src/jetson_multimedia_api/argus/samples/utils/nvmmapi/NvNativeBuffer.cpp
# Linking: argus_camera_recording