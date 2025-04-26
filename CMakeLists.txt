cmake_minimum_required(VERSION 3.10)
project(PointCloudViewer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV (version 4.2.0 or compatible)
find_package(OpenCV 4.2.0 REQUIRED)

# Find GStreamer components
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}
)

link_directories(
    ${GSTREAMER_LIBRARY_DIRS}
)

add_executable(lidar_visualization_from_gst lidar_visualization_from_gst.cpp)
target_link_libraries(lidar_visualization_from_gst
    ${OpenCV_LIBS}
    ${GSTREAMER_LIBRARIES}
    pthread
)
