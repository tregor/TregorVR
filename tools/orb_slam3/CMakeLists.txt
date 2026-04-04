cmake_minimum_required(VERSION 3.10)
project(tregor_orb_slam3_live LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED ORB_SLAM3_ROOT)
  set(ORB_SLAM3_ROOT "$ENV{ORB_SLAM3_ROOT}")
endif()

if(NOT ORB_SLAM3_ROOT)
  message(FATAL_ERROR "Set ORB_SLAM3_ROOT to your ORB-SLAM3 checkout")
endif()

list(APPEND CMAKE_MODULE_PATH "${ORB_SLAM3_ROOT}/cmake_modules")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Threads REQUIRED)

set(ORB_SLAM3_LIB "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so")
if(NOT EXISTS "${ORB_SLAM3_LIB}")
  message(FATAL_ERROR "Expected ORB-SLAM3 library at ${ORB_SLAM3_LIB}")
endif()

add_executable(orbslam3_stereo_live live_stereo_rectified.cpp)

target_include_directories(orbslam3_stereo_live PRIVATE
  "${ORB_SLAM3_ROOT}"
  "${ORB_SLAM3_ROOT}/include"
  "${ORB_SLAM3_ROOT}/include/CameraModels"
  "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
)

if(DEFINED Pangolin_INCLUDE_DIRS)
  target_include_directories(orbslam3_stereo_live PRIVATE ${Pangolin_INCLUDE_DIRS})
endif()

target_link_libraries(orbslam3_stereo_live PRIVATE
  "${ORB_SLAM3_LIB}"
  ${OpenCV_LIBS}
  Eigen3::Eigen
  Threads::Threads
)

if(TARGET Pangolin::pangolin)
  target_link_libraries(orbslam3_stereo_live PRIVATE Pangolin::pangolin)
else()
  target_link_libraries(orbslam3_stereo_live PRIVATE ${Pangolin_LIBRARIES})
endif()
