cmake_minimum_required(VERSION 3.16)
project(tregor_orb_slam3_live LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED ORB_SLAM3_ROOT)
  set(ORB_SLAM3_ROOT "$ENV{ORB_SLAM3_ROOT}")
endif()

if(NOT ORB_SLAM3_ROOT)
  message(FATAL_ERROR "Set ORB_SLAM3_ROOT to your ORB-SLAM3 checkout (e.g. tools/vendor/ORB_SLAM3)")
endif()

list(APPEND CMAKE_MODULE_PATH "${ORB_SLAM3_ROOT}/cmake_modules")

find_package(OpenCV REQUIRED)
if(WIN32)
  find_package(Boost REQUIRED COMPONENTS serialization)
  find_package(Eigen3 CONFIG REQUIRED)
  find_package(Pangolin CONFIG REQUIRED)
else()
  find_package(Eigen3 REQUIRED)
  find_package(Pangolin REQUIRED)
endif()
find_package(Threads REQUIRED)

if(WIN32)
  set(_ORB_CANDIDATES
    "${ORB_SLAM3_ROOT}/build_win/Release/ORB_SLAM3.lib"
    "${ORB_SLAM3_ROOT}/lib/Release/ORB_SLAM3.lib"
    "${ORB_SLAM3_ROOT}/lib/ORB_SLAM3.lib"
    "${ORB_SLAM3_ROOT}/lib/Debug/ORB_SLAM3.lib")
  set(ORB_SLAM3_LIB "")
  foreach(_c ${_ORB_CANDIDATES})
    if(EXISTS "${_c}")
      set(ORB_SLAM3_LIB "${_c}")
      break()
    endif()
  endforeach()
  if(NOT ORB_SLAM3_LIB)
    message(FATAL_ERROR "ORB-SLAM3 Windows import library not found. Build ORB_SLAM3 first (MSVC Release). Searched: ${_ORB_CANDIDATES}")
  endif()
else()
  set(ORB_SLAM3_LIB "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so")
  if(NOT EXISTS "${ORB_SLAM3_LIB}")
    message(FATAL_ERROR "Expected ORB-SLAM3 library at ${ORB_SLAM3_LIB}")
  endif()
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
if(WIN32)
  target_link_libraries(orbslam3_stereo_live PRIVATE Boost::serialization)
endif()

if(TARGET Pangolin::pangolin)
  target_link_libraries(orbslam3_stereo_live PRIVATE Pangolin::pangolin)
else()
  target_link_libraries(orbslam3_stereo_live PRIVATE ${Pangolin_LIBRARIES})
endif()
