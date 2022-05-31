cmake_minimum_required(VERSION 2.4)
project(videoCapture)

message("msg CMAKE_PREFIX_PATH:::${CMAKE_PREFIX_PATH}")
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4/OpenCVConfig.cmake")
find_package( OpenCV  REQUIRED )
add_compile_options(-std=c++11 )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -pthread -fopenmp")

add_executable(videoCapture main.cpp)
target_link_libraries( videoCapture ${OpenCV_LIBS} )
