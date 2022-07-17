cmake_minimum_required(VERSION 2.4)
project(videoCapture)

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
    endif(COMMAND cmake_policy)


message("msg CMAKE_PREFIX_PATH:::${CMAKE_PREFIX_PATH}")
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4/OpenCVConfig.cmake")
find_package( OpenCV 3 REQUIRED )
add_compile_options(-std=c++11 )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -pthread -fopenmp")


set(COMMONCLS /home/nx/xavier1MulitimediaAPI/samples/common/classes)
set(COMMONCLSFILES
    ${COMMONCLS}/NvBuffer.cpp
    ${COMMONCLS}/NvElement.cpp
    ${COMMONCLS}/NvElementProfiler.cpp
    ${COMMONCLS}/NvEglRenderer.cpp
    ${COMMONCLS}/NvLogging.cpp
    ${COMMONCLS}/NvV4l2Element.cpp
    ${COMMONCLS}/NvV4l2ElementPlane.cpp
    ${COMMONCLS}/NvVideoEncoder.cpp
    ${COMMONCLS}/NvVideoDecoder.cpp
    ${COMMONCLS}/NvLogging.cpp)

include_directories(
    /home/nx/xavier1MulitimediaAPI/include
    /home/nx/xavier1MulitimediaAPI/samples/common/algorithm/cuda/
    )

add_executable(videoCapture main.cpp )
target_link_libraries( videoCapture ${OpenCV_LIBS} )

add_executable(nvvideoc camera_v4l2_cuda.cpp ${COMMONCLSFILES})
target_link_libraries(nvvideoc  
    -L/usr/lib/aarch64-linux-gnu/tegra/ -lnvbuf_utils  
    -L/usr/lib/aarch64-linux-gnu/ -lv4l2 -lEGL -lGLESv2 -lX11 
    ${CUDA_LIBRARIES} ${GLUT_LIBRARY} ${OpenCV_LIBS} 
)


find_package(CSerialPort QUIET)


# IF( UNIX )
#     find_package(PkgConfig)
#     pkg_search_module( CSERIAL REQUIRED cserial )
# ENDIF( UNIX )

add_compile_options(-std=c++11 )

#add_executable( cserial-example serialrecorder.cpp )
#target_link_libraries( cserial-example ${CSERIAL_LIBRARIES} )

add_executable( serialrecorder CSerialPortDemoNoGui.cpp)
include_directories(${CSerialPort_INCLUDE_DIR})
target_link_libraries (serialrecorder ${CSerialPort_LIBRARY})
