cmake_minimum_required(VERSION 2.4)
project(videoCapture)

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
    endif(COMMAND cmake_policy)


message("msg CMAKE_PREFIX_PATH:::${CMAKE_PREFIX_PATH}")
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4/OpenCVConfig.cmake")
find_package( OpenCV 3 REQUIRED )
find_package(CSerialPort QUIET)
find_package(Protobuf REQUIRED)


add_compile_options(-std=c++11 )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -pthread -fopenmp")


set(COMMONCLS /home/nx/xavier1MulitimediaAPI/samples/common/classes)
# set(COMMONCLS /usr/src/jetson_multimedia_api/samples/common/classes)
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

set(PB_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/record/test.pb.cpp)

include_directories(
    /home/nx/xavier1MulitimediaAPI/include
    /home/nx/xavier1MulitimediaAPI/samples/common/algorithm/cuda/
    ${CMAKE_CURRENT_SOURCE_DIR}/record/
    )

# include_directories(
#     /usr/src/jetson_multimedia_api/include
#     /usr/src/jetson_multimedia_api/samples/common/algorithm/cuda/
#     ${CMAKE_CURRENT_SOURCE_DIR}/record/
#     )

# add_executable(videoCapture main.cpp )
# target_link_libraries( videoCapture ${OpenCV_LIBS} )

add_executable(readfile readfile.cpp )
target_link_libraries( readfile ${OpenCV_LIBS} )

# add_executable(nvvideoc camera_v4l2_cuda.cpp ${COMMONCLSFILES} ${PB_SOURCE})
# target_link_libraries(nvvideoc  
#     -L/usr/lib/aarch64-linux-gnu/tegra/ -lnvbuf_utils  
#     -L/usr/lib/aarch64-linux-gnu/ -lv4l2 -lEGL -lGLESv2 -lX11 
#     ${CUDA_LIBRARIES} ${GLUT_LIBRARY} ${OpenCV_LIBS} 
#     ${CSerialPort_LIBRARY}
#     ${Protobuf_LIBRARIES}
# )

add_executable(nvvideoc1024 camera_v4l2_cuda-1024.cpp serial.cpp ${COMMONCLSFILES} ${PB_SOURCE})
target_link_libraries(nvvideoc1024  
    -L/usr/lib/aarch64-linux-gnu/tegra/ -lnvbuf_utils  
    -L/usr/lib/aarch64-linux-gnu/ -lv4l2 -lEGL -lGLESv2 -lX11 
    ${CUDA_LIBRARIES} ${GLUT_LIBRARY} ${OpenCV_LIBS} 
    ${CSerialPort_LIBRARY}
    ${Protobuf_LIBRARIES}
)





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
