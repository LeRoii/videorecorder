cmake_minimum_required(VERSION 3.10)
project(MeshLab)


find_package(Protobuf REQUIRED)

include_directories(${Protobuf_INCLUDE_DIRS})
	
set(SOURCES
	test.pb.cpp
	test.cpp)

set(HEADERS
	test.pb.h
	)

set(RESOURCES 
    )

set(UI
	#mainwindow.ui
    )

add_executable(main WIN32 MACOSX_BUNDLE
${SOURCES} ${HEADERS} ${RESOURCES}
${UI})

target_include_directories(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(
	main
	PUBLIC
	${Protobuf_LIBRARIES}
	)
