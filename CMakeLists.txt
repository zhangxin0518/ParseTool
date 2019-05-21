cmake_minimum_required(VERSION 2.8)
project(ParseTool)

set(CMAKE_CXX_STANDARD 11)

set(OpenCV_DIR /usr/local/share/OpenCV/)
find_package(OpenCV REQUIRED)


set(LIBS
        ${OpenCV_LIBS}
	"pthread"
    )

set(SOURCE_FILES src/main.cpp 
	src/data_type.h 
	src/7zFile.h 
	src/7zTypes.h 
	src/7zVersion.h 
	src/Alloc.h 
	src/LzmaDec.h 
	src/LzmaEnc.h 
	src/LzmaDec.c
        src/Precomp.h 
	src/Compiler.h 
	src/Alloc.c )

add_executable(ParseTool ${SOURCE_FILES})
target_link_libraries(ParseTool ${LIBS} )
