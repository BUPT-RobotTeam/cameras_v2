cmake_minimum_required(VERSION 3.12)
project(Cam_V2)

#------------------------------基本设置------------------------------
set(CMAKE_CXX_STANDARD 20)
set(MVCAM_COMMON_PATH /opt/MVS)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
file(GLOB CPP_SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB HPP_SRC ${PROJECT_SOURCE_DIR}/include/*.hpp)


#------------------------------包含头文件------------------------------
include_directories(
    ${MVCAM_COMMON_PATH}/include
    ${PROJECT_SOURCE_DIR}/include
)

#------------------------------查找库文件------------------------------
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(yaml-cpp REQUIRED)


add_executable(test 
    main.cpp 
    ${CPP_SRC}
    ${HPP_SRC}
)

target_link_libraries(test
    MvCameraControl 
    ${OpenCV_LIBS} 
    ${realsense2_LIBRARY}
    yaml-cpp::yaml-cpp
    pthread
)
