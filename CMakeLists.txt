cmake_minimum_required(VERSION 2.8.3)
project(sekonix_camera)

set(CMAKE_CXX_STANDARD 14)

set(VIBRANTE TRUE)
add_definitions(-DVIBRANTE)
# this is the path I placed the driveworks cmake files in
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(glfw_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/linux-aarch64/glfw-3.1.1/include)
set(glfw_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/linux-aarch64/glfw-3.1.1/lib)
set(drive_t186ref_linux_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/drive-t186ref-linux/include)

link_directories(${glfw_LIBRARY})

find_package(Threads REQUIRED)
find_package(Driveworks REQUIRED)
find_package(OpenCV REQUIRED)

# SET THESE VARIABLES TO YOUR DRIVEWORKS LOCATIONS
set(Driveworks_INCLUDE_DIR /usr/local/driveworks/include/)
set(Driveworks_LIBRARY /usr/local/driveworks/lib/)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        cv_bridge
        image_transport
        camera_info_manager
        gmsl_frame_msg
        )

catkin_package(
        LIBRARIES ${PROJECT_NAME}
)
link_directories(
        ${Driveworks_LIBRARY})


MESSAGE("Driveworks include directory ${Driveworks_INCLUDE_DIR}")

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Driveworks_INCLUDE_DIR}
        ${CUDA_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${glfw_INCLUDE_DIR}
        ${drive_t186ref_linux_INCLUDE_DIR}
        include
        include/framework
)

# ros node name template
set(NODE_NAME ${PROJECT_NAME}_node)

add_executable(${NODE_NAME}
        src/main.cpp
        src/SekonixCamera.cpp
        src/DriveWorksApi.cpp
        src/DeviceArguments.cpp
        src/cv_connection.cpp
        src/CameraPort.cpp
        src/StopWatch.cpp
        src/PrintEventHandler.cpp)

target_link_libraries(${NODE_NAME}
        ${catkin_LIBRARIES}
        nvmedia
        driveworks
        GLESv2
        EGL
        drm
        ${CUDA_LIBRARIES}
        ${OpenCV_LIBS}
        )
