# Copyright (c) 2016-2020 NVIDIA Corporation. All rights reserved.
set(CODEOWNERS @jdokter)

project(samples_framework C CXX)

#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------
set(SOURCES
    Checks.hpp
    SampleFramework.hpp
    SampleFramework.cpp

    DriveWorksSample.hpp
    DriveWorksSample.cpp

    ProgramArguments.hpp
    ProgramArguments.cpp

    Window.hpp
    Window.cpp

    Grid.hpp
    Grid.cpp

    WindowGLFW.hpp
    WindowGLFW.cpp

    MathUtils.hpp
    MathUtils.cpp
    MouseView3D.cpp
    MouseView3D.hpp

    Mat3.hpp
    Mat4.hpp

    Log.hpp
    Log.cpp

    StatsCounter.hpp
    CudaTimer.hpp

    ProfilerCUDA.hpp
    ProfilerCUDA.cpp

    SimpleStreamer.hpp
    CameraFramePipeline.hpp
    CameraFramePipeline.cpp
    SimpleCamera.hpp
    SimpleCamera.cpp

    SimpleRenderer.cpp
    SimpleRenderer.hpp
    SimpleRecordingPlayer.hpp
    SimpleRecordingPlayer.cpp

    ScreenshotHelper.hpp
    ScreenshotHelper.cpp
    RenderUtils.hpp
    RenderUtils.cpp

    DataPath.hpp
    DataPath.cpp
)

set(LIBRARIES
    glfw-src
    lodepng-src
    drm
    cuda-libs
    ${Driveworks_LIBRARIES}
    )

if(DW_USE_EGL)
    list(APPEND SOURCES
         WindowEGL.hpp
         WindowEGL.cpp
    )

    if(VIBRANTE_V5Q)
        list(APPEND SOURCES
            WindowQNXEGL.hpp
            WindowQNXEGL.cpp
        )
        list(APPEND LIBRARIES
            screen
        )
    elseif(VIBRANTE_V5L OR LINUX)
        list(APPEND SOURCES
            WindowLinuxEGL.hpp
            WindowLinuxEGL.cpp
        )
    endif()
endif()

if(LINUX)
    list(APPEND LIBRARIES
        glew-src
    )
endif()

if(VIBRANTE)
    list(APPEND LIBRARIES
        vibrante
        nvmedia
        gles
    )

    if(VIBRANTE_V5L)
        list(APPEND LIBRARIES
            vibrante_Xlibs
        )
    endif()
endif()

#-------------------------------------------------------------------------------
# Final target
#-------------------------------------------------------------------------------
add_library(${PROJECT_NAME} STATIC ${SOURCES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBRARIES})
target_include_directories(${PROJECT_NAME}
                           PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}/..
                           ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Samples")

#-------------------------------------------------------------------------------
# Special aligned allocator library
#-------------------------------------------------------------------------------
add_library(samples_allocator STATIC Allocator.cpp)
target_link_libraries(samples_allocator PUBLIC dwdynamicmemory)

#------------------------------------------------------------------------------
# No Cuda
#------------------------------------------------------------------------------
set(NOCUDA_PROJECT_NAME ${PROJECT_NAME}_nocuda)

set(SOURCES
    ProgramArguments.cpp
    ProgramArguments.hpp
)

add_library(${NOCUDA_PROJECT_NAME} STATIC ${SOURCES})
target_include_directories(${NOCUDA_PROJECT_NAME}
                           PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}/..
                           ${CMAKE_CURRENT_SOURCE_DIR}/../..)
