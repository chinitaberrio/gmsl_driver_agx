# Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# CUDA dependency (needs to be after defining all configurations)
#-------------------------------------------------------------------------------
find_package(CUDA REQUIRED)
include_directories(${CUDA_TOOLKIT_INCLUDE})

#-------------------------------------------------------------------------------
# Build flags
#-------------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wunused -Wunused-value -Wunused-parameter")
    if((LINUX OR VIBRANTE_V5L) AND
       (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "5.0" OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL "5.0") AND
       (NOT ${CMAKE_PROJECT_NAME} STREQUAL DriveworksSDK-Samples))
        message(WARNING "Compiling with gcc >= 5.0 is experimental. Warnings will not be reported as errors.")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wno-error=deprecated-declarations")
    endif()

    if(NOT VIBRANTE)
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined -Wl,--as-needed")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-undefined -Wl,--as-needed")
    endif()
endif()

#-------------------------------------------------------------------------------
# CUDA configuration
#-------------------------------------------------------------------------------
set(CUDA_NVCC_FLAGS "-arch=sm_30;-lineinfo;-std=c++11;")
# Add debug info
# Note: This is disabled because it takes very long to do JIT compilation of kernels
#       with debug info. Enable this if you intend to do CUDA debugging.
# set(CUDA_NVCC_FLAGS_DEBUG "--debug;--device-debug;${CUDA_NVCC_FLAGS_DEBUG}")

#-------------------------------------------------------------------------------
# Configured headers
#-------------------------------------------------------------------------------
include_directories(${SDK_BINARY_DIR}/configured)
