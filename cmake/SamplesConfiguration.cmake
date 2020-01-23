# Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# CUDA dependency (needs to be after defining all configurations)
#-------------------------------------------------------------------------------
find_package(CUDA REQUIRED)
include_directories(${CUDA_TOOLKIT_INCLUDE})
include_directories(${CUDA_CUBLAS_INCLUDE_DIR})

#-------------------------------------------------------------------------------
# Build flags
#-------------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wunused -Wunused-value -Wunused-parameter")

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
