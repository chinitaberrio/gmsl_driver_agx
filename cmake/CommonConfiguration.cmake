# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# Debug symbols
#-------------------------------------------------------------------------------
# Enable minimal (level 1) debug info on experimental builds for
# informative stack trace including function names
if(SDK_BUILD_EXPERIMENTAL AND NOT CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g1")
endif()

# WAR for 32 bit linker - needed for DRIVE platform only    
# Store debug symbols in a separate file
# This is currently not supported by QCC
if(NOT VIBRANTE_V5Q)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -gsplit-dwarf")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -gsplit-dwarf")
endif()

#-------------------------------------------------------------------------------
# Enable C++11
#-------------------------------------------------------------------------------
if(CMAKE_VERSION VERSION_GREATER 3.1)
    set(CMAKE_CXX_STANDARD 14)
else()
    if(LINUX OR VIBRANTE)
        include(CheckCXXCompilerFlag)
        CHECK_CXX_COMPILER_FLAG(-std=c++14 COMPILER_SUPPORTS_CXX14)
        CHECK_CXX_COMPILER_FLAG(-std=c++1y COMPILER_SUPPORTS_CXX1Y)
        if(COMPILER_SUPPORTS_CXX14)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
        elseif(COMPILER_SUPPORTS_CXX1Y)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")
        else()
            message(ERROR "Compiler ${CMAKE_CXX_COMPILER} has no C++14 support")
        endif()
    endif()
endif()

#-------------------------------------------------------------------------------
# Dependencies
#-------------------------------------------------------------------------------
find_package(Threads REQUIRED)

#-------------------------------------------------------------------------------
# Profiling
#-------------------------------------------------------------------------------
if (CMAKE_BUILD_TYPE MATCHES "Profile")
    add_definitions(-DDW_PROFILING)
    set(DW_PROFILING TRUE)
endif()

#-------------------------------------------------------------------------------
# Find Nvidia driver version
#-------------------------------------------------------------------------------
# Determine currently installed linux driver major version hint
set(nvidia-driver-version_FOUND FALSE)
set(nvidia-driver-version "current")
if(NOT WIN32)
  if (EXISTS /proc/driver/nvidia/version)
    file(READ /proc/driver/nvidia/version nvidia-driver-version-file)
    if(${nvidia-driver-version-file} MATCHES "NVIDIA UNIX.*Kernel Module  ([0123456789]+)\\.[0123456789]+")
      set(nvidia-driver-version_FOUND TRUE)
      set(nvidia-driver-version ${CMAKE_MATCH_1})
    endif()
  endif()
endif()
