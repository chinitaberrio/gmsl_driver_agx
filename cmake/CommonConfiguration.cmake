# Copyright (c) 2016-2019 NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# Debug symbols
#-------------------------------------------------------------------------------
# Enable dynamic symbols (.dynsym) info on experimental-release builds for
# informative stack trace including function names
if(SDK_BUILD_EXPERIMENTAL AND CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -rdynamic")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic")
endif()

# WAR for 32 bit linker - needed for DRIVE platform only
# Store debug symbols in a separate file
# This is currently not supported by QCC
if(NOT VIBRANTE_V5Q)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -gsplit-dwarf")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -gsplit-dwarf")
endif()

#-------------------------------------------------------------------------------
# Enable C++14 and C11
#-------------------------------------------------------------------------------
if(CMAKE_VERSION VERSION_GREATER 3.1)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
    set(CMAKE_C_STANDARD_REQUIRED TRUE)
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

        include(CheckCCompilerFlag)
        CHECK_C_COMPILER_FLAG(-std=c11 COMPILER_SUPPORTS_C11)
        if(COMPILER_SUPPORTS_C11)
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
        else()
            message(ERROR "Compiler ${CMAKE_C_COMPILER} has no C11 support")
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
if (EXISTS /proc/driver/nvidia/version)
  file(READ /proc/driver/nvidia/version nvidia-driver-version-file)
  if(${nvidia-driver-version-file} MATCHES "NVIDIA UNIX.*Kernel Module  ([0123456789]+)\\.[0123456789]+")
    set(nvidia-driver-version_FOUND TRUE)
    set(nvidia-driver-version ${CMAKE_MATCH_1})
  endif()
endif()
if(NOT nvidia-driver-version_FOUND)
    message(WARNING "Couldn't determine the nvidia driver version: /proc/driver/nvidia/version was not found. libnvcuvid and nivida-ml might be missing.")
endif()

