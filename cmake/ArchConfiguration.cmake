# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# Platform selection
#-------------------------------------------------------------------------------

if(VIBRANTE)
    message(STATUS "Cross Compiling for Vibrante")
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
    set(WINDOWS TRUE)
    add_definitions(-DWINDOWS)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        set(LINUX TRUE)
        add_definitions(-DLINUX)
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
        set(VIBRANTE_V5Q TRUE)
        add_definitions(-DVIBRANTE_V5Q)
    #message(FATAL_ERROR "Direct compilation not supported for ${CMAKE_SYSTEM_PROCESSOR}, use cross compilation.")
    else()
        message(FATAL_ERROR "Unsupported Linux CPU architecture ${CMAKE_SYSTEM_PROCESSOR}.")
    endif()
else()
    message(FATAL_ERROR "Cannot identify OS")
endif()

# Position independent code enforce for 64bit systems and armv7l
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
   set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

#-------------------------------------------------------------------------------
# Architecture selection
#-------------------------------------------------------------------------------
if(VIBRANTE)
    # Select device architecture
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        if(VIBRANTE_V5Q)
            # Qnx arm64
            set(ARCH_DIR     "qnx-aarch64")
            set(C_ARCH_DIR   "qnx-aarch64/abiC")
            set(CPP_ARCH_DIR "qnx-aarch64/abi11") # V5Q is already C++11ABI exclusively
        else()
            # Linux arm64
            set(ARCH_DIR     "linux-aarch64")
            set(C_ARCH_DIR   "linux-aarch64/abiC")
            set(CPP_ARCH_DIR "linux-aarch64/abi98")
        endif()
    else()
        # Linux arm7
        set(ARCH_DIR     "linux-armv7l")
        set(C_ARCH_DIR   "linux-armv7l")
        set(CPP_ARCH_DIR "linux-armv7l")
    endif()
else()
    # Linux x86_64
    set(ARCH_DIR     "linux-x86")
    set(C_ARCH_DIR   "linux-x86/abiC")
    set(CPP_ARCH_DIR "linux-x86/abi98")
endif()

if(NOT ${CMAKE_PROJECT_NAME} STREQUAL DriveworksSDK-Samples)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND NOT VIBRANTE_V5Q) # V5Q is C++11ABI exclusively
        if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "5.0" OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL "5.0")
            option(DW_EXPERIMENTAL_ENABLE_ABI11 "Enable experimental C++11ABI build (needs to be explicitly requested as non-official)" OFF)

            if(NOT DW_EXPERIMENTAL_ENABLE_ABI11)
                message(FATAL_ERROR "Compilation with >g++-4.9 is not officially supported "
                                    "(partial experimental mode is enabled with 'DW_EXPERIMENTAL_ENABLE_ABI11') "
                                    ""
                                    "Either switch to a compatible compiler, e.g., with the `CXX=/usr/bin/g++-4.9 CC=/usr/bin/gcc-4.9` "
                                    "environment variables, or set compiler in cmake with -DCMAKE_CXX_COMPILER=g++-4.9 -DCMAKE_C_COMPILER=gcc-4.9,"
                                    "or enable 'DW_EXPERIMENTAL_ENABLE_ABI11'")
            endif()

            message(WARNING "Using gcc (${CMAKE_CXX_COMPILER_VERSION}) >= 5.0, switching to C++11 ABI. "
                    "This is not officially supported and many modules will be disabled due to missing dependencies")

            set(CPP_ARCH_DIR "${ARCH_DIR}/abi11")
        endif()
    endif()
endif()

# Dependencies that are C++ abi dependent are stored under SDK_CPP_ARCH_DIR
unset(SDK_CPP_ARCH_DIR CACHE)
set(SDK_CPP_ARCH_DIR ${CPP_ARCH_DIR} CACHE INTERNAL "")

# Dependencies that are C only and don't care about abi are stored under SDK_C_ARCH_DIR
unset(SDK_C_ARCH_DIR CACHE)
set(SDK_C_ARCH_DIR ${C_ARCH_DIR} CACHE INTERNAL "")

# Generic top-level architecture subfolder name SDK_ARCH_DIR
unset(SDK_ARCH_DIR CACHE)
set(SDK_ARCH_DIR ${ARCH_DIR} CACHE INTERNAL "")
