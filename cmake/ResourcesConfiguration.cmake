# Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# Set CuDNN and TensorRT version
# TRT_VERSION is cmake command line parameter
#-------------------------------------------------------------------------------

if(DEFINED TRT_VERSION)
    if(VIBRANTE_BUILD)        
        if(TRT_VERSION STREQUAL "5.0.3.2" AND VIBRANTE_PDK_VERSION STREQUAL 5.1.0.2)
            set(CUDNN_VERSION 7.3)
            set(DIT_VERSION_STR 05_00_03_02 CACHE INTERNAL "")
            set(RESOURCES_VERSION trt_05_00_03_02)
            set(NVDLA_VERSION 01_01_00)
            set(REQUIRED_CUDA_VERSION 10.0)
            message(STATUS "Building with TRT version ${TRT_VERSION} and "
                    "cudnn version ${CUDNN_VERSION}")
        elseif(TRT_VERSION STREQUAL "5.1.4.2" AND (VIBRANTE_PDK_VERSION STREQUAL 5.1.6.0 OR VIBRANTE_PDK_VERSION STREQUAL 5.1.6.1))
            set(CUDNN_VERSION 7.5.1.14)
            set(DIT_VERSION_STR 05_01_04_02 CACHE INTERNAL "")
            set(RESOURCES_VERSION trt_05_01_04_02)
            set(NVDLA_VERSION 01_03_00)
            set(REQUIRED_CUDA_VERSION 10.2)
            message(STATUS "Building with TRT version ${TRT_VERSION} and cudnn version "
                    "${CUDNN_VERSION}, pdk version ${VIBRANTE_PDK_VERSION}")
        elseif(TRT_VERSION STREQUAL "6.0.1.0" AND VIBRANTE_PDK_VERSION STREQUAL 5.1.9.0)
            set(CUDNN_VERSION 7.6.3.8)
            set(DIT_VERSION_STR 06_00_01_00 CACHE INTERNAL "")
            set(RESOURCES_VERSION trt_06_00_01_00)
            set(NVDLA_VERSION 01_04_00)
            set(REQUIRED_CUDA_VERSION 10.2)
            message(STATUS "Building with TRT version ${TRT_VERSION} and cudnn version "
                    "${CUDNN_VERSION}, pdk version ${VIBRANTE_PDK_VERSION}")
        else()
            message(FATAL_ERROR "Combination of TRT version ${TRT_VERSION} and "
                    "PDK version ${VIBRANTE_PDK_VERSION} not supported")
        endif()
    else()
       if(TRT_VERSION STREQUAL "5.0.3.2")
            set(CUDNN_VERSION 7.3)
            set(DIT_VERSION_STR 05_00_03_02 CACHE INTERNAL "")
            set(RESOURCES_VERSION trt_05_00_03_02)
            set(NVDLA_VERSION 01_01_00)
            set(REQUIRED_CUDA_VERSION 10.0)
            message(STATUS "Building with TRT version ${TRT_VERSION} and "
                    "cudnn version ${CUDNN_VERSION}")
        elseif(TRT_VERSION STREQUAL "5.1.4.2")
            set(CUDNN_VERSION 7.5.1.14)
            set(DIT_VERSION_STR 05_01_04_02 CACHE INTERNAL "")
            set(RESOURCES_VERSION trt_05_01_04_02)
            set(NVDLA_VERSION 01_03_00)
            set(REQUIRED_CUDA_VERSION 10.2)
            message(STATUS "Building with TRT version ${TRT_VERSION} and "
                    "cudnn version ${CUDNN_VERSION}")
        elseif(TRT_VERSION STREQUAL "5.1.5.0")
            set(CUDNN_VERSION 7.5.0.56)
            set(DIT_VERSION_STR 05_01_05_00 CACHE INTERNAL "")
            set(RESOURCES_VERSION trt_05_01_05_00)
            set(NVDLA_VERSION 01_03_00)
            set(REQUIRED_CUDA_VERSION 10.1)
            message(STATUS "Building with TRT version ${TRT_VERSION} and "
                    "cudnn version ${CUDNN_VERSION}")
        elseif(TRT_VERSION STREQUAL "6.0.1.0")
            set(CUDNN_VERSION 7.6.3.8)
            set(DIT_VERSION_STR 06_00_01_00 CACHE INTERNAL "")
            set(RESOURCES_VERSION trt_06_00_01_00)
            set(NVDLA_VERSION 01_04_00)
            set(REQUIRED_CUDA_VERSION 10.2)
            message(STATUS "Building with TRT version ${TRT_VERSION} and "
                    "cudnn version ${CUDNN_VERSION}")

        else()
            set(CUDNN_VERSION 7.5.1.14)
            set(DIT_VERSION_STR 05_01_04_02 CACHE INTERNAL "")
            set(RESOURCES_VERSION trt_05_01_04_02)
            set(NVDLA_VERSION 01_03_00)
            set(REQUIRED_CUDA_VERSION 10.2)
            message(STATUS "Building with TRT version ${TRT_VERSION} and "
                    "cudnn version ${CUDNN_VERSION}")
        endif()
    endif()
else()
    if(VIBRANTE_BUILD)        
        if(VIBRANTE_PDK_VERSION STREQUAL 5.1.0.2)
            set(CUDNN_VERSION 7.3)
            set(DIT_VERSION_STR 05_00_03_02 CACHE INTERNAL "")
            set(RESOURCES_VERSION trt_05_00_03_02)
            set(NVDLA_VERSION 01_01_00)
            set(REQUIRED_CUDA_VERSION 10.0)
            message(STATUS "Default - Building with TRT version 5.0.3.2 and "
                    "cudnn version ${CUDNN_VERSION}")
        elseif(VIBRANTE_PDK_VERSION STREQUAL 5.1.6.0 OR VIBRANTE_PDK_VERSION STREQUAL 5.1.6.1)
            set(CUDNN_VERSION 7.5.1.14)
            set(DIT_VERSION_STR 05_01_04_02 CACHE INTERNAL "")
            set(RESOURCES_VERSION trt_05_01_04_02)
            set(NVDLA_VERSION 01_03_00)
            set(REQUIRED_CUDA_VERSION 10.2)
            message(STATUS "Default - Building with TRT version 5.1.4.2 and "
                    "cudnn version ${CUDNN_VERSION}")
        elseif(VIBRANTE_PDK_VERSION STREQUAL 5.1.9.0)
            set(CUDNN_VERSION 7.6.3.8)
            set(DIT_VERSION_STR 06_00_01_00 CACHE INTERNAL "")
            set(RESOURCES_VERSION trt_06_00_01_00)
            set(NVDLA_VERSION 01_04_00)
            set(REQUIRED_CUDA_VERSION 10.2)
            message(STATUS "Default - Building with TRT version 6.0.1.0 and "
                    "cudnn version ${CUDNN_VERSION}")

        endif()
    else()
        set(CUDNN_VERSION 7.5.1.14)
        set(DIT_VERSION_STR 05_01_04_02 CACHE INTERNAL "")
        set(RESOURCES_VERSION trt_05_01_04_02)
        set(NVDLA_VERSION 01_03_00)
        set(REQUIRED_CUDA_VERSION 10.2)
        message(STATUS "Building with TRT version ${TRT_VERSION} and "
                "cudnn version ${CUDNN_VERSION}")
    endif()
endif()

# Remove the prefix "DIT_" and create a list
string(REGEX REPLACE "DIT_" "" TRT_VERSION_ALL ${DIT_VERSION_STR})
string(REGEX REPLACE "_" ";" TRT_VERSION_ALL ${TRT_VERSION_ALL})
# Extract TRT sub versions
list(GET TRT_VERSION_ALL 0 TRT_MAJOR)
list(GET TRT_VERSION_ALL 1 TRT_MINOR)
list(GET TRT_VERSION_ALL 2 TRT_PATCH)
list(GET TRT_VERSION_ALL 3 TRT_BUILD)

math(EXPR TRT_VERSION_DECIMAL "${TRT_MAJOR} * 1000000 + \
                               ${TRT_MINOR} * 10000 + \
                               ${TRT_PATCH} * 100 + \
                               ${TRT_BUILD}")

string(REGEX REPLACE "_" ";" NVDLA_VERSION_ALL ${NVDLA_VERSION})
list(GET NVDLA_VERSION_ALL 0 NVDLA_MAJOR)
list(GET NVDLA_VERSION_ALL 1 NVDLA_MINOR)
list(GET NVDLA_VERSION_ALL 2 NVDLA_PATCH)

math(EXPR NVDLA_VERSION_DECIMAL "${NVDLA_MAJOR} * 10000 + \
                                 ${NVDLA_MINOR} * 100 + \
                                 ${NVDLA_PATCH}")

#-------------------------------------------------------------------------------
# Set resources folder and filename
#-------------------------------------------------------------------------------
set(DW_RESOURCES_PARENT_DIR "resources")
set(DW_RESOURCES_DATA_DIR "${DW_RESOURCES_PARENT_DIR}/${RESOURCES_VERSION}")
set(DW_RESOURCES_PAK_NAME "resources.pak")
