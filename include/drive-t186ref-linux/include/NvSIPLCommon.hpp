/*
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef NVSIPLCOMMON_HPP
#define NVSIPLCOMMON_HPP

/**
 * @file
 *
 * @brief <b> NVIDIA SIPL: Common Data Structures  - @ref NvSIPL </b>
 *
 */

/** @namespace nvsipl
 *  @brief Contains the classes and variables for implementation of @ref NvSIPL.
 */
namespace nvsipl
{
/** @defgroup NvSIPL SIPL
 *
 * @brief  SIPL provides abstract and simple API to capture the output of image sensors
 * with optional image processing. SIPL is implemented on top of NvMedia API.
 *
 * @ingroup nvmedia_image_top
 */
/** @addtogroup NvSIPL
 * @{
 */

/** @brief Defines the status codes returned by functions in @ref NvSIPL modules. */
enum SIPLStatus
{
    // New status code must be added after NVSIPL_STATUS_OK and before NVSIPL_STATUS_ERROR.

    /** Indicates the operation completed successfully without errors. */
    NVSIPL_STATUS_OK = 0,

    // Error codes.
    /** Indicates one or more invalid arguments was encountered. */
    NVSIPL_STATUS_BAD_ARGUMENT,
    /** Indicates an unsupported operation or argument was encountered. */
    NVSIPL_STATUS_NOT_SUPPORTED,
    /** Indicates an out of memory or other system resource error was encountered. */
    NVSIPL_STATUS_OUT_OF_MEMORY,
    /** Indicates a resource error was encountered. */
    NVSIPL_STATUS_RESOURCE_ERROR,
    /** Indicates an operation timed out. */
    NVSIPL_STATUS_TIMED_OUT,
    /** Indicates a module is in an invalid state. */
    NVSIPL_STATUS_INVALID_STATE,
    /** Indicates that end of file has been reached. */
    NVSIPL_STATUS_EOF,

    /** Indicates a module was not initialized.
     * @note This enum will be deprecated in the future.
     */
    NVSIPL_STATUS_NOT_INITIALIZED,

    /** Indicates an unspecified error that is used when no other error code applies. */
    NVSIPL_STATUS_ERROR=127 // Assuming that we would not need more than 127 codes.
};

/** @} */

}  // namespace nvsipl


#endif // NVSIPLCOMMON_HPP
