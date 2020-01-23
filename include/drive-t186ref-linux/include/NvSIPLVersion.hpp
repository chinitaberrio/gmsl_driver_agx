/*
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#ifndef _NVSIPLVERSION_HPP_
#define _NVSIPLVERSION_HPP_

#include "NvSIPLCommon.hpp"

#include <cstdint>

/**
 * @file
 *
 * <b> NVIDIA Sensor Input Processing Library: Version Interface - @ref NvSIPLCamera_API </b>
 *
 */

namespace nvsipl
{

/** @ingroup NvSIPLCamera_API
* @{
*/

/** @brief Holds the version information of @ref NvSIPLCamera_API */
typedef struct __NvSIPLVersion
{
    std::uint32_t uMajor; /**< Holds the major revision. */
    std::uint32_t uMinor; /**< Holds the minor revision. */
    std::uint32_t uPatch; /**< Holds the patch revision. */
} NvSIPLVersion;

#define NVSIPL_MAJOR_VER 0 /**< Indicates the major revision. */
#define NVSIPL_MINOR_VER 0 /**< Indicates the minor revision. */
#define NVSIPL_PATCH_VER 0 /**< Indicates the patch revision. */

/** @brief Gets the library version.
*
* Static function to get the version information of the library.
* @param[out] rVersion A reference to the object to copy the version information. */
void NvSIPLGetVersion(NvSIPLVersion& rVersion);

/** @} */

}  // namespace nvsipl

#endif //_NVSIPLVERSION_HPP_
