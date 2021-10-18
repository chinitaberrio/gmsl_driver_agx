/*
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#ifndef NVSIPLVERSION_HPP
#define NVSIPLVERSION_HPP

#include <cstdint>

namespace nvsipl
{

/**
 *  @defgroup NvSIPLVersion NvSIPL Version
 *
 * Holds the version information for @ref NvSIPLCamera_API and
 * @ref NvSIPLClient_API.
 *
 * @ingroup NvSIPLCamera_API
 * @{
 */

/** @brief Holds the version information of @ref NvSIPLCamera_API and @ref NvSIPLClient_API.
*
* @par
* Version history of @ref NvSIPLCamera_API and @ref NvSIPLClient_API
* @par
* Version <b> 1.0.0 </b> - 9th September 2019 - Alpha version.
*/
struct NvSIPLVersion
{
    std::uint32_t uMajor; /**< Holds the major revision. */
    std::uint32_t uMinor; /**< Holds the minor revision. */
    std::uint32_t uPatch; /**< Holds the patch revision. */
};

#define NVSIPL_MAJOR_VER 1 /**< Indicates the major revision. */
#define NVSIPL_MINOR_VER 0 /**< Indicates the minor revision. */
#define NVSIPL_PATCH_VER 0 /**< Indicates the patch revision. */

/** @brief Gets the library version.
*
* Static function to get the version information of the library.
* @param[out] rVersion A reference to the object to copy the version information.
*/
void NvSIPLGetVersion(NvSIPLVersion& rVersion);

/** @} */

}  // namespace nvsipl

#endif // NVSIPLVERSION_HPP
