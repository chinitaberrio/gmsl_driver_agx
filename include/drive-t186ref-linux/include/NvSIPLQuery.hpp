/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#ifndef NVSIPLQUERY_HPP
#define NVSIPLQUERY_HPP

#include "NvSIPLCommon.hpp"
#include "NvSIPLPlatformCfg.hpp"

#include <string>
#include <cstdint>
#include <memory>
#include <vector>

/**
 * @file
 *
 * @brief <b> NVIDIA SIPL: Query Interface - @ref NvSIPLQuery_API </b>
 *
 */

namespace nvsipl
{

/**
 * @defgroup NvSIPLQuery_API NvSIPL Query
 *
 * @brief Manages a database of information about external devices (deserializer, serializer,
 * sensor, and EEPROM) and the camera platform configurations supported by SIPL Device Block drivers.
 *
 * @ingroup NvSIPL
 */

/** @addtogroup NvSIPLQuery_API
 * @{
 */

/**
 * @class INvSIPLQuery NvSIPLQuery.hpp
 *
 * @brief Defines the public data structures and describes the interfaces
 * for NvSIPLQuery.
 */
class INvSIPLQuery
{
public:

    static constexpr std::uint32_t MAJOR_VER  = 1u; /**< Indicates a major revision. */
    static constexpr std::uint32_t MINOR_VER  = 0u; /**< Indicates a minor revision. */
    static constexpr std::uint32_t PATCH_VER  = 0u; /**< Indicates a patch revision. */

    /** @brief Defines the version information for NvSIPLQuery_API.
     * @par
     * Version history of @ref NvSIPLQuery_API
     * @par
     * Version <b> 1.0.0 </b> - 9th September 2019 - Alpha version.
     */
    struct Version
    {
        std::uint32_t uMajor = MAJOR_VER; /**< Holds a major revision. */
        std::uint32_t uMinor = MINOR_VER; /**< Holds a minor revision. */
        std::uint32_t uPatch = PATCH_VER; /**< Holds a patch revision. */
    };

    /** @brief Returns the library version.
     *
     * @param[out] version A reference to the object containing the version
     * information.
     */
    static void GetVersion(Version& version);

    /** @brief Gets a handle to an instance of INvSIPLQuery.
     *
     * This static function creates an instance of INvSIPLQuery and returns
     * a handle to the object.
     * The object is automatically destroyed when the variable holding the
     * return value goes out of scope.
     *
     * @returns unique_ptr A pointer to an instance of INvSIPLQuery.
     */
    static std::unique_ptr <INvSIPLQuery> GetInstance(void);

    /** @brief Parses the built-in JSON database and updates the internal state of
     * the implementation class. This function must be called before any
     * other non-static functions.
     *
     * @returns::SIPLStatus The status of the operation. */
    virtual SIPLStatus ParseDatabase(void) = 0;

    /** @brief Parses the input JSON file containing a list of
     * user-specified camera platform configuration. This function
     * must be called only after ParseDatabase().
     *
     * @param[in] fileName Full name, including the path, of the
     * JSON file containing the platform configuration. The external
     * devices referenced in the platform configuration must be supported
     * by the library.
     *
     * @returns::SIPLStatus The status of the operation. */
    virtual SIPLStatus ParseJsonFile(std::string fileName) = 0;

    /** @brief Returns a pointer to the list of all external image devices
     * supported by @ref NvSIPLQuery_API and SIPL Device Block drivers.
     * This function must be called only after ParseDatabase().
     *
     * @returns A @c const pointer to DeviceInfoList. */
    const virtual DeviceInfoList* GetDeviceInfoList() const = 0;

    /** @brief Returns a list of camera platform configurations supported by
     * @ref NvSIPLQuery_API and @ref NvSIPLCamera_API.
     * This function must be called only after ParseDatabase().
     *
     * @returns A vector of @c const pointers to PlatformCfg. */
    virtual std::vector <const PlatformCfg*> GetPlatformCfgList() const = 0;

    /** @brief Returns a PlatformCfg object by name.
     * This function must be called only after ParseDatabase().
     *
     * @returns::SIPLStatus The status of the operation. */
    virtual SIPLStatus GetPlatformCfg(std::string name,
                                      PlatformCfg& oConfig) const = 0;

    /** @brief Defines link enable masks for deserializers.
     *
     * To construct a complete mask for a deserializer, perform an <em>OR</em>
     * operation between the link mask values.
     * For example, to enable link 0 and 3 of a deserializer, set the mask to
     * @c LINK_0 | @c LINK_3. */
    enum EnableMask
    {
        LINK_0 = 0x0001, /**< 1st Link */
        LINK_1 = 0x0010, /**< 2nd Link */
        LINK_2 = 0x0100, /**< 3rd Link */
        LINK_3 = 0x1000  /**< 4th Link */
    };

    /** @brief Applies masks to the input platform configuration.
     *
     * Creates a custom platform configuration by modifying the input platform configuration
     * object to enable only specific links of the deserializer/deviceblock using the input masks.
     * This function must be called only after ParseDatabase().
     *
     * @param[in,out] platCfg A reference to the platform configuration that is modified.
     * @param[in] vMasks A vector of integers describing the mask value for each deserializer
     * in the platform configuration. The number of masks in the vector must be same as the number
     * of deserializers in the platform configuration.
     * @returns::SIPLStatus The status of the operation. */
    virtual SIPLStatus ApplyMask(PlatformCfg& platCfg,
                                 const std::vector <std::uint32_t>& vMasks) const = 0;

    /** @brief Default destructor. */
    virtual ~INvSIPLQuery() = default;

}; //INvSIPLQuery

/** @} */

}// namespace nvsipl



#endif //NVSIPLQUERY_HPP
