/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#ifndef _NVSIPLPLATFORMCFG_HPP_
#define _NVSIPLPLATFORMCFG_HPP_

#include "NvSIPLCommon.hpp"
#include "NvSIPLDeviceBlock.hpp"

#include "nvmedia_icp.h"

#include <string>
#include <cstdint>
#include <memory>
#include <vector>

/**
 * @file
 *
 * <b> NVIDIA Sensor Input Processing Library: Camera Platform Configuration </b>
 *
 */

namespace nvsipl
{

/** @addtogroup NvSIPL
 * @{
 */

/** @brief Defines a list of all external image devices supported by
 * @ref NvSIPLQuery_API and @ref NvSIPLDevBlk_API. */
struct DeviceInfoList
{
    /** Holds a vector of @ref SensorInfo objects. */
    std::vector<SensorInfo> sensorInfoList;
    /** Holds a vector of @ref EEPROMInfo objects. */
    std::vector<EEPROMInfo> eepromInfoList;
    /** Holds a vector of @ref SerInfo objects. */
    std::vector<SerInfo> serInfoList;
    /** Holds a vector of @ref CameraModuleInfo objects. */
    std::vector<CameraModuleInfo> cameraModuleList;
    /** Holds a vector of @ref DeserInfo objects. */
    std::vector<DeserInfo> deserInfoList;
};

/** @brief Defines the camera platform configuration.
 *
 * Describes up to @ref MAX_DEVICEBLOCKS_PER_PLATFORM deserializers connected to Tegra. */
struct PlatformCfg
{
    /** Holds the platform name. For example, "ddpx-a". */
    std::string platform = "";
    /** Holds the platform configuration name. */
    std::string platformConfig = "";
    /** Holds the platform configuration description. */
    std::string description = "";

    /** Holds the number of device blocks.
     * This value must be less than @ref MAX_DEVICEBLOCKS_PER_PLATFORM. */
    uint32_t numDeviceBlocks = 0;
    /** Holds an array of @ref DeviceBlockInfo. */
    DeviceBlockInfo deviceBlockList[MAX_DEVICEBLOCKS_PER_PLATFORM];
};

/** @} */

}// namespace nvsipl


#endif //_NVSIPLPLATFORMCFG_HPP_
