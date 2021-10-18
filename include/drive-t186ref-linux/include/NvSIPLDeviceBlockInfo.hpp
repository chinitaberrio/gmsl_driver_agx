/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef NVSIPLDEVICEBLOCKINFO_HPP
#define NVSIPLDEVICEBLOCKINFO_HPP

#include "devblk_cdi.h"
#include "nvmedia_icp.h"

#include <string>
#include <memory>

/**
 * @file
 *
 * @brief <b> NVIDIA SIPL: DeviceBlock Information - @ref NvSIPLDevBlkInfo </b>
 *
 */

namespace nvsipl
{
/** @defgroup NvSIPLDevBlkInfo NvSIPL DeviceBlock Information
 *
 *  @brief Describes information about devices supported by SIPL Device Block.
 *
 *  @ingroup NvSIPLCamera_API
 */

 /** @addtogroup NvSIPLDevBlkInfo
 * @{
 */

/** Indicates the maximum number of device blocks per platform. */
static const std::uint32_t MAX_DEVICEBLOCKS_PER_PLATFORM = 4U;

/** Indicates the maximum number of camera modules per device block. */
static const std::uint32_t MAX_CAMERAMODULES_PER_BLOCK = 4U;

/** Indicates the maximum number of camera modules per platform. */
static const std::uint32_t MAX_CAMERAMODULES_PER_PLATFORM = MAX_DEVICEBLOCKS_PER_PLATFORM * MAX_CAMERAMODULES_PER_BLOCK;

/** Indicates the maximum number of sensors per platform. */
static const std::uint32_t MAX_SENSORS_PER_PLATFORM = MAX_CAMERAMODULES_PER_PLATFORM;

/** @brief Defines the image sensor information. */
struct SensorInfo
{
    /** @brief Defines the image resolution. */
    struct Resolution
    {
        std::uint32_t width = 0U;  /**< Holds the width in pixels in the range
                                   * from NVMEDIA_ICP_MIN_IMAGE_WIDTH to NVMEDIA_ICP_MAX_IMAGE_WIDTH.
                                   */
        std::uint32_t height = 0U; /**< Holds the height in pixels in the range
                                   * from NVMEDIA_ICP_MIN_IMAGE_HEIGHT to NVMEDIA_ICP_MAX_IMAGE_HEIGHT.
                                   */
    };

    /** @brief Defines the information of a virtual channel/single exposure. */
    struct VirtualChannelInfo
    {
        /**
         * Holds the Bayer color filter array order of the sensor. NVIDIA
         * recommends setting this element with the
         * NVM_SURF_ATTR_COMPONENT_ORDER_* family of macros, e.g.
         * \ref NVM_SURF_ATTR_COMPONENT_ORDER_ALPHA.
         */
        std::uint32_t cfa = 0U;
        /** Holds the number of top embedded lines. */
        std::uint32_t embeddedTopLines = UINT32_MAX;
        /** Holds the number of bottom embedded lines. */
        std::uint32_t embeddedBottomLines = UINT32_MAX;
        /** Holds the input format */
        NvMediaICPInputFormatType inputFormat = NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_RAW12;
        /** Holds the @ref Resolution of the captured frame. */
        Resolution resolution;
        /** Holds the average number of frames per second
        *  in the range from NVMEDIA_ICP_MIN_FRAME_RATE to NVMEDIA_ICP_MAX_FRAME_RATE. */
        float fps = 0;
        /** Indicates whether the embedded data is coming in CSI packet with different data. The default value is <em>false</em>. */
        bool isEmbeddedDataTypeEnabled = false;
    };

    /** Holds the identification of the pipeline index in the platform configuration. */
    std::uint32_t id = UINT32_MAX;
    /** Holds the name of the image sensor, for example, "AR0231". */
    std::string name = "";
#if !NV_IS_SAFETY
    /** Holds the description of the image sensor. */
    std::string description = "";
#endif
    /** Holds the native I2C address of the image sensor. */
    std::uint8_t i2cAddress = UINT8_MAX;
    /** Holds virtual channel information. */
    VirtualChannelInfo vcInfo;
    /** Holds a flag which indicates whether trigger mode is enabled. The
     default value is <em>false</em>. */
    bool isTriggerModeEnabled = false;
#if !NV_IS_SAFETY
    /** Holds a flag which indicates whether the sensor requires configuration
     in Test Pattern Generator (TPG) mode. The default value is
     <em>false</em>. */
    bool isTPGEnabled = false;
    /** Holds the Test Pattern Generator (TPG) pattern mode. */
    std::uint32_t patternMode = 0;
#endif
};

/** @brief Defines the EEPROM information. */
struct EEPROMInfo
{
    /** Holds the name of the EEPROM, for example, "N24C64". */
    std::string name = "";
#if !NV_IS_SAFETY
    /** Holds the description of the EEPROM. */
    std::string description = "";
#endif
    /** Holds the native I2C address. */
    std::uint8_t i2cAddress = UINT8_MAX;
};

/** @brief Defines the serializer information. */
struct SerInfo
{
    /** Holds the name of the serializer, for example, "MAX96705". */
    std::string name = "";
#if !NV_IS_SAFETY
    /** Holds the description of the serializer. */
    std::string description = "";
#endif
    /** Holds the native I2C address. */
    std::uint8_t i2cAddress = UINT8_MAX;
#if !NV_IS_SAFETY
    /** Holds long cable support */
    bool longCable = false;
#endif
};

/** @brief Defines information for the camera module.
 *
 * A camera module is a physical grouping of a serializer,
 * image sensor(s), and associated EEPROM(s). */
struct CameraModuleInfo
{
    /** Holds the name of the camera module, for example, "SF3324". */
    std::string name = "";
#if !NV_IS_SAFETY
    /** Holds the description of the camera module. */
    std::string description = "";
#endif
    /** Holds  the index of the deserializer link to which this module is
     connected. */
    std::uint32_t linkIndex = UINT32_MAX;
    /** Holds the @ref SerInfo of the serializer. */
    SerInfo serInfo;
    /** Holds EEPROM support */
    bool isEEPROMSupported = false;
    /** Holds the information about EEPROM device
     in a camera module. */
    EEPROMInfo eepromInfo;
    /** Holds the information about the sensor in a camera module. */
    SensorInfo sensorInfo;
};

/** @brief Defines the deserializer information. */
struct DeserInfo
{
    /** Holds the name of the deserializer, for example, "MAX96712". */
    std::string name = "";
#if !NV_IS_SAFETY
    /** Holds the description of the deserializer. */
    std::string description = "";
#endif
    /** Holds the native I2C address of the deserializer. */
    std::uint8_t i2cAddress = UINT8_MAX;
};

/**
 * @brief  Defines the DeviceBlock information.
 *
 * A DeviceBlock represents a grouping of a deserializer (which is connected
 * to the SoC's CSI interface) and the camera modules connected
 * to the links of the deserializer. */
struct DeviceBlockInfo
{
    /** Holds the @ref NvMediaICPInterfaceType that specifies the CSI port of
     the SoC to which the deserializer is connected. */
    NvMediaICPInterfaceType csiPort = NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_A;
    /** Holds the @ref NvMediaICPCsiPhyMode Phy mode. */
    NvMediaICPCsiPhyMode phyMode = NVMEDIA_ICP_CSI_DPHY_MODE;
    /** Holds the I2C device bus number used to connect the deserializer with
     the SoC. */
    std::uint32_t i2cDevice = UINT32_MAX;
    /** Holds the @ref DeserInfo deserializer information. */
    DeserInfo deserInfo;
    /** Holds the number of camera modules connected to the deserializer.
     * This value must be less than or equal to
     * @ref MAX_CAMERAMODULES_PER_BLOCK. */
    std::uint32_t numCameraModules = 0;
    /** Holds an array of information about each camera module
     in the device block. */
    CameraModuleInfo cameraModuleInfoList[MAX_CAMERAMODULES_PER_BLOCK];
#if !NV_IS_SAFETY
    /** Holds a flag which indicates whether simulator mode has been enabled.
     Used for the ISP reprocessing use case to simulate the presence of a
     device block. */
    bool isSimulatorModeEnabled = false;
    /** Holds a flag which indicates whether slave mode must be enabled.
     Used when a Jetson AGX Xavier&trade; SoC connected to the deserializer
     does not have an I2C connection to control it. */
    bool isSlaveModeEnabled = false;
    /** Holds a flag which indicates whether group initialization is enabled. */
    bool isGroupInitProg = false;
    /** Holds a flag which indicates whether power control is disabled on the
     platform. */
    bool isPwrCtrlDisabled = false;
    /** Holds long cable support */
    bool longCables[MAX_CAMERAMODULES_PER_BLOCK] = {false, false, false, false};
#endif
};

/** @} */

} // namespace nvsipl

#endif //NVSIPLDEVICEBLOCKINFO_HPP

