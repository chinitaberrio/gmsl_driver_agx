/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef _NVSIPLDEVICEBLOCK_HPP_
#define _NVSIPLDEVICEBLOCK_HPP_

#include "NvSIPLCommon.hpp"

#include "nvmedia_isc.h"

#include <string>
#include <cstdint>
#include <memory>
#include <vector>
/**
 * @file
 *
 * <b> NVIDIA Sensor Input Processing Library: DeviceBlock Interface - @ref NvSIPLDevBlk_API </b>
 *
 */

namespace nvsipl
{

/** @defgroup NvSIPLDevBlk_API NvSIPL DeviceBlock  (libnvsipl_devblk.so)
 *
 * @brief Manages the programming of the external image devices using NvMediaISC
 * compatible drivers.
 *
 * @ingroup NvSIPLCamera_API
 */

/** @addtogroup NvSIPLDevBlk_API
 * @{
 */

/** Indicates the maximum number of device blocks per platform. */
static const std::uint32_t MAX_DEVICEBLOCKS_PER_PLATFORM = 4U;

/** Indicates the maximum number of camera modules per device block. */
static const std::uint32_t MAX_CAMERAMODULES_PER_BLOCK = 4U;

/** Indicates the maximum number of image sensors per camera module. */
static const std::uint32_t MAX_SENSORS_PER_CAMERA_MODULE = 2U;

/** Indicates the maximum number of EEPROMs per camera module. */
static const std::uint32_t MAX_EEPROMS_PER_CAMERA_MODULE = MAX_SENSORS_PER_CAMERA_MODULE;

/**Indicates the maximum number of virtual channels per image sensor. */
static const std::uint32_t MAX_VIRTUAL_CHANNELS_PER_SENSOR = 4U;

/** Indicates the maximum number of camera modules per platform. */
static const std::uint32_t MAX_CAMERAMODULES_PER_PLATFORM = MAX_DEVICEBLOCKS_PER_PLATFORM * MAX_CAMERAMODULES_PER_BLOCK;

/** Indicates the maximum number of sensors per platform. */
static const std::uint32_t MAX_SENSORS_PER_PLATFORM = MAX_CAMERAMODULES_PER_PLATFORM * MAX_SENSORS_PER_CAMERA_MODULE;

/** @brief Defines the image sensor information. */
struct SensorInfo
{

    /** @brief Defines the image resolution. */
    struct Resolution
    {
        std::uint32_t width = 0U;  /**< Indicates the width in pixels. */
        std::uint32_t height = 0U; /**< Indicates the height in pixels. */
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
        std::uint32_t embeddedTopLines = -1U;
        /** Holds the number of bottom embedded lines. */
        std::uint32_t embeddedBottomLines = -1U;
        /** Holds the input format. For example, "raw12". */
        std::string inputFormat = "";
        /** Holds the @ref Resolution of the captured frame. */
        Resolution resolution;
        /** Holds the average number of frames per second. */
        float fps = 0;
        /** Indicates whether embedded data type is enabled. The default value is <em>false</em>. */
        bool isEmbeddedDataTypeEnabled = false;
    };

    /** Holds the identification of the sensor in the platform configuration. */
    std::uint32_t id = -1U;
    /** Holds the name of the image sensor. For example, "AR0231". */
    std::string name = "";
    /** Holds the description of the image sensor. */
    std::string description = "";
    /** Holds the native I2C address of the image sensor. */
    uint32_t i2cAddress = -1U;
    /** Holds the number of virtual channels used. This value must be less than @ref MAX_VIRTUAL_CHANNELS_PER_SENSOR. */
    uint32_t numVirtualChannels;
    /** Defines an array of virtual channel information. */
    VirtualChannelInfo vcInfoList[MAX_VIRTUAL_CHANNELS_PER_SENSOR];
    /** Indicates whether trigger mode is enabled. The default value is <em>false</em>. */
    bool isTriggerModeEnabled = false;
    /** Indicates whether the sensor requires configuration in Test Pattern Generator (TPG) mode.
     * The default value is <em>false</em>. */
    bool isTPGEnabled = false;
    /** Holds the Test Pattern Generator (TPG) pattern mode. */
    uint32_t patternMode = 0;
};

/** @brief Defines the EEPROM information. */
struct EEPROMInfo
{
    /** Holds the name of the EEPROM. For example, "N24C64". */
    std::string name = "";
    /** Holds the description of the EEPROM. */
    std::string description = "";
    /** Holds the native I2C address. */
    std::uint32_t i2cAddress = -1U;
};

/** @brief Defines the serializer information. */
struct SerInfo
{
    /** Holds the name of the serializer. For example, "MAX96705". */
    std::string name = "";
    /** Holds the description of the serializer. */
    std::string description = "";
    /** Holds the native I2C address. */
    std::uint32_t i2cAddress = -1U;
};

/** @brief Defines information for the Camera module.
 *
 * A Camera module, is a physical grouping of a serializer,
 * image sensor(s), and associated EEPROM(s). */
struct CameraModuleInfo
{
    /** Holds the name of the Camera module. For example, "SF3324". */
    std::string name = "";
    /** Holds the description of the Camera module. */
    std::string description = "";
    /** Holds  the index of the deserializer link on which this module is connected. */
    std::uint32_t linkIndex = -1U;
    /** Holds the @ref SerInfo of the serializer. */
    SerInfo serInfo;
    /** Holds the number of EEPROMs in the module. This value must be
    * less than or equal to @ref MAX_EEPROMS_PER_CAMERA_MODULE. */
    uint32_t numEEPROMs = 0;
    /** Holds an array of information about each EEPROM device
     in a camera module. */
    EEPROMInfo eepromInfoList[MAX_EEPROMS_PER_CAMERA_MODULE];
    /** Holds the number of image sensors in the module. This value must be
     less than or equal to @ref MAX_SENSORS_PER_CAMERA_MODULE. */
    uint32_t numSensors = 0;
    /** Holds an array of information about each sensor in a camera module. */
    SensorInfo sensorInfoList[MAX_SENSORS_PER_CAMERA_MODULE];
};

/** @brief Defines the deserializer information. */
struct DeserInfo
{
    /** Holds the name of the deserializer. For example, "MAX96712". */
    std::string name = "";
    /** Holds the description of the deserializer. */
    std::string description = "";
    /** Holds the native I2C address of the deserializer. */
    std::uint32_t i2cAddress = -1U;
};

/** @brief Defines the DeviceBlock information.
 *
 * A DeviceBlock represents a grouping of a deserializer (connected CSI interface of Tegra) and
 * the Camera modules connected on the links of the deserializer. */
struct DeviceBlockInfo
{
    /** Indicates whether simulator mode has been enabled.
     *  This flag is used for ISP re-process use case to simulate presence of a device block. */
    bool isSimulatorModeEnabled = false;
    /** Indicates whether slave mode needs to be enabled.
     * This flag is used when the Xavier connected to the deserializer
     * does not have an I2C connection to control it. */
    bool isSlaveModeEnabled = false;
    /** Specifies the @ref NvMediaICPInterfaceType CSI port of Tegra on which the deserializer is connected. */
    NvMediaICPInterfaceType csiPort = NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_A;
    /** Specifies the @ref NvMediaICPCsiPhyMode Phy mode. */
    NvMediaICPCsiPhyMode phyMode = NVMEDIA_ICP_CSI_DPHY_MODE;
    /** Indicates whether the group initialization is enabled */
    bool isGroupInitProg = false;
    /** Holds the I2C device bus number used to connect the deserializer with Tegra. */
    std::uint32_t i2cDevice = -1U;
    /** Holds the @ref DeserInfo deserializer information. */
    DeserInfo deserInfo;
    /** Indicates whether power control is disabled on the platform. */
    bool isPwrCtrlDisabled = false;

    /** Holds the number of camera modules connected to the deserializer.
     * This value must be less than or equal to
     * @ref MAX_CAMERAMODULES_PER_BLOCK. */
    uint32_t numCameraModules = 0;
    /** Holds an array of information about each camera module
     in the device block. */
    CameraModuleInfo cameraModuleInfoList[MAX_CAMERAMODULES_PER_BLOCK];
};

/** @class INvSIPLDeviceBlock NvSIPLDeviceBlock.hpp
 *
 * @brief Defines the public data structures and interfaces for NvSIPL DeviceBlock.
 */
class INvSIPLDeviceBlock
{
public:
    /** @brief Defines camera module properties. */
    typedef struct
    {
        /** Holds the number of image sensors in the module.
         * Must be less than or equal to @ref MAX_SENSORS_PER_CAMERA_MODULE. */
        std::uint32_t numOfSensors;

        /** @brief Defines image sensor properties. */
        typedef struct
        {
            /** Holds the number of virtual channels.
             * This number will be less than @ref MAX_VIRTUAL_CHANNELS_PER_SENSOR. */
            std::uint32_t numOfVCs;

            /** @brief Defines virtual channel properties. */
            typedef struct
            {
                /** Holds the virtual channel index.
                 * This number will be less than @ref MAX_VIRTUAL_CHANNELS_PER_SENSOR. */
                std::uint32_t virtualChannelIndex;

                /** Holds the @ref NvMediaICPSettings to configure NvMediaICP
                 * to capture the virtual channel. */
                NvMediaICPSettings icpSettings;

                /** Holds the @ref NvMediaRect clipping rectangle. */
                NvMediaRect clipRect;

                /** Holds the number of embedded top lines. */
                std::uint32_t embeddedTop;

                /** Holds the number of embedded bottom lines. */
                std::uint32_t embeddedBot;

                /** Holds the channel frameRate. */
                std::uint32_t frameRate;
            } VirtualChannelProperty;

            /** Holds the ID of the sensor specified in the
             platform configuration file. */
            std::uint32_t sensorIndex;

            /** Holds an array of @ref VirtualChannelProperty objects. */
            VirtualChannelProperty virtualChannelProperty[MAX_VIRTUAL_CHANNELS_PER_SENSOR];

            /** Holds the @ref NvMediaISCDevice handle of the image sensor. */
            NvMediaISCDevice *sensorHandle;

            /** Holds the @ref NvMediaISCDevice handle of the EEPROM. */
            NvMediaISCDevice *EEPROMHandle;
        } SensorProperty;

        /** Holds an array of @ref SensorProperty objects. */
        SensorProperty sensorProperty[MAX_SENSORS_PER_CAMERA_MODULE];

    } CameraModuleProperty;

    /** @brief Defines DeviceBlock properties. */
    typedef struct
    {
        /**  Holds the number of camera modules connected to the deserializer.
         * This value will be less than @ref MAX_CAMERAMODULES_PER_BLOCK. */
        std::uint32_t numOfModules;

        /** Holds an array of @ref CameraModuleProperty objects. */
        CameraModuleProperty cameraModuleProperty[MAX_CAMERAMODULES_PER_BLOCK];
    } DeviceBlockProperty;

    static constexpr std::uint32_t MAJOR_VER  = 0U; /**< Indicates a major revision. */
    static constexpr std::uint32_t MINOR_VER  = 0U; /**< Indicates a minor revision. */
    static constexpr std::uint32_t PATCH_VER  = 0U; /**< Indicates a patch revision. */

    /** @brief Defines @ref NvSIPLDevBlk_API version information. */
    struct Version
    {
        std::uint32_t uMajor = MAJOR_VER; /**< Holds a major revision. */
        std::uint32_t uMinor = MINOR_VER; /**< Holds a minor revision. */
        std::uint32_t uPatch = PATCH_VER; /**< Holds a patch revision. */
    };

    /** @brief Gets the version of the library.
     *
     * @param[out] version A reference to the object containing the version information. */
    static void GetVersion(Version& version);

    /** @brief Creates an instance of INvSIPLDeviceBlock implementation.
     *
     * Static function to create an instance of implementation class and
     * to return a handle. The object is automatically destroyed when the
     * variable holding the return value goes out of scope.
     *
     * @returns unique_ptr A pointer to INvSIPLDeviceBlock instance. */
    static std::unique_ptr <INvSIPLDeviceBlock> Create(void);

    /** @brief Sets the configuration for @ref DeviceBlockInfo.
     *
     * @param[in] deviceBlockInfo A @c const pointer
     * to a DeviceBlockInfo object.
     *
     * @return  A status code indicating the outcome of the operation.
     */
    virtual SIPLStatus SetConfig(const DeviceBlockInfo* deviceBlockInfo) = 0;

    /** @brief Initializes @ref NvSIPLDevBlk_API for the selected configuration.
     *
     * This function must be called after SetConfig().
     *
     * @return  A status code indicating the outcome of the operation.
     */
    virtual SIPLStatus Init(void) = 0;

    /** @brief Starts the streaming from sensors.
     *
     * This function must be called after Init().
     *
     * @return  A status code indicating the outcome of the operation.
     */
    virtual SIPLStatus Start(void) = 0;

    /** @brief Stops the streaming from sensors.
     *
     * @return  A status code indicating the outcome of the operation.
     */
    virtual SIPLStatus Stop(void) = 0;

    /** @brief De-initializes @ref NvSIPLDevBlk_API.
     *
     * @return  A status code indicating the outcome of the operation.
     */
    virtual SIPLStatus Deinit(void) = 0;

    /** @brief Gets the properties of the @ref NvSIPLDevBlk_API after initializing
     * with specific @ref DeviceBlockInfo configuration.
     *
     * The output properties can be used to create and manage image processing pipelines.
     * @param[out] A reference to a @ref DeviceBlockProperty.
     *
     * @return  A status code indicating the outcome of the operation.
     */
    virtual SIPLStatus GetProperty(DeviceBlockProperty& property) const = 0;

    /** @brief Wait for an error to occur.
     *
     * A blocking function call that waits for an error to occour
     * on the camera modules.
     *
     * @return  A status code indicating the outcome of the operation.
     */
    virtual SIPLStatus WaitForError(void) = 0;
    /** @brief Detect errors on the cameras device(s).
     *
     * Function to detect errors on the camera module(s).
     *
     * The output can be used to recover an error found on the camera module(s).
     * @param[out] Camera module(s) that have an error.
     * @return  A status code indicating the outcome of the operation.
     */
    virtual SIPLStatus DetectErrors(std::uint8_t& cameraModules) = 0;

    /** @brief Recover from a camera module error.
     *
     * Function to recover camera module(s) that have an error.
     *
     * Must be called after @ref DetectErrors
     * @param[in] Camera module(s) to reocver from an error.
     *
     * @param[in] cameraModules Camera module(s) to reocver from an error.
     * @return  A status code indicating the outcome of the operation.
     */
    virtual SIPLStatus ReconfigureModule(std::uint8_t cameraModules) = 0;

    /** @brief Default destructor. */
    virtual ~INvSIPLDeviceBlock() = default;
};

/** @} */

} // namespace nvsipl

#endif //_NVSIPLDEVICEBLOCK_HPP_
