/*
 * Copyright (c) 2018-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#ifndef NVSIPLCAMERA_HPP
#define NVSIPLCAMERA_HPP

#include "NvSIPLCommon.hpp"
#include "NvSIPLPlatformCfg.hpp"
#include "NvSIPLPipelineMgr.hpp"
#include "INvSiplControlAuto.hpp"
#include "NvSIPLClient.hpp"

#include "nvmedia_image.h"

#include "devblk_cdi.h"

#ifdef NVMEDIA_NVSCI_ENABLE
#include "nvscisync.h"
#include "nvscistream.h"
#endif // NVMEDIA_NVSCI_ENABLE

#include <cstdint>
#include <memory>
#include <vector>

/**
 * @file
 *
 * @brief <b> NVIDIA SIPL: Camera Interface - @ref NvSIPLCamera_API </b>
 *
 */

/** @defgroup NvSIPLCamera_API NvSIPL Camera
 *
 * @brief Provides top-level interfaces to program external image devices and Tegra to create and
 * manage image processing pipelines to receive outputs in NvMediaImage surfaces.
 *
 * @ingroup NvSIPL */

namespace nvsipl
{

/** @ingroup NvSIPLCamera_API
 * @{
 */

/** @class INvSIPLCamera NvSIPLCamera.hpp
 *
 * @brief Defines public data structures and describes the interfaces for NvSIPLCamera.
 */
class INvSIPLCamera
{
 public:
    /** @brief Gets a handle to an INvSIPLCamera instance.
     *
     * Static function to create an instance of the implementation class and return the handle. The
     * object is automatically destroyed when the variable holding the return value goes out of
     * scope.
     *
     * @returns a pointer to an INvSIPLCamera instance.
     */
    static std::unique_ptr<INvSIPLCamera> GetInstance(void);

    /** @brief Sets a platform configuration.
     *
     * The method sets a @ref PlatformCfg camera platform configuration. NvSIPLCamera uses the
     * information in the platform configuration to create the necessary DeviceBlock(s)
     * and Pipeline Manager(s). This function must be called before @ref SetPipelineCfg().
     *
     * @param[in] platformCfg @ref PlatformCfg
     * The external devices referenced in the platform configuration must be supported by the
     * SIPL Device Block drivers.
     *
     * @returns::SIPLStatus the completion status of the operation.
     */
    virtual SIPLStatus SetPlatformCfg(const PlatformCfg* platformCfg) = 0;

#if !NV_IS_SAFETY

    /** @brief Sets a pipeline configuration.
     *
     * The method sets a camera pipeline configuration. NvSIPLCamera uses the information in the
     * pipeline configuration to initialize the Pipeline Manager. This function must be called after
     * @ref SetPlatformCfg() but before @ref Init().
     *
     * @param[in] index The ID of the sensor.
     * @param[in] pipelineCfg An @ref NvSIPLPipelineCfg to set.
     *
     * @returns::SIPLStatus the completion status of the operation.
     * @deprecated This version of SetPipelineCfg() will be removed in a future release.
     */
    virtual SIPLStatus SetPipelineCfg(std::uint32_t index, const NvSIPLPipelineCfg &pipelineCfg) = 0;

#endif // NV_IS_SAFETY

    /** @brief Sets a pipeline configuration.
     *
     * The method sets a camera pipeline configuration. NvSIPLCamera uses the information in the
     * pipeline configuration to initialize the Pipeline Manager. This function must be called after
     * @ref SetPlatformCfg() but before @ref Init().
     *
     * @param[in] index The ID of the sensor.
     * @param[in] pipelineCfg An @ref NvSIPLPipelineConfiguration to set.
     * @param[out] queues The queues that will deliver completed frames and events to the client.
     *
     * @returns::SIPLStatus the completion status of the operation.
     */
    virtual SIPLStatus SetPipelineCfg(std::uint32_t index,
                                      const NvSIPLPipelineConfiguration &pipelineCfg,
                                      NvSIPLPipelineQueues& queues) = 0;

    /** @brief Register Auto Control plugin to be used for specific pipeline.
     *
     * This method must be called for every pipeline with ISP output enabled. This function must be
     * called after @ref RegisterImages() but before @ref Start().
     *
     * @param[in] index The ID of the sensor.
     * @param[in] type Plugin type.
     * @param[in] autoControl @ref ISiplControlAuto Handle to plugin implementation.
     * @param[in] blob Reference to binary blob containing the ISP configuration.
     *
     * @note Support for registering more than one auto control plugin is deprecated and will be removed in the future release.
     *
     * @returns::SIPLStatus the completion status of the operation.
     */
    virtual SIPLStatus RegisterAutoControlPlugin(std::uint32_t index,
                                                 PluginType type,
                                                 ISiplControlAuto* autoControl,
                                                 const std::vector<std::uint8_t>& blob) = 0;

    /** @brief Initializes @ref NvSIPLCamera_API for the selected platform configuration.
     *
     * The method internally initializes the camera module(s) and deserializer for each device block
     * in the selected platform configuration and creates and initializes the image processing
     * pipelines based on the number and type of the consumers set via @ref SetPipelineCfg.
     *
     * This method must be called after @ref SetPipelineCfg() but before @ref RegisterImageGroups().
     *
     * @returns::SIPLStatus the completion status of the operation.
     */
    virtual SIPLStatus Init(void) = 0;

    /** @brief Gets image attributes.
     *
     * The method can be used to get the attributes of the images to be used with the image
     * processing pipeline. The user must reconcile the attributes returned by this function with
     * the attributes required by the downstream consumers of the output of the pipeline and
     * allocate the images.
     *
     * This method must be called after @ref Init() but before @ref Start().
     *
     * @param[in] index The ID of the sensor.
     * @param[in] outType @ref nvsipl::INvSIPLClient::ConsumerDesc::OutputType.
     * @param[out] imageAttr Reference to the image attributes structure.
     *
     * @returns::SIPLStatus the completion status of the operation.
     */
    virtual SIPLStatus GetImageAttributes(std::uint32_t index,
                                          INvSIPLClient::ConsumerDesc::OutputType outType,
                                          NvSIPLImageAttr &imageAttr) = 0;

    /** @brief Read from an EEPROM in a camera module.
     *
     * This method can be used to perform data reads from an EEPROM in a camera
     * module. This method can only be called after @ref Init() but before
     * @ref Start().
     *
     * @param[in] index The ID of the sensor to which the EEPROM is associated.
     * @param[in] address The start address to read from in the EEPROM.
     * @param[in] length Contiguous size of data to be read. [byte]
     * @param[out] buffer Buffer that EEPROM data is to be written into, must be
     *                    at least size length.
     *
     * @returns::SIPLStatus. The completion status of the operation. */
    virtual SIPLStatus ReadEEPROMData(const std::uint32_t index,
                                      const std::uint16_t address,
                                      const std::uint32_t length,
                                      std::uint8_t * const buffer) = 0;

#if !NV_IS_SAFETY
    /** @brief Get EEPROM CDI handle.
     *
     * This method can be used to get the EEPROM CDI handle. This method can only be called after
     * @ref Init() but before @ref Start().
     *
     * WARNING: This API will be deprecated in an imminent future release in favour of
     *          ReadEEPROMData().
     *
     * @param[in] index The ID of the sensor.
     *
     * @returns a pointer to the EEPROM CDI handle.
     */
    virtual DevBlkCDIDevice* GetE2PHandle(std::uint32_t index) = 0;
#endif // !NV_IS_SAFETY

    /** @brief Registers image groups.
     *
     * The method can be used to register the NvMedia image groups to be used within the image
     * processing pipelines. @ref NvMediaImageGroup serve as the output of ICP and input to ISP.
     *
     * This method must be called after @ref Init() but before @ref Start() and if ISP output is
     * enabled also before @ref RegisterImages().
     *
     * @param[in] index The ID of the sensor.
     * @param[in] imageGroups Vector of @ref NvMediaImageGroup pointers to be registered.
     *
     * @returns::SIPLStatus the completion status of the operation.
     */
    virtual SIPLStatus RegisterImageGroups(std::uint32_t index,
                                           const std::vector<NvMediaImageGroup*> &imageGroups) = 0;

    /** @brief Registers images.
     *
     * The method can be used to register the images to be used within the image processing
     * pipelines. @ref NvMediaImage serve as the output of ISP.
     *
     * If ISP output is enabled, this method must be called after @ref RegisterImageGroups() but
     * before @ref RegisterAutoControlPlugin().
     *
     * @param[in] index The ID of the sensor.
     * @param[in] outType @ref nvsipl::INvSIPLClient::ConsumerDesc::OutputType, can be ISP0 or ISP1.
     * @param[in] images Vector of @ref NvMediaImage pointers to be registered.
     *
     * @returns::SIPLStatus the completion status of the operation.
     */
    virtual SIPLStatus RegisterImages(std::uint32_t index,
                                      INvSIPLClient::ConsumerDesc::OutputType outType,
                                      const std::vector<NvMediaImage*> &images) = 0;

    /** @brief Starts @ref NvSIPLCamera_API for the selected platform configuration.
     *
     * The method internally starts the streaming from sensors belonging to each device block
     * in the selected platform configuration and starts the associated image processing pipelines.
     *
     * This method must be called after @ref RegisterImageGroups() @if (NVMEDIA_NVSCI_ENABLE), @else
     *  and @endif (if ISP output is enabled) @ref RegisterImages()@if (NVMEDIA_NVSCI_ENABLE), and
     * (if using NvSci) @ref SetNvSciSyncObjForEOF()@endif.
     *
     * @returns::SIPLStatus the completion status of the operation.
     */
    virtual SIPLStatus Start(void) = 0;

    /** @brief Stops @ref NvSIPLCamera_API for the selected platform configuration.
     *
     * The method internally stops the streaming from sensors belonging to each device block
     * in the selected platform configuration and stops the associated image processing pipelines.
     *
     * This method must be called after @ref Start().
     *
     * @returns::SIPLStatus the completion status of the operation.
     */
    virtual SIPLStatus Stop(void) = 0;

    /** @brief Deinitializes @ref NvSIPLCamera_API for the selected platform configuration.
     *
     * The method internally deinitializes the camera module(s) and deserializer for each device block in the
     * selected platform configuration and deinitializes and destroys the image processing
     * pipelines.
     *
     * Any registered images are automatically deregistered and can be safely destroyed.
     *
     * This method must be called after @ref Stop().
     *
     * @returns::SIPLStatus the completion status of the operation.
     */
    virtual SIPLStatus Deinit(void) = 0;

    /** @brief Attempts to recover a given link.
     *
     * The method tries to recover a link that has failed.
     *
     * If this method is called on a link that has not failed, nothing will happen.
     *
     * This method should only be called after @ref Start() and before @ref Stop().
     *
     * @param[in] index The ID of the sensor.
     *
     * @returns::SIPLStatus the completion status of the operation.
     */
    virtual SIPLStatus RecoverLink(std::uint32_t index) = 0;

#if !NV_IS_SAFETY
    /** @brief Control the LED on the associated camera module.
     *
     * The method tries to enable or disable the LED on the specific module.
     *
     * This method is valid only if there is an LED on the camera module and it is controlled by the sensor.
     *
     * This method should only be called after @ref Start() and before @ref Stop().
     *
     * @param[in] index The ID of the sensor.
     * @param[in] enable Enable or disable LED.
     *
     * @returns::SIPLStatus the completion status of the operation.
     */
    virtual SIPLStatus ToggleLED(std::uint32_t index, bool enable) = 0;
#endif // !NV_IS_SAFETY

    /** @brief Default destructor. */
    virtual ~INvSIPLCamera(void) = default;

#ifdef NVMEDIA_NVSCI_ENABLE
    /** @brief Fills an @ref NvSciSyncAttrList.
     *
     * The method can be used to fetch the NvSciSync attributes required for compatiblility
     * with the underlying image processing pipelines.
     *
     * If using NvSci, this method must be called after @ref Init() and before @ref
     * RegisterNvSciSyncObj().
     *
     * @param[in] index The ID of the sensor.
     * @param[in] outType @ref nvsipl::INvSIPLClient::ConsumerDesc::OutputType.
     * @param[out] attrList @ref NvSciSyncAttrList to be filled.
     * @param[in] clientType Waiter, signaler, or both.
     *
     * @returns::SIPLStatus the completion status of the operation.
     */
    virtual SIPLStatus FillNvSciSyncAttrList(std::uint32_t index,
                                             INvSIPLClient::ConsumerDesc::OutputType outType,
                                             NvSciSyncAttrList attrList,
                                             NvMediaNvSciSyncClientType clientType) = 0;

    /** @brief Register an @ref NvSciSyncObj.
     *
     * If using NvSci, this method must be called after @ref FillNvSciSyncAttrList() and before @ref
     * SetNvSciSyncObjForEOF().
     *
     * @param[in] index The ID of the sensor.
     * @param[in] outType @ref nvsipl::INvSIPLClient::ConsumerDesc::OutputType.
     * @param[in] syncobjtype Presync, EOF sync, or presync and EOF.
     * @param[in] syncobj @ref NvSciSyncObj to be registered.
     *
     * @returns::SIPLStatus the completion status of the operation.
     * @note Support for registering more than one NvSciSyncObj of type NVMEDIA_EOFSYNCOBJ is deprecated and will be removed in a future release.
     */
    virtual SIPLStatus RegisterNvSciSyncObj(std::uint32_t index,
                                            INvSIPLClient::ConsumerDesc::OutputType outType,
                                            NvMediaNvSciSyncObjType syncobjtype,
                                            NvSciSyncObj syncobj) = 0;

#if !NV_IS_SAFETY
    /** @brief Set the EOF @ref NvSciSyncObj.
     *
     * Set the engine's current EOF NvSciSync object.
     *
     * If using NvSci, this method must be called after @ref RegisterNvSciSyncObj() and before @ref
     * Start().
     *
     * @param[in] index The ID of the sensor.
     * @param[in] outType @ref nvsipl::INvSIPLClient::ConsumerDesc::OutputType.
     * @param[in] syncobj @ref NvSciSyncObj to be registered.
     *
     * @returns::SIPLStatus the completion status of the operation.
     * @deprecated This method is deprecated and will be removed in a future release.
     */
    virtual SIPLStatus SetNvSciSyncObjForEOF(std::uint32_t index,
                                             INvSIPLClient::ConsumerDesc::OutputType outType,
                                             NvSciSyncObj syncobj) = 0;
#endif // !NV_IS_SAFETY
#endif // NVMEDIA_NVSCI_ENABLE

}; // INvSIPLCamera

/** @} */
} // namespace nvsipl



#endif // NVSIPLCAMERA_HPP
