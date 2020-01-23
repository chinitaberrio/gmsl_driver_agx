/*
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#ifndef _NVSIPLCAMERA_HPP_
#define _NVSIPLCAMERA_HPP_

#include "NvSIPLCommon.hpp"
#include "NvSIPLPlatformCfg.hpp"
#include "NvSIPLPipelineMgr.hpp"

#include <memory>

/**
 * @file
 *
 * <b> NVIDIA Sensor Input Processing Library: Camera Interface - @ref NvSIPLCamera_API </b>
 *
 */

/** @defgroup NvSIPLCamera_API NvSIPL Camera (libnvsipl.so)
 *
 * @brief Provides top-level interfaces to program external image devices and Tegra to create and
 * manage image processing pipelines to receives outputs in NvMediaImage surfaces.
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
    /** @brief Gets a handle to INvSIPLCamera instance.
     *
     * Static function to create an instance of implementation class and return the handle. The object
     * is automatically destroyed when the variable holding the return value goes out of scope.
     *
     * @returns unique_ptr A pointer to an INvSIPLCamera instance. */
    static std::unique_ptr<INvSIPLCamera> GetInstance(void);

    /** @brief Sets a platform configuration.
     *
     * Function sets a PlatformCfg camera platform configuration. NvSIPLCamera uses the
     * information in the platform configuration to create the necessary DeviceBlock(s)
     * and Pipeline Manager(s).
     *
     * @param[in] platformCfg @ref PlatformCfg
     * The external devices referenced in the platform configurations must be
     * supported by the @ref NvSIPLDevBlk_API
     *
     * @returns::SIPLStatus. The completion status of the operation. */
    virtual SIPLStatus SetPlatformCfg(const PlatformCfg* platformCfg) = 0;

    /** @brief Gets the camera platform configuration set by @ref SetPlatformCfg.
     *
     * @returns PlatformCfg* A pointer to the PlatformCfg.
     */
    virtual const PlatformCfg* GetPlatformCfg(void) const = 0;

    /** @brief Sets a handler for events from an image processing pipeline.
     *
     * Sets a @ref NvSIPLPipelineNotifier to handle the events from the image processing pipeline.
     * @param[in] uIndex The ID of the sensor.
     * @param[in] pNotifier A pointer to @ref NvSIPLPipelineNotifier.
     *
     * @returns::SIPLStatus. The completion status of the operation. */
    virtual SIPLStatus SetNotifier(std::uint32_t uIndex, NvSIPLPipelineNotifier* pNotifier) = 0;


    /** @brief Sets an image group writer for an image processing pipeline.
     *
     * Sets a @ref NvSIPLImageGroupWriter to feed the images to image processing pipeline.
     * The API can be used to re-process the RAW frames captured from sensor via Tegra HW ISP.
     *
     * The @ref SetPlatformCfg() method must be called with appropriate
     * platform configuration before calling this API.
     *
     * @param[in] uIndex The ID of the sensor.
     * @param[in] pImageGroupWriter A pointer to @ref NvSIPLImageGroupWriter.
     *
     * @returns @ref SIPLStatus. */
    virtual SIPLStatus SetImageGroupWriterCallback(std::uint32_t uIndex, NvSIPLImageGroupWriter* pImageGroupWriter) = 0;

    /** @brief Sets a vector of @ref INvSIPLClient::ConsumerDesc of consumers of image processing pipeline(s).
     *
     * This function must be called after @ref SetPlatformCfg() and before @ref Init()
     * Each @ref INvSIPLClient::ConsumerDesc describes a consumer of an image processing pipeline output.
     *
     * @returns::SIPLStatus. The completion status of the operation. */
    virtual SIPLStatus SetOutputDesc(std::vector<INvSIPLClient::ConsumerDesc> vDescs) = 0;

    /** @brief Sets attributes of the image pool used by ICP and ISP components of
     * an image processing pipeline.
     *
     * The API can be used to override the default output format of the ISP.
     *
     * @param[in] index The ID of the sensor.
     * @param[in] outType @ref nvsipl::INvSIPLClient::ConsumerDesc::OutputType.
     * @param[in] numOfImages Number of images in the pool.
     * @param[in] imageAttr An @ref NvSIPLImageAttr.
     *
     * @returns @ref SIPLStatus. */
    virtual SIPLStatus SetImagePoolAttributes(std::uint32_t index,
                                              INvSIPLClient::ConsumerDesc::OutputType outType,
                                              uint32_t numOfImages,
                                              const NvSIPLImageAttr &imageAttr) = 0;

    /** @brief Initializes @ref NvSIPLCamera_API for the selected platform configuration.
     *
     * The function internally initializes the @ref NvSIPLDevBlk_API for each device block
     * in the selected platform configuration and creates and initializes the image processing
     * pipelines based on the number and type of the consumers set via @ref SetOutputDesc.
     *
     * This function must be called after @ref SetPlatformCfg and @ref SetOutputDesc.
     *
     * @returns::SIPLStatus. The completion status of the operation. */
    virtual SIPLStatus Init(void) = 0;

    /** @brief Gets @ref NvSIPLClient_API descriptor for an image processing pipeline output.
     *
     * The function returns a handle to @ref INvSIPLClient::ClientDesc for a specific
     * image processing pipeline output. This descriptor can be used to
     * initialize a @ref NvSIPLClient_API object.
     *
     * @param[in] index The ID of the sensor.
     * @param[in] outType @ref INvSIPLClient::ConsumerDesc::OutputType.
     *
     * this function must be called after @ref Init().
     *
     * @returns::SIPLStatus. The completion status of the operation. */
    virtual INvSIPLClient::ClientDesc* GetClientDesc(std::uint32_t index,
                                                     INvSIPLClient::ConsumerDesc::OutputType outType) = 0;

    /** @brief Starts @ref NvSIPLCamera_API for the selected platform configuration.
     *
     * The function internally starts the streaming from sensors belonging to each device block
     * in the selected platform configuration and starts the associated image processing pipelines.
     *
     * This function must be called after @ref Init().
     *
     * @returns::SIPLStatus. The completion status of the operation. */
    virtual SIPLStatus Start(void) = 0;

    /** @brief Stops @ref NvSIPLCamera_API for the selected platform configuration.
     *
     * The function internally stops the streaming from sensors belonging to each device block
     * in the selected platform configuration and stops the associated image processing pipelines.
     *
     * This function must be called after @ref Start().
     *
     * @returns::SIPLStatus. The completion status of the operation. */
    virtual SIPLStatus Stop(void) = 0;

    /** @brief De-initializes @ref NvSIPLCamera_API for the selected platform configuration.
     *
     * The function internally De-initializes the @ref NvSIPLDevBlk_API for each device block in
     * the selected platform configuration and De-initializes and destroys the image
     * processing pipelines.
     *
     * This function must be called after @ref Stop().
     *
     * @returns @ref SIPLStatus. */

    virtual SIPLStatus Deinit(void) = 0;

    /** @brief Default destructor. */
    virtual ~INvSIPLCamera(void) = default;

    // TODO: Add APIs to get deviceblock errors and handle recovery.

}; // INvSIPLCamera

/** @} */
} // namespace nvsipl



#endif //_NVSIPLCAMERA_HPP_
