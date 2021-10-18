/*
 * Copyright (c) 2018-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef NVSIPLCLIENT_HPP
#define NVSIPLCLIENT_HPP

#include "NvSIPLCommon.hpp"

#include "nvmedia_image.h"
#include "devblk_cdi.h"
#include "nvmedia_isp_stat.h"

#ifdef NVMEDIA_NVSCI_ENABLE
#include "nvscisync.h"
#include "nvscistream.h"
#endif // NVMEDIA_NVSCI_ENABLE

#include <cstdint>
#include <string>
#include <memory>

/**
 * @file
 *
 * @brief <b> NVIDIA SIPL: Client Interface - @ref NvSIPLClient_API </b>
 *
 */

/** @defgroup NvSIPLClient_API NvSIPL Client
 *
 * @brief Provides interfaces to retrieve the output of the SIPL Pipeline Manager.
 *
 * @ingroup NvSIPLCamera_API
 */

namespace nvsipl
{

/** @ingroup NvSIPLClient_API
 * @{
 */

/** @class INvSIPLClient NvSIPLClient.hpp
 *
 * @brief Defines the public data structures
 * and describes the interfaces for @ref NvSIPLClient_API.
 *
 */
class INvSIPLClient
{
 public:
    /** @brief Defines the metadata associated with the image. */
    struct ImageMetaData
    {
        /** Holds the TSC timestamp of the frame capture. */
        std::uint64_t frameCaptureTSC;
        /** Holds the parsed embedded data frame number of exposures info for the captured frame.*/
        std::uint32_t numExposures;
        /** Holds the parsed embedded data sensor exposure info for the captured frame. */
        DevBlkCDIExposure sensorExpInfo;
        /** Holds the parsed embedded data sensor white balance info for the captured frame. */
        DevBlkCDIWhiteBalance sensorWBInfo;
        /** Holds the parsed embedded data sensor PWL info for the captured frame. */
        DevBlkCDIPWL sensorPWLInfo;
        /** Holds the parsed embedded data sensor crc info for the captured frame. */
        DevBlkCDICRC sensorCRCInfo;
        /** Holds the parsed embedded data frame report info for the captured frame. */
        DevBlkCDIFrameReport sensorReportInfo;
        /** Holds the parsed embedded data sensor temperature info for the captured frame. */
        DevBlkCDITemperature sensorTempInfo;
        /** Holds the parsed embedded data frame sequence number info for the captured frame. */
        DevBlkCDIFrameSeqNum frameSeqNumInfo;
        /** Holds a flag indicating if the ISP bad pixel statistics are valid. */
        bool badPixelStatsValid;
        /** Holds the ISP bad pixel statistics for the previous ISP output frame. */
        NvMediaISPBadPixelStatsData badPixelStats;
    };

    /** @class INvSIPLBuffer
     *
     * @brief Describes the interfaces of SIPL buffer.
     *
     */
    class INvSIPLBuffer
    {
     public:
        /** @brief Adds a reference.
         *
         * Adding a reference to the buffer ensures that this buffer is not re-used by another
         * producer/consumer of the buffer. */
        virtual void AddRef(void) = 0;

        /** @brief Release a reference.
         *
         * Releasing reference implies that the user has finished working with the buffer and
         * the buffer is available for re-use.
         *
         * @returns::SIPLStatus The completion status of the operation. */
        virtual SIPLStatus Release(void) = 0;

#ifdef NVMEDIA_NVSCI_ENABLE
        /** @brief Add an NvSciSync prefence.
         *
         * Add an NvSciSync prefence to be used with the next ISP or ICP operation. This function
         * creates its own duplicate of the fence, so the caller must clear their copy of the fence
         * by calling @ref NvSciSyncFenceClear().
         *
         * @param[in] prefence Prefence to be added.
         *
         * @returns::SIPLStatus The completion status of the operation. */
        virtual SIPLStatus AddNvSciSyncPrefence(NvSciSyncFence &prefence) = 0;

        /** @brief Retrieve the latest NvSciSync EOF fence.
         *
         * Retrieve the buffer's latest NvSciSync EOF fence associated with the engine's set
         * NvSciSync EOF object. The caller must clear the returned fence by calling
         * @ref NvSciSyncFenceClear().
         *
         * This method should only be called after @ref nvsipl::INvSIPLCamera::Start().
         *
         * @param[out] postfence EOF fence being returned.
         *
         * @returns::SIPLStatus The completion status of the operation. */
        virtual SIPLStatus GetEOFNvSciSyncFence(NvSciSyncFence *postfence) = 0;
#endif // NVMEDIA_NVSCI_ENABLE
    };

    /** @class INvSIPLNvMBuffer
     *
     * @brief Describes a SIPL buffer containing an @ref NvMediaImage or @ref NvMediaImageGroup.
     *
     * INvSIPLNvMBuffer holds either @ref NvMediaImage or @ref NvMediaImageGroup at a time. */
    class INvSIPLNvMBuffer : public INvSIPLBuffer
    {
     public:
        /** @brief Gets a handle to @ref NvMediaImage.
         *
         * @returns A pointer to @ref NvMediaImage. */
        virtual NvMediaImage* GetImage() = 0;

        /** @brief Gets a handle to @ref NvMediaImageGroup.
         *
         * This function holds the output of an image sensor in HDR mode.
         *
         * @returns A pointer to @ref NvMediaImageGroup. */
        virtual NvMediaImageGroup* GetImageGroup() = 0;

        /** @brief Gets an @ref nvsipl::INvSIPLClient::ImageMetaData
         * associated with @ref NvMediaImage or @ref NvMediaImageGroup.
         *
         * @returns A copy of @ref nvsipl::INvSIPLClient::ImageMetaData. */
        virtual ImageMetaData GetImageData() = 0;
    };

    /** @class INvMCallback
     *
     * @brief Describes a class with callback functions that must be implemented by
     * the consumer of the SIPL image processing pipeline.
     *
     * The consumer of the @ref INvSIPLBuffer must implement a class derived from this class. */
    class INvMCallback
    {
     public:
        /** @brief A new output frame is available.
         *
         * This method is overridden by the implementation of the consumer.
         * This function is called by SIPL pipeline thread.
         *
         * The consumer must use @ref INvSIPLBuffer::AddRef and @ref INvSIPLBuffer::Release on
         * the buffer if it intends to consume the buffer asynchronously in a different thread.
         *
         * @param[in] pBuffer A pointer to @ref INvSIPLBuffer.
         *
         * @returns::SIPLStatus Indicates if the frame was consumed correctly. */
        virtual SIPLStatus OnFrameAvailable(INvSIPLBuffer* pBuffer) = 0;
    };

    /** @brief Describes a client of the pipeline. */
    struct ConsumerDesc
    {
        /** @brief Defines the types of the SIPL pipeline output. */
        enum class OutputType
        {
            ICP,  /**< Indicates the unprocessed output of the image sensor. */
            ISP0, /**< Indicates the first output of NvMediaISP. */
            ISP1, /**< Indicates the second output of NvMediaISP. */
        };

        /** Holds an @ref OutputType. */
        OutputType eOutputType;
        /** Holds a pointer to a user-implemented class object inheriting from @ref INvMCallback. */
        INvMCallback* callback;
    };
};

/** @} */

} // namespace nvsipl



#endif // NVSIPLCLIENT_HPP
