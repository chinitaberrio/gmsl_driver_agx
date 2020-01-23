/*
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef _NVSIPLCLIENT_HPP_
#define _NVSIPLCLIENT_HPP_

#include "NvSIPLCommon.hpp"

#include "nvmedia_image.h"
#include "nvmedia_isc.h"

#include <cstdint>
#include <string>
#include <memory>

/**
 * @file
 *
 * <b> NVIDIA Sensor Input Processing Library: Client Interface - @ref NvSIPLClient_API </b>
 *
 */

/** @defgroup NvSIPLClient_API NvSIPL Client (libnvsipl.so)
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
        uint32_t numExposures;
        /** Holds the parsed embedded data sensor exposure info for the captured frame. */
        NvMediaISCExposure sensorExpInfo;
        /** Holds the parsed embedded data sensor white balance info for the captured frame. */
        NvMediaISCWhiteBalance sensorWBInfo;
        /** Holds the parsed embedded data sensor PWL info for the captured frame. */
        NvMediaISCPWL sensorPWLInfo;
        /** Holds the parsed embedded data sensor crc info for the captured frame. */
        NvMediaISCCRC sensorCRCInfo;
        /** Holds the parsed embedded data frame report info for the captured frame. */
        NvMediaISCFrameReport sensorReportInfo;
        /** Holds the parsed embedded data sensor temperature info for the captured frame. */
        NvMediaISCTemperature sensorTempInfo;
        /** Holds parsed embedded data frame sequence number info for the captured frame. */
        NvMediaISCFrameSeqNum frameSeqNumInfo;
    };

    /** @brief Defines a consumer of the output of the SIPL pipeline.
     *
     * This object must be allocated and set by the user of @ref NvSIPLCamera_API to specify
     * the properties of the intended consumers of the pipeline output(s). */
    struct ConsumerDesc
    {
        /** @brief Defines the types of the SIPL pipeline output. */
        enum class OutputType
        {
            ICP,  /**< Indicates the unprocessed output of the image sensor. */
            ISP0, /**< Indicates the first output of NvMediaISP. */
            ISP1, /**< Indicates the second output of NvMediaISP. */
        };

        /** Holds the name of the consumer. */
        std::string sName;
        /** Holds the ID of the sensor. This must be one of the IDs in PlatformCfg. */
        std::uint32_t uSensorIndex;
        /** Holds the OutputType. */
        OutputType eOutputType;
    };

    /** @brief Describes a client of the pipeline.
     *
     * This is returned by @ref NvSIPLCamera_API after initialization.
     * and needed to create the consumer. */
    struct ClientDesc
    {
        /** @ref ConsumerDesc */
        ConsumerDesc oConsDesc;

        virtual ~ClientDesc() = default;
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
         * producer/consumer of the buffer.
         *
         * @returns Ref count after the increment. */
        virtual std::uint32_t AddRef(void) = 0;

        /** @brief Release a reference.
         *
         * Releasing reference implies that the user has finished working with the buffer and
         * the buffer is available for re-use.
         *
         * @returns Ref count after the decrement. */
        virtual std::uint32_t Release(void) = 0;
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

        /** @brief An error occurred in the image processing pipeline.
         *
         * This method is overridden by the implementation of the consumer.
         * This function is called by SIPL pipeline thread.
         *
         * TODO: Update this to include the error information and corrective action.
         *
         * @returns::SIPLStatus. The status of the operation. */
        virtual SIPLStatus OnError(void) = 0;

        /** @brief Default destructor. */
        virtual ~INvMCallback(void) = default;
    };

    /** @brief Create an instance of @ref NvSIPLClient_API.
     *
     * Static function to create an instance of implementation class and return handle. The object
     * is automatically destroyed when the variable holding the return value goes out of scope.
     *
     * @returns unique_ptr A handle to an INvSIPLClient instance. */
    static std::unique_ptr<INvSIPLClient> Create();

    /** @brief Initializes @ref NvSIPLClient_API.
     *
     * @param[in] pDesc Pointer @ref ClientDesc returned by @ref INvSIPLCamera::GetClientDesc
     * @param[in] pCallback A pointer to user implemented object of a class inheriting
     * @ref INvMCallback
     *
     * @returns::SIPLStatus. The completion status of the operation. */
    virtual SIPLStatus Init(ClientDesc* pDesc, INvMCallback* pCallback) = 0;

    /** @brief Starts @ref NvSIPLClient_API.
     *
     * The function is a placeholder for any operations that are necessary before
     * @ref NvSIPLClient_API starts streaming out the buffer.
     *
     * Currently there is no impact of calling this API.
     *
     * @returns::SIPLStatus. The completion status of the operation. */
    virtual SIPLStatus Start(void) = 0;

    /** @brief Stops @ref NvSIPLClient_API.
     *
     * The function is a placeholder for any operations that are necessary before
     * @ref NvSIPLClient_API stops streaming out the buffer.
     *
     * Currently there is no impact of calling this API.
     *
     * @returns::SIPLStatus. The completion status of the operation.*/
    virtual SIPLStatus Stop(void) = 0;

    /** @brief De-initializes @ref NvSIPLClient_API.
     *
     * @returns::SIPLStatus. The completion status of the operation. */
    virtual SIPLStatus Deinit(void) = 0;

    /** @brief Default destructor. */
    virtual ~INvSIPLClient(void) = default;
};

/** @} */

} // namespace nvsipl



#endif //_NVSIPLCLIENT_HPP_
