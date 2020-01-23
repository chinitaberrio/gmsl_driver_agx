/*
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef _NVSIPLPIPELINEMGR_HPP_
#define _NVSIPLPIPELINEMGR_HPP_

#include "NvSIPLCommon.hpp"
#include "NvSIPLClient.hpp"

#include <string>
#include <cstdint>
#include <memory>
#include <vector>

/**
 * @file
 *
 * <b> NVIDIA Sensor Input Processing Library: Pipeline Manager - @ref NvSIPLPipelineMgr </b>
 *
 */

/** @defgroup NvSIPLPipelineMgr NvSIPL Pipeline Manager (libnvsipl.so)
 *
 * @brief Programs Video Input (VI) and Image Signal Processor (ISP) HW block
 * using NvMedia API to create image processing pipelines for each sensor.
 *
 * @ingroup NvSIPLCamera_API
 */

namespace nvsipl
{

/** @ingroup NvSIPLPipelineMgr
 *
 * @class NvSIPLPipelineNotifier
 *
 * @brief Describes the interfaces of SIPL pipeline notification handler.
 *
 * This class defines data structures and interfaces that must be implemented by
 * SIPL pipeline notification handler.
 */
class NvSIPLPipelineNotifier
{
public:
    /** @brief Defines the events of the image processing pipeline. */
    enum NotificationType
    {
        /** Indicates ISP processing has finished. */
        NOTIF_INFO_ISP_PROCESSING_DONE = 0,
        /** Indicates sensor programming has finished. */
        NOTIF_INFO_ISC_PROCESSING_DONE,
        /** Indicates algorithm processing finished. */
        NOTIF_INFO_ALG_PROCESSING_DONE,
        /** Indicates a frame was captured. */
        NOTIF_INFO_FRAME_CAPTURE,
        /** Indicates an end-of-file was encountered. */
        NOTIF_INFO_EOF,

        /** Indicates pipeline was forced to drop a frame due to a slow consumer or system issues. */
        NOTIF_WARNING_CAPTURE_FRAME_DROP = 100,
        /** Indicates CSI frame discontinuity was detected. */
        NOTIF_WARNING_CSI_FRAME_DISCONTINUITY,
        /** Indicates an unspecified internal failure. */
        NOTIF_ERROR_INTERNAL_FAILURE = 200,
        /** indicates I2C transmission failure */
        NOTIF_ERROR_I2C_TRANSMISSION_FAILURE,
        /** Indicates CSI input stream error. */
        NOTIF_ERROR_CSI_INPUT_STREAM_FAILURE,
        /** Indicates CSI link error. */
        NOTIF_ERROR_CSI_LINK_FAILURE,
        /** Indicates Deserializer link error. */
        NOTIF_ERROR_DESER_LINK_FAILURE,
        /** Indicates Deserializer link error recovery failure. */
        NOTIF_ERROR_DESER_LINK_RECOVERY_FAILURE,
    };

    /** @brief Defines the actions to be performed after certain event */
    enum ActionType
    {
        /** Indicates no action to be taken. */
        ACTION_NONE = 0,
        /** Indicates attempt to recover the pipeline. */
        ACTION_ATTEMPT_RECOVERY,
    };

    /** @brief Defines the notification data. */
    struct NotificationData
    {
        /** Holds the @ref NotificationType event type. */
        NotificationType eNotifType;
        /** Holds the ID of the pipeline. This is the same as the Sensor ID in PlatformCfg. */
        std::uint32_t uIndex;
    };

    /** @brief Handles a pipeline event.
     *
     * This method is overridden by implementation of the consumer.
     * This function is called by SIPL pipeline thread.
     *
     * Consumer can use the events to track the flow of buffers in the image processing pipeline
     * and take any corrective action in case of an error or unexpected events.
     *
     * @param[in] oNotificationData A reference to @ref NotificationData. */
    virtual ActionType OnEvent(NotificationData &oNotificationData)
    {
        return ACTION_NONE;
    }

    /** @brief Default destructor. */
    virtual ~NvSIPLPipelineNotifier(void) = default;
};

/** @ingroup NvSIPLPipelineMgr
 *
 * @class NvSIPLImageGroupWriter
 *
 * @brief Describes the interfaces of SIPL pipeline feeder.
 *
 * This class defines data structures and interfaces that must be implemented by
 * SIPL pipeline feeder in case of ISP re-process mode.
 *
 * In ISP re-process mode, the user can feed unprocessed sensor output captured during
 * data collection process and then process them through HW ISP.
 *
 * The user must have configured @ref NvSIPLCamera_API using appropriate PlatformCfg to be able
 * to use this mode.
 */
class NvSIPLImageGroupWriter
{
public:
    /** @brief Describes an unprocessed sensor output buffer. */
    struct RawBuffer
    {
        /** Holds a pointer to @ref NvMediaImageGroup. */
        NvMediaImageGroup *imageGroup;
        /** Holds the ID of the sensor in PlatformCfg. */
        std::uint32_t uIndex;
    };

    /** @brief Populates the buffer with RAW data.
     *
     * This method is overridden by implementation of the consumer.
     * This function is called by SIPL pipeline thread.
     *
     * Feeder must populate the @ref RawBuffer with appropriate RAW data.
     *
     * @param[out] oRawBuffer A reference to @ref RawBuffer that will be populated by the function.
     *
     * @returns::SIPLStatus. The completion status of this operation. */
    virtual SIPLStatus FillRawBuffer(RawBuffer &oRawBuffer) = 0;

    /** @brief Default destructor. */
    virtual ~NvSIPLImageGroupWriter(void) = default;
};

/** @ingroup NvSIPLPipelineMgr
 *
 * @brief Describes attributes of images used in image processing pipeline. */
struct NvSIPLImageAttr {
    /** Holds the surface type (@ref NvMediaSurfaceType) of the image. */
    NvMediaSurfaceType surfaceType = NvMediaSurfaceType_Unsupported;

    /** Holds a vector of @ref NvMediaSurfAllocAttr objects. */
    std::vector<NvMediaSurfAllocAttr> surfaceAllocAttr;
};

}  // namespace nvsipl


#endif //_NVSIPLPIPELINEMGR_HPP_
