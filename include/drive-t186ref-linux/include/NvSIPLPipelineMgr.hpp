/*
 * Copyright (c) 2018-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef NVSIPLPIPELINEMGR_HPP
#define NVSIPLPIPELINEMGR_HPP

#include "NvSIPLCommon.hpp"
#include "NvSIPLClient.hpp"

#include "nvmedia_image.h"
#include "nvmedia_surface.h"

#include <cstdint>
#include <vector>

/**
 * @file
 *
 * @brief <b> NVIDIA SIPL: Pipeline Manager -
 *     @ref NvSIPLPipelineMgr </b>
 *
 */

namespace nvsipl
{

/** @defgroup NvSIPLPipelineMgr NvSIPL Pipeline Manager
 *
 * @brief Programs Video Input (VI) and Image Signal Processor (ISP)
 *  hardware blocks using NvMedia API to create image processing pipelines
 *  for each sensor.
 *
 * @ingroup NvSIPLCamera_API
 * @{
 */

/**
  *  Defines types of SIPL Control Auto plug-ins.
  */
enum PluginType {
   NV_PLUGIN = 0,  /**< NVIDIA plug-in */
   CUSTOM_PLUGIN0, /**< Custom plug-in 0 */
   MAX_NUM_PLUGINS /**< Maximum number of plug-ins supported. */
};

/**
 *
 * @class NvSIPLPipelineNotifier
 *
 * @brief Describes the interfaces of the SIPL pipeline notification handler.
 *
 * This class defines data structures and interfaces that must be implemented by
 * a SIPL pipeline notification handler.
 */
class NvSIPLPipelineNotifier
{
public:
    /** @brief Defines the events of the image processing pipeline. */
    enum NotificationType
    {
        /** Indicates ICP processing is finished. */
        NOTIF_INFO_ICP_PROCESSING_DONE = 0,
        /** Indicates ISP processing is finished. */
        NOTIF_INFO_ISP_PROCESSING_DONE,
        /** Indicates auto control processing is finished. */
        NOTIF_INFO_ACP_PROCESSING_DONE,
        /** Indicates CDI processing is finished. */
        NOTIF_INFO_CDI_PROCESSING_DONE,

        /** Indicates pipeline was forced to drop a frame due to a slow consumer or system issues.
        * @note Only eNotifType & uIndex members are valid in NotificationData for this event. */
        NOTIF_WARN_ICP_FRAME_DROP = 100,
        /** Indicates a discontinuity was detected in parsed embedded data frame sequence number.
        * @note Only eNotifType & uIndex members are valid in NotificationData for this event. */
        NOTIF_WARN_ICP_FRAME_DISCONTINUITY,
        /** Indicates occurrence of timeout while capturing.
        * @note Only eNotifType & uIndex members are valid in NotificationData for this event. */
        NOTIF_WARN_ICP_CAPTURE_TIMEOUT,

        /** Indicates ICP bad input stream.
        * @note Only eNotifType & uIndex members are valid in NotificationData for this event. */
        NOTIF_ERROR_ICP_BAD_INPUT_STREAM = 200,
        /** Indicates ICP capture failure.
        * @note Only eNotifType & uIndex members are valid in NotificationData for this event. */
        NOTIF_ERROR_ICP_CAPTURE_FAILURE,
        /** Indicates embedded data parsing failure. */
        NOTIF_ERROR_ICP_EMB_DATA_PARSE_FAILURE,
        /** Indicates ISP processing failure. */
        NOTIF_ERROR_ISP_PROCESSING_FAILURE,
        /** Indicates auto control processing failure. */
        NOTIF_ERROR_ACP_PROCESSING_FAILURE,
        /** Indicates CDI set sensor control failure. */
        NOTIF_ERROR_CDI_SET_SENSOR_CTRL_FAILURE,
        /** Indicates a deserializer link error.
        * @note Only eNotifType & uIndex members are valid in NotificationData for this event. */
        NOTIF_ERROR_DESER_LINK_FAILURE,

        /** Indicates an unexpected internal failure. */
        NOTIF_ERROR_INTERNAL_FAILURE = 300,
    };

    /** @brief Defines the notification data.
     * @note A few members are not valid for certain events, please see @ref NotificationType. */
    struct NotificationData
    {
        /** Holds the @ref NotificationType event type. */
        NotificationType eNotifType;
        /** Holds the ID of the pipeline. This is the same as the Sensor ID in PlatformCfg. */
        std::uint32_t uIndex;
        /** Holds the TSC timestamp of the frame capture. */
        std::uint64_t frameCaptureTSC;
    };

    /** @brief Handles a pipeline event.
     *
     * The consumer's implementation overrides this method.
     * The method is called by the SIPL pipeline thread.
     *
     * The consumer can use pipeline events to track the flow of buffers
     * in the image processing pipeline and take any necessary
     * corrective action in case of an error or unexpected events.
     *
     * @param[in] oNotificationData A reference to @ref NotificationData. */
    virtual void OnEvent(NotificationData &oNotificationData)
    {
        return;
    }

    /** @brief Default destructor. */
    virtual ~NvSIPLPipelineNotifier(void) = default;
};

/**
 *
 * @class NvSIPLImageGroupWriter
 *
 * @brief Describes the interfaces of SIPL pipeline feeder.
 *
 * This class defines data structures and interfaces that must be implemented by
 * the SIPL pipeline feeder in case of ISP reprocess mode.
 *
 * In ISP reprocess mode, the user can feed unprocessed sensor output captured
 * during the data collection process and then process it through HW ISP.
 *
 * The user must have configured @ref NvSIPLCamera_API using an appropriate
 * PlatformCfg to be able
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
        /** Holds a flag to signal discontinuity for the current raw buffer from the previous one. */
        bool discontinuity;
    };

    /** @brief Populates the buffer with RAW data.
     *
     * The consumer's implementation overrides this method.
     * The method is called by SIPL pipeline thread.
     *
     * The feeder must populate the @ref RawBuffer with appropriate RAW data.
     *
     * @param[out] oRawBuffer   A reference to the @ref RawBuffer that
     *                           the function is to populate.
     *
     * @returns::SIPLStatus. The completion status of this operation. */
    virtual SIPLStatus FillRawBuffer(RawBuffer &oRawBuffer) = 0;

    /** @brief Default destructor. */
    virtual ~NvSIPLImageGroupWriter(void) = default;
};

/**
 *
 * @brief Describes attributes of images used in image processing pipeline. */
struct NvSIPLImageAttr {
    /** Holds the surface type of the image. */
    NvMediaSurfaceType surfaceType = NvMediaSurfaceType_Unsupported;

    /** Holds a vector of @ref NvMediaSurfAllocAttr objects. */
    std::vector<NvMediaSurfAllocAttr> surfaceAllocAttr;
};

/** @brief Downscale and crop configuration. */
struct NvSIPLDownscaleCropCfg
{
    /** Indicates if ISP input crop is enabled. */
    bool ispInputCropEnable {false};
    /** ISP input crop rectangle. */
    NvMediaRect ispInputCrop;

    /** Indicates if ISP0 output crop is enabled. */
    bool isp0OutputCropEnable {false};
    /** ISP0 output crop rectangle. */
    NvMediaRect isp0OutputCrop;

    /** Indicates if ISP1 output crop is enabled. */
    bool isp1OutputCropEnable {false};
    /** ISP1 output crop rectangle. */
    NvMediaRect isp1OutputCrop;

    /** Indicates if ISP0 downscale is enabled. */
    bool isp0DownscaleEnable {false};
    /** ISP0 downscale width. */
    std::uint32_t isp0DownscaleWidth;
    /** ISP0 downscale height. */
    std::uint32_t isp0DownscaleHeight;

    /** Indicates if ISP1 downscale is enabled. */
    bool isp1DownscaleEnable {false};
    /** ISP1 downscale width. */
    std::uint32_t isp1DownscaleWidth;
    /** ISP1 downscale height. */
    std::uint32_t isp1DownscaleHeight;
};

/**
 * @brief Defines the camera pipeline configuration.
 *
 * @deprecated This struct is deprecated and will be removed in a future release.
 * Use @ref NvSIPLPipelineConfiguration instead.
 */
struct NvSIPLPipelineCfg
{
    /** Holds a vector of @ref INvSIPLClient::ConsumerDesc. */
    std::vector<INvSIPLClient::ConsumerDesc> clients;

    /** Holds a downscale and crop configuration. */
    NvSIPLDownscaleCropCfg downscaleCropCfg {};

    /** Holds a pointer to an @ref NvSIPLPipelineNotifier. */
    NvSIPLPipelineNotifier* notifier {nullptr};

    /** Holds a pointer to an @ref NvSIPLImageGroupWriter. */
    NvSIPLImageGroupWriter* imageGroupWriter {nullptr};
};

/**
 * @brief Defines the camera pipeline configuration.
 *
 */
struct NvSIPLPipelineConfiguration
{
    /** <tt>true</tt> if the client wants capture output frames to be delivered */
    bool captureOutputRequested {false};

    /** <tt>true</tt> if the client wants frames to be delivered from the first ISP output */
    bool isp0OutputRequested {false};

    /** <tt>true</tt> if the client wants frames to be delivered from the second ISP output */
    bool isp1OutputRequested {false};

    /** Holds a downscale and crop configuration. */
    NvSIPLDownscaleCropCfg downscaleCropCfg {};

    /** Holds a pointer to an @ref NvSIPLImageGroupWriter. */
    NvSIPLImageGroupWriter* imageGroupWriter {nullptr};
};

/**
 * @brief The interface to the frame completion queue.
 */
class INvSIPLFrameCompletionQueue
{
public:

    /**
     * @brief Retrieve the next item from the queue.
     *
     * The buffer returned will have a single reference that must be released by the client
     * when it has finished with the buffer. This is done by calling item->Release().
     *
     * @param[out] item The item retrieved from the queue.
     * @param[in] timeoutUsec The timeout of the request, in microseconds.
     * If the queue is empty at the time of the call,
     * this method will wait up to @c timeoutUsec microseconds
     * for a new item to arrive in the queue and be returned.
     *
     * @retval NVSIPL_STATUS_OK if @c item has been successfully retrieved from the queue.
     * @retval NVSIPL_STATUS_TIMED_OUT if an item was not available within the timeout interval.
     * @retval NVSIPL_STATUS_EOF if the queue has been shut down.
     * In this case, no further calls can be made on the queue object.
     * @retval NVSIPL_STATUS_ERROR if a system error occurred.
     */
    virtual SIPLStatus Get(INvSIPLClient::INvSIPLBuffer*& item,
                           size_t timeoutUsec) = 0;

    /**
     * @brief Return the current queue length.
     *
     * @returns the number of elements currently in the queue.
     */
    virtual size_t GetCount() const = 0;

protected:

    INvSIPLFrameCompletionQueue() = default;
    virtual ~INvSIPLFrameCompletionQueue() = default;

private:

    INvSIPLFrameCompletionQueue(INvSIPLFrameCompletionQueue const& other) = delete;
    INvSIPLFrameCompletionQueue& operator=(INvSIPLFrameCompletionQueue const& rhs) = delete;
};

/**
 * @brief The interface to the notification queue.
 */
class INvSIPLNotificationQueue
{
public:

    /**
     * @brief Retrieve the next item from the queue.
     *
     * @param[out] item The item retrieved from the queue.
     * @param[in] timeoutUsec The timeout of the request, in microseconds.
     * If the queue is empty at the time of the call,
     * this method will wait up to @c timeoutUsec microseconds
     * for a new item to arrive in the queue and be returned.
     *
     * @retval NVSIPL_STATUS_OK if @c item has been successfully retrieved from the queue.
     * @retval NVSIPL_STATUS_TIMED_OUT if an item was not available within the timeout interval.
     * @retval NVSIPL_STATUS_EOF if the queue has been shut down.
     * In this case, no further calls can be made on the queue object.
     * @retval NVSIPL_STATUS_ERROR if a system error occurred.
     */
    virtual SIPLStatus Get(NvSIPLPipelineNotifier::NotificationData& item,
                           size_t timeoutUsec) = 0;

    /**
     * @brief Return the current queue length.
     *
     * @returns the number of elements currently in the queue.
     */
    virtual size_t GetCount() const = 0;

protected:

    INvSIPLNotificationQueue() = default;
    virtual ~INvSIPLNotificationQueue() = default;

private:

    INvSIPLNotificationQueue(INvSIPLNotificationQueue const& other) = delete;
    INvSIPLNotificationQueue& operator=(INvSIPLNotificationQueue const& rhs) = delete;
};

/**
 * @brief This is the output structure for the new version of SetPipelineCfg().
 * It contains the queues used by the client to receive completed frames
 * and event notifications.
 */
struct NvSIPLPipelineQueues
{
    /**
     * The queue for completed capture frames.
     * Will be null if capture output was not requested.
     */
    INvSIPLFrameCompletionQueue* captureCompletionQueue;

    /**
     * The queue for completed frames from the first ISP output.
     * Will be null if the first ISP output was not requested.
     */
    INvSIPLFrameCompletionQueue* isp0CompletionQueue;

    /**
     * The queue for completed frames from the second ISP output.
     * Will be null if the second ISP output was not requested.
     */
    INvSIPLFrameCompletionQueue* isp1CompletionQueue;

    /** The queue for event notifications. */
    INvSIPLNotificationQueue* notificationQueue;
};

/** @} */

}  // namespace nvsipl


#endif // NVSIPLPIPELINEMGR_HPP
