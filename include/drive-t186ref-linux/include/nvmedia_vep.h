/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: NvMedia Video Encode Processing API </b>
 *
 * This file contains the \ref encoder_api "Video Encode Processing API".
 */

#ifndef NVMEDIA_VEP_H
#define NVMEDIA_VEP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_common_encode.h"
#include "nvmedia_core.h"
#include "nvmedia_surface.h"
#include "nvmedia_video.h"

/**
 * \defgroup encoder_api Video Encoder
 * \ingroup nvmedia_video_top
 * The NvMediaVideoEncoder object takes uncompressed video data and tuns it
 * into a codec specific bitstream. Currently only H.264 encoding is supported.
 *
 * @{
 */

/** \brief Major Version number */
#define NVMEDIA_VEP_VERSION_MAJOR   2
/** \brief Minor Version number */
#define NVMEDIA_VEP_VERSION_MINOR   8

/**
 * \brief Video encoder codec type
 * @ingroup encoder_api
 */
typedef enum {
    /** \brief H.264 codec */
    NVMEDIA_VIDEO_ENCODE_CODEC_H264,
    /** \brief HEVC codec */
    NVMEDIA_VIDEO_ENCODE_CODEC_HEVC,
    /** \brief VP9 codec */
    NVMEDIA_VIDEO_ENCODE_CODEC_VP9,
    /** \brief VP8 codec */
    NVMEDIA_VIDEO_ENCODE_CODEC_VP8
} NvMediaVideoEncodeType;

/**
 * \brief Video encoder object created by \ref NvMediaVideoEncoderCreate.
 * @ingroup encoder_api
 */
typedef struct {
    /** Codec type */
    NvMediaVideoEncodeType codec;
    /** Input surface format */
    NvMediaSurfaceType inputFormat;
    /** Instance ID */
    NvMediaEncoderInstanceId instanceId;
    /** An Opaque pointer for internal use */
    struct NvMediaVideoEncoderPriv_ *encPriv;
} NvMediaVideoEncoder;


/** \brief Gets the version information for the NvMedia Video Encoder library.
 *
 * \pre  None
 * \post None
 *
 * <b>Approved usage</b>:
 * - Operation Mode: Init
 *
 * \param[in] version A pointer to a \ref NvMediaVersion structure to be filled
 * by the function.
 * \return \ref NvMediaStatus The completion status of the operation.
 * Possible values are:
 * \n \ref NVMEDIA_STATUS_OK
 * \n \ref NVMEDIA_STATUS_BAD_PARAMETER if the pointer is NULL.
 */
NvMediaStatus
NvMediaVideoEncoderGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Create an NvMediaVideoEncoder object instance
 *
 * Creates an encoder object capable of turning a stream of surfaces
 * of the "inputFormat" into a bitstream of the specified "codec".
 * Surfaces are feed to the encoder with \ref NvMediaVideoEncoderFeedFrame
 * and bitstream buffers are retrieved with \ref NvMediaVideoEncoderGetBits.
 *
 * \pre NvMediaVideoEncoderGetVersion()
 * \post NvMediaVideoEncoder object is created
 *
 * <b>Approved usage</b>:
 * - Operation Mode: Init
 *
 * \param[in] device The \ref NvMediaDevice "device" this video encoder will use.
 * \param[in] codec The codec to be used, which can be:
 *      \n \ref NVMEDIA_VIDEO_ENCODE_CODEC_H264
 *      \n \ref NVMEDIA_VIDEO_ENCODE_CODEC_HEVC
 *      \n \ref NVMEDIA_VIDEO_ENCODE_CODEC_VP9
 * \param[in] initParams Encode parameters, which can be:
 * \n Supported encode parameter structures:
 * \n \ref NvMediaEncodeInitializeParamsH264
 * \n \ref NvMediaEncodeInitializeParamsH265
 * \n \ref NvMediaEncodeInitializeParamsVP9
 * \param[in] inputFormat Must be obtained by /ref NvMediaSurfaceFormatGetType with:
 *      \n \ref NVM_SURF_FMT_SET_ATTR_YUV(attr, YUV, 420, SEMI_PLANAR, UINT, [8/10], BL)
 *      \n \ref NVM_SURF_FMT_SET_ATTR_YUV(attr, YUV, 444, SEMI_PLANAR, UINT, [8/10], BL)
 * \param[in] maxInputBuffering
 *      Maximum number of frames in the encode pipeline
 *      at any time. This parameter is no longer in use and kept here
 *      to maintain backward compatibility.
 * \param[in] maxOutputBuffering
 *      Maximum number of frames of encoded bitstream that can be held
 *      by the NvMediaVideoEncoder before it must be retrieved using
 *      \ref NvMediaVideoEncoderGetBits. This number must be less than or equal to 16.
 *      If \a maxOutputBuffering frames worth of encoded bitstream is yet to be
 *      retrived, \ref NvMediaVideoEncoderFeedFrame returns
 *      \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING. In this case, encoded output
 *      of one or more frames must be retrived with \ref NvMediaVideoEncoderGetBits
 *      before feeding more frames using \ref NvMediaVideoEncoderFeedFrame
 * \param[in] instanceId The ID of the encoder engine instance. The following
 *      instances are supported:
 *      \n \ref NVMEDIA_ENCODER_INSTANCE_0
 *      \n \ref NVMEDIA_ENCODER_INSTANCE_1
 *      \n \ref NVMEDIA_ENCODER_INSTANCE_AUTO
 *
 * \return \ref NvMediaVideoEncoder The new video encoder's handle or NULL if unsuccessful.
 */
NvMediaVideoEncoder *
NvMediaVideoEncoderCreate(
    const NvMediaDevice *device,
    NvMediaVideoEncodeType codec,
    const void *initParams,
    NvMediaSurfaceType inputFormat,
    uint8_t maxInputBuffering,
    uint8_t maxOutputBuffering,
    NvMediaEncoderInstanceId instanceId
);

/**
 * \brief Destroys an NvMediaVideoEncoder object.
 *
 * \pre NvMediaVideoEncoderSurfaceUnRegister()
 * \post NvMediaVideoEncoder object is destroyed
 *
 * <b>Approved usage</b>:
 * - Operation Mode: De-init
 *
 * \param[in] encoder A pointer to the encoder to destroy.
 */
void NvMediaVideoEncoderDestroy(const NvMediaVideoEncoder *encoder);

/**
 * \brief Submits the specified frame for encoding
 *
 * The encoding process is asynchronous, as a result, the encoded output may
 * not be available when the API returns. Refer \ref NvMediaVideoEncoderBitsAvailable
 * and \ref NvMediaVideoEncoderGetBits for more details regarding how to retrieve
 * the encoded output.
 *
 * \pre NvMediaVideoEncoderSurfaceRegister()
 * \pre NvMediaVideoEncoderSetConfiguration() must be called at least once to
 *      configure NvMediaVideoEncoder
 * \post Encoding task is submitted
 *
 * <b>Approved usage</b>:
 * - Operation Mode: Runtime
 *
 * \param[in] encoder The encoder to use.
 * \param[in] frame The frame, which
 *      must be of the same sourceType as in the NvMediaVideoEncoder.
 *      The size of this surface should be similar to the configured
 *      encoding size passed in initialization parameter of \ref NvMediaVideoEncoderCreate.
 * \param[in] sourceRect
 *       This parameter is not used anymore and kept here to maintain
 *       backward compatibility.
 * \param[in] picParams Picture parameters used for the frame.
 * \n Supported picture parameter structures:
 * \n \ref NvMediaEncodePicParamsH264
 * \n \ref NvMediaEncodePicParamsH265
 * \n \ref NvMediaEncodePicParamsVP9
 * \param[in] instanceId The specific ID of the encoder engine instance.
 *      \n <b>Note</b>: \ref NVMEDIA_ENCODER_INSTANCE_AUTO should not be
 *          passed to \ref NvMediaVideoEncoderFeedFrame
 *      \n If \ref NVMEDIA_ENCODER_INSTANCE_AUTO was \a not passed to
 *          \ref NvMediaVideoEncoderCreate API, then the same \a instanceId passed to
 *          \ref NvMediaVideoEncoderCreate API needs to be passed here as well.
 *      \n If \ref NVMEDIA_ENCODER_INSTANCE_AUTO was passed to
 *          \ref NvMediaVideoEncoderCreate API, then the following
 *          values of \a instanceId are supported for \ref NvMediaVideoEncoderFeedFrame
 *          API:
 *                \n \ref NVMEDIA_ENCODER_INSTANCE_0
 *                \n \ref NVMEDIA_ENCODER_INSTANCE_1
 *
 * \return \ref NvMediaStatus The completion status of the operation.
 * Possible values are:
 * \n \ref NVMEDIA_STATUS_OK
 * \n \ref NVMEDIA_STATUS_BAD_PARAMETER if any of the input parameters are invalid.
 * \n \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING if the rate at which the input
 *      frames are being fed to NvMediaVideoEncoder using \ref NvMediaVideoEncoderFeedFrame is
 *      much greater than the rate at which the encoded output is being
 *      retrieved using \ref NvMediaVideoEncoderGetBits. This case typically happens
 *      when \ref NvMediaVideoEncoderGetBits has not been called frequently enough and
 *      the internal queue containing encoded/encoding frames awaiting to be
 *      retrieved is full. The \a maxOutputBuffering argument passed to
 *      \ref NvMediaVideoEncoderCreate specifies the maximum number of calls that can be
 *      made to \ref NvMediaVideoEncoderFeedFrame after which at least one or more calls
 *      to \ref NvMediaVideoEncoderGetBits has to be made.
 * \n \ref NVMEDIA_STATUS_ERROR if there is a run-time error encountered during
 *      encoding
 */
NvMediaStatus
NvMediaVideoEncoderFeedFrame(
    const NvMediaVideoEncoder *encoder,
    const NvMediaVideoSurface *frame,
    const NvMediaRect *sourceRect,
    const void *picParams,
    NvMediaEncoderInstanceId instanceId
);

/**
 * \brief Sets the encoder configuration. The values in the configuration
 * take effect only at the start of the next GOP.
 *
 * \pre NvMediaVideoEncoderCreate()
 * \post NvMediaVideoEncoder object is configured
 *
 * <b>Approved usage</b>:
 * - Operation Mode: Runtime
 *
 * \param[in] encoder A pointer to the encoder to use.
 * \param[in] configuration Configuration data.
 * \n Supported configuration structures:
 * \n \ref NvMediaEncodeConfigH264
 * \n \ref NvMediaEncodeConfigH265
 * \n \ref NvMediaEncodeConfigVP9
 *
 * \return \ref NvMediaStatus The completion status of the operation.
 * Possible values are:
 * \n \ref NVMEDIA_STATUS_OK
 * \n \ref NVMEDIA_STATUS_BAD_PARAMETER if any of the input parameters are invalid.
 */
NvMediaStatus
NvMediaVideoEncoderSetConfiguration(
    const NvMediaVideoEncoder *encoder,
    const void *configuration
);

/**
 * \brief Returns the bitstream for a frame
 *
 *  It is safe to call this function from a separate thread.
 *
 * Copies a frame's worth of bitstream into the provided \a buffer. \a numBytes
 * returns the size of this bitstream.
 *
 * The return value and behavior is the same as that of
 * \ref NvMediaVideoEncoderBitsAvailable when called with
 * \ref NVMEDIA_ENCODE_BLOCKING_TYPE_NEVER except that when
 * \ref NVMEDIA_STATUS_OK is returned, the "buffer" will be filled in addition
 * to the \a numBytes.
 *
 * \pre NvMediaVideoEncoderFeedFrame()
 * \post Encoded bitstream corresponding to the submitted task is retrieved.
 *
 * <b>Approved usage</b>:
 * - Operation Mode: Runtime
 *
 * \param[in] encoder A pointer to the encoder to use.
 * \param[out] numBytes Size of the filled bitstream.
 * \param[in] buffer
 *      The buffer to be filled with the encoded data.
 *      \n Minimum Size: Size returned by \ref NvMediaVideoEncoderBitsAvailable
 *      \n Maximum Size: 16*1024*1024 bytes
 * \return \ref NvMediaStatus The completion status of the operation.
 * Possible values are:
 * \n \ref NVMEDIA_STATUS_OK
 * \n \ref NVMEDIA_STATUS_BAD_PARAMETER if any of the input parameters are invalid.
 * \n \ref NVMEDIA_STATUS_PENDING if an encode is in progress but not yet completed.
 * \n \ref NVMEDIA_STATUS_NONE_PENDING if no encode is in progress.
 * \n \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING if the size of the provided
 *         bitstream buffers are insufficient to hold the available output.
 * \n \ref NVMEDIA_STATUS_ERROR if there was an error during encoding.
 */
NvMediaStatus
NvMediaVideoEncoderGetBits(
    const NvMediaVideoEncoder *encoder,
    uint32_t *numBytes,
    void *buffer
);

/**
 * \brief Returns the status of an encoding task submitted using
 * \ref NvMediaVideoEncoderFeedFrame, whose encoded output is to be retrieved next.
 *
 * The number of bytes of encoded output that is available (if ready), is also
 * retrieved along with the status. The specific behavior depends on the
 * specified \ref NvMediaBlockingType.
 *
 * It is safe to call this function from a separate thread.
 *
 * \pre NvMediaVideoEncoderFeedFrame()
 * \post Status of the submitted encoding task is retrieved.
 *
 * <b>Approved usage</b>:
 * - Operation Mode: Runtime
 *
 * \param[in] encoder The encoder to use.
 * \param[out] numBytesAvailable
 *      The number of bytes of encoded output that is available. This output
 *      corresponds to the next encoding task in the queue for which
 *      the output is yet to be retrieved. The value is valid only when the
 *      return value from this API is \ref NVMEDIA_STATUS_OK.
 * \param[in] blockingType Blocking type.
 *      The following are supported blocking types:
 * \n \ref NVMEDIA_ENCODE_BLOCKING_TYPE_NEVER
 *        This type never blocks so \a millisecondTimeout is ignored.
 *        The following are possible return values:  \ref NVMEDIA_STATUS_OK
 *        \ref NVMEDIA_STATUS_PENDING or \ref NVMEDIA_STATUS_NONE_PENDING.
 * \n
 * \n \ref NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING
 *        Blocks if an encoding task is pending, until it completes or till it
 *        times out. If no encoding task is pending, doesn't block and returns
 *        \ref NVMEDIA_STATUS_NONE_PENDING.
 *        Possible return values: \ref NVMEDIA_STATUS_OK,
 *        \ref NVMEDIA_STATUS_NONE_PENDING, \ref NVMEDIA_STATUS_TIMED_OUT.
 * \param[in] millisecondTimeout
 *       Timeout in milliseconds or \ref NVMEDIA_VIDEO_ENCODER_TIMEOUT_INFINITE
 *       if a timeout is not desired.
 *
 * \return \ref NvMediaStatus The completion status of the operation.
 * Possible values are:
 * \n \ref NVMEDIA_STATUS_OK
 * \n \ref NVMEDIA_STATUS_BAD_PARAMETER if any of the input parameters are invalid.
 * \n \ref NVMEDIA_STATUS_PENDING if an encode is in progress but not yet completed.
 * \n \ref NVMEDIA_STATUS_NONE_PENDING if no encode is in progress.
 * \n \ref NVMEDIA_STATUS_TIMED_OUT if the operation timed out.
 * \n \ref NVMEDIA_STATUS_ERROR if there was an error during encoding.
 */
NvMediaStatus
NvMediaVideoEncoderBitsAvailable(
    const NvMediaVideoEncoder *encoder,
    uint32_t *numBytesAvailable,
    NvMediaBlockingType blockingType,
    uint32_t millisecondTimeout
);

/**
 * \brief Registers \ref NvMediaVideoSurface for use with a NvMediaVideoEncoder handle.
 * NvMediaVideoEncoder handle maintains a record of all the surfaces registered using this API.
 *
 * This API needs to be called in order to ensure deterministic execution time
 * of \ref NvMediaVideoEncoderFeedFrame on QNX platform. Although optional on
 * other platform configurations, it is highly recommended to use this API.
 *
 * To ensure deterministic execution time of \ref NvMediaVideoEncoderFeedFrame
 * API:
 * - \ref NvMediaVideoEncoderSurfaceRegister must be called for every input
 *   \ref NvMediaVideoSurface that will be used with NvMediaVideoEncoder
 * - All \ref NvMediaVideoEncoderSurfaceRegister calls must be made before first
 *   \ref NvMediaVideoEncoderFeedFrame API call.
 *
 * Maximum of 32 \ref NvMediaVideoSurface handles can be registered per access mode.
 *
 * \pre NvMediaVideoEncoderCreate()
 * \post NvMediaVideoSurface is registered with NvMediaVideoEncoder object
 *
 * <b>Approved usage</b>:
 * - Operation Mode: Init
 *
 * \param[in] encoder - A pointer to the encoder to use.
 * \param[in] frame - A pointer to NvMediaVideoSurface
 * \param[in] accessMode : \ref NvMediaAccessMode required for the surface
 *
 * \return \ref NvMediaStatus, the completion status of operation:
 * - \ref NVMEDIA_STATUS_OK if successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if encoder, surface or accessMode is invalid
 * - \ref NVMEDIA_STATUS_ERROR in following cases
 *          - user registers more than 32 surfaces.
 *          - user registers same surface with more than one accessModes.
 *          - user registers same surface multiple times
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED if API is not functional
 *
 **/
NvMediaStatus
NvMediaVideoEncoderSurfaceRegister(
    const NvMediaVideoEncoder *encoder,
    const NvMediaVideoSurface *frame,
    NvMediaAccessMode  accessMode
);

/**
 * \brief  Un-registers \ref NvMediaVideoSurface which is registered with NvMediaVideoEncoder
 *
 * For all \ref NvMediaVideoSurface handles registered with NvMediaVideoEncoder using
 * \ref NvMediaVideoEncoderSurfaceRegister API, \ref NvMediaVideoEncoderSurfaceUnRegister must
 * be called before calling \ref NvMediaVideoEncoderDestroy API. For unregistration to succeed,
 * it should be ensured that none of the submitted tasks on the surfaces are
 * pending prior to calling \ref NvMediaVideoEncoderSurfaceUnRegister API.
 * In order to ensure this, \ref NvMediaVideoEncoderGetBits API needs to be called
 * prior to unregistration, untill the output of all the submitted tasks are
 * available.
 *
 * This API needs to be called in order to ensure deterministic execution time
 * of \ref NvMediaVideoEncoderFeedFrame on QNX platform. Although optional on
 * other platform configurations, it is highly recommended to use this API.
 *
 * To ensure deterministic execution time of \ref NvMediaVideoEncoderFeedFrame
 * API:
 * - \ref NvMediaVideoEncoderSurfaceUnRegister must be called only after last
 *   \ref NvMediaVideoEncoderFeedFrame call.
 *
 * \pre NvMediaVideoEncoderFeedFrame()
 * \pre NvMediaVideoEncoderGetBits() [verify that processing is complete]
 * \post NvMediaVideoSurface is un-registered from NvMediaVideoEncoder object
 *
 * <b>Approved usage</b>:
 * - Operation Mode: De-init
 *
 * \param[in] encoder - A pointer to the encoder to use.
 * \param[in] frame - A pointer to NvMediaVideoSurface
 * \return \ref NvMediaStatus, the completion status of operation:
 * - \ref NVMEDIA_STATUS_OK if successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if encoder is invalid or unregister is
 * called for surface which is not registered using \ref NvMediaVideoEncoderSurfaceRegister
 * API
 * - \ref NVMEDIA_STATUS_PENDING if the enqueued operations on the NvMediaVideoSurface
 *   are still pending. In this case, the application can choose to wait for
 *   all the operations to complete through calls to \ref NvMediaVideoEncoderGetBits
 *   API, or re-try the NvMediaVideoEncoderSurfaceUnRegister API call, untill
 *   the status returned is not NVMEDIA_STATUS_PENDING
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED if API is not functional
 *
 **/
NvMediaStatus
NvMediaVideoEncoderSurfaceUnRegister(
    const NvMediaVideoEncoder *encoder,
    const NvMediaVideoSurface *frame
);

/** @} <!-- ends continuation of encoder_api group --> */

/*
 * \defgroup history_nvmedia_vep History
 * Provides change history for the NvMedia Video Encode API.
 *
 * \section history_nvmedia_vcp Version History
 *
 * <b> Version 1.0 </b> March 21, 2017
 * - Initial release
 *
 * <b> Version 1.1 </b/> April 18, 2017
 * - Added \ref NvMediaVideoEncodeType
 * - Added \ref NvMediaVideoEncoderGetVersion
 * - Added instance ID parameter to \ref NvMediaVideoEncoderCreate()
 * - Added instance ID parameter to \ref NvMediaVideoEncoderFeedFrame()
 * - Added instanceId to \ref NvMediaVideoEncoder object
 * - Changed \ref NvMediaVideoEncoderCreate codec parameter to
 *   \ref NvMediaVideoEncodeType
 * - Changed to use standard data type
 *
 * <b> Version 2.0 </b> May 9, 2017
 * - Removed optionalDevice param from NvMediaVideoEncoderCreate()
 *
 * <b> Version 2.1 </b> Sept 1, 2017
 * - Added NVMEDIA_VIDEO_ENCODE_CODEC_VP8 in \ref NvMediaVideoEncodeType enum
 *
 * <b> Version 2.2 </b> Dec 14, 2018
 * - Fix MISRA violationis 21.1 and 21.2
 *
 * <b> Version 2.3 </b> January 15, 2019
 * - Fix MISRA violations 8.13
 *
 * <b> Version 2.4 </b> Feb 7, 2019
 * - Added opaque handle for encoder
 *   internal usage into Video encoder object
 * - Fix MISRA violations 11.3
 * - Fix MISRA violations 8.13
 *
 * <b> Version 2.5 </b> Feb 28, 2019
 * - Added required header includes nvmedia_core.h and nvmedia_surface.h
 *
 * <b> Version 2.6 </b> July 10, 2019
 * - Header include nvmedia_common.h is replaced with nvmedia_common_encode.h
 *
 * <b> Version 2.7 </b> July 11, 2019
 * - Fix MISRA violations 8.13
 *
 * <b> Version 2.8 </b> Aug 14, 2019
 * - Added \ref NvMediaVideoEncoderSurfaceRegister API
 * - Added \ref NvMediaVideoEncoderSurfaceUnRegister API
 *
 */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_VEP_H */
