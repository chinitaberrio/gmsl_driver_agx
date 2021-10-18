/*
 * Copyright (c) 2014-2020, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: NvMedia Image Encode Processing API </b>
 *
 * This file contains the \ref image_encode_api "Image Encode Processing API".
 */

#ifndef NVMEDIA_IEP_H
#define NVMEDIA_IEP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_common_encode.h"
#include "nvmedia_core.h"
#include "nvmedia_image.h"
#include "nvmedia_surface.h"

/**
 * \defgroup image_encode_api Image Encoder
 *
 * The NvMediaIEP object takes uncompressed image data and turns it
 * into a codec specific bitstream. Only H.264, H.265 and VP9 encoding is supported.
 *
 * @ingroup nvmedia_image_top
 * @{
 */

/** \brief Major Version number */
#define NVMEDIA_IEP_VERSION_MAJOR   2
/** \brief Minor Version number */
#define NVMEDIA_IEP_VERSION_MINOR   12

/**
 * \brief Image encode type
 */
typedef enum {
    /** \brief H.264 encode */
    NVMEDIA_IMAGE_ENCODE_H264,
    /** \brief HEVC codec */
    NVMEDIA_IMAGE_ENCODE_HEVC,
    /** \brief VP9 codec */
    NVMEDIA_IMAGE_ENCODE_VP9,
    /** \brief VP8 codec */
    NVMEDIA_IMAGE_ENCODE_VP8
} NvMediaIEPType;

/**
 * \brief Holds the image encoder object created by \ref NvMediaIEPCreate.
 */
typedef struct NvMediaIEPRec {
    /** Codec type */
    NvMediaIEPType encodeType;
    /** Input Image surface type. */
    NvMediaSurfaceType inputFormat;
    /** Instance ID */
    NvMediaEncoderInstanceId instanceId;
    /** An Opaque pointer for internal use */
    struct NvMediaIEPPriv_ *encPriv;
} NvMediaIEP;

/**
 * \brief Checks the version compatibility for the NvMedia IEP library.
 *
 * \pre  None
 * \post None
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Init
 *
 * \param[in] version A pointer to a \ref NvMediaVersion structure
 *                    of the client.
 * \return \ref NvMediaStatus The status of the operation.
 * Possible values are:
 * \n \ref NVMEDIA_STATUS_OK
 * \n \ref NVMEDIA_STATUS_BAD_PARAMETER if the pointer is NULL.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIEPGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Create an NvMediaIEP object instance
 *
 * \pre NvMediaIEPGetVersion()
 * \pre NvMediaIEPNvSciSyncGetVersion() [for use with IEP-NvSciSync APIs]
 * \post NvMediaIEP object is created
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Init
 *
 * Creates an encoder object capable of turning a stream of surfaces
 * of the "inputFormat" into a bitstream of the specified "codec".
 * Surfaces are feed to the encoder with \ref NvMediaIEPFeedFrame
 * and bitstream buffers are retrieved with \ref NvMediaIEPGetBitsEx.
 *
 * \param[in] device The \ref NvMediaDevice "device" this image encoder will use.
 * \param[in] encodeType The encode type, which can be:
 *      \n \ref NVMEDIA_IMAGE_ENCODE_H264
 *      \n \ref NVMEDIA_IMAGE_ENCODE_HEVC
 *      \n \ref NVMEDIA_IMAGE_ENCODE_VP9
 * \param[in] initParams The encode parameters, which can be:
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
 *      by the NvMediaIEP before it must be retrieved using
 *      \ref NvMediaIEPGetBitsEx. This number must be less than or equal to 16.
 *      If \a maxOutputBuffering frames worth of encoded bitstream is yet to be
 *      retrived, \ref NvMediaIEPFeedFrame returns
 *      \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING. In this case, encoded output
 *      of one or more frames must be retrived with \ref NvMediaIEPGetBitsEx
 *      before feeding more frames using \ref NvMediaIEPFeedFrame
 * \param[in] instanceId The ID of the encoder engine instance. The following
 *      instances are supported:
 *      \n \ref NVMEDIA_ENCODER_INSTANCE_0
 *      \n \ref NVMEDIA_ENCODER_INSTANCE_1      [Unsupported in safety build]
 *      \n \ref NVMEDIA_ENCODER_INSTANCE_AUTO   [Unsupported in safety build]
 *
 * \return \ref NvMediaIEP The new image encoder's handle or NULL if unsuccessful.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaIEP *
NvMediaIEPCreate(
    const NvMediaDevice *device,
    NvMediaIEPType encodeType,
    const void *initParams,
    NvMediaSurfaceType inputFormat,
    uint8_t maxInputBuffering,
    uint8_t maxOutputBuffering,
    NvMediaEncoderInstanceId instanceId
);

/**
 * \brief Destroys an NvMedia image encoder.
 *
 * \pre NvMediaIEPImageUnRegister()
 * \pre NvMediaIEPUnregisterNvSciSyncObj() [for use with IEP-NvSciSync APIs]
 * \post NvMediaIEP object is destroyed
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: De-init
 *
 * \param[in] encoder The encoder to destroy.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
void NvMediaIEPDestroy(const NvMediaIEP *encoder);

/**
 * \brief Submits the specified frame for encoding
 *
 * The encoding process is asynchronous, as a result, the encoded output may
 * not be available when the API returns. Refer \ref NvMediaIEPBitsAvailable
 * and \ref NvMediaIEPGetBitsEx for more details regarding how to retrieve
 * the encoded output.
 *
 * \pre NvMediaIEPImageRegister()
 * \pre NvMediaIEPSetConfiguration() must be called at least once to configure
 *      NvMediaIEP
 * \pre NvMediaIEPSetNvSciSyncObjforEOF() [optional - for use with IEP-
 *                                         NvSciSync APIs]
 * \pre NvMediaIEPInsertPreNvSciSyncFence() [optional - for use with IEP-
 *                                           NvSciSync APIs]
 * \post Image encoding task is submitted
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Runtime
 *
 * \param[in] encoder The encoder to use.
 * \param[in] frame The frame, which
 *      must be of the same sourceType as in the NvMediaIEP.
 *      The size of this surface should be similar to the configured
 *      encoding size passed in initialization parameter of \ref NvMediaIEPCreate.
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
 *          passed to \ref NvMediaIEPFeedFrame
 *      \n If \ref NVMEDIA_ENCODER_INSTANCE_AUTO was \a not passed to
 *          \ref NvMediaIEPCreate API, then the same \a instanceId passed to
 *          \ref NvMediaIEPCreate API needs to be passed here as well.
 *      \n If \ref NVMEDIA_ENCODER_INSTANCE_AUTO was passed to
 *          \ref NvMediaIEPCreate API (non-safety build), then the following
 *          values of \a instanceId are supported for \ref NvMediaIEPFeedFrame
 *          API:
 *                \n \ref NVMEDIA_ENCODER_INSTANCE_0
 *                \n \ref NVMEDIA_ENCODER_INSTANCE_1
 *
 * \return \ref NvMediaStatus The completion status of the operation.
 * Possible values are:
 * \n \ref NVMEDIA_STATUS_OK
 * \n \ref NVMEDIA_STATUS_BAD_PARAMETER if any of the input parameters are invalid.
 * \n \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING if the rate at which the input
 *      frames are being fed to NvMediaIEP using \ref NvMediaIEPFeedFrame is
 *      much greater than the rate at which the encoded output is being
 *      retrieved using \ref NvMediaIEPGetBitsEx. This case typically happens
 *      when \ref NvMediaIEPGetBitsEx has not been called frequently enough and
 *      the internal queue containing encoded/encoding frames awaiting to be
 *      retrieved is full. The \a maxOutputBuffering argument passed to
 *      \ref NvMediaIEPCreate specifies the maximum number of calls that can be
 *      made to \ref NvMediaIEPFeedFrame after which at least one or more calls
 *      to \ref NvMediaIEPGetBitsEx has to be made.
 * \n \ref NVMEDIA_STATUS_ERROR if there is a run-time error encountered during
 *      encoding
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIEPFeedFrame(
    const NvMediaIEP *encoder,
    const NvMediaImage *frame,
    NvMediaRect *sourceRect,
    const void *picParams,
    NvMediaEncoderInstanceId instanceId
);

/**
 * \brief Sets the encoder configuration. The values in the configuration
 * take effect only at the start of the next GOP.
 *
 * \pre NvMediaIEPCreate()
 * \post NvMediaIEP object is configured
 *
 * <b>Considerations for Safety</b>:
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
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIEPSetConfiguration(
    const NvMediaIEP *encoder,
    const void *configuration
);

/**
 * \brief Returns the bitstream for a slice or frame.
 *
 *  It is safe to call this function from a separate thread.
 *
 *  The return value and behavior of \ref NvMediaIEPGetBitsEx is the same as
 *  that of \ref NvMediaIEPBitsAvailable when called with
 *  \ref NVMEDIA_ENCODE_BLOCKING_TYPE_NEVER, except that when
 *  \ref NVMEDIA_STATUS_OK is returned, \a bitstreams is filled in addition
 *  to \a numBytes.
 *
 * \pre NvMediaIEPFeedFrame()
 * \post Encoded bitstream corresponding to the submitted task is retrieved.
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Runtime
 *
 * \param[in] encoder A pointer to the encoder to use.
 * \param[out] numBytes Size of the filled bitstream.
 * \param[in] numBitstreamBuffers The length of the \a bitstreams array
 *      \n <b>Range</b>: 1 to 64 in steps of 1
 * \param[in,out] bitstreams A pointer to an array of type
 *      \ref NvMediaBitstreamBuffer, the length of the array being passed in
 *      \a numBitstreamBuffers. Encoded bitstream data will be filled in the
 *      entries of the array, starting from index 0 to
 *      (\a numBitstreamBuffers - 1), as and when each entry in the array gets
 *      filled up. The minimum combined bitstream size needs to be at least
 *      equal to the \a numBytesAvailable returned by
 *      \ref NvMediaIEPBitsAvailable call.
 *      \n Members of each \ref NvMediaBitstreamBuffer are to be set as follows:
 *      \n \ref NvMediaBitstreamBuffer.bitstream is assigned a pointer to an
 *          array of type uint8_t, where the encoded bitstream output will be
 *          written to by NvMediaIEP
 *      \n \ref NvMediaBitstreamBuffer.bitstreamSize is assigned with the size
 *          of the \ref NvMediaBitstreamBuffer.bitstream array
 *      \n \ref NvMediaBitstreamBuffer.bitstreamBytes will be populated by
 *          NvMediaIEP to indicate the number of encoded bitstream bytes that
 *          are populated in \ref NvMediaBitstreamBuffer.bitstream once the
 *          function call returns
 * \param[in,out] extradata This is an internal feature. This parameter can no
 *          longer be used in external applications and is retained only for
 *          backward compatibility. This should be set to NULL.
 *
 * \return \ref NvMediaStatus The completion status of the operation.
 * Possible values are:
 * \n \ref NVMEDIA_STATUS_OK
 * \n \ref NVMEDIA_STATUS_BAD_PARAMETER if any of the input parameters are invalid.
 * \n \ref NVMEDIA_STATUS_PENDING if an encode is in progress but not yet completed.
 * \n \ref NVMEDIA_STATUS_NONE_PENDING if no encode is in progress.
 * \n \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING if the size of the provided
 *         bitstream buffers are insufficient to hold the available output.
 * \n \ref NVMEDIA_STATUS_ERROR if there was an error during image encoding.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIEPGetBitsEx(
    const NvMediaIEP *encoder,
    uint32_t *numBytes,
    uint32_t numBitstreamBuffers,
    const NvMediaBitstreamBuffer *bitstreams,
    void *extradata
);

/**
 * \brief Returns the status of an encoding task submitted using
 * \ref NvMediaIEPFeedFrame, whose encoded output is to be retrieved next.
 *
 * The number of bytes of encoded output that is available (if ready), is also
 * retrieved along with the status. The specific behavior depends on the
 * specified \ref NvMediaBlockingType.
 *
 * It is safe to call this function from a separate thread.
 *
 * \pre NvMediaIEPFeedFrame()
 * \post Status of the submitted encoding task is retrieved.
 *
 * <b>Considerations for Safety</b>:
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
 * \n \ref NVMEDIA_STATUS_ERROR if there was an error during image encoding.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIEPBitsAvailable(
    const NvMediaIEP *encoder,
    uint32_t *numBytesAvailable,
    NvMediaBlockingType blockingType,
    uint32_t millisecondTimeout
);


/**
 * \brief Gets the encoder attribute for the current encoding session.
 * This function can be called after passing the first frame for encoding.
 * It can be used to get header information (SPS/PPS/VPS) for the
 * current encoding session. Additionally, it can be extended for further
 * requirements, by implementing proper data structures.
 *
 * Before calling this function, you must pass the first frame for encoding.
 *
 * \pre NvMediaIEPFeedFrame() called at least once
 * \post Value of the required attribute is retrieved.
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Runtime
 *
 * \param[in] encoder A pointer to the encoder to use.
 * \param[in] attrType Attribute type as defined in \ref NvMediaEncAttrType.
 * \param[in] attrSize Size of the data structure associated with attribute.
 * \param[out] AttributeData A pointer to data structure associated with the attribute.
 *
 * \return \ref NvMediaStatus The completion status of the operation.
 * Possible values are:
 * \n \ref NVMEDIA_STATUS_OK
 * \n \ref NVMEDIA_STATUS_BAD_PARAMETER if any of the input parameters are invalid.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIEPGetAttribute(
    const NvMediaIEP *encoder,
    NvMediaEncAttrType attrType,
    uint32_t attrSize,
    void *AttributeData
);

/**
 * \brief Registers \ref NvMediaImage for use with a NvMediaIEP handle.
 * NvMediaIEP handle maintains a record of all the images registered using this API.
 *
 * This is a mandatory API on QNX safety build to ensure deterministic execution
 * time of \ref NvMediaIEPFeedFrame. Although optional on other platform
 * configurations, it is highly recommended to use this API.
 *
 * To ensure deterministic execution time of \ref NvMediaIEPFeedFrame API:
 * - \ref NvMediaIEPImageRegister must be called for every input
 *   \ref NvMediaImage that will be used with NvMediaIEP
 * - All \ref NvMediaIEPImageRegister calls must be made before first
 *   \ref NvMediaIEPFeedFrame API call.
 *
 * Maximum of 32 \ref NvMediaImage handles can be registered per access mode.
 *
 * \pre NvMediaIEPCreate()
 * \post NvMediaImage is registered with NvMediaIEP object
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Init
 *
 * \param[in] encoder    : NvMedia IEP device handle.
 * \param[in] image      : A pointer to NvMedia image
 * \param[in] accessMode : \ref NvMediaAccessMode required for the image
 *
 * \return \ref NvMediaStatus, the completion status of operation:
 * - \ref NVMEDIA_STATUS_OK if successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if encoder, image or accessMode is invalid
 * - \ref NVMEDIA_STATUS_ERROR in following cases
 *          - user registers more than 32 images.
 *          - user registers same image with more than one accessModes.
 *          - user registers same image multiple times
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED if API is not functional
 *
 **/
NvMediaStatus
NvMediaIEPImageRegister(
    const NvMediaIEP   *encoder,
    const NvMediaImage *image,
    NvMediaAccessMode  accessMode
);

/**
 * \brief  Un-registers \ref NvMediaImage which is registered with NvMediaIEP
 *
 * For all \ref NvMediaImage handles registered with NvMediaIEP using
 * \ref NvMediaIEPImageRegister API, \ref NvMediaIEPImageUnRegister must be called
 * before calling \ref NvMediaIEPDestroy API. For unregistration to succeed,
 * it should be ensured that none of the submitted tasks on the image are
 * pending prior to calling \ref NvMediaIEPImageUnRegister API.
 * In order to ensure this, \ref NvMediaIEPGetBitsEx API needs to be called
 * prior to unregistration, untill the output of all the submitted tasks are
 * available.
 *
 * This is a mandatory API on QNX safety build to ensure deterministic execution
 * time of \ref NvMediaIEPFeedFrame. Although optional on other platform
 * configurations, it is highly recommended to use this API.
 *
 * To ensure deterministic execution time of \ref NvMediaIEPFeedFrame API:
 * - \ref NvMediaIEPImageUnRegister should be called only after the last
 *   \ref NvMediaIEPFeedFrame call
 *
 * \pre NvMediaIEPFeedFrame()
 * \pre NvMediaIEPGetBitsEx() [verify that processing is complete]
 * \post NvMediaImage is un-registered from NvMediaIEP object
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: De-init
 *
 * \param[in] encoder    : NvMedia IEP device handle.
 * \param[in] image      : A pointer to NvMedia image
 *
 * \return \ref NvMediaStatus, the completion status of operation:
 * - \ref NVMEDIA_STATUS_OK if successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if encoder is invalid or unregister is
 *   called for image which is not registered using \ref NvMediaIEPImageRegister
 *   API
 * - \ref NVMEDIA_STATUS_PENDING if the enqueued operations on the NvMediaImage
 *   are still pending. In this case, the application can choose to wait for
 *   all the operations to complete through calls to \ref NvMediaIEPGetBitsEx
 *   API, or re-try the NvMediaIEPImageUnRegister API call, untill the status
 *   returned is not NVMEDIA_STATUS_PENDING
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED if API is not functional
 *
 **/
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIEPImageUnRegister(
    const NvMediaIEP   *encoder,
    const NvMediaImage *image
);

/*
 * \defgroup history_nvmedia_iep History
 * Provides change history for the NvMedia IEP API.
 *
 * \section history_nvmedia_iep Version History
 *
 * <b> Version 1.0 </b> March 14, 2016
 * - Initial release
 *
 * <b> Version 1.1 </b> May 11, 2016
 * - Added \ref NvMediaICPCheckVersion API
 *
 * <b> Version 1.2 </b> May 5, 2017
 * - Added \ref NvMediaIEPGetBitsEx and \ref NvMediaIEPSetInputExtraData API
 * - Added \ref NvMediaIEPGetAttribute
 *
 * <b> Version 2.0 </b> May 9, 2017
 * - Added instance ID parameter to NvMediaIEPCreate() and NvMediaIEPFeedFrame()
 * - optionalDevice parameter is removed from NvMediaIEPCreate()
 * - NvMediaIEPGetVersion() is added to get the version of NvMedia Image Encoder library
 * - NvMediaIEPGetBits() is deprecated. Use NvMediaIEPGetBitsEx() instead.
 * - All NvMedia data types are moved to standard data types
 *
 * <b> Version 2.1 </b> June 30, 2017
 * - Removed \ref NvMediaIEPSetInputExtraData API
 *
 * <b> Version 2.2 </b> Sept 1, 2017
 * - Added NVMEDIA_IMAGE_ENCODE_VP8 in \ref NvMediaIEPType enum
 *
 * <b> Version 2.3 </b> Dec 14, 2018
 * - Fix MISRA violations 21.1 and 21.2
 *
 * <b> Version 2.4 </b> January 15, 2019
 * - Fix MISRA violations 8.13
 *
 * <b> Version 2.5 </b> Feb 7, 2019
 * - Added opaque handle for encoder
 *   internal usage into Image encoder object
 * - Fix MISRA violations 11.3
 * - Fix MISRA violations 8.13
 *
 * <b> Version 2.6 </b> Feb 28, 2019
 * - Added required header includes nvmedia_core.h and nvmedia_surface.h
 *
 * <b> Version 2.7 </b> July 10, 2019
 * - Header include nvmedia_common.h is replaced with nvmedia_common_encode.h
 *
 *  <b> Version 2.8 </b> July 13, 2019
 * - Corrected parameter description for \ref NvMediaIEPCreate and
 *   \ref NvMediaIEPFeedFrame.
 *
 *  <b> Version 2.9 </b> July 15, 2019
 * - Fix MISRA violations 8.13
 *
 *  <b> Version 2.10 </b> Jul 16, 2019
 * - Added \ref NvMediaIEPImageRegister API
 * - Added \ref NvMediaIEPImageUnRegister API
 *
 *  <b> Version 2.11 </b> October 5, 2019
 * - Fix MISRA violations 8.13
 * - Added Coverity Code annotations to suppress partial 8.7 violations
 *
 *  <b> Version 2.12 </b> November 25, 2019
 * - Fix MISRA violations 8.13
 */

/** @} */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_IEP_H */
