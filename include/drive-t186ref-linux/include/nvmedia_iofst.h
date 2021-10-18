/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: NvMedia Image OpticalFlow/StereoDisparity
 *        (OFST) Estimator API </b>
 */

#ifndef NVMEDIA_IOFST_H
#define NVMEDIA_IOFST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "nvmedia_image.h"
#include "nvmedia_common_encode_ofst.h"
#include "nvmedia_core.h"
#include "nvmedia_surface.h"

/**
 * \defgroup image_ofst_api Image OpticalFlow/StereoDisparity (OFST) Estimator
 *
 * The NvMediaIOFST object takes an uncompressed image frame pair and turns them
 * into opticalflow/stereodisparity estimation data.
 *
 * @ingroup nvmedia_image_top
 * @{
 */

/** \brief Major version number. */
#define NVMEDIA_IOFST_VERSION_MAJOR   1
/** \brief Minor version number. */
#define NVMEDIA_IOFST_VERSION_MINOR   14

/**
 * \brief Defines the image estimation type.
 */
typedef enum {
    /** High Performance OpticalFlow.
     *  \n Relative comparison with respect to other Optical Flow modes:
     *  - Performance: Moderate
     *  - Estimation Precision: Moderate */
    NVMEDIA_IMAGE_OPTICALFLOW_ESTIMATION_HP_MODE,
    /** High Performance StereoDisparity.
     *  \n Relative comparison with respect to other Stereo Disparity modes:
     *  - Performance: Moderate
     *  - Estimation Precision: Moderate */
    NVMEDIA_IMAGE_STEREODISPARITY_ESTIMATION_HP_MODE,
    /** High Quality OpticalFlow.
     *  \n Relative comparison with respect to other Optical Flow modes:
     *  - Performance: Least
     *  - Estimation Precision: Best */
    NVMEDIA_IMAGE_OPTICALFLOW_ESTIMATION_HQ_MODE,
    /** High Quality StereoDisparity.
     *  \n Relative comparison with respect to other Stereo Disparity modes:
     *  - Performance: Least
     *  - Estimation Precision: Best */
    NVMEDIA_IMAGE_STEREODISPARITY_ESTIMATION_HQ_MODE,
    /** Ultra High Performance OpticalFlow.
     *  \n Relative comparison with respect to other Optical Flow modes:
     *  - Performance: Best
     *  - Estimation Precision: Least */
    NVMEDIA_IMAGE_OPTICALFLOW_ESTIMATION_UHP_MODE,
    /** Ultra High Performance StereoDisparity.
     *  \n Relative comparison with respect to other Stereo Disparity modes:
     *  - Performance: Best
     *  - Estimation Precision: Least */
    NVMEDIA_IMAGE_STEREODISPARITY_ESTIMATION_UHP_MODE
} NvMediaIOFSTType;

/**
 * Defines OFST estimation configuration features.
 */
typedef enum {
    /** Enables OFST profiling.
      * \n This is an internal feature. */
    NVMEDIA_OFST_CONFIG_ENABLE_PROFILING                = (1 << 0)
} NvMediaOFSTConfigFeatures;

/**
 * \brief Holds an OFST object created and returned by NvMediaIOFSTCreate().
 */
typedef struct {
    /** Motion estimation type. */
    NvMediaIOFSTType estimationType;
    /** Input surface format. */
    NvMediaSurfaceType inputFormat;
    /** Output surface format. */
    NvMediaSurfaceType outputFormat;
    /** Instance ID. */
    NvMediaEncoderInstanceId instanceId;
    /** An Opaque pointer for internal use. */
    struct NvMediaIOFSTPriv_ *ofstPriv;
} NvMediaIOFST;

/**
 * Holds OFST estimation initialization parameters.
 */
typedef struct {
    /** Input width
     *  \inputrange The values between 160 to 4096, in increments of 2.
     *  \n Width of the input \ref NvMediaImage that needs to be processed
     *  should be equal to the value which is passed here.
     */
    uint16_t width;
    /** Input height
     *  \inputrange The values between 64 to 4096, in increments of 2.
     *  \n Height of the input \ref NvMediaImage that needs to be processed
     *  should be equal to the value which is passed here.
     */
    uint16_t height;
    /** Specifies bitwise OR'ed configuration feature flags.
      * Refer \ref NvMediaOFSTConfigFeatures for the list of supported features
      * that can be bit-OR'd here. */
    uint32_t features;
} NvMediaOFSTInitializeParams;

/**
 * Holds OFST estimation parameters. This structure is used on a per-frame
 * basis.
 *
 * This is used to improve the OF/ST performance/quality. Hints are contained
 * in a proprietary format, typically generated using Vision Programming
 * Interface.
 */
typedef struct {
    /** Specifies a pointer to ME external hints for the current frame.
     *  \n This field is not supported on QNX Safety build.
     */
    void *meExternalHints;
    /** Enables the external segment ID map, as follows:
     *  - A value of @c NVMEDIA_TRUE enables it.
     *  - A value of @c NVMEDIA_FALSE disables it.
     *  \n This field should be enabled only when \a meExternalHints has
     *  segment map information also.
     *  \n This field is not supported on QNX Safety build.
     */
    NvMediaBool enableSegmentMap;
} NvMediaOFSTExternalHintParams;

/**
 * \brief Returns the version information for the NvMedia IOFST library.
 *
 * \pre  None
 * \post None
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Init
 *
 * \param[out] version   A pointer to a \ref NvMediaVersion structure
 *                       filled by the IOFST library.
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if the \a version pointer is NULL.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIOFSTGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Creates an \ref NvMediaIOFST object that can compute motion vectors or
 * a disparity map based on differences between two images.
 *
 * \pre NvMediaIOFSTGetVersion()
 * \pre NvMediaIOFSTNvSciSyncGetVersion() [for use with IOFST-NvSciSync APIs]
 * \post NvMediaIOFST object is created
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Init
 *
 * \param[in] device
 *      A pointer to the \ref NvMediaDevice this OFST is to use.
 *      \inputrange Non-NULL - valid address
 * \param[in] estimationType OF/ST estimation type.
 *      \inputrange One of the following:
 *          \n \ref NVMEDIA_IMAGE_OPTICALFLOW_ESTIMATION_HP_MODE
 *          \n \ref NVMEDIA_IMAGE_STEREODISPARITY_ESTIMATION_HP_MODE
 *          \n \ref NVMEDIA_IMAGE_OPTICALFLOW_ESTIMATION_HQ_MODE
 *          \n \ref NVMEDIA_IMAGE_STEREODISPARITY_ESTIMATION_HQ_MODE
 *          \n \ref NVMEDIA_IMAGE_OPTICALFLOW_ESTIMATION_UHP_MODE
 *          \n \ref NVMEDIA_IMAGE_STEREODISPARITY_ESTIMATION_UHP_MODE
 * \param[in] inputFormat
 *      Format of the input \ref NvMediaImage.
 *      \inputrange Input format returned by a call to
 *      NvMediaSurfaceFormatGetType(), with \a attr being set as follows:
 *      \n <b>QNX Safety build:</b>
 *          \n \ref NVM_SURF_FMT_SET_ATTR_YUV(attr, YUV, 420, SEMI_PLANAR, UINT,
 *                                            [8/16], BL)
 *      \n <b>Non-safety build:</b>
 *          \n %NVM_SURF_FMT_SET_ATTR_YUV(attr, LUMA, NONE, PACKED, UINT,
 *                                        [8/10/16], BL)
 *          \n %NVM_SURF_FMT_SET_ATTR_YUV(attr, YUV, 420, SEMI_PLANAR, UINT,
 *                                        [8/10/12/16], BL)
 *          \n %NVM_SURF_FMT_SET_ATTR_YUV(attr, YUV, 444, SEMI_PLANAR, UINT,
 *                                        [8/10/12/16], BL)
 *
 * \param[in] outputFormat
 *      Format of the output \ref NvMediaImage.
 *      \inputrange Output format obtained by a call to
 *      NvMediaSurfaceFormatGetType(), with \a attr being set as follows:
 *          \n NVM_SURF_FMT_SET_ATTR_RGBA(attr, RG, INT, 16, BL) for Optical
 *              Flow
 *          \n %NVM_SURF_FMT_SET_ATTR_RGBA(attr, ALPHA, INT, 16, BL) for Stereo
 *              Disparity
 *
 * \param[in] initParams
 *      A pointer to a structure that specifies initialization parameters.
 *      \inputrange Non-NULL - valid address.
 *          \n Ranges specific to each member in the structure can be found in
 *          \ref NvMediaOFSTInitializeParams.
 *
 * \param[in] maxInputBuffering
 *      Maximum number of NvMediaIOFSTProcessFrame() operations that can be
 *      queued by NvMediaIOFST. If more than @a maxInputBuffering operations
 *      are queued, %NvMediaIOFSTProcessFrame() returns an error to indicate
 *      insufficient buffering.
 *      \inputrange The values between 1 to 16, in increments of 1
 *
 * \param[in] instanceId
 *      ID of the encoder engine instance.
 *      \inputrange One of the following:
 *           \n \ref NVMEDIA_ENCODER_INSTANCE_0 [Supported only on Non-safety
 *                                               builds]
 *           \n \ref NVMEDIA_ENCODER_INSTANCE_1
 *           \n \ref NVMEDIA_ENCODER_INSTANCE_AUTO [Supported only on Non-safety
 *                                                  builds]
 *
 * \return Created NvMediaIOFST estimator handle if successful, or NULL
 *         otherwise.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaIOFST *
NvMediaIOFSTCreate(
    const NvMediaDevice *device,
    NvMediaIOFSTType estimationType,
    NvMediaSurfaceType inputFormat,
    NvMediaSurfaceType outputFormat,
    const NvMediaOFSTInitializeParams *initParams,
    uint8_t maxInputBuffering,
    NvMediaEncoderInstanceId instanceId
);

/**
 * \brief Destroys the created \ref NvMediaIOFST object and frees associated
 * resources.
 *
 * \pre NvMediaIOFSTImageUnRegister()
 * \pre NvMediaIOFSTUnregisterNvSciSyncObj() [for use with IOFST-NvSciSync APIs]
 * \post NvMediaIOFST object is destroyed
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: De-init
 *
 * \param[in] ofst The \ref NvMediaIOFST object to destroy, returned by
 * NvMediaIOFSTCreate().
 *      \inputrange Non-NULL - valid pointer address
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
void NvMediaIOFSTDestroy(const NvMediaIOFST *ofst);

/**
 * \brief Performs OF/ST estimation on a specified frame pair.
 *
 * Estimation is based on the difference between @a refFrame and @a frame.
 * The output of Optical Flow processing is motion vectors, and that of Stereo
 * Disparity processing is a disparity surface.
 *
 * \pre NvMediaIOFSTImageRegister()
 * \pre NvMediaIOFSTSetNvSciSyncObjforEOF() [for use with IOFST-NvSciSync APIs]
 * \pre NvMediaIOFSTInsertPreNvSciSyncFence() [optional - for use with IOFST-
 *                                             NvSciSync APIs]
 * \post Optical Flow/Stereo Disparity estimation task is submitted
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Runtime
 *
 * \param[in] ofst      A pointer to the \ref NvMediaIOFST estimator to use.
 *                      \inputrange Non-NULL - valid pointer address
 * \param[in] frame     <b>Optical Flow Processing:</b> A pointer to an input
 *                      \ref NvMediaImage at time T+1 (current frame).
 *                      \n <b>Stereo Disparity Processing:</b> A pointer to an
 *                      input NvMediaImage containing the rectified left view.
 *                      \n It must be the same sourceType as
 *                      @a ofst-&gt;inputformat.
 *                      \n \a frame is allocated through a call to
 *                      NvMediaImageCreateFromNvSciBuf().
 *                      \inputrange Non-NULL - valid pointer address.
 * \param[in] refFrame  <b>Optical Flow Processing:</b> A pointer to an input
 *                      \ref NvMediaImage at time T, which acts as a reference
 *                      to current frame.
 *                      \n <b>Stereo Disparity Processing:</b> A pointer to an
 *                      input NvMediaImage containing the rectified right view.
 *                      \n It must be the same sourceType as
 *                      @a ofst-&gt;inputFormat.
 *                      \n \a refFrame is allocated through a call to
 *                      NvMediaImageCreateFromNvSciBuf().
 *                      \inputrange Non-NULL - valid pointer address.
 * \param[in] mvs       A pointer to an \ref NvMediaImage where motion vector
 *                      output (for Optical Flow) and disparity surface (for
 *                      Stereo Disparity) will be populated as a result of OF/ST
 *                      processing. As the processing in not synchronous \a mvs
 *                      will not be populated as soon as this API returns.
 *                      Clients can optionally call NvMediaImageGetStatus()
 *                      API to know the status of processing once this API
 *                      returns.
 *                      \n \a mvs must be of the same sourceType as
 *                      @a ofst-&gt;outputFormat.
 *                      \n \a mvs is allocated through a
 *                      call to NvMediaImageCreateFromNvSciBuf().
 *                      \n Motion vector output is in S10.5 format, with a range
 *                      of [-1024, 1023] and quarter pixel accuracy. This means
 *                      that the MSB represents the sign of the flow vector,
 *                      next 10bits represent the integer value and the last
 *                      5 bits represent the fractional value. As we support
 *                      quarter pixel precision, out of the 5 bits that
 *                      represent the fractional component, only the most
 *                      significant 2 bits are in use. The last 3 bits of the
 *                      fractional part are zero all the time.
 *                      \n For Stereo Disparity, output corresponds
 *                      to left view and absolute value of output will provide
 *                      actual disparity.
 *                      \inputrange Non-NULL - valid pointer address.
 * \param[in] extHintParams
 *                      A pointer to a structure containing external hint
 *                      parameters corresponding to the input frame pair.
 *                      \n This feature is not supported on QNX Safety build,
 *                      the field should be set to NULL.
 *                      \inputrange
 *                      \n NULL to disable ME hints
 *                      \n Non-NULL - valid pointer address to enable ME hints
 * \param[in] instanceId The ID of the encoder engine instance.
 *                      \inputrange
 *                      \n In the case of QNX safety build, this needs to be set
 *                      to the same value of \a instanceId passed to
 *                      NvMediaIOFSTCreate().
 *                      \n In the case of non-safety builds, if
 *                      \a instanceId passed to %NvMediaIOFSTCreate() was \a not
 *                      \ref NVMEDIA_ENCODER_INSTANCE_AUTO, then the value
 *                      of to \a instanceId be passed here, should be the same
 *                      as that of the value passed to %NvMediaIOFSTCreate().
 *                      If NVMEDIA_ENCODER_INSTANCE_AUTO was passed to
 *                      %NvMediaIOFSTCreate(), the following instances are
 *                      supported:
 *                      \n \ref NVMEDIA_ENCODER_INSTANCE_0
 *                      \n \ref NVMEDIA_ENCODER_INSTANCE_1
 *                      \n If any other value is passed, an error is returned.
 * \return The completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the call is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if any of the input parameters are
 *        invalid.
 * - \ref NVMEDIA_STATUS_ERROR if there is an internal error in processing.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIOFSTProcessFrame(
    const NvMediaIOFST *ofst,
    const NvMediaImage *frame,
    const NvMediaImage *refFrame,
    const NvMediaImage *mvs,
    const NvMediaOFSTExternalHintParams *extHintParams,
    NvMediaEncoderInstanceId instanceId
);

/**
 * \brief Registers an \ref NvMediaImage for use with an \ref NvMediaIOFST
 * handle. The NvMediaIOFST handle maintains a record of all the images
 * registered using this API.
 *
 * This is a mandatory API in safety build as skipping it will result in
 * non-deterministic NvMediaIOFSTProcessFrame() execution time.
 *
 * For deterministic execution time of %NvMediaIOFSTProcessFrame() API :
 * - NvMediaIOFSTImageRegister() must be called for every input and output
 *   NvMediaImage that will be used with NvMediaIOFST.
 * - All %NvMediaIOFSTImageRegister() calls must be made before first
 *   %NvMediaIOFSTProcessFrame() API call.
 *
 * Maximum of 32 NvMediaImage handles can be registered per access mode.
 *
 * \pre NvMediaIOFSTCreate()
 * \post NvMediaImage is registered with NvMediaIOFST object
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Init
 *
 * \param[in] ofst         \ref NvMediaIOFST handle.
 *                         \inputrange Non-NULL - valid pointer address
 * \param[in] image        A pointer to NvMediaImage object.
 *                         \inputrange Non-NULL - valid pointer address
 * \param[in] accessMode   \ref NvMediaAccessMode required for the image.
 *                         \inputrange One of the following based on the type of
 *                         NvMediaImage being registered:
 *                         - \ref NVMEDIA_ACCESS_MODE_READ       [Input image]
 *                         - \ref NVMEDIA_ACCESS_MODE_READ_WRITE [Output image]

 * \return \ref NvMediaStatus, the completion status of operation:
 * - \ref NVMEDIA_STATUS_OK if successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if ofst, image or accessMode is invalid.
 * - \ref NVMEDIA_STATUS_ERROR in following cases:
 *          - User registers more than 32 images.
 *          - User registers same image with more than one accessModes.
 *          - User registers same image multiple times.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED if API is not supported in a specific
 * platform configuration.
 *
 **/
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIOFSTImageRegister(
    const NvMediaIOFST *ofst,
    const NvMediaImage *image,
    NvMediaAccessMode  accessMode
);

/**
 * \brief Un-registers \ref NvMediaImage which was previously registered with
 * \ref NvMediaIOFST using NvMediaIOFSTImageRegister().
 *
 * For all NvMediaImage handles registered with NvMediaIOFST using
 * %NvMediaIOFSTImageRegister() API, %NvMediaIOFSTImageUnRegister() must be
 * called before calling NvMediaIOFSTDestroy() API. For unregistration to
 * succeed, it should be ensured that none of the submitted tasks on the image
 * are pending prior to calling %NvMediaIOFSTImageUnRegister() API.
 * In order to ensure this, %NvMediaImageGetStatus() API needs to be called
 * on all output NvMediaImage, to wait for their task completion. Post this
 * %NvMediaIOFSTImageUnRegister() can be successfully called on valid
 * NvMediaImage.
 *
 * For deterministic execution of %NvMediaIOFSTProcessFrame() API,
 * %NvMediaIOFSTImageUnRegister() must be called only after last
 * %NvMediaIOFSTProcessFrame() call.
 *
 * \pre NvMediaIOFSTProcessFrame()
 * \pre NvMediaImageGetStatus() [verify that processing is complete]
 * \post NvMediaImage is un-registered from NvMediaIOFST object
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: De-init
 *
 * \param[in] ofst         NvMediaIOFST handle.
 *                         \inputrange Non-NULL - valid pointer address
 * \param[in] image        A pointer to NvMediaImage to unregister.
 *                         \inputrange Non-NULL - valid pointer address
 * \return \ref NvMediaStatus, the completion status of operation:
 * - \ref NVMEDIA_STATUS_OK if successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if ofst or image is invalid
 *   %NvMediaIOFSTImageRegister() API.
 * - \ref NVMEDIA_STATUS_PENDING if the enqueued operations on the
 *   \ref NvMediaImage are still pending. In this case, the application can
 *   choose to wait for operations to complete on the output surface using
 *   %NvMediaImageGetStatus() or re-try the %NvMediaIOFSTImageUnRegister()
 *   API call, untill the status returned is not \ref NVMEDIA_STATUS_PENDING.
 * - \ref NVMEDIA_STATUS_ERROR in following cases:
 *          - User unregisters an NvMediaImage which is not previously
 *            registered using %NvMediaIOFSTImageRegister() API.
 *          - User unregisters an NvMediaImage multiple times.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED if API is not supported in a specific
 * platform configuration.
 *
 **/
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIOFSTImageUnRegister(
    const NvMediaIOFST *ofst,
    const NvMediaImage *image
);

/*
 * \defgroup history_nvmedia_iofst History
 * Provides change history for the NvMedia IOFST API.
 *
 * \section history_nvmedia_iofst Version History
 *
 * <b> Version 1.0 </b> April 22, 2017
 * - Initial release
 *
 * <b> Version 1.1 </b> June 08, 2017
 * - Added enableSegmentMap parameter to \ref NvMediaOFSTInitializeParams
 * - Added enableSegmentMap parameter to \ref NvMediaOFSTExternalHintParams
 *
 * <b> Version 1.2 </b/> June 13, 2017
 * - Added instance ID parameter to NvMediaIOFSTCreate()
 * - Added instance ID parameter to NvMediaIOFSTProcessFrame()
 * - Added instanceId to \ref NvMediaIOFST object
 *
 * <b> Version 1.3 </b/> June 19, 2017
 * - Added NVMEDIA_IMAGE_OPTICALFLOW_ESTIMATION_HQ_MODE and
 *   NVMEDIA_IMAGE_OPTICALFLOW_ESTIMATION_UHP_MODE and
 *   NVMEDIA_IMAGE_STEREODISPARITY_ESTIMATION_UHP_MODE
 *   parameters \ref NvMediaIOFSTType enum
 *
 * <b> Version 1.4 </b/> June 19, 2018
 * - Added features parameter to \ref NvMediaOFSTInitializeParams
 *
 * <b> Version 1.5 </b/> Dec 14, 2018
 * - Fix MISRA violations 21.1 and 21.2
 *
 * <b> Version 1.6 </b> January 15, 2019
 * - Fix MISRA violations 8.13
 *
 * <b> Version 1.7 </b> Feb 7, 2019
 * - Added opaque handle for ofst
 *   internal usage into OFST object
 * - Fix MISRA violations 11.3
 * - Fix MISRA violations 8.13
 *
 * <b> Version 1.8 </b> February 28, 2019
 * - Added required header includes nvmedia_core.h and nvmedia_surface.h
 *
 * <b> Version 1.9 </b> March 14, 2019
 * - Removed ExternalHints related parameters from
 *   \ref NvMediaOFSTInitializeParams
 * - Updated meExternalHints pointer type in
 *   \ref NvMediaOFSTExternalHintParams
 *
 * <b> Version 1.10 </b> April 03, 2019
 * - Added NvMediaIOFSTGetVersion() API
 *
 * <b> Version 1.11 </b> July 10, 2019
 * - Header include nvmedia_common.h is replaced with nvmedia_common_encode_ofst.h
 *
 * <b> Version 1.12 </b> July 16, 2019
 * - Added NvMediaIOFSTImageRegister() API
 * - Added NvMediaIOFSTImageUnRegister() API
 *
 * <b> Version 1.13 </b> July 17, 2019
 * - Removed nvmedia_array dependency
 *
 * <b> Version 1.14 </b> October 5, 2019
 * - Fix MISRA violations 8.13
 * - Added Coverity Code annotations to suppress partial 8.7 violations
 */

/** @} */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_IOFST_H */
