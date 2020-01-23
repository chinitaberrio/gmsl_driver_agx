/*
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.  All
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
#include "nvmedia_common.h"
#include "nvmedia_core.h"
#include "nvmedia_surface.h"
#include "nvmedia_array.h"

/**
 * \defgroup image_ofst_api Image OpticalFlow/StereoDisparity (OFST) Estimator
 *
 * The NvMediaIOFST object takes an uncompressed image frame pair and turns them
 * into opticalflow/stereodisparity estimation data.
 *
 * @{
 */

/** \brief Major version number. */
#define NVMEDIA_IOFST_VERSION_MAJOR   1
/** \brief Minor version number. */
#define NVMEDIA_IOFST_VERSION_MINOR   10

/**
 * \brief Defines the image estimation type.
 */
typedef enum {
    /** High Performance OpticalFlow. */
    NVMEDIA_IMAGE_OPTICALFLOW_ESTIMATION_HP_MODE,
    /** High Performance StereoDisparity. */
    NVMEDIA_IMAGE_STEREODISPARITY_ESTIMATION_HP_MODE,
    /** High Quality OpticalFlow. */
    NVMEDIA_IMAGE_OPTICALFLOW_ESTIMATION_HQ_MODE,
    /** High Quality StereoDisparity. */
    NVMEDIA_IMAGE_STEREODISPARITY_ESTIMATION_HQ_MODE,
    /** Ultra High Performance OpticalFlow. */
    NVMEDIA_IMAGE_OPTICALFLOW_ESTIMATION_UHP_MODE,
    /** Ultra High Performance StereoDisparity. */
    NVMEDIA_IMAGE_STEREODISPARITY_ESTIMATION_UHP_MODE
} NvMediaIOFSTType;

/**
 * Defines OFST estimation configuration features.
 */
typedef enum {
    /** Enables OFST profiling. */
    NVMEDIA_OFST_CONFIG_ENABLE_PROFILING                = (1 << 0)
} NvMediaOFSTConfigFeatures;

/**
 * \brief Holds an OFST object created by \ref NvMediaIOFSTCreate.
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
    /** An Opaque pointer for internal use */
    struct NvMediaIOFSTPriv_ *ofstPriv;
} NvMediaIOFST;

/**
 * Holds OFST estimation initialization parameters.
 */
typedef struct {
    /** OFST width.*/
    uint16_t width;
    /** OFST height.*/
    uint16_t height;
    /** Specifies bitwise OR'ed configuration feature flags. See the enum
     NvMediaOFSTConfigFeatures. */
    uint32_t features;
} NvMediaOFSTInitializeParams;

/**
 * Holds OFST estimation parameters. This structure is used on a per-frame
 * basis.
 */
typedef struct {
    /** Specifies a pointer to ME external hints for the current frame. */
    NvMediaArray *meExternalHints;
    /** @c Enables the external segment ID map, as follows:
     \n A value of @c NVMEDIA_TRUE enables it.
     \n A value of @c NVMEDIA_FALSE disables it. */
    NvMediaBool enableSegmentMap;
} NvMediaOFSTExternalHintParams;

/**
 * \brief Returns the version information for the NvMedia IOFST library.
 * \param[out] version : A pointer to a \ref NvMediaVersion structure
 *                       filled by the IOFST library.
 * \return \ref NvMediaStatus, the completion status of the operation:
 *  \ref NVMEDIA_STATUS_OK if successful, or
 *  \ref NVMEDIA_STATUS_BAD_PARAMETER if the pointer is invalid.
 */
NvMediaStatus
NvMediaIOFSTGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Creates an OFST object that can create motion vectors based on the
 *        difference between two frames.
 *
 * Feeds surfaces to the OFST estimator with \ref NvMediaIOFSTProcessFrame()
 * and retrieves estimated data with \ref NvMediaIOFSTProcessFrame()
 * motion vector surface.
 *
 * \param[in] device
 *      A pointer to the \ref NvMediaDevice this OFST is to use.
 * \param[in] estimationType Estimation type. Supported types are:
 *      \n \ref NVMEDIA_IMAGE_OPTICALFLOW_ESTIMATION_HP_MODE
 *      \n \ref NVMEDIA_IMAGE_STEREODISPARITY_ESTIMATION_HP_MODE
 *      \n \ref NVMEDIA_IMAGE_OPTICALFLOW_ESTIMATION_HQ_MODE
 *      \n \ref NVMEDIA_IMAGE_STEREODISPARITY_ESTIMATION_HQ_MODE
 *      \n \ref NVMEDIA_IMAGE_OPTICALFLOW_ESTIMATION_UHP_MODE
 *      \n \ref NVMEDIA_IMAGE_STEREODISPARITY_ESTIMATION_UHP_MODE
 * \param[in] inputFormat
 *      Input format. Must be obtained by a call to
 *      NvMediaSurfaceFormatGetType() with:
 *      \n \ref NVM_SURF_FMT_SET_ATTR_YUV(attr, LUMA, NONE, PACKED, UINT, [8/10/16],
 *                                   BL)
 *      \n \ref NVM_SURF_FMT_SET_ATTR_YUV(attr, YUV, 420, SEMI_PLANAR, UINT,
 *                                   [8/10/12/16], BL)
 *      \n \ref NVM_SURF_FMT_SET_ATTR_YUV(attr, YUV, 444, SEMI_PLANAR, UINT,
 *                                   [8/10/12/16], BL)
 * \param[in] outputFormat
 *      Output format. Obtained by a call to NvMediaSurfaceFormatGetType() with:
 *      \n NVM_SURF_FMT_SET_ATTR_RGBA(attr, RG, INT, 16, BL)
 *      \n NVM_SURF_FMT_SET_ATTR_RGBA(attr, ALPHA, INT, 16, BL)
 * \param[in] initParams
 *      A pointer to a structure that specifies initialization parameters.
 * \param[in] maxInputBuffering
 *      Number of NvMediaIOFSTProcessFrame() operations that can be queued
 *      by the \ref NvMediaIOFST. If more than @a maxInputBuffering operations
 *      are queued, NvMediaIOFSTProcessFrame() blocks until the excess
 *      operations are dequeued.
 * \param[in] instanceId
 *      ID of the encoder engine instance. Supported instances are:
 *      \n \ref NVMEDIA_ENCODER_INSTANCE_0
 *      \n \ref NVMEDIA_ENCODER_INSTANCE_1
 *      \n \ref NVMEDIA_ENCODER_INSTANCE_AUTO
 * \return The new Image OFST Estimator handle if successful, or NULL otherwise.
 */
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
 * \brief Destroys an NvMediaIOFST object.
 * \param[in] ofst    The OFST object to destroy.
 */
void NvMediaIOFSTDestroy(const NvMediaIOFST *ofst);

/**
 * \brief Performs OFST estimation on a specified frame pair.
 *
 * Estimation is based on the difference between @a refFrame and @a frame.
 * The OFST produces motion vectors.
 *
 * \param[in] ofst      A pointer to the OFSTestimator to use.
 * \param[in] frame     A pointer to current frame YUV data.
 *                      It must be the same sourceType as
 *                      @a ofst-&gt;inputformat.
 * \param[in] refFrame  A pointer to reference frame YUV
 *                      data. It must be the same sourceType as
 *                      @a ofst-&gt;inputformat.
 * \param[in] mvs       A pointer to a frame which contains motion vector
 *                      output. It must be of the same sourceType as
 *                      @a ofst-&gt;outputformat.
 * \param[in] extHintParams
 *                      A pointer to a structure containing external hint
 *                      parameters. It is used for ME hinting. Set it to NULL
 *                      if ME hinting is not needed.
 * \param[in] instanceId The ID of the encoder engine instance. If
 *                 NVMEDIA_ENCODER_INSTANCE_AUTO is used the call to
 *                 NvMediaVideoEncoderCreate(), the following instances are
 *                 supported:
 *                 - \ref NVMEDIA_ENCODER_INSTANCE_0
 *                 - \ref NVMEDIA_ENCODER_INSTANCE_1
 *                 Otherwise this parameter is ignored.
 * \return The completion status of the operation: \ref NVMEDIA_STATUS_OK
 *  if the call is successful, or \ref NVMEDIA_STATUS_BAD_PARAMETER if any of
 *  the pointer input parameters is NULL.
 */
NvMediaStatus
NvMediaIOFSTProcessFrame(
    const NvMediaIOFST *ofst,
    NvMediaImage *frame,
    NvMediaImage *refFrame,
    NvMediaImage *mvs,
    const NvMediaOFSTExternalHintParams *extHintParams,
    NvMediaEncoderInstanceId instanceId
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
 * - Added instance ID parameter to \ref NvMediaIOFSTCreate()
 * - Added instance ID parameter to \ref NvMediaIOFSTProcessFrame()
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
 * - Added \ref NvMediaIOFSTGetVersion API
 */

/** @} */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_IOFST_H */
