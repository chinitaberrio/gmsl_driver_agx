/*
 * Copyright (c) 2017-2019 NVIDIA CORPORATION. All rights reserved. All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */


/**
 * \file
 * \brief <b> NVIDIA Media Interface: Common Types for Video/Image Encode/Decode</b>
 *
 * @b Description: This file contains common types and definitions for image and video
 * decode and encode operations.
 */

#ifndef NVMEDIA_COMMON_H
#define NVMEDIA_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"

/**
 * \defgroup common_types_top Video and Image Encode/Decode: Common Types
 * \ingroup nvmedia_common_top
 *
 * \brief Defines common types and declarations for image and video
 * decode and encode operations.
 * @{
 */

#include "nvmedia_common_decode.h"
#include "nvmedia_common_encode.h"

/** \brief Major Version number */
#define NVMEDIA_COMMON_VERSION_MAJOR   1
/** \brief Minor Version number */
#define NVMEDIA_COMMON_VERSION_MINOR   29


/** \brief Defines filter quality levels.
 */
typedef enum {
    /** \hideinitializer \brief Low (default) filter quality. */
    NVMEDIA_FILTER_QUALITY_LOW = 0,
    /** \hideinitializer \brief Medium filter quality. */
    NVMEDIA_FILTER_QUALITY_MEDIUM,
    /** \hideinitializer \brief High filter quality. */
    NVMEDIA_FILTER_QUALITY_HIGH
} NvMediaFilterQuality;

/**
 * \defgroup display_attributes Display Attributes
 * \ingroup common_types_top
 * Defines display attribute bit masks for constructing attribute masks.
 * @{
 */

/**
 * \hideinitializer
 * \brief Sets the layer composition mode.
 */
#define NVMEDIA_DISP_ATTR_COMPOSITION     (1<<6)
/**
 * \hideinitializer
 * \brief Sets video output depth
 */
#define NVMEDIA_DISP_ATTR_SET_DEPTH       (1<<7)
/**
 * \hideinitializer
 * \brief Sets video output position
 */
#define NVMEDIA_DISP_ATTR_SET_POSITION    (1<<8)

/**@} <!-- Ends display_attributes sub-group --> */

/**
 * \brief Specifies composition modes for layers.
 */
typedef enum {
    /*! A lower depth overlay is fully covering any higher depth overlay.*/
    NVMEDIA_DISP_COMP_MODE_OPAQUE,
    /*! A lower depth overlay is per-pixel alpha blended over any higher depth overlay.
        The blending mode is Source Alpha. */
    NVMEDIA_DISP_COMP_MODE_SRC_ALPHA,
    /*! A lower depth overlay is per-pixel alpha blended over any higher depth overlay.
        The blending mode is Pre-multiplied Source Alpha. */
    NVMEDIA_DISP_COMP_MODE_PREMULTI_SRC_ALPHA
} NvMediaDispCompMode;

/**
 * \brief Holds NvMedia display attributes.
 */
typedef struct {
    /** \brief Defines the composition mode for the layer.
     * \n \ref NVMEDIA_DISP_COMP_MODE_OPAQUE
     * \n \ref NVMEDIA_DISP_COMP_MODE_SRC_ALPHA
     * \n \ref NVMEDIA_DISP_COMP_MODE_PREMULTI_SRC_ALPHA
     * \ref NVMEDIA_DISP_ATTR_COMPOSITION.
     */
    NvMediaDispCompMode compositionMode;
    /** \brief Holds a positive value (up to 255) that specifies the video displays
     *  relative to the top-most layer @code(depth == 0)@endcode.
     * \ref NVMEDIA_DISP_ATTR_SET_DEPTH.
     */
    uint32_t depth;
    /** \brief The rectangle where the video renders.
     * \ref NVMEDIA_DISP_ATTR_SET_POSITION.
     */
    NvMediaRect position;
} NvMediaDispAttributes;

/** \brief Noise Reduction Algorithm
 */
typedef enum {
/** \hideinitializer \brief Original (default) noise reduction algorithm. */
    NVMEDIA_NOISE_REDUCTION_ALGORITHM_ORIGINAL = 0,
/** \hideinitializer \brief Noise reduction algorithm for outdoor low light condition. */
    NVMEDIA_NOISE_REDUCTION_ALGORITHM_OUTDOOR_LOW_LIGHT,
/** \hideinitializer \brief Noise reduction algorithm for outdoor medium light condition. */
    NVMEDIA_NOISE_REDUCTION_ALGORITHM_OUTDOOR_MEDIUM_LIGHT,
/** \hideinitializer \brief Noise reduction algorithm for outdoor high light condition. */
    NVMEDIA_NOISE_REDUCTION_ALGORITHM_OUTDOOR_HIGH_LIGHT,
/** \hideinitializer \brief Noise reduction algorithm for indoor low light condition. */
    NVMEDIA_NOISE_REDUCTION_ALGORITHM_INDOOR_LOW_LIGHT,
/** \hideinitializer \brief Noise reduction algorithm for indoor medium light condition. */
    NVMEDIA_NOISE_REDUCTION_ALGORITHM_INDOOR_MEDIUM_LIGHT,
/** \hideinitializer \brief Noise reduction algorithm for indoor high light condition. */
    NVMEDIA_NOISE_REDUCTION_ALGORITHM_INDOOR_HIGH_LIGHT
} NvMediaNoiseReductionAlgorithm;

/** @} <!-- Ends common_types_top group NvMedia Types and Structures common to Video & Image --> */

/*
 * \defgroup history_nvmedia_common History
 * Provides change history for the NvMedia Common Types.
 *
 * \section history_nvmedia_common Version History
 *
 * <b> Version 1.0 </b> March 21, 2017
 * - Initial release
 *
 * <b> Version 1.1 </b> April 18, 2017
 * - Added \ref NvMediaDecoderInstanceId enum and
 *   \ref NvMediaEncoderInstanceId enum
 * - Changed to use standard data types
 * - Removed input surface rotation and mirroring enumerations and members from
 *   \ref NvMediaEncodeInitializeParamsH264 and \ref NvMediaEncodeInitializeParamsH265
 *
 * <b> Version 1.2 </b> May 4, 2017
 * - Added SLICE_LEVEL_OUTPUT, RTP_MODE_OUTPUT, EXT_PIC_RC_HINT, DYNAMIC_RPS,
 *   MV_BUFFER_DUMP and RECON_CRC features for \ref NvMediaEncodeH264Features
 *   and \ref NvMediaEncodeH265Features
 * - Add bConstFrameQP parameter in \ref NvMediaEncodeRCParams.
 * - Add NvMediaEncAttrType enum and NvMediaNalData data structure.
 *
 * <b> Version 1.3 </b> May 8, 2017
 * - Added \ref numMacroblocksPerSlice in \ref NvMediaEncodeConfigH264
 *   and \ref numCTUsPerSlice in \ref NvMediaEncodeConfigH265.
 *
 * <b> Version 1.4 </b> May 10, 2017
 * - Added LOOP_FILTER_PARAMS, QUANTIZATION_PARAMS, TRANSFORM_MODE, HIGH_PRECISION_MV
 *   and ERROR_RESILIENT features in \ref NvMediaEncodeVP9Features.
 * - Added filter_type, filter_level, sharpness_level, ref_lf_deltas, mode_lf_deltas,
 *   bmode_ref_lf_delta_enabled, bmode_ref_lf_delta_update, base_qindex, delta_y_dc_q,
 *   delta_uv_dc, delta_uv_ac, transform_mode, high_prec_mv and error_resilient
 *   parameters in \ref NvMediaEncodeConfigVP9.
 *
 * <b> Version 1.5 </b> May 17, 2017
 * - Added display_attributes enum defines
 * - Added \ref NvMediaDispCompMode and \ref NvMediaDispAttributes
 *
 * <b> Version 1.6 </b> May 18, 2017
 * - Added NvMediaTransform enum defines
 *
 * <b> Version 1.7 </b> June 05, 2017
 * - Add \ref bSliceDecEnable and \ref NvMediaSliceDecodeData structure member in
 *   \ref NvMediaPictureInfoH265 structure.
 *
 * <b> Version 1.8 </b> June 07, 2017
 * - Added \ref NvMediaEncodeH264POCType enum for H.264 pic_order_cnt_type.
 *   The support is added for pic_order_cnt_type 0 and 2. pic_order_cnt_type 1 is not
 *   not supported.
 * - Added variable pocType of type \ref NvMediaEncodeH264POCType in \ref NvMediaEncodeConfigH264.
 *
 * <b> Version 1.9 </b> June 09, 2017
 * -Added PROFILING feature in \ref NvMediaEncodeH264Features,
 *  \ref NvMediaEncodeH265Features and \ref NvMediaEncodeVP9Features.
 *
 * <b> Version 1.10 </b> June 09, 2017
 * - Removed RECON_CRC features from \ref NvMediaEncodeH264Features
 *   and \ref NvMediaEncodeH265Features and added it as enableReconCRC
 *   for \ref NvMediaEncodeInitializeParamsH264 and \ref
 *   NvMediaEncodeInitializeParamsH265.
 * - Added enableMVC, enableROIEncode and enableSliceEncode
 *   in \ref NvMediaEncodeInitializeParamsH264 and \ref
 *   NvMediaEncodeInitializeParamsH265.
 *
 * <b> Version 1.11 </b> June 22, 2017
 * -Added levelTier parameter in \ref NvMediaEncodeInitializeParamsH265
 *  Added initQP, maxQP parameters in \ref NvMediaEncodeConfigH264,
 *  \ref NvMediaEncodeConfigH265 and \ref NvMediaEncodeConfigVP9.
 *
 * <b> Version 1.12 </b> June 29, 2017
 * -Added feature in \ref NvMediaEncodeH264Features and \ref NvMediaEncodeH265Features,
 *  to enable 4 byte start code.
 *
 * <b> Version 1.13</b> June 30, 2017
 * -Added HEVC MV interface members in \ref NvMediaPictureInfoH265 structure
 *
 * <b> Version 1.14</b> July 26, 2017
 * -Added enableWeightedPrediction in \ref NvMediaEncodeConfigH264 structure *
 *
 * <b> Version 1.15</b> Sept 1, 2017
 * -Added NvMediaEncodeVP8Features, NvMediaEncodeConfigVP8, NvMediaEncodeInitializeParamsVP8
 *  and NvMediaEncodePicParamsVP8 structures *
 *
 * <b> Version 1.16 </b> Sept 13, 2017
 * -Added frameRateNum and frameRateDen parameters in \ref NvMediaEncodePicParamsVP8
 *
 * <b> Version 1.17 </b> Oct 26, 2017
 * -Added useBFramesAsRef parameter in \ref NvMediaEncodeInitializeParamsH264
 *  and \ref NvMediaEncodeInitializeParamsH265
 *
 * <b> Version 1.18 </b> Nov 6, 2017
 * - Added enableTwoPassRC and enableSourceHalfScaled for \ref NvMediaEncodeInitializeParamsH264 and
 *   \ref NvMediaEncodeInitializeParamsH265.
 *
 * <b> Version 1.19 </b> Nov 24, 2017
 * - Added mvNumViews for \ref NvMediaEncodeInitializeParamsH265 and viewId for
 *   \ref NvMediaEncodePicParamsH265. Added mvcNumViews for \ref NvMediaEncodeInitializeParamsH264
 *   and viewId for \ref NvMediaEncodePicParamsH264.
 *
 * <b> Version 1.20 </b> March 16, 2018
 * - Added \ref NvMediaNoiseReductionAlgorithm structure
 *
 * <b> Version 1.21 </b> October 4, 2018
 * - Added Ultrafast encoding support for H.264 for platforms >=T194 in
 *   \ref NvMediaEncodeH264Features
 *
 * <b> Version 1.22 </b> October 29, 2018
 * - Added enableExternalPictureRC in \ref NvMediaEncodeInitializeParamsH264
 *  and \ref NvMediaEncodeInitializeParamsH265
 *
 * <b> Version 1.23 </b> Dec 14, 2018
 * - Defined Macro constants as unsigned to fix MISRA issues
 * - Fix MISRA violations 21.1 and 21.2
 *
 * <b> Version 1.24 </b> Jan 02, 2019
 * - Added enableAllIFrames in \ref NvMediaEncodeInitializeParamsH264
 *  and \ref NvMediaEncodeInitializeParamsH265
 *
 * <b> Version 1.25 </b> Jan 21, 2019
 * - Moved \ref NvMediaTransform to nvmedia_2d.h
 *
 * <b> Version 1.26 </b> Feb 6, 2019
 * - Deprecated the following display attributes \ref NVMEDIA_DISP_ATTR_BRIGHTNESS,
 *   \ref NVMEDIA_DISP_ATTR_COLOR_STANDARD, \ref NVMEDIA_DISP_ATTR_SATURATION,
 *   \ref NVMEDIA_DISP_ATTR_HUE, \ref NVMEDIA_DISP_ATTR_CONTRAST, \ref NVMEDIA_DISP_ATTR_LIMITED_RGB
 * - Deprecated parameters brightness, hue, saturation, contrast, limitedRGB and
 *   colorStandard from \ref NvMediaDispAttributes
 *
 * <b> Version 1.27 </b> March 26, 2019
 * - Added \ref NvMediaContentLightLevelInfo structure
 *
 * <b> Version 1.28 </b> April 4, 2019
 * - Deprecate NVMEDIA_MIN_CAPTURE_FRAME_BUFFERS, NVMEDIA_MAX_CAPTURE_FRAME_BUFFERS
 *
 * <b> Version 1.29 </b> July 5, 2019
 * - Moved Encode structures to nvmedia_common_encode.h
 * - Moved Decode structures to nvmedia_common_decode.h
 */


#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_COMMON_H */
