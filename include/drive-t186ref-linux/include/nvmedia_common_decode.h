/*
 * Copyright (c) 2019-2020 NVIDIA CORPORATION. All rights reserved. All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */


/**
 * \file
 * \brief <b> NVIDIA Media Interface: Common Types for Video/Image Decode</b>
 *
 * @b Description: This file contains common types and definitions for image and video
 * decode operations.
 */

#ifndef NVMEDIA_COMMON_DECODE_H
#define NVMEDIA_COMMON_DECODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"
#include "nvmedia_common_encode_decode.h"

/**
 * \defgroup common_types_decode_top Video and Image Decode: Common Types
 * \ingroup nvmedia_common_top
 *
 * \brief Defines common types and declarations for image and video
 * decode operations.
 * @{
 */

/**
 * \defgroup decoder_api Video Decoder: Common Types
 * \ingroup nvmedia_video_top
 * Defines and manages objects that decode video.
 *
 * The NvMediaVideoDecoder object decodes compressed video data, writing
 * the results to a \ref NvMediaVideoSurface "NvMediaVideoSurface".
 *
 * A specific NvMedia implementation may support decoding multiple
 * types of compressed video data. However, NvMediaVideoDecoder objects
 * are able to decode a specific type of compressed video data.
 * This type must be specified during creation.
 *
 * @{
 */

/** \brief Major Version number */
#define NVMEDIA_COMMON_DECODE_VERSION_MAJOR   1
/** \brief Minor Version number */
#define NVMEDIA_COMMON_DECODE_VERSION_MINOR   1

/**
 * \brief Specifies the decoder instance ID
 */
typedef enum  {
    /** \brief Specifies the decoder instance ID 0 */
    NVMEDIA_DECODER_INSTANCE_0 = 0,
    /** \brief Specifies the decoder instance ID 1 */
    NVMEDIA_DECODER_INSTANCE_1,
   /** \brief Specifies that the decoder instance ID
    * can be set dynamically during decode
    */
    NVMEDIA_DECODER_INSTANCE_AUTO,
} NvMediaDecoderInstanceId;

/**
 * \brief NAL types found in a bitstream packet. This is used for TK1 H264 OTF cases.
 */
enum
{
    /** \brief slice header data present or not */
    NVMEDIA_SLH_PRESENT = 0x1,
    /** \brief sequence header data present or not */
    NVMEDIA_SPS_PRESENT = 0x2,
    /** \brief picture header data present or not */
    NVMEDIA_PPS_PRESENT = 0x4
};

/**
 * \brief A generic "picture information" pointer type.
 *
 * This type serves solely to document the expected usage of a
 * generic (void *) function parameter. In actual usage, the
 * application is expected to physically provide a pointer to an
 * instance of one of the "real" NvMediaPictureInfo* structures,
 * picking the type appropriate for the decoder object in
 * question.
 */
typedef void NvMediaPictureInfo;

/**
 * \brief A generic "reference surface" pointer type.
 *
 * This type serves solely to document the expected usage of a
 * generic (void *) function parameter. In actual usage, the
 * application is expected to physically provide a pointer to an
 * instance of NvMediaVideoSurface or NvMediaImage, depending on
 * the type of decoder API used.
 */

typedef void NvMediaRefSurface;

/**@} <!-- Ends decoder_api Video Decoder --> */

/**
 * \defgroup h264decoder_api H.264 Structures
 * \ingroup basic_api_top
 * Provides structures for defining the H.264 reference frame.
 * @{
 */

/**
 * \brief Information about an H.264 reference frame
 *
 * @note References to "copy of bitstream field" in the field descriptions may:
 * - Refer to data literally parsed from the bitstream or
 * - Be derived from
 * the bitstream using a mechanism described in the specification.
 */
typedef struct {
    /**
     * The surface that contains the reference image.
     * Set to NULL for unused entries.
     */
    NvMediaRefSurface   *surface;
    /** Is this a long term reference (else short term). */
    NvMediaBool         is_long_term;
    /**
     * Is the top field used as a reference.
     * Set to NVMEDIA_FALSE for unused entries.
     */
    NvMediaBool         top_is_reference;
    /**
     * Is the bottom field used as a reference.
     * Set to NVMEDIA_FALSE for unused entries.
     */
    NvMediaBool         bottom_is_reference;
    /** [0]: top, [1]: bottom */
    int32_t                 field_order_cnt[2];
    /**
     * Copy of the H.264 bitstream field:
     * frame_num from slice_header for short-term references,
     * LongTermPicNum from decoding algorithm for long-term references.
     */
    uint16_t     FrameIdx;

    /** Parser only: Non-existing reference frame flag
     * (corresponding PicIdx should be set to -1)
     */
    uint16_t     not_existing;
} NvMediaReferenceFrameH264;

/**  \brief Maximum user defined sei payload size */
#define MAX_USER_SEI_PAYLOAD    128U

/**
 * \brief H.264 SEI payload information
 *  Used by the parser only.
 *
 */
typedef struct {
    /** Indicate the type of packing arrangement of the frames,
        as described in Annex D */
    uint8_t frame_packing_arrangement_type;
    /** Indicate whether each colour component plane of each consituent
        frame is quincunx sampled or not, as described in Annex D */
    uint8_t quincunx_sampling_flag;
    /** Indicates the intended interpretation of the constituent frames,
        as described in Annex D */
    uint8_t content_interpretation_flag;
    /** Indicates that whether one of the two constituent frames is
        spatially flipped relative to its intended orientation for
        display, as described in Annex D */
    uint8_t spatial_flipping_flag;
    /** Indicates which one of the two constituent frames is flipped,
        as described in Annex D */
    uint8_t frame0_flipped_flag;
    /** Indicates whether all pictures in the current coded video sequence
        are coded as complementary field pairs, as described in Annex D */
    uint8_t field_views_flag;
    /** Indicate whether current frame is frame 0, as described in Annex D */
    uint8_t current_frame_is_frame0_flag;
    /** Flag whether stereo is enabled or not */
    NvMediaBool bStereoEnabled;
    /** Specify the length of user data unregistered SEI message,
        as described in Annex D */
    uint32_t uUserSeiPayloadLength;
    /** Holds user data unregistered SEI message, as described in Annex D */
    uint8_t UserDefinedSeiPayload[MAX_USER_SEI_PAYLOAD];
} NvMediaSEIPayloadH264;

/**
 * \brief Picture parameter information for an H.264 picture.
 *
 * @note The \ref referenceFrames array must contain the "DPB" as
 * defined by the H.264 specification. In particular, once a
 * reference frame has been decoded to a surface, that surface must
 * continue to appear in the DPB until no longer required to predict
 * any future frame. Once a surface is removed from the DPB, it can
 * no longer be used as a reference, unless decoded again.
 *
 * Also note that only surfaces previously generated using \ref
 * NvMediaVideoDecoderRenderEx may be used as reference frames.
 *
 * @note References to "copy of bitstream field" in the field descriptions
 * may refer to data literally parsed from the bitstream, or derived from
 * the bitstream using a mechanism described in the specification.
 */
typedef struct {
    /** [0]: top, [1]: bottom */
    int32_t            field_order_cnt[2];
    /** Will the decoded frame be used as a reference later. */
    NvMediaBool    is_reference;

    /** Copy of the H.264 bitstream field. */
    uint16_t chroma_format_idc;
    /** Copy of the H.264 bitstream field. */
    uint16_t frame_num;
    /** Copy of the H.264 bitstream field. */
    uint8_t  field_pic_flag;
    /** Copy of the H.264 bitstream field. */
    uint8_t  bottom_field_flag;
    /** Copy of the H.264 bitstream field. */
    uint8_t  num_ref_frames;
    /** Copy of the H.264 bitstream field. */
    uint8_t  mb_adaptive_frame_field_flag;
    /** Copy of the H.264 bitstream field. */
    uint8_t  constrained_intra_pred_flag;
    /** Copy of the H.264 bitstream field. */
    uint8_t  weighted_pred_flag;
    /** Copy of the H.264 bitstream field. */
    uint8_t  weighted_bipred_idc;
    /** Copy of the H.264 bitstream field. */
    uint8_t  frame_mbs_only_flag;
    /** Copy of the H.264 bitstream field. */
    uint8_t  transform_8x8_mode_flag;
    /** Copy of the H.264 bitstream field. */
    int8_t           chroma_qp_index_offset;
    /** Copy of the H.264 bitstream field. */
    int8_t           second_chroma_qp_index_offset;
    /** Copy of the H.264 bitstream field. */
    int8_t           pic_init_qp_minus26;
    /** Copy of the H.264 bitstream field. */
    uint8_t  num_ref_idx_l0_active_minus1;
    /** Copy of the H.264 bitstream field. */
    uint8_t  num_ref_idx_l1_active_minus1;
    /** Copy of the H.264 bitstream field. */
    uint8_t  log2_max_frame_num_minus4;
    /** Copy of the H.264 bitstream field. */
    uint8_t  pic_order_cnt_type;
    /** Copy of the H.264 bitstream field. */
    uint8_t  log2_max_pic_order_cnt_lsb_minus4;
    /** Copy of the H.264 bitstream field. */
    uint8_t  delta_pic_order_always_zero_flag;
    /** Copy of the H.264 bitstream field. */
    uint8_t  direct_8x8_inference_flag;
    /** Copy of the H.264 bitstream field. */
    uint8_t  entropy_coding_mode_flag;
    /** Copy of the H.264 bitstream field. */
    uint8_t  pic_order_present_flag;
    /** Copy of the H.264 bitstream field. */
    uint8_t  deblocking_filter_control_present_flag;
    /** Copy of the H.264 bitstream field. */
    uint8_t  redundant_pic_cnt_present_flag;
    /** Copy of the H.264 bitstream field. */
    uint8_t  num_slice_groups_minus1;
    /** Copy of the H.264 bitstream field. */
    uint8_t  slice_group_map_type;
    /** Copy of the H.264 bitstream field. */
    uint32_t   slice_group_change_rate_minus1;
    /** Slice group map */
    uint8_t *slice_group_map;
    /** Copy of the H.264 bitstream field. */
    uint8_t fmo_aso_enable;
    /** Copy of the H.264 bitstream field. */
    uint8_t scaling_matrix_present;

    /** Copy of the H.264 bitstream field, converted to raster order. */
    uint8_t  scaling_lists_4x4[6][16];
    /** Copy of the H.264 bitstream field, converted to raster order. */
    uint8_t  scaling_lists_8x8[2][64];

    /** See \ref NvMediaPictureInfoH264 for instructions regarding this field. */
    NvMediaReferenceFrameH264 referenceFrames[16];
    /** Number of slices in this picture. \a nNumSlices entries contain the offset
     of each slice within the bitstream data buffer. Required for nvdec. */
    uint32_t nNumSlices;
    /** Passing NULL for \a pSliceDataOffsets disables error-concealment. */
    uint32_t *pSliceDataOffsets;
    /** 0:FrameType_B  1:FrameType_P  2:FrameType_I */
    uint8_t frameType;

    /** MVC extension */
    struct {
        /** Copy of the H.264 mvc bitstream field. */
        uint16_t num_views_minus1;
        /** Copy of the H.264 mvc bitstream field. */
        uint16_t view_id;
        /** Copy of the H.264 mvc bitstream field. */
        uint8_t inter_view_flag;
        /** Copy of the H.264 mvc bitstream field. */
        uint8_t num_inter_view_refs_l0;
        /** Copy of the H.264 mvc bitstream field. */
        uint8_t num_inter_view_refs_l1;
        /** Copy of the H.264 mvc bitstream field. */
        uint8_t MVCReserved8Bits;
        /** Copy of the H.264 mvc bitstream field. */
        uint16_t InterViewRefsL0[16];
        /** Copy of the H.264 mvc bitstream field. */
        uint16_t InterViewRefsL1[16];
    } mvcext;

    /** Parser only: SEI payload info */
    NvMediaSEIPayloadH264 seiPayloadInfo;
    /** Copy of the H.264 bitstream field. Required for OTF */
    uint32_t pic_width_in_mbs_minus1;
    /** Copy of the H.264 bitstream field. Required for OTF */
    uint32_t pic_height_in_map_units_minus1;
    /** Copy of the H.264 bitstream field. Required for OTF */
    int32_t last_sps_id;
    /** Copy of the H.264 bitstream field. Required for OTF */
    int32_t last_pps_id;
    /** Copy of the H.264 bitstream field, qpprime_y_zero_transform_bypass_flag */
    int32_t qpprime_y_zero_transform_bypass_flag;
} NvMediaPictureInfoH264;

/** @} <!-- Ends h264decoder_api Basic Types sub-group --> */

/**
 * \defgroup h265decoder_api H.265 Structures
 * \ingroup basic_api_top
 * Provides structures for defining the H.265 reference frame.
 * @{
 */

/**
 * \brief Mastering display data for an H.265 picture.
 *  Used by the parser only.
 *
 *  Array indexing 0 : G, 1 : B, 2 : R
 */
typedef struct
{
    /** Normalized x chromaticity cordinate. It shall be in the range of 0 to 50000 */
    uint16_t display_primaries_x[3];
    /** Normalized y chromaticity cordinate. It shall be in the range of 0 to 50000 */
    uint16_t display_primaries_y[3];
    /** Normalized x chromaticity cordinate of white point of mastering display */
    uint16_t white_point_x;
    /** Normalized y chromaticity cordinate of white point of mastering display */
    uint16_t white_point_y;
    /** Nominal maximum display luminance in units of candelas per square metre */
    uint16_t max_display_parameter_luminance;
    /** Nominal minimum display luminance in units of 0.0001 candelas per square metre */
    uint16_t min_display_parameter_luminance;
} NvMediaMasteringDisplayData;

/**
 * \brief Content Light Level info for an H.265 picture.
 *  Used by the parser only.
 *
 * Optional parameter
 */
typedef struct
{
    /** Maximum content light level in units of candelas per square metre */
    uint16_t max_content_light_level;
    /** Maximum frame average light level in units of candelas per square metre */
    uint16_t max_pic_average_light_level;
} NvMediaContentLightLevelInfo;



/** \brief slice level data used with slice level decoding
 *
 * @note This slice level information is passed with \ref NvMediaParserClientCb::SliceDecode
 * callback. Client will call NvMediaVideoDecoderSliceDecode() to
 * program hardware for decoding current slice. This feature is available
 * only for specific hardware/codecs.
 */
typedef struct
{
    /** Bitstream data*/
    /** Number of bytes in bitstream data buffer */
    uint32_t uBitstreamDataLen;
    /** Ptr to bitstream data for this picture (slice-layer) */
    uint8_t *pBitstreamData;
    /** Number of slices in this SliceData */
    uint32_t uNumSlices;
    /** If not NULL, nNumSlices entries, contains offset of each slice within the bitstream data buffer */
    uint32_t *pSliceDataOffsets;
    /** Number of CTB present in this CTB */
    uint32_t uCTBCount;
    /** CTB number of first CTB in the slice data */
    uint32_t uFirstCtbAddr;
    /** First slice flag: whether this SliceData contains first slice of frame */
    NvMediaBool bFirstSlice;
    /** Last slice flag: whether this SliceData contains last slice of frame */
    NvMediaBool bLastSlice;
    /** Error flag if some parsing error detected */
    NvMediaBool bErrorFlag;
} NvMediaSliceDecodeData;

/**
 * \brief Holds picture parameter information for an H.265 picture.
 *
 * \note The NvMediaPictureInfoH264.referenceFrames array must contain the "DPB" as
 * defined by the H.265 specification. Once a
 * reference frame has been decoded to a surface, that surface must
 * continue to appear in the DPB until it is no longer required to predict
 * any future frame. Once a surface is removed from the DPB, it is
 * no longer used as a reference unless decoded again.
 * Only surfaces previously generated using \ref
 * NvMediaVideoDecoderRenderEx may be used as reference frames.
 * References to "copy of bitstream field" in the field descriptions
 * may refer to data literally parsed from the bitstream, or derived from
 * the bitstream using a mechanism described in the specification.
 */
typedef struct {
    // sps
    /** Holds a copy of the H.265 bitstream field. */
    uint32_t pic_width_in_luma_samples;
    /** Holds a copy of the H.265 bitstream field. */
    uint32_t pic_height_in_luma_samples;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t log2_min_luma_coding_block_size_minus3;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t log2_diff_max_min_luma_coding_block_size;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t log2_min_transform_block_size_minus2;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t log2_diff_max_min_transform_block_size;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t pcm_enabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t log2_min_pcm_luma_coding_block_size_minus3;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t log2_diff_max_min_pcm_luma_coding_block_size;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t bit_depth_luma;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t bit_depth_chroma;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t pcm_sample_bit_depth_luma_minus1;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t pcm_sample_bit_depth_chroma_minus1;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t pcm_loop_filter_disabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t strong_intra_smoothing_enabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t max_transform_hierarchy_depth_intra;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t max_transform_hierarchy_depth_inter;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t amp_enabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t separate_colour_plane_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t log2_max_pic_order_cnt_lsb_minus4;
    /** Holds a copy of the H.265 bitstream field. */

    /** Holds a copy of the H.265 bitstream field. */
    uint8_t num_short_term_ref_pic_sets;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t long_term_ref_pics_present_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t num_long_term_ref_pics_sps;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t sps_temporal_mvp_enabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t sample_adaptive_offset_enabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t scaling_list_enable_flag;
    /** Holds a copy of the chroma_format_idc. */
    uint8_t chroma_format_idc;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t reserved1[3];

    // pps
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t dependent_slice_segments_enabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t slice_segment_header_extension_present_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t sign_data_hiding_enabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t cu_qp_delta_enabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t diff_cu_qp_delta_depth;
    /** Holds a copy of the H.265 bitstream field. */
    int8_t init_qp_minus26;
    /** Holds a copy of the H.265 bitstream field. */
    int8_t pps_cb_qp_offset;
    /** Holds a copy of the H.265 bitstream field. */
    int8_t pps_cr_qp_offset;

    /** Holds a copy of the H.265 bitstream field. */
    uint8_t constrained_intra_pred_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t weighted_pred_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t weighted_bipred_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t transform_skip_enabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t transquant_bypass_enabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t entropy_coding_sync_enabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t log2_parallel_merge_level_minus2;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t num_extra_slice_header_bits;

    /** Holds a copy of the H.265 bitstream field. */
    uint8_t loop_filter_across_tiles_enabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t loop_filter_across_slices_enabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t output_flag_present_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t num_ref_idx_l0_default_active_minus1;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t num_ref_idx_l1_default_active_minus1;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t lists_modification_present_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t cabac_init_present_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t pps_slice_chroma_qp_offsets_present_flag;

    /** Holds a copy of the H.265 bitstream field. */
    uint8_t deblocking_filter_control_present_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t deblocking_filter_override_enabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t pps_deblocking_filter_disabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    int8_t pps_beta_offset_div2;
    /** Holds a copy of the H.265 bitstream field. */
    int8_t pps_tc_offset_div2;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t tiles_enabled_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t uniform_spacing_flag;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t num_tile_columns_minus1;
    /** Holds a copy of the H.265 bitstream field. */
    uint8_t num_tile_rows_minus1;

    /** Holds a copy of the H.265 bitstream field. */
    uint16_t column_width_minus1[22];
    /** Holds a copy of the H.265 bitstream field. */
    uint16_t row_height_minus1[20];

    // RefPicSets
    /** Holds a copy of the H.265 bitstream field. */
    uint32_t iCur;
    /** Holds a copy of the H.265 bitstream field. */
    uint32_t IDRPicFlag;
    /** Holds a copy of the H.265 bitstream field. */
    uint32_t RAPPicFlag;
    /** Holds a copy of the H.265 bitstream field. */
    uint32_t NumDeltaPocsOfRefRpsIdx;
    /** Holds a copy of the H.265 bitstream field. */
    uint32_t NumPocTotalCurr;
    /** Holds a copy of the H.265 bitstream field. */
    uint32_t NumPocStCurrBefore;
    /** Holds a copy of the H.265 bitstream field. */
    uint32_t NumPocStCurrAfter;
    /** Holds a copy of the H.265 bitstream field. */
    uint32_t NumPocLtCurr;
    /** Holds a copy of the H.265 bitstream field. */
    uint32_t NumBitsToSkip;
    /** Holds a copy of the H.265 bitstream field. */
    int32_t CurrPicOrderCntVal;
    /**
     * Holds the surface that contains the reference image.
     * Set to NULL for unused entries.
     */
    NvMediaRefSurface *RefPics[16];
    int32_t PicOrderCntVal[16];
    /** 1 = long-term reference. */
    uint8_t IsLongTerm[16];
    int8_t RefPicSetStCurrBefore[8];
    int8_t RefPicSetStCurrAfter[8];
    int8_t RefPicSetLtCurr[8];

    // scaling lists (diag order)
    /** Holds a copy of the H.265 bitstream field. [matrixId][i]*/
    uint8_t ScalingList4x4[6][16];
    /** Holds a copy of the H.265 bitstream field. [matrixId][i]*/
    uint8_t ScalingList8x8[6][64];
    /** Holds a copy of the H.265 bitstream field. [matrixId][i]*/
    uint8_t ScalingList16x16[6][64];
    /** Holds a copy of the H.265 bitstream field. [matrixId][i]*/
    uint8_t ScalingList32x32[2][64];
    /** Holds a copy of the H.265 bitstream field. [matrixId]*/
    uint8_t ScalingListDCCoeff16x16[6];
    /** Holds a copy of the H.265 bitstream field. [matrixId]*/
    uint8_t ScalingListDCCoeff32x32[2];

    /** Holds a copy of the H.265 bitstream field. */
    uint32_t NumDeltaPocs[64];

    uint8_t sps_range_extension_present_flag;
    // sps extension HEVC-main 444
    /** Holds the SPS extension for \a transform_skip_rotation_enabled_flag. */
    uint8_t transformSkipRotationEnableFlag;
    /** Holds the SPS extension for \a transform_skip_context_enabled_flag. */
    uint8_t transformSkipContextEnableFlag;
    /** Holds the SPS \a implicit_rdpcm_enabled_flag. */
    uint8_t implicitRdpcmEnableFlag;
    /** Holds the SPS \a explicit_rdpcm_enabled_flag. */
    uint8_t explicitRdpcmEnableFlag;
    /** Holds the SPS \a extended_precision_processing_flag; always 0 in current profile. */
    uint8_t extendedPrecisionProcessingFlag;
    /** Holds the SPS \a intra_smoothing_disabled_flag. */
    uint8_t intraSmoothingDisabledFlag;
    /** Holds the SPS \a high_precision_offsets_enabled_flag. */
    uint8_t highPrecisionOffsetsEnableFlag;
    /** Holds the SPS \a fast_rice_adaptation_enabled_flag. */
    uint8_t fastRiceAdaptationEnableFlag;
    /** Holds the SPS \a cabac_bypass_alignment_enabled_flag; always 0 in current profile. */
    uint8_t cabacBypassAlignmentEnableFlag;
    /** Holds the SPS \a intraBlockCopyEnableFlag; always 0 not currently used by the spec. */
    uint8_t intraBlockCopyEnableFlag;

    uint8_t pps_range_extension_present_flag;
    // pps extension HEVC-main 444
    /** Holds the PPS extension \a log2_max_transform_skip_block_size_minus2, 0...5 */
    uint8_t log2MaxTransformSkipSize;
    /** Holds the PPS \a cross_component_prediction_enabled_flag. */
    uint8_t crossComponentPredictionEnableFlag;
    /** Holds the PPS \a chroma_qp_adjustment_enabled_flag .*/
    uint8_t chromaQpAdjustmentEnableFlag;
    /** Holds the PPS \a diff_cu_chroma_qp_adjustment_depth, 0...3 */
    uint8_t diffCuChromaQpAdjustmentDepth;
    /** Holds the PPS \a chroma_qp_adjustment_table_size_minus1+1, 1...6 */
    uint8_t chromaQpAdjustmentTableSize;
    /** Holds the PPS \a log2_sao_offset_scale_luma, max(0,bitdepth-10), maxBitdepth 16 for future. */
    uint8_t log2SaoOffsetScaleLuma;
    /** Holds the PPS \a log2_sao_offset_scale_chroma. */
    uint8_t log2SaoOffsetScaleChroma;
    /** Holds -[12,+12]. */
    int8_t cb_qp_adjustment[6];
    /** Holds -[12,+12]. */
    int8_t cr_qp_adjustment[6];
    /** Ensures alignment to 4 bytes. */
    uint8_t reserved2;
    /** 0:FrameType_B  1:FrameType_P  2:FrameType_I */
    uint8_t frameType;

    /** Parser Only: Flag to indicated mastering display data is present */
    uint8_t masterin_display_data_present;
    /** Parser Only: Mastering display data if present */
    NvMediaMasteringDisplayData MasteringDispData;
    /** Flag to indicate slice decode is enabled */
    NvMediaBool  bSliceDecEnable;
    /** Slice decode data when Slice decode is enabled */
    NvMediaSliceDecodeData sliceDecData;

    /** Holds multiview video extensions. */
    struct {
        /** Indicates hecv-mv is present in the stream. */
        uint32_t mv_hevc_enable;
        /** Holds a copy of the H.265-MV bitstream field. */
        uint32_t nuh_layer_id;
        /** Holds a copy of the H.265-MV bitstream field. */
        uint32_t default_ref_layers_active_flag;
        /** Holds a copy of the H.265-MV bitstream field. */
        uint32_t NumDirectRefLayers;
        /** Holds a copy of the H.265-MV bitstream field. */
        uint32_t max_one_active_ref_layer_flag;
        /** Holds a copy of the H.265-MV bitstream field. */
        uint32_t NumActiveRefLayerPics;
        /** Holds a copy of the H.265-MV bitstream field. */
        uint32_t poc_lsb_not_present_flag;
        /** Holds a copy of the H.265-MV bitstream field. */
        uint32_t  NumActiveRefLayerPics0;
        /** Holds a copy of the H.265-MV bitstream field. */
        uint32_t  NumActiveRefLayerPics1;
        /** Holds a copy of the H.265-MV bitstream field. */
        int32_t  RefPicSetInterLayer0[32];
        /** Holds a copy of the H.265-MV bitstream field. */
        int32_t  RefPicSetInterLayer1[32];
    } mvext;
    /** Parser Only: Flag to indicated content light level data is present */
    NvMediaBool content_light_level_info_present;
    /** Parser Only: Content light level info data if present */
    NvMediaContentLightLevelInfo ContentLightLevelInfo;
} NvMediaPictureInfoH265;
/** @} <!-- Ends h265decoder_api Basic Types sub-group --> */

/**
 * \defgroup mpeg1and2decoder_api MPEG-1 and MPEG-2 Structures
 * \ingroup basic_api_top
 * Provides a structure for defining the MPEG-1 and MPEG-2 picture parameter
 * information.
 * @{
 */

/**
 * \brief Holds picture parameter information for an MPEG 1 or MPEG 2
 *        picture.
 *
 * @note References to "copy of bitstream field" in the field descriptions
 * may refer to data literally parsed from the bitstream, or derived from
 * the bitstream using a mechanism described in the specification.
 */
typedef struct {
    /**
     * Reference used by B and P frames.
     * Set to NULL when not used.
     */
    NvMediaRefSurface *forward_reference;
    /**
     * Reference used by B frames.
     * Set to NULL when not used.
     */
    NvMediaRefSurface *backward_reference;

    /** Holds a copy of the MPEG bitstream field. */
    uint8_t picture_structure;
    /** Holds a copy of the MPEG bitstream field. */
    uint8_t picture_coding_type;
    /** Holds a copy of the MPEG bitstream field. */
    uint8_t intra_dc_precision;
    /** Holds a copy of the MPEG bitstream field. */
    uint8_t frame_pred_frame_dct;
    /** Holds a copy of the MPEG bitstream field. */
    uint8_t concealment_motion_vectors;
    /** Holds a copy of the MPEG bitstream field. */
    uint8_t intra_vlc_format;
    /** Holds a copy of the MPEG bitstream field. */
    uint8_t alternate_scan;
    /** Holds a copy of the MPEG bitstream field. */
    uint8_t q_scale_type;
    /** Holds a copy of the MPEG bitstream field. */
    uint8_t top_field_first;
    /** Holds a copy of the MPEG-1 bitstream field. For MPEG-2, set to 0. */
    uint8_t full_pel_forward_vector;
    /** Holds a copy of the MPEG-1 bitstream field. For MPEG-2, set to 0. */
    uint8_t full_pel_backward_vector;
    /**
     * Holds a copy of the MPEG bitstream field.
     * For MPEG-1, fill both horizontal and vertical entries.
     */
    uint8_t f_code[2][2];
    /** Holds a copy of the MPEG bitstream field, converted to raster order. */
    uint8_t intra_quantizer_matrix[64];
    /** Holds a copy of the MPEG bitstream field, converted to raster order. */
    uint8_t non_intra_quantizer_matrix[64];
    /** Holds the number of slices in this picture.
      * \a nNumSlices entries contain the offset
      * of each slice within the bitstream data buffer. Required for nvdec. */
    uint32_t nNumSlices;
    /** Passing NULL for \a pSliceDataOffsets disables error-concealment. */
    uint32_t *pSliceDataOffsets;
    /** Indicates whether the MPEG slices span across multiple rows. */
    uint8_t flag_slices_across_multiple_rows;
} NvMediaPictureInfoMPEG1Or2;

/** @} <!-- Ends mpeg1and2decoder_api MPEG4 and MPEG Structures --> */

/**
 * \defgroup mpeg4part2decoder_api MPEG4 Part 2 Structures
 * \ingroup basic_api_top
 * Provides a structure for defining picture parameters for the
 * MPEG-4 Part 2 picture.
 * @{
 */

/**
 * \brief Holds picture parameter information for an MPEG-4 Part 2 picture.
 *
 * @note References to "copy of bitstream field" in the field descriptions may:
 * - Refer to data literally parsed from the bitstream or
 * - Be derived from
 * the bitstream using a mechanism described in the specification.
 */
typedef struct {
    /**
     * Reference used by B and P frames.
     * Set to NULL when not used.
     */
    NvMediaRefSurface *forward_reference;
    /**
     * Reference used by B frames.
     * Set to NULL when not used.
     */
    NvMediaRefSurface *backward_reference;

    /** Holds a copy of the bitstream field. */
    int32_t            trd[2];
    /** Holds a copy of the bitstream field. */
    int32_t            trb[2];
    /** Holds a copy of the bitstream field. */
    uint16_t vop_time_increment_resolution;
    /** Holds a copy of the bitstream field. */
    uint32_t   vop_time_increment_bitcount;
    /** Holds a copy of the bitstream field. */
    uint8_t  vop_coding_type;
    /** Holds a copy of the bitstream field. */
    uint8_t  vop_fcode_forward;
    /** Holds a copy of the bitstream field. */
    uint8_t  vop_fcode_backward;
    /** Holds a copy of the bitstream field. */
    uint8_t  resync_marker_disable;
    /** Holds a copy of the bitstream field. */
    uint8_t  interlaced;
    /** Holds a copy of the bitstream field. */
    uint8_t  quant_type;
    /** Holds a copy of the bitstream field. */
    uint8_t  quarter_sample;
    /** Holds a copy of the bitstream field. */
    uint8_t  short_video_header;
    /** Derived from vop_rounding_type bitstream field. */
    uint8_t  rounding_control;
    /** Holds a copy of the bitstream field. */
    uint8_t  alternate_vertical_scan_flag;
    /** Holds a copy of the bitstream field. */
    uint8_t  top_field_first;
    /** Holds a copy of the bitstream field. */
    uint8_t  intra_quantizer_matrix[64];
    /** Holds a copy of the bitstream field. */
    uint8_t  non_intra_quantizer_matrix[64];
    /** Holds a copy of the bitstream field. */
    uint8_t  data_partitioned;
    /** Holds a copy of the bitstream field. */
    uint8_t  reversible_vlc;
    /** Number of slices in this picture. \a nNumSlices entries contain the offset
     of each slice within the bitstream data buffer. Required for nvdec. */
    uint32_t nNumSlices;
    /** Passing NULL for \a pSliceDataOffsets disables error-concealment. */
    uint32_t *pSliceDataOffsets;

    /** Parser Only: Video object layer width */
    uint16_t video_object_layer_width;
    /** Parser Only: Video object layer height */
    uint16_t video_object_layer_height;
    /** Parser Only: DivX flags */
    uint32_t divx_flags;
    /** Parser only: DivX GMC Concealment
     *  Flag to prevent decoding of non I-VOPs during a GMC sequence
     * and indicate beginning / end of a GMC sequence. */
    NvMediaBool bGMCConceal;
} NvMediaPictureInfoMPEG4Part2;

/** @} <!-- Ends mpeg4part2decoder_api MPEG4 Part 2 structures -> */

/**
 * \defgroup vc1decoder_api VC1 Structures
 * \ingroup nvmedia_video_top
 * Defines a structure for defining picture information for a VC1 picture.
 * @{
 */

/**
 * \brief Holds picture parameter information for a VC1 picture.
 *
 * @note References to "copy of bitstream field" in the field descriptions may:
 * - Refer to data literally parsed from the bitstream or
 * - Be derived from
 * the bitstream using a mechanism described in the specification.
 */
typedef struct {
    /**
     * Reference used by B and P frames.
     * Set to NULL when not used.
     */
    NvMediaRefSurface *forward_reference;
    /**
     * Reference used by B frames.
     * Set to NULL when not used.
     */
    NvMediaRefSurface *backward_reference;
    /**
     * Reference used for range mapping.
     * Set to NULL when not used.
     */
    NvMediaRefSurface *range_mapped;

    /** I=0, P=1, B=3, BI=4  from 7.1.1.4. */
    uint8_t  picture_type;
    /** Progressive=0, Frame-interlace=2, Field-interlace=3; see VC-1 7.1.1.15. */
    uint8_t  frame_coding_mode;
    /** Bottom field flag TopField=0 BottomField=1 */
    uint8_t  bottom_field_flag;


    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.1.5. */
    uint8_t postprocflag;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.1.8. */
    uint8_t pulldown;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.1.9. */
    uint8_t interlace;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.1.10. */
    uint8_t tfcntrflag;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.1.11. */
    uint8_t finterpflag;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.1.3. */
    uint8_t psf;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.2.8. */
    uint8_t dquant;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.2.3. */
    uint8_t panscan_flag;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.2.4. */
    uint8_t refdist_flag;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.2.11. */
    uint8_t quantizer;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.2.7. */
    uint8_t extended_mv;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.2.14. */
    uint8_t extended_dmv;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.2.10. */
    uint8_t overlap;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.2.9. */
    uint8_t vstransform;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.2.5. */
    uint8_t loopfilter;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.2.6. */
    uint8_t fastuvmc;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.12.15. */
    uint8_t range_mapy_flag;
    /** Holds a copy of the VC-1 bitstream field. */
    uint8_t range_mapy;
    /** Holds a copy of the VC-1 bitstream field. See VC-1 6.2.16. */
    uint8_t range_mapuv_flag;
    /** Holds a copy of the VC-1 bitstream field. */
    uint8_t range_mapuv;

    /**
     * Copy of the VC-1 bitstream field. See VC-1 J.1.10.
     * Only used by simple and main profiles.
     */
    uint8_t multires;
    /**
     * Copy of the VC-1 bitstream field. See VC-1 J.1.16.
     * Only used by simple and main profiles.
     */
    uint8_t syncmarker;
    /**
     * VC-1 SP/MP range reduction control. See VC-1 J.1.17.
     * Only used by simple and main profiles.
     */
    uint8_t rangered;
    /**
     * Copy of the VC-1 bitstream field. See VC-1 7.1.13
     * Only used by simple and main profiles.
     */
    uint8_t rangeredfrm;
    /**
     * Copy of the VC-1 bitstream field. See VC-1 J.1.17.
     * Only used by simple and main profiles.
     */
    uint8_t maxbframes;
    /** Number of slices in this picture. \a nNumSlices entries contain the offset
     of each slice within the bitstream data buffer. Required for nvdec. */
    uint32_t nNumSlices;
    /** Passing NULL for \a pSliceDataOffsets disables error-concealment. */
    uint32_t *pSliceDataOffsets;

    /** Parser only: Profile */
    uint8_t profile;
    /** Parser only: Actual frame width */
    uint16_t frameWidth;
    /** Parser only: Actual frame height */
    uint16_t frameHeight;
} NvMediaPictureInfoVC1;
/** @} <!-- Ends vc1decoder_api VC1 structures -> */

/**
 * \defgroup vp8decoder_api VP8 Structures
 * \ingroup nvmedia_video_top
 * Defines a structure for defining picture information for a VP8 picture.
 * @{
 */

/**
 * \brief Picture parameter information for a VP8 picture.
 *
 * @note References to "copy of bitstream field" in the field descriptions may:
 * - Refer to data literally parsed from the bitstream or
 * - Be derived from
 * the bitstream using a mechanism described in the specification.
 */
typedef struct {
    /** Last reference frame. */
    NvMediaRefSurface *LastReference;
    /** Golden reference frame. */
    NvMediaRefSurface *GoldenReference;
    /** Alternate reference frame. */
    NvMediaRefSurface *AltReference;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t key_frame;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t version;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t show_frame;
    /**
     * Copy of the VP8 bitstream field.
     * 0 = clamp needed in decoder, 1 = no clamp needed
     */
    uint8_t clamp_type;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t segmentation_enabled;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t update_mb_seg_map;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t update_mb_seg_data;
    /**
     * Copy of the VP8 bitstream field.
     * 0 means delta, 1 means absolute value
     */
    uint8_t update_mb_seg_abs;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t filter_type;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t loop_filter_level;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t sharpness_level;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t mode_ref_lf_delta_enabled;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t mode_ref_lf_delta_update;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t num_of_partitions;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t dequant_index;
    /** Holds a copy of the VP8 bitstream field. */
    int8_t deltaq[5];

    /** Holds a copy of the VP8 bitstream field. */
    uint8_t golden_ref_frame_sign_bias;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t alt_ref_frame_sign_bias;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t refresh_entropy_probs;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t CbrHdrBedValue;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t CbrHdrBedRange;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t mb_seg_tree_probs [3];

    /** Holds a copy of the VP8 bitstream field. */
    int8_t seg_feature[2][4];
    /** Holds a copy of the VP8 bitstream field. */
    int8_t ref_lf_deltas[4];
    /** Holds a copy of the VP8 bitstream field. */
    int8_t mode_lf_deltas[4];

    /** Bits consumed for the current bitstream byte. */
    uint8_t BitsConsumed;
    /** Holds a copy of the VP8 bitstream field. */
    uint8_t AlignByte[3];
    /** Remaining header parition size */
    uint32_t hdr_partition_size;
    /** Start of header partition */
    uint32_t hdr_start_offset;
    /** Offset to byte which is parsed in cpu */
    uint32_t hdr_processed_offset;
    /** Holds a copy of the VP8 bitstream field. */
    uint32_t coeff_partition_size[8];
    /** Holds a copy of the VP8 bitstream field. */
    uint32_t coeff_partition_start_offset[8];
    /** Number of slices in this picture. \a nNumSlices entries contain the offset
     of each slice within the bitstream data buffer. Required for nvdec. */
    uint32_t nNumSlices;
    /** Passing NULL for \a pSliceDataOffsets disables error-concealment. */
    uint32_t *pSliceDataOffsets;
    /** Number of bytes in VP8 Coeff partition (for OTF case) */
    uint32_t uCoeffPartitionDataLen;
    /** Handle to VP8 Coeff partition (for OTF case). */
    uint32_t uCoeffPartitionBufferHandle;

    /** Parser only: RetRefreshGoldenFrame */
    uint32_t RetRefreshGoldenFrame;
    /** Parser only: RetRefreshAltFrame */
    uint32_t RetRefreshAltFrame;
    /** Parser only: RetRefreshLastFrame */
    uint32_t RetRefreshLastFrame;
} NvMediaPictureInfoVP8;
/** @} <!-- Ends vp8decoder_api VP8 Structures -> */

/**
 * \defgroup vp9decoder_api VP9 Structures
 * \ingroup nvmedia_video_top
 * Provides structures for defining the VP9 reference frame.
 * @{
 */

/**
 * \brief Holds VP9 counters for adaptive entropy contexts.
 */
typedef struct {
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t inter_mode_counts[7][3][2];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t sb_ymode_counts[4][10];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t uv_mode_counts[10][10];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t partition_counts[16][4];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t switchable_interp_counts[4][3];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t intra_inter_count[4][2];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t comp_inter_count[5][2];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t single_ref_count[5][2][2];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t comp_ref_count[5][2];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t tx32x32_count[2][4];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t tx16x16_count[2][3];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t tx8x8_count[2][2];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t mbskip_count[3][2];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t joints[4];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t sign[2][2];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t classes[2][11];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t class0[2][2];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t bits[2][10][2];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t class0_fp[2][2][4];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t fp[2][4];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t class0_hp[2][2];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t hp[2][2];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t countCoeffs[2][2][6][6][4];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t countCoeffs8x8[2][2][6][6][4];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t countCoeffs16x16[2][2][6][6][4];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t countCoeffs32x32[2][2][6][6][4];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t countEobs[4][2][2][6][6];
} NvMediaVP9BackwardUpdates;

/**
 * \brief Holds VP9 entropy contexts.
 * Table formatted for 256 bits memory; probs 0 to 7 for all tables followed by
 * probs 8 to N for all tables.
 * Compile with TRACE_PROB_TABLES to print bases for each table.
 */
typedef struct {

    /** Holds a copy of the VP9 bitstream field. */
    uint8_t kf_bmode_prob[10][10][8];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t kf_bmode_probB[10][10][1];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t ref_pred_probs[3];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t mb_segment_tree_probs[7];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t segment_pred_probs[3];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t ref_scores[4];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t prob_comppred[2];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t pad1[9];

    /** Holds a copy of the VP9 bitstream field. */
    uint8_t kf_uv_mode_prob[10][8];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t kf_uv_mode_probB[10][1];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t pad2[6];

    /** Holds a copy of the VP9 bitstream field. */
    uint8_t inter_mode_prob[7][4];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t intra_inter_prob[4];

    /** Holds a copy of the VP9 bitstream field. */
    uint8_t uv_mode_prob[10][8];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t tx8x8_prob[2][1];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t tx16x16_prob[2][2];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t tx32x32_prob[2][3];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t sb_ymode_probB[4][1];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t sb_ymode_prob[4][8];

    /** Holds a copy of the VP9 bitstream field. */
    uint8_t partition_prob[2][16][4];

    /** Holds a copy of the VP9 bitstream field. */
    uint8_t uv_mode_probB[10][1];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t switchable_interp_prob[4][2];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t comp_inter_prob[5];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t mbskip_probs[3];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t pad3[1];

    /** Holds a copy of the VP9 bitstream field. */
    uint8_t joints[3];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t sign[2];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t class0[2][1];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t fp[2][3];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t class0_hp[2];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t hp[2];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t classes[2][10];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t class0_fp[2][2][3];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t bits[2][10];

    /** Holds a copy of the VP9 bitstream field. */
    uint8_t single_ref_prob[5][2];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t comp_ref_prob[5];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t pad4[17];

    /** Holds a copy of the VP9 bitstream field. */
    uint8_t probCoeffs[2][2][6][6][4];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t probCoeffs8x8[2][2][6][6][4];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t probCoeffs16x16[2][2][6][6][4];
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t probCoeffs32x32[2][2][6][6][4];
} NvmediaVP9EntropyProbs;

/**
 * \brief Holds picture parameter information for a VP9 picture.
 *
 * \note References to "copy of bitstream field" in the field descriptions
 * either refer to data literally parsed from the bitstream or data derived from
 * the bitstream using a mechanism described in the specification.
 */

typedef struct {
    /** Holds a pointer to the last reference frame. */
    NvMediaRefSurface *LastReference;
    /** Holds a pointer to the golden reference frame. */
    NvMediaRefSurface *GoldenReference;
    /** Holds a pointer to the alternate reference frame. */
    NvMediaRefSurface *AltReference;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t    width;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t    height;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t    ref0_width;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t    ref0_height;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t    ref1_width;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t    ref1_height;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t    ref2_width;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t    ref2_height;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t keyFrame;
    /** Holds the depth per pixel. */
    uint32_t bit_depth;
    /** If previous frame is key frame. */
    uint32_t prevIsKeyFrame;
    /** Previous frame is show frame.*/
    uint32_t PrevShowFrame;
    /** Resolution change. */
    uint32_t resolutionChange;
    /** Error Resilient. */
    uint32_t errorResilient;
    /** Intra only. */
    uint32_t intraOnly;
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t frameContextIdx;
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t refFrameSignBias[4];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t loopFilterLevel;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t loopFilterSharpness;
    /** Holds a copy of the VP9 bitstream field. */
    int32_t qpYAc;
    /** Holds a copy of the VP9 bitstream field. */
    int32_t qpYDc;
    /** Holds a copy of the VP9 bitstream field. */
    int32_t qpChAc;
    /** Holds a copy of the VP9 bitstream field. */
    int32_t qpChDc;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t lossless;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t transform_mode;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t allow_high_precision_mv;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t allow_comp_inter_inter;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t mcomp_filter_type;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t comp_pred_mode;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t comp_fixed_ref;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t comp_var_ref[2];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t log2_tile_columns;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t log2_tile_rows;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t segmentEnabled;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t segmentMapUpdate;
    /** Holds a copy of the VP9 bitstream field. */
    int32_t segmentMapTemporalUpdate;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t segmentFeatureMode;
    /** Holds a copy of the VP9 bitstream field. */
    uint8_t segmentFeatureEnable[8][4];
    /** Holds a copy of the VP9 bitstream field. */
    short segmentFeatureData[8][4];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t modeRefLfEnabled;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t mbRefLfDelta[4];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t mbModeLfDelta[2];
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t offsetToDctParts;
    /** Holds a copy of the VP9 bitstream field. */
    uint32_t frameTagSize;
    /** Holds a copy of the VP9 bitstream field. */
    NvmediaVP9EntropyProbs entropy;

    /** Parser only: Backward update counts */
    NvMediaVP9BackwardUpdates backwardUpdateCounts;
} NvMediaPictureInfoVP9;
/** @} <!-- Ends vp9decoder_api VP9 Structures --> */

#if NV_BUILD_CONFIGURATION_EXPOSING_T23X
#define NV_NVMEDIA_VIDEOCODECNEW_T23X
    #include "nvmedia_common_decode_t23x.h"
#undef NV_NVMEDIA_VIDEOCODECNEW_T23X
#endif

/** @} <!-- Ends common_types_decode_top group NvMedia Types and Structures common to Video & Image --> */

/*
 * \defgroup history_nvmedia_common_decode History
 * Provides change history for the NvMedia Common Decode Types.
 *
 * \section history_nvmedia_common_decode Version History
 *
 * <b> Version 1.0 </b> July 10, 2019
 * - Initial release
 * <b> Version 1.1 </b> July 17, 2019
 * - Added change to consider scaling factor for max_display_parameter_luminance
 */


#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_COMMON_DECODE_H */
