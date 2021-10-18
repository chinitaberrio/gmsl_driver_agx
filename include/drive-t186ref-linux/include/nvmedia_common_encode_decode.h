/*
 * Copyright (c) 2019-2020 NVIDIA CORPORATION. All rights reserved. All
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

#ifndef NVMEDIA_COMMON_ENCODE_DECODE_H
#define NVMEDIA_COMMON_ENCODE_DECODE_H

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

/**
 * \brief Video codec type
 */
typedef enum {
    /** \brief H.264 codec */
    NVMEDIA_VIDEO_CODEC_H264,
    /** \brief VC-1 simple and main profile codec */
    NVMEDIA_VIDEO_CODEC_VC1,
    /** \brief VC-1 advanced profile codec */
    NVMEDIA_VIDEO_CODEC_VC1_ADVANCED,
    /** \brief MPEG1 codec */
    NVMEDIA_VIDEO_CODEC_MPEG1,
    /** \brief MPEG2 codec */
    NVMEDIA_VIDEO_CODEC_MPEG2,
    /** \brief MPEG4 Part 2 codec */
    NVMEDIA_VIDEO_CODEC_MPEG4,
    /** \brief MJPEG codec */
    NVMEDIA_VIDEO_CODEC_MJPEG,
    /** \brief VP8 codec */
    NVMEDIA_VIDEO_CODEC_VP8,
    /** \brief H265 codec */
    NVMEDIA_VIDEO_CODEC_HEVC,
    /** \brief VP9 codec */
    NVMEDIA_VIDEO_CODEC_VP9,
    /** \brief H.264 Multiview Video Coding codec */
    NVMEDIA_VIDEO_CODEC_H264_MVC,
    /** \brief H265 Multiview Video Coding codec */
    NVMEDIA_VIDEO_CODEC_HEVC_MV,
#if NV_BUILD_CONFIGURATION_EXPOSING_T23X
    /** \brief AV1 Video Coding codec */
    NVMEDIA_VIDEO_CODEC_AV1,
#endif
} NvMediaVideoCodec;

/**
 * \brief Holds an application data buffer containing compressed video
 *        data.
 */
typedef struct {
    /** A pointer to the bitstream data bytes. */
    uint8_t *bitstream;
    /** The number of data bytes */
    uint32_t bitstreamBytes;
    /** Size of bitstream array */
    uint32_t bitstreamSize;
} NvMediaBitstreamBuffer;

/** @} <!-- Ends common_types_top group NvMedia Types and Structures common to Video & Image --> */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_COMMON_ENCODE_DECODE_H */
