/*
 * Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: The NvMedia Image Decode Processing API </b>
 *
 * This file contains the \ref image_decoder_api "Image Decode Processing API".
 */

#ifndef NVMEDIA_IMGDEC_H
#define NVMEDIA_IMGDEC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_common_decode.h"
#include "nvmedia_core.h"
#include "nvmedia_image.h"

/**
 * \defgroup image_decoder_api Image Decoder
 * \ingroup nvmedia_image_top
 *
 * Defines and manages objects that decode video.
 *
 * The NvMediaImageDecoder object decodes compressed video data, writing
 * the results to a \ref NvMediaImage "NvMediaImage".
 *
 * A specific NvMedia implementation may support decoding multiple
 * types of compressed video data. However, NvMediaImageDecoder objects
 * are able to decode a specific type of compressed video data.
 * This type must be specified during creation.
 *
 * @{
 */

/** \brief Major Version number */
#define NVMEDIA_IMAGEDEC_VERSION_MAJOR   2
/** \brief Minor Version number */
#define NVMEDIA_IMAGEDEC_VERSION_MINOR   8

/**
 * \brief Image codec type
 */
typedef enum {
    /** \brief H.264 codec */
    NVMEDIA_IMAGE_CODEC_H264 = 0,
    /** \brief H265 codec */
    NVMEDIA_IMAGE_CODEC_HEVC = 8,
    /** \brief VP9 codec */
    NVMEDIA_IMAGE_CODEC_VP9  = 9,
} NvMediaImageCodec;

/**
 * \brief A handle representing a IMAGE decoder object.
 */
typedef struct {
    /** \brief Codec type */
    NvMediaImageCodec codec;
    /** \brief Decoder width */
    uint16_t width;
    /** \brief Decoder height */
    uint16_t height;
    /** \brief Maximum number of reference pictures */
    uint16_t maxReferences;
    /** \brief Instance ID of the decoder */
    NvMediaDecoderInstanceId instanceId;
    /** An Opaque pointer for internal use */
    struct NvMediaImageDecoderPriv_ *decoderPriv;
} NvMediaImageDecoder;

/**
 * \defgroup decoder_create_flag Decoder Creation Flag
 * Defines decoder flag bit masks for constructing the decoder.
 * @{
 */

/**
 * \hideinitializer
 * \brief Progressive sequence
 */
#define NVMEDIA_IMAGE_DECODER_ATTRIBUTE_PROGRESSIVE_SEQUENCE       (1U<<0)

/**
 * \hideinitializer
 * \brief Defines 10-bit decode.
 */

#define NVMEDIA_IMAGE_DECODER_10BIT_DECODE                         (1U<<1)

/**
 * \hideinitializer
 * \brief Rec_2020 color format for the decoded surface
 */

#define NVMEDIA_IMAGE_DECODER_PIXEL_REC_2020                       (1U<<2)

/**
 * \hideinitializer
 * \brief Enable decoder profiling support
 */

#define NVMEDIA_IMAGE_DECODER_PROFILING                            (1U<<5)

/**@} <!-- Ends decoder_create_flag sub-group --> */

/**
 * \brief Checks the version compatibility for the NvMedia Image decoder library.
 * \param[in] version A pointer to a \ref NvMediaVersion structure
 *                    of the client.
 * \return \ref NvMediaStatus The status of the operation.
 * Possible values are:
 * \n \ref NVMEDIA_STATUS_OK
 * \n \ref NVMEDIA_STATUS_BAD_PARAMETER if the pointer is invalid.
 */
NvMediaStatus
NvMediaImageDecoderGetVersion(
    NvMediaVersion *version
);

/** \brief Creates a image decoder object.
 *
 * Creates a \ref NvMediaImageDecoder object for the specified codec. Each
 * decoder object may be accessed by a separate thread. The object
 * must be destroyed with \ref NvMediaImageDecoderDestroy(). All surfaces
 * used with the \c NvMediaImageDecoder must be obtained by /ref NvMediaSurfaceFormatGetType
 * with:
 * - \ref NVM_SURF_FMT_SET_ATTR_YUV(attr, YUV, 420, SEMI_PLANAR, UINT, 8/10/12, BL)
 * - \ref NVM_SURF_FMT_SET_ATTR_YUV(attr, YUV, 422, SEMI_PLANAR, UINT, 8/10/12, BL)
 * - \ref NVM_SURF_FMT_SET_ATTR_YUV(attr, YUV, 444, SEMI_PLANAR, UINT, 8/10/12, BL)
 *
 * \param[in] device A pointer to the \ref NvMediaDevice "device" this video decoder will use.
 * \param[in] codec Codec type. The following types are supported:
 * - \ref NVMEDIA_IMAGE_CODEC_HEVC
 * - \ref NVMEDIA_IMAGE_CODEC_H264
 * \param[in] width Decoder width in luminance pixels.
 * \param[in] height Decoder height in luminance pixels.
 * \param[in] maxReferences The maximum number of reference frames used.
 * This limits internal allocations.
 * \param[in] maxBitstreamSize The maximum size for bitstream.
 * This limits internal allocations.
 * \param[in] inputBuffering How many frames can be in flight at any given
 * time. If this value is 1, NvMediaImageDecoderRender() blocks until the
 * previous frame has finished decoding. If this is 2, \c NvMediaImageDecoderRender
 * blocks if two frames are pending but does not block if one is pending.
 * This value is clamped internally to between 1 and 8.
 * \param[in] flags Set the flags of the decoder.
 * The following flags are supported:
 * \n \ref NVMEDIA_IMAGE_DECODER_ATTRIBUTE_PROGRESSIVE_SEQUENCE
 * \param[in] instanceId The ID of the engine instance.
 * The following instances are supported:
 * \n \ref NVMEDIA_DECODER_INSTANCE_0
 * \n \ref NVMEDIA_DECODER_INSTANCE_1
 * \n \ref NVMEDIA_DECODER_INSTANCE_AUTO
 * \return NvMediaVideoDecoder The new video decoder's handle or NULL if unsuccessful.
 */

NvMediaImageDecoder *
NvMediaImageDecoderCreate(
    const NvMediaDevice *device,
    NvMediaImageCodec codec,
    uint16_t width,
    uint16_t height,
    uint16_t maxReferences,
    uint64_t maxBitstreamSize,
    uint8_t inputBuffering,
    uint32_t flags,
    NvMediaDecoderInstanceId instanceId
);


/** \brief Unpin all the Pinned decode Surfaces used by decoder.
 *
 * It is Mandatory for NvMedia Applications to call this API before
 * Freeing Allocated Output Surfaces.
 *
 * \param[in] decoder The decoder object that will perform the
 *       decode operation.
 *
 */
void
NvMediaImageDecoderUnRegisterPinnedSurfaces(
    const NvMediaImageDecoder *decoder
);

/** \brief Destroys a image decoder object.
 * \param[in] decoder The decoder to be destroyed.
 * \return void
 */
void
NvMediaImageDecoderDestroy(
   const NvMediaImageDecoder *decoder
);

/**
 * \brief Decodes a compressed field/frame and render the result
 *        into a \ref NvMediaImage "NvMediaImage".
 * \param[in] decoder The decoder object that will perform the
 *       decode operation.
 * \param[in] target The image surface to render to.
 * \param[in] pictureInfo A (pointer to a) structure containing
 *       information about the picture to be decoded. Note that
 *       the appropriate type of NvMediaPictureInfo* structure must
 *       be provided to match to profile that the decoder was
 *       created for.
 * \param[in] numBitstreamBuffers The number of bitstream
 *       buffers containing compressed data for this picture.
 * \param[in] bitstreams An array of bitstream buffers.
 * \param[in] instanceId The ID of the engine instance.
 * The following instances are supported if NVMEDIA_DECODER_INSTANCE_AUTO
 * was used in \ref NvMediaImageDecoderCreate API, else this parameter is ignored:
 * \n \ref NVMEDIA_DECODER_INSTANCE_0
 * \n \ref NVMEDIA_DECODER_INSTANCE_1
 * \return \ref NvMediaStatus The completion status of the operation.
 * Possible values are:
 * \n \ref NVMEDIA_STATUS_OK
 * \n \ref NVMEDIA_STATUS_BAD_PARAMETER
 */
NvMediaStatus
NvMediaImageDecoderRender(
    const NvMediaImageDecoder *decoder,
    NvMediaImage        *target,
    const NvMediaPictureInfo *pictureInfo,
    uint32_t numBitstreamBuffers,
    const NvMediaBitstreamBuffer *bitstreams,
    NvMediaDecoderInstanceId instanceId
);


/** \brief NvMediaImageDecoder get backward updates counters for VP9
 *       adaptive entropy contexts.
 *
 * \param[in] decoder A pointer to the decoder object that performs the
 *       decoding operation.
 * \param[in] backupdates A pointer to a structure that holds the
 *       backward update counters.
 */
NvMediaStatus
NvMediaImageDecoderGetBackwardUpdates(
    const NvMediaImageDecoder *decoder,
    void *backupdates
);


/*
 * \defgroup history_nvmedia_imgdec History
 * Provides change history for the NvMedia Image Decode API.
 *
 * \section history_nvmedia_imgdec Version History
 *
 * <b> Version 1.0 </b> May 1, 2016
 * - Initial release
 *
 * <b> Version 1.1 </b> May 27, 2016
 * - Added \ref NvMediaImageDecoderCheckVersion API
 *
 * <b> Version 2.0 </b> May 9, 2017
 * - Added instance ID parameter to \ref NvMediaImageDecoderCreate()
 *   and NvMediaImageDecoderRender()
 * - Added NvMediaDevice parameter to NvMediaImageDecoderCreate()
 * - NvMediaImageDecoderCheckVersion is now deprecated.
 *   Use NvMediaImageDecoderGetVersion() instead.
 * - All NvMedia data types are moved to standard data types
 *
 * <b> Version 2.1 </b> Dec 14, 2018
 * - Defined Macro constants as unsigned to fix MISRA issues
 * - Fix MISRA violations 21.1 and 21.2
 *
 * <b> Version 2.2 </b> January 15, 2019
 * - Fix MISRA violations 8.13
 *
 * <b> Version 2.3 </b> Feb 7, 2019
 * - Added opaque handle for decoder
 *   internal usage into Image decoder object
 * - Fix MISRA violations 11.3
 * - Fix MISRA violations 8.13
 *
 * <b> Version 2.4 </b> February 28, 2019
 * - Added required header include nvmedia_core.h
 *
 * <b> Version 2.5 </b> July 10, 2019
 * - Header include nvmedia_common.h is replaced with nvmedia_common_decode.h
 *
 * <b> Version 2.6 </b> August 20, 2019
 * - Added New API \ref NvMediaImageDecoderUnRegisterPinnedSurfaces for
 *   Unpinning of Pinned Surfaces.
 *
 * <b> Version 2.7 </b> August 31, 2019
 * - Added NVMEDIA_IMAGE_CODEC_VP9 code type entry in \ref NvMediaImageCodec
 * - Added New API \ref NvMediaImageDecoderGetBackwardUpdates to
 *   get the backward probality update.
 *
 * <b> Version 2.8 </b> November 21, 2019
 * - Added NVMEDIA_IMAGE_DECODER_PROFILING bit as a part of flags to enable
 *   decoder per frame profiling information.
 */
/**@} <!-- Ends decoder_api Image Decoder --> */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_IMGDEC_H */
