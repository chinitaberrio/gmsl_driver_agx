/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: Surface Handling </b>
 *
 * @b Description: This file contains the
 *                 \ref surface_handling_api "NvMedia Surface Handling API".
 */

#ifndef NVMEDIA_SURFACE_H
#define NVMEDIA_SURFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"

/**
 * \defgroup surface_handling_api Surface Handling API
 *
 * Defines and manages image and video surface objects.
 *
 * NvMedia Surfaces are video RAM surfaces
 * storing YUV, RGBA or RAW data. They can store one or more surfaces depending
 * on the surface types.
 *
 * - For guidance on creating and destroying image surfaces,
 *   see \ref nvmedia_image_top.
 * - For guidance on creating and destroying video surfaces
 *   \ref nvmedia_video_top.
 *
 * @ingroup nvmedia_common_top
 * @{
 */

/** \brief Major Version number */
#define NVMEDIA_SURFACE_VERSION_MAJOR   (1u)
/** \brief Minor Version number */
#define NVMEDIA_SURFACE_VERSION_MINOR   (14u)


/**
 * \brief Defines NvMedia Surface format attribute types.
 *
 * The range of \ref NvMediaSurfFormatAttrType values depend on the surface
 * format attribute type. See the individual enum for the valid range.
 */
typedef enum {
    /** Specifies the surface type. Valid range is:
        [\ref NVM_SURF_ATTR_SURF_TYPE_YUV,
        \ref NVM_SURF_ATTR_SURF_TYPE_RAW]. */
    NVM_SURF_ATTR_SURF_TYPE = 0u,
    /** Specifies the surface layout type. Valid range is:
        [\ref NVM_SURF_ATTR_LAYOUT_BL,
        \ref NVM_SURF_ATTR_LAYOUT_PL]. */
    NVM_SURF_ATTR_LAYOUT,
    /** Specifies the surface data type. Valid range is:
        [\ref NVM_SURF_ATTR_DATA_TYPE_UINT,
        \ref NVM_SURF_ATTR_DATA_TYPE_FLOATISP]. */
    NVM_SURF_ATTR_DATA_TYPE,
    /** Specifies the surface memory type flags. Valid range is:
        [\ref NVM_SURF_ATTR_MEMORY_PLANAR,
        \ref NVM_SURF_ATTR_MEMORY_PACKED]. */
    NVM_SURF_ATTR_MEMORY,
    /** Specifies the surface sub sampling type. Valid range is:
        [\ref NVM_SURF_ATTR_SUB_SAMPLING_TYPE_NONE,
        \ref NVM_SURF_ATTR_SUB_SAMPLING_TYPE_422]. */
    NVM_SURF_ATTR_SUB_SAMPLING_TYPE,
    /** Specifies bits per component. Valid range is:
        [\ref NVM_SURF_ATTR_BITS_PER_COMPONENT_8,
         \ref NVM_SURF_ATTR_BITS_PER_COMPONENT_20]. */
    NVM_SURF_ATTR_BITS_PER_COMPONENT,
    /** Specifies the Pixel order. Valid range is:
        [\ref NVM_SURF_ATTR_COMPONENT_ORDER_LUMA,
        \ref NVM_SURF_ATTR_COMPONENT_ORDER_CCCC]. */
    NVM_SURF_ATTR_COMPONENT_ORDER,
    /** Specifies the maximum number of surface format attributes. */
    NVM_SURF_FMT_ATTR_MAX,
} NvMediaSurfFormatAttrType;

/** \brief NVM_SURF_ATTR_SURF_TYPE flags. */
/** YUV surface type flag. */
#define NVM_SURF_ATTR_SURF_TYPE_YUV                       (0x00000001u)
/** RGBA surface type flag. */
#define NVM_SURF_ATTR_SURF_TYPE_RGBA                      (0x00000002u)
/** RAW surface type flag. */
#define NVM_SURF_ATTR_SURF_TYPE_RAW                       (0x00000003u)

/** \brief NVM_SURF_ATTR_LAYOUT flags */
/** Block Linear (BL) surface layout flag. */
#define NVM_SURF_ATTR_LAYOUT_BL                           (0x00000001u)
/** Pitch Linear (PL) surface layout flag. */
#define NVM_SURF_ATTR_LAYOUT_PL                           (0x00000002u)

/** \brief NVM_SURF_ATTR_DATA_TYPE flags. */
/** Unsigned Integer surface data type flag. */
#define NVM_SURF_ATTR_DATA_TYPE_UINT                      (0x00000001u)
/** Integer surface data type flag. */
#define NVM_SURF_ATTR_DATA_TYPE_INT                       (0x00000002u)
/** Float surface data type flag. */
#define NVM_SURF_ATTR_DATA_TYPE_FLOAT                     (0x00000003u)
/** FloatISP surface data type flag. */
#define NVM_SURF_ATTR_DATA_TYPE_FLOATISP                  (0x00000004u)

/** \brief NVM_SURF_ATTR_MEMORY flags. */
/** Planar surface memory type flag. */
#define NVM_SURF_ATTR_MEMORY_PLANAR                       (0x00000001u)
/** Semi-Planar surface memory type flag. */
#define NVM_SURF_ATTR_MEMORY_SEMI_PLANAR                  (0x00000002u)
/** Packed surface memory type flag. */
#define NVM_SURF_ATTR_MEMORY_PACKED                       (0x00000003u)

/** \brief NVM_SURF_ATTR_SUB_SAMPLING_TYPE flags for YUV surface types
  * "R" indicates a transpose.
  */
/** 4:2:0 sub-sampling type flag. */
#define NVM_SURF_ATTR_SUB_SAMPLING_TYPE_420               (0x00000001u)
/** 4:2:2 sub-sampling type flag. */
#define NVM_SURF_ATTR_SUB_SAMPLING_TYPE_422               (0x00000002u)
/** 4:4:4 sub-sampling type flag. */
#define NVM_SURF_ATTR_SUB_SAMPLING_TYPE_444               (0x00000003u)
/** 4:2:2 transposed sub-sampling type flag. */
#define NVM_SURF_ATTR_SUB_SAMPLING_TYPE_422R              (0x00000004u)
/** sub-sampling type not applicable flag. */
#define NVM_SURF_ATTR_SUB_SAMPLING_TYPE_NONE              (0x00000000u)

/** \brief NVM_SURF_ATTR_BITS_PER_COMPONENT flags.
  * If a layout is not specified, it is a uniform layout across components.
  */
/** 8-bit per component flag. */
#define NVM_SURF_ATTR_BITS_PER_COMPONENT_8                 (0x00000001u)
/** 10-bit per component flag. */
#define NVM_SURF_ATTR_BITS_PER_COMPONENT_10                (0x00000002u)
/** 12-bit per component flag. */
#define NVM_SURF_ATTR_BITS_PER_COMPONENT_12                (0x00000003u)
/** 14-bit per component flag. */
#define NVM_SURF_ATTR_BITS_PER_COMPONENT_14                (0x00000004u)
/** 16-bit per component flag. */
#define NVM_SURF_ATTR_BITS_PER_COMPONENT_16                (0x00000005u)
/** 32-bit per component flag. */
#define NVM_SURF_ATTR_BITS_PER_COMPONENT_32                (0x00000006u)

/** 16:8:8 bits per component layout flag. */
#define NVM_SURF_ATTR_BITS_PER_COMPONENT_LAYOUT_16_8_8     (0x00000007u)
/** 10:8:8 bits per component layout flag. */
#define NVM_SURF_ATTR_BITS_PER_COMPONENT_LAYOUT_10_8_8     (0x00000008u)
/** 2:10:10:10 bits per component layout flag. */
#define NVM_SURF_ATTR_BITS_PER_COMPONENT_LAYOUT_2_10_10_10 (0x00000009u)
/** 20-bit per component flag. */
#define NVM_SURF_ATTR_BITS_PER_COMPONENT_20                (0x0000000Au)

/** \brief NVM_SURF_ATTR_COMPONENT_ORDER flags for YUV surface type. */
/** Luma component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_LUMA                (0x00000001u)
/** YUV component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_YUV                 (0x00000002u)
/** YVU component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_YVU                 (0x00000003u)
/** YUYV component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_YUYV                (0x00000004u)
/** YVYU component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_YVYU                (0x00000005u)
/** VYUY component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_VYUY                (0x00000006u)
/** UYVY component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_UYVY                (0x00000007u)
/** XUYV component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_XUYV                (0x00000008u)
/** XYUV component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_XYUV                (0x00000009u)
/** VUYX component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_VUYX                (0x0000000Au)

/** \brief NVM_SURF_ATTR_PIXEL_ORDER flags for RGBA surface type. */
/** Alpha component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_ALPHA               (0x00000011u)
/** RGBA component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_RGBA                (0x00000012u)
/** ARGB component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_ARGB                (0x00000013u)
/** BGRA component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_BGRA                (0x00000014u)
/** RG component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_RG                  (0x00000015u)

/** \brief NVM_SURF_ATTR_PIXEL_ORDER flags for RAW surface type. */
/** RGGB component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_RGGB                (0x00000021u)
/** BGGR component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_BGGR                (0x00000022u)
/** GRBG component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_GRBG                (0x00000023u)
/** GBRG component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_GBRG                (0x00000024u)

/** RCCB component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_RCCB                (0x00000025u)
/** BCCR component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_BCCR                (0x00000026u)
/** CRBC component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_CRBC                (0x00000027u)
/** CBRC component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_CBRC                (0x00000028u)

/** RCCC component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_RCCC                (0x00000029u)
/** CCCR component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_CCCR                (0x0000002Au)
/** CRCC component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_CRCC                (0x0000002Bu)
/** CCRC component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_CCRC                (0x0000002Cu)

/** CCCC component order flag. */
#define NVM_SURF_ATTR_COMPONENT_ORDER_CCCC                (0x0000002Du)

/**
 * \brief Holds NvMedia Surface format attributes.
 */
typedef struct {
    /** \brief Holds surface format attribute type. */
    NvMediaSurfFormatAttrType type;
    /** \brief Holds surface format attribute value. */
    uint32_t value;
} NvMediaSurfFormatAttr;

/**
 * \brief A helper macro to define surface format attributes.
 */
#define NVM_SURF_FMT_DEFINE_ATTR(x)                                                   \
    NvMediaSurfFormatAttr x[] = {                                                     \
        {                                                                             \
            .type = NVM_SURF_ATTR_SURF_TYPE,                                          \
            .value = 0u,                                                               \
        },                                                                            \
        {                                                                             \
            .type = NVM_SURF_ATTR_LAYOUT,                                             \
            .value = 0u,                                                               \
        },                                                                            \
        {                                                                             \
            .type = NVM_SURF_ATTR_DATA_TYPE,                                          \
            .value = 0u,                                                               \
        },                                                                            \
        {                                                                             \
            .type = NVM_SURF_ATTR_MEMORY,                                             \
            .value = 0u,                                                               \
        },                                                                            \
        {                                                                             \
            .type = NVM_SURF_ATTR_SUB_SAMPLING_TYPE,                                  \
            .value = 0u,                                                               \
        },                                                                            \
        {                                                                             \
            .type = NVM_SURF_ATTR_BITS_PER_COMPONENT,                                 \
            .value = 0u,                                                               \
        },                                                                            \
        {                                                                             \
            .type = NVM_SURF_ATTR_COMPONENT_ORDER,                                    \
            .value = 0u,                                                               \
        },                                                                            \
    };                                                                                \

/**
 * \brief A helper macro to set YUV surface format attributes.
 * The \a attr parameter must be defined before setting the values using
 * \ref NVM_SURF_FMT_SET_ATTR_YUV. For \ref NVM_SURF_ATTR_COMPONENT_ORDER_XUYV,
 * \ref NVM_SURF_ATTR_COMPONENT_ORDER_XYUV, \ref NVM_SURF_ATTR_COMPONENT_ORDER_VUYX
 * and \ref NVM_SURF_ATTR_COMPONENT_ORDER_LUMA, set the sampling type to be
 * \ref NVM_SURF_ATTR_SUB_SAMPLING_TYPE_NONE.
 */
#define NVM_SURF_FMT_SET_ATTR_YUV(attr, order, samplingtype, memory, datatype, bpc, layout)       \
{                                                                                                 \
    attr[0].type = NVM_SURF_ATTR_SURF_TYPE;                                                       \
    attr[0].value = NVM_SURF_ATTR_SURF_TYPE_YUV;                                                  \
                                                                                                  \
    attr[1].type = NVM_SURF_ATTR_LAYOUT;                                                          \
    attr[1].value = NVM_SURF_ATTR_LAYOUT_##layout;                                                \
                                                                                                  \
    attr[2].type = NVM_SURF_ATTR_DATA_TYPE;                                                       \
    attr[2].value = NVM_SURF_ATTR_DATA_TYPE_##datatype;                                           \
                                                                                                  \
    attr[3].type = NVM_SURF_ATTR_MEMORY;                                                          \
    attr[3].value = NVM_SURF_ATTR_MEMORY_##memory;                                                \
                                                                                                  \
    attr[4].type = NVM_SURF_ATTR_SUB_SAMPLING_TYPE;                                               \
    attr[4].value = NVM_SURF_ATTR_SUB_SAMPLING_TYPE_##samplingtype;                               \
                                                                                                  \
    attr[5].type = NVM_SURF_ATTR_BITS_PER_COMPONENT;                                              \
    attr[5].value = NVM_SURF_ATTR_BITS_PER_COMPONENT_##bpc;                                       \
                                                                                                  \
    attr[6].type = NVM_SURF_ATTR_COMPONENT_ORDER;                                                 \
    attr[6].value = NVM_SURF_ATTR_COMPONENT_ORDER_##order;                                        \
}

/**
 * \brief A helper macro to set RGBA surface format attributes.
 * The \a attr parameter must be defined before setting the values using
 * \ref NVM_SURF_FMT_SET_ATTR_RGBA.
 */
#define NVM_SURF_FMT_SET_ATTR_RGBA(attr, order, datatype, bpc, layout)                            \
{                                                                                                 \
    attr[0].type = NVM_SURF_ATTR_SURF_TYPE;                                                       \
    attr[0].value = NVM_SURF_ATTR_SURF_TYPE_RGBA;                                                 \
                                                                                                  \
    attr[1].type = NVM_SURF_ATTR_LAYOUT;                                                          \
    attr[1].value = NVM_SURF_ATTR_LAYOUT_##layout;                                                \
                                                                                                  \
    attr[2].type = NVM_SURF_ATTR_DATA_TYPE;                                                       \
    attr[2].value = NVM_SURF_ATTR_DATA_TYPE_##datatype;                                           \
                                                                                                  \
    attr[3].type = NVM_SURF_ATTR_MEMORY;                                                          \
    attr[3].value = NVM_SURF_ATTR_MEMORY_PACKED;                                                  \
                                                                                                  \
    attr[4].type = NVM_SURF_ATTR_SUB_SAMPLING_TYPE;                                               \
    attr[4].value = NVM_SURF_ATTR_SUB_SAMPLING_TYPE_NONE;                                                                            \
                                                                                                  \
    attr[5].type = NVM_SURF_ATTR_BITS_PER_COMPONENT;                                              \
    attr[5].value = NVM_SURF_ATTR_BITS_PER_COMPONENT_##bpc;                                       \
                                                                                                  \
    attr[6].type = NVM_SURF_ATTR_COMPONENT_ORDER;                                                 \
    attr[6].value = NVM_SURF_ATTR_COMPONENT_ORDER_##order;                                        \
}

/**
 * \brief A helper macro to set RAW surface format attributes.
 * Before setting the values, use \ref NVM_SURF_FMT_SET_ATTR_RAW to define \a attr.
 */
#define NVM_SURF_FMT_SET_ATTR_RAW(attr, order, datatype, bpc, layout)                             \
{                                                                                                 \
    attr[0].type = NVM_SURF_ATTR_SURF_TYPE;                                                       \
    attr[0].value = NVM_SURF_ATTR_SURF_TYPE_RAW;                                                  \
                                                                                                  \
    attr[1].type = NVM_SURF_ATTR_LAYOUT;                                                          \
    attr[1].value = NVM_SURF_ATTR_LAYOUT_##layout;                                                \
                                                                                                  \
    attr[2].type = NVM_SURF_ATTR_DATA_TYPE;                                                       \
    attr[2].value = NVM_SURF_ATTR_DATA_TYPE_##datatype;                                           \
                                                                                                  \
    attr[3].type = NVM_SURF_ATTR_MEMORY;                                                          \
    attr[3].value = NVM_SURF_ATTR_MEMORY_PACKED;                                                  \
                                                                                                  \
    attr[4].type = NVM_SURF_ATTR_SUB_SAMPLING_TYPE;                                               \
    attr[4].value = NVM_SURF_ATTR_SUB_SAMPLING_TYPE_NONE;                                                                            \
                                                                                                  \
    attr[5].type = NVM_SURF_ATTR_BITS_PER_COMPONENT;                                              \
    attr[5].value = NVM_SURF_ATTR_BITS_PER_COMPONENT_##bpc;                                       \
                                                                                                  \
    attr[6].type = NVM_SURF_ATTR_COMPONENT_ORDER;                                                 \
    attr[6].value = NVM_SURF_ATTR_COMPONENT_ORDER_##order;                                        \
}

/**
 * \brief Defines NvMedia Surface allocation attribute types.
 *
 * The range of \ref NvMediaSurfAllocAttrType values depend on the surface
 * allocation attribute type. See the individual enum for the valid range.
 *
 * @note The Application must set the relevant color standard based on the Surface type.
 *       If 0 is passed as value to \ref NVM_SURF_ATTR_CPU_ACCESS and
 *       \ref NVM_SURF_ATTR_COLOR_STD_TYPE appropriate default attribute value will
 *       be set by NvMedia-Surface.
 */
typedef enum {
    /** Specifies the surface width.
        Valid range is: [\ref NVM_SURF_ATTR_MIN_WIDTH, \ref NVM_SURF_ATTR_MAX_WIDTH]. */
    NVM_SURF_ATTR_WIDTH = 0u,
    /** Specifies the surface height (excluding embedded data lines).
        Valid range is: [\ref NVM_SURF_ATTR_MIN_HEIGHT, \ref NVM_SURF_ATTR_MAX_HEIGHT]. */
    NVM_SURF_ATTR_HEIGHT,
    /** Specifies the embedded lines top.
        Valid range is: [\ref NVM_SURF_ATTR_MIN_EMB_LINES_TOP,
                         \ref NVM_SURF_ATTR_MAX_EMB_LINES_TOP]. */
    NVM_SURF_ATTR_EMB_LINES_TOP,
    /** Specifies the embedded lines bottom.
        Valid range is: [\ref NVM_SURF_ATTR_MIN_EMB_LINES_BOTTOM,
                         \ref NVM_SURF_ATTR_MAX_EMB_LINES_BOTTOM]. */
    NVM_SURF_ATTR_EMB_LINES_BOTTOM,
    /** Specifies the CPU access to surface flags. The default value is:
        NVM_SURF_ATTR_CPU_ACCESS_UNCACHED.
        Valid range is: [\ref NVM_SURF_ATTR_CPU_ACCESS_UNCACHED,
                         \ref NVM_SURF_ATTR_CPU_ACCESS_UNMAPPED]. */
    NVM_SURF_ATTR_CPU_ACCESS,
    /** Specifies the allocation type. The default value is : none.
        Valid range is: [\ref NVM_SURF_ATTR_ALLOC_DEFAULT,
                         \ref NVM_SURF_ATTR_ALLOC_SECURED]. */
    NVM_SURF_ATTR_ALLOC_TYPE,
    /** Specifies the peer VM ID in case of shared buffers.
        This value must be a valid Peer VM-ID. */
    NVM_SURF_ATTR_PEER_VM_ID,
    /** Specifies the surface scan type. The default value is:
        NVM_SURF_ATTR_SCAN_PROGRESSIVE.
        Valid range is: [\ref NVM_SURF_ATTR_SCAN_PROGRESSIVE] */
    NVM_SURF_ATTR_SCAN_TYPE,
    /** Specifies the color standard type. The default is
        YCbCr Rec.601 (Extended Range) for YUV surface types, and
        sRGB for RGB surface types.
        Valid range is: [\ref NVM_SURF_ATTR_COLOR_STD_SRGB,
                         \ref NVM_SURF_ATTR_COLOR_STD_REC2020PQ_ER]. */
    NVM_SURF_ATTR_COLOR_STD_TYPE,
    /** Specifies the maximum number of surface allocation attributes. */
    NVM_SURF_ALLOC_ATTR_MAX,
} NvMediaSurfAllocAttrType;

/** \brief NVM_SURF_ATTR_WIDTH flags.
  */
/** Minimum width of NvMedia Surface . */
#define NVM_SURF_ATTR_MIN_WIDTH (16U)
/** Maximum width of NvMedia Surface . */
#define NVM_SURF_ATTR_MAX_WIDTH (16384U)

/** \brief NVM_SURF_ATTR_HEIGHT flags.
  */
/** Minimum height of NvMedia Surface . */
#define NVM_SURF_ATTR_MIN_HEIGHT (16U)
/** Maximum height of NvMedia Surface . */
#define NVM_SURF_ATTR_MAX_HEIGHT (16384U)

/** \brief NVM_SURF_ATTR_EMB_LINES_TOP flags.
  */
/** Minimum Top Embedded Data Lines. */
#define NVM_SURF_ATTR_MIN_EMB_LINES_TOP (0U)
/** Maximum Top Embedded Data Lines. */
#define NVM_SURF_ATTR_MAX_EMB_LINES_TOP (128U)

/** \brief NVM_SURF_ATTR_EMB_LINES_BOTTOM flags.
  */
/** Minimum Bottom Embedded Data Lines. */
#define NVM_SURF_ATTR_MIN_EMB_LINES_BOTTOM (0U)
/** Maximum Bottom Embedded Data Lines. */
#define NVM_SURF_ATTR_MAX_EMB_LINES_BOTTOM (128U)

/** \brief NVM_SURF_ATTR_CPU_ACCESS flags.
  */
/** Uncached (mapped) access type flag. */
#define NVM_SURF_ATTR_CPU_ACCESS_UNCACHED                 (0x00000001u)
/** Cached (mapped) access type flag. */
#define NVM_SURF_ATTR_CPU_ACCESS_CACHED                   (0x00000002u)
/** Unmapped access type flag. */
#define NVM_SURF_ATTR_CPU_ACCESS_UNMAPPED                 (0x00000003u)

/** \brief NVM_SURF_ATTR_ALLOC_TYPE flags.
  */
/** Default buffer allocation flag. */
#define NVM_SURF_ATTR_ALLOC_DEFAULT                       (0x00000000u)
/** Isochronous buffer allocation flag. */
#define NVM_SURF_ATTR_ALLOC_ISOCHRONOUS                   (0x00000001u)
/** Secured buffer allocation flag. */
#define NVM_SURF_ATTR_ALLOC_SECURED                       (0x00000002u)

/** \brief NVM_SURF_ATTR_SCAN_TYPE flags.
  */
/** Default buffer allocation flag */
#define NVM_SURF_ATTR_ALLOC_DEFAULT                       (0x00000000u)
/** Progressive surface scan type flag. */
#define NVM_SURF_ATTR_SCAN_PROGRESSIVE                    (0x00000001u)

#if !defined(NV_IS_SAFETY) || (!NV_IS_SAFETY)
/** Interlaced surface scan type flag.
 *  Interlaced Scan Type is not supported on
 *  safety builds.
 */
#define NVM_SURF_ATTR_SCAN_INTERLACED                     (0x00000002u)
#endif

/** \brief NVM_SURF_ATTR_COLOR_STD_TYPE flags.
  */
/** sRGB Color Std flag
  * Range [RGB:0-255]
  */
#define NVM_SURF_ATTR_COLOR_STD_SRGB                      (0x00000001u)
/** YCbCr Rec.601 (Studio Range) Color Std flag
  * Range [Y:16-235 CbCr:16-240]
  */
#define NVM_SURF_ATTR_COLOR_STD_REC601_SR                 (0x00000002u)
/** YCbCr Rec.601 (Extended Range) Color Std flag
  * Range [YCbCr:0-255]
  */
#define NVM_SURF_ATTR_COLOR_STD_REC601_ER                 (0x00000003u)
/** YCbCr Rec.709 (Studio Range) Color Std flag
  * Range [Y:16-235 CbCr:16-240]
  */
#define NVM_SURF_ATTR_COLOR_STD_REC709_SR                 (0x00000004u)
/** YCbCr Rec.709 (Extended Range) Color Std flag
  * Range [YCbCr:0-255]
  */
#define NVM_SURF_ATTR_COLOR_STD_REC709_ER                 (0x00000005u)
/** RGB Rec.2020 Color Std flag
  * Range [RGB:0-1023 (10-bit),
  *            0-4095 (12-bit),
  *            0-65535(16-bit)]
  */
#define NVM_SURF_ATTR_COLOR_STD_REC2020_RGB               (0x00000006u)
/** YCbCr Rec.2020 (Studio Range) Color Std flag
  * Range [Y:64-940     CbCr:64-960    (10-bit),
  *        Y:256-3760   CbCr:256-3840  (12-bit),
  *        Y:1024-60160 CbCr:1024-61440(16-bit)]
  */
#define NVM_SURF_ATTR_COLOR_STD_REC2020_SR                (0x00000007u)
/** YCbCr Rec.2020 (Extended Range) Color Std flag
  * Range [YCbCr:0-1023 (10-bit),
  *              0-4095 (12-bit),
  *              0-65535(16-bit)]
  */
#define NVM_SURF_ATTR_COLOR_STD_REC2020_ER                (0x00000008u)
/** YcCbcCrc Rec.2020 (Studio Range) Color Std flag
  * Range [Y:64-940     CbCr:64-960    (10-bit),
  *        Y:256-3760   CbCr:256-3840  (12-bit),
  *        Y:1024-60160 CbCr:1024-61440(16-bit)]
  */
#define NVM_SURF_ATTR_COLOR_STD_YcCbcCrc_SR               (0x00000009u)
/** YcCbcCrc Rec.2020 (Extended Range) Color Std flag
  * Range [YCbCr:0-1023 (10-bit),
  *              0-4095 (12-bit),
  *              0-65535(16-bit)]
  */
#define NVM_SURF_ATTR_COLOR_STD_YcCbcCrc_ER               (0x0000000Au)

/** Sensor RGBA Color Std flag.
  * This color std flag is used to represent
  * demosaiced and white-balanced sensor bayer
  * data in linear space.
  * Range [RGBA: 0.0 -1.0 (16-bit float)]
  */
#define NVM_SURF_ATTR_COLOR_STD_SENSOR_RGBA               (0x0000000Bu)

/** YCbCr Rec.2020PQ (Extended Range) Color Std flag
  * Range [YCbCr:0-65535 (16-bit)]
  */
#define NVM_SURF_ATTR_COLOR_STD_REC2020PQ_ER              (0x0000000Cu)

/**
 * \brief Holds NvMedia Surface allocation attributes.
 */
typedef struct {
    /** \brief Holds the surface allocation attribute type. */
    NvMediaSurfAllocAttrType type;
    /** \brief Holds the surface allocation attribute value. */
    uint32_t value;
} NvMediaSurfAllocAttr;

/**
 * \brief Defines the set of NvMedia surface types.
 */
#define NvMediaSurfaceType    uint32_t

/** Unsupported types */
#define NvMediaSurfaceType_Unsupported                                (99999u)

#if (NV_IS_SAFETY == 0)
/**
 * @note The following surface type macros are deprecated and
 * will be removed in a future release.
 * Use NvMediaSurfaceFormatGetType() API instead to get the surface type.
 */
/** Obsolete Video surface for 4:2:0 format video decoders.
  The actual physical surface format is selected based
  on the chip architecture. */
#define NvMediaSurfaceType_Video_420                                  (1000u)
#define NvMediaSurfaceType_Video_420_10bit                            (1001u)
#define NvMediaSurfaceType_Video_420_12bit                            (1002u)
/** Obsolete Video surface for 4:2:2 format video decoders.
  The actual physical surface format is selected based
  on the chip architecture. */
#define NvMediaSurfaceType_Video_422                                  (1003u)
#define NvMediaSurfaceType_Video_422_10bit                            (1004u)
#define NvMediaSurfaceType_Video_422_12bit                            (1005u)
/** Obsolete Video surface for 4:4:4 format video decoders.
  The actual physical surface format is selected based
  on the chip architecture. */
#define NvMediaSurfaceType_Video_444                                  (1006u)
#define NvMediaSurfaceType_Video_444_10bit                            (1007u)
#define NvMediaSurfaceType_Video_444_12bit                            (1008u)
/** Obsolete Video capture surface for 4:2:2 format. */
#define NvMediaSurfaceType_VideoCapture_422                           (1009u)
/** Obsolete Video capture surface for YUYV format. */
#define NvMediaSurfaceType_VideoCapture_YUYV_422                      (1010u)
/** Obsolete R8G8B8A8 surface type. */
#define NvMediaSurfaceType_R8G8B8A8                                   (1011u)
/** Obsolete R8G8B8A8 surface type used for video rendering.
    Surfaces that are interacting with EGL Stream
    functions must be in this format. */
#define NvMediaSurfaceType_R8G8B8A8_BottomOrigin                      (1012u)
/** Obsolete Monochrome image. */
#define NvMediaSurfaceType_Image_Monochrome                           (1013u)
/** Obsolete 4:2:0 format image. */
#define NvMediaSurfaceType_Image_YUV_420                              (1014u)
/** Obsolete 4:2:2 format image. */
#define NvMediaSurfaceType_Image_YUV_422                              (1015u)
/** Obsolete 4:4:4 format image. */
#define NvMediaSurfaceType_Image_YUV_444                              (1016u)
/** Obsolete 4:2:2 format packed image with YUYV component order. */
#define NvMediaSurfaceType_Image_YUYV_422                             (1017u)
/** Obsolete RGBA image type */
#define NvMediaSurfaceType_Image_RGBA                                 (1018u)
/** Obsolete RAW image type */
#define NvMediaSurfaceType_Image_RAW                                  (1019u)
/** Obsolete 4:4:4:4 format packed image with VYUX component order. */
#define NvMediaSurfaceType_Image_V16Y16U16X16                         (1020u)
/** Obsolete 16-bit Y data image. */
#define NvMediaSurfaceType_Image_Y16                                  (1021u)
/** Obsolete 4:4:4:4 format packed image with XUYV component order. */
#define NvMediaSurfaceType_Image_X2U10Y10V10                          (1022u)
/** Obsolete 4:2:0 semi-planar YUV */
#define NvMediaSurfaceType_Image_Y10U8V8_420                          (1023u)
/** Obsolete 10-bit Y data image. */
#define NvMediaSurfaceType_Image_Y10                                  (1024u)
/** Obsolete A8 alpha surface */
#define NvMediaSurfaceType_A8                                         (1025u)

/** Obsolete 4:2:0 video surface type */
#define NvMediaSurfaceType_YV12 NvMediaSurfaceType_Video_420
/** Obsolete 4:2:2 video surface type */
#define NvMediaSurfaceType_YV16 NvMediaSurfaceType_Video_422
/** Obsolete 4:4:4 video surface type */
#define NvMediaSurfaceType_YV24 NvMediaSurfaceType_Video_444
/** Obsolete 4:2:2 video capture surface type */
#define NvMediaSurfaceType_YV16x2 NvMediaSurfaceType_VideoCapture_422
#endif

/**
 * \brief Gets \ref NvMediaSurfaceType for the input \ref NvMediaSurfFormatAttr.
 *
 * This API takes surface format attributes of a given surface as input parameters and
 * returns the surface type of the corresponding surface.
 *
 * \param[in] attrs Pointer to an array of surface format attributes. It should be a valid
 *                  non-null pointer.
 * \param[in] numAttrs Number of attributes in the array.
 *            The application has to pass \ref NVM_SURF_FMT_ATTR_MAX number of attributes.
 * \return \ref NvMediaSurfaceType for the input attributes.
 *         \ref NvMediaSurfaceType_Unsupported if unsuccessful.
 */
NvMediaSurfaceType
NvMediaSurfaceFormatGetType(
    const NvMediaSurfFormatAttr *attrs,
    uint32_t numAttrs
);

/**
 * \brief Gets \ref NvMediaSurfFormatAttr for the input surface type.
 * \param[in] type \ref NvMediaSurfaceType of the surface.
 *                  This must be a valid value returned by
 *                  NvMediaSurfaceFormatGetType().
 * \param[in] attrs A pointer to array of \ref NvMediaSurfFormatAttr.
 *                  The attribute value is returned for the input attribute
 *                  type \ref NvMediaSurfFormatAttrType for each array element.
 *                  It should be a valid non-null pointer.
 * \param[in] numAttrs Number of attributes in the array.
 *            Must be a valid value in range [1, \ref NVM_SURF_FMT_ATTR_MAX].
 * \return \ref NvMediaStatus The completion status of the operation.
 * Possible values are:
 * \n \ref NVMEDIA_STATUS_OK : On Successful completion
 * \n \ref NVMEDIA_STATUS_ERROR : On invalid \ref NvMediaSurfFormatAttrType
 * \n \ref NVMEDIA_STATUS_NOT_SUPPORTED : On invalid \ref NvMediaSurfaceType
 */
NvMediaStatus
NvMediaSurfaceFormatGetAttrs(
    NvMediaSurfaceType type,
    NvMediaSurfFormatAttr *attrs,
    uint32_t numAttrs
);

/**
 * \brief Gets the surface version information for the NvMediaSurface component
 * \param[in] version A pointer to an \ref NvMediaVersion structure
 *                    to be filled by the NvMediaSurface component.
 *                    It should be a valid non-null pointer.
 * \return  NVMEDIA_STATUS_OK if the operation was successful, or
 *  NVMEDIA_STATUS_BAD_PARAMETER if @a version was invalid
 *                               or if a NULL Pointer is provided as input argument.
 */
NvMediaStatus
NvMediaSurfaceGetVersion(
    NvMediaVersion *version
);

/*
 * \defgroup history_nvmedia_surface History
 * Provides change history for the NvMedia Surface API.
 *
 * \section history_nvmedia_surface Version History
 *
 * <b> Version 1.0 </b> March 1, 2017
 * - Initial release
 *
 *
 * <b> Version 1.1 </b> April 24, 2017
 * - Added NVM_SURF_ATTR_COMPONENT_ORDER_XYUV and
 *   NVM_SURF_ATTR_COMPONENT_ORDER_RG component order flags
 *
 * <b> Version 1.2 </b> May 18, 2017
 * - Added NVM_SURF_ATTR_COLOR_STD_TYPE and
 *   NVM_SURF_ATTR_COLOR_STD flags
 *
 * <b> Version 1.3 </b> June 08, 2017
 * - Removed NvMediaSurfaceType_Image_NonColor_S16_XY and
 *   NvMediaSurfaceType_Image_NonColor_S16_X surface types
 *
 * <b> Version 1.4 </b> June 12, 2017
 * - Added NVM_SURF_ATTR_COMPONENT_ORDER_VUYX flag
 *
 * <b> Version 1.5 </b> October 09, 2017
 * - Added NVM_SURF_ATTR_COLOR_STD_SENSOR_RGBA and
 *   NVM_SURF_ATTR_COLOR_STD_REC2020PQ_ER flags
 *
 * <b> Version 1.6 </b> October 31, 2017
 * - Added NVM_SURF_ATTR_DATA_TYPE_FLOATISP type
 *
 * <b> Version 1.7 </b> November 07, 2017
 * - Added NVM_SURF_ATTR_BITS_PER_COMPONENT_20
 *
 * <b> Version 1.8 </b> June 04, 2018
 * - Added NVM_SURF_ATTR_COMPONENT_ORDER_CCCC component order
 *
 * <b> Version 1.9 </b> July 10, 2018
 * - Added NvMediaSurfaceGetVersion API
 *
 * <b> Version 1.10 </b> December 11, 2018
 * - Fixed MISRA-C rule 10.4, 20.7 and 21.1 violations
 *   resulting from this header.
 *
 * <b> Version 1.11 </b> December 03, 2019
 * - Updated the comments to deprecate old NvMediaSurfaceType_ macros
 *
 * <b> Version 1.12 </b> March 22, 2019
 * - Fixed MISRA-C rule 8.13 violations
 *   resulting from this header.
 *
 * <b> Version 1.13 </b> November 27, 2019
 * - Fixed MISRA-C rule 4.6 violations by changing unsigned int
 *   arguments and struct members to uint32_t type.
 *
 * <b> Version 1.14 </b> December 4, 2019
 * - Fixed MISRA-C rule 2.5 violations by defining non-safety
 *   surface types under non-safety conditional compilation macro.
 */
/**@}*/

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_SURFACE_H */
