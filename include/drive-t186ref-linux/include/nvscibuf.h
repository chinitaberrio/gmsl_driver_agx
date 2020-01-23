/*
 * Header file for NvSciBuf APIs
 *
 * Copyright (c) 2018-2019, NVIDIA CORPORATION. All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
/**
 * @file
 *
 * @brief <b> NVIDIA Software Communications Interface (SCI) : NvSciBuf </b>
 *
 * Allows applications to allocate and exchange buffers in memory.
 */
#ifndef INCLUDED_NVSCIBUF_H
#define INCLUDED_NVSCIBUF_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "nvscierror.h"
#include <nvsciipc.h>

#if defined(__cplusplus)
extern "C"
{
#endif
/**
 * @defgroup nvsci_buf Buffer Allocation APIs
 *
 * The NvSciBuf library contains the APIs for applications to allocate
 * and exchange buffers in memory.
 *
 * @ingroup nvsci_group_stream
 * @{
 */
/**
 * @defgroup nvscibuf_datatype NvSciBuf Datatype Definitions
 * Contains a list of all NvSciBuf datatypes.
 * @{
 */

/**
 * @brief Enum definitions of NvSciBuf Buffer datatype.
 */
typedef enum {
    /*! Reserved for General keys.
     *! Shouldn't be used as valid value for  NvSciBufGeneralAttrKey_Types. */
    NvSciBufType_General,
    NvSciBufType_RawBuffer,
    NvSciBufType_Image,
    NvSciBufType_Tensor,
    NvSciBufType_Array,
    NvSciBufType_Pyramid,
    NvSciBufType_MaxValid,
    NvSciBufType_UpperBound
} NvSciBufType;

/**
 * @}
 */

/**
 * @defgroup nvscibuf_constants NvSciBuf Global Constants
 * Definitions of all NvSciBuf Global Constants/Macros
 *
 * @{
 */
/**
 * @brief API Major version number.
 */
static const uint32_t NvSciBufMajorVersion = 1U;

/**
 * @brief API Minor version number.
 */
static const uint32_t NvSciBufMinorVersion = 0U;

#if defined(__cplusplus)

/**
 * @brief Maximum number of dimensions supported by tensor datatype.
 */
static const int NV_SCI_BUF_TENSOR_MAX_DIMS = 8;

/**
 * @brief Maximum number of planes supported by image datatype.
 */
static const int NV_SCI_BUF_IMAGE_MAX_PLANES = 3;

/**
 * @brief Maximum number of levels supported by pyramid datatype.
 */
static const int NV_SCI_BUF_PYRAMID_MAX_LEVELS = 10;

/**
 * @brief Indicates the size of export descriptor.
 */
static const int NVSCIBUF_EXPORT_DESC_SIZE = 32;

/**
 * @brief Indicates number of bits used for defining an attribute key.
 * Note: Maximum 16K attribute Keys per datatype.
 */
static const int NV_SCI_BUF_ATTRKEY_BIT_COUNT = 16;

/**
 * @brief Indicates number of bits used for defining an datatype of a key.
 * Note: Maximum 1K datatypes.
 */
static const int NV_SCI_BUF_DATATYPE_BIT_COUNT = 10;

/**
 * @brief Indicates the attribute key is a public key type.
 */
static const int NV_SCI_BUF_ATTR_KEY_TYPE_PUBLIC = 0;

/*
 * @brief Global constant to specify the start-bit of attribute Keytype.
 */
static const int NV_SCI_BUF_KEYTYPE_BIT_START =
        (NV_SCI_BUF_DATATYPE_BIT_COUNT + NV_SCI_BUF_ATTRKEY_BIT_COUNT);

/**
 * @brief Indicates starting value of General attribute keys.
 */
static const int NV_SCI_BUF_GENERAL_ATTR_KEY_START =
           (NV_SCI_BUF_ATTR_KEY_TYPE_PUBLIC << NV_SCI_BUF_KEYTYPE_BIT_START) |
           (NvSciBufType_General << NV_SCI_BUF_ATTRKEY_BIT_COUNT);

/**
 * @brief Indicates the start of Raw-buffer Datatype keys.
 */
static const int NV_SCI_BUF_RAW_BUF_ATTR_KEY_START =
           (NV_SCI_BUF_ATTR_KEY_TYPE_PUBLIC << NV_SCI_BUF_KEYTYPE_BIT_START) |
           (NvSciBufType_RawBuffer << NV_SCI_BUF_ATTRKEY_BIT_COUNT);

/**
 * @brief Indicates the start of Image Datatype keys.
 */
static const int NV_SCI_BUF_IMAGE_ATTR_KEY_START =
           (NV_SCI_BUF_ATTR_KEY_TYPE_PUBLIC << NV_SCI_BUF_KEYTYPE_BIT_START) |
           (NvSciBufType_Image << NV_SCI_BUF_ATTRKEY_BIT_COUNT);
/**
 * @brief Indicates the start of ImagePyramid Datatype keys.
 */
static const int NV_SCI_BUF_PYRAMID_ATTR_KEY_START =
           (NV_SCI_BUF_ATTR_KEY_TYPE_PUBLIC << NV_SCI_BUF_KEYTYPE_BIT_START) |
           (NvSciBufType_Pyramid << NV_SCI_BUF_ATTRKEY_BIT_COUNT);

/**
 * @brief Indicates the start of NvSciBuf Array Datatype keys.
 */
static const int NV_SCI_BUF_ARRAY_ATTR_KEY_START =
           (NV_SCI_BUF_ATTR_KEY_TYPE_PUBLIC << NV_SCI_BUF_KEYTYPE_BIT_START) |
           (NvSciBufType_Array << NV_SCI_BUF_ATTRKEY_BIT_COUNT);

/**
 * @brief Indicates the start of Tensor Datatype keys.
 */
static const int NV_SCI_BUF_TENSOR_ATTR_KEY_START =
           (NV_SCI_BUF_ATTR_KEY_TYPE_PUBLIC << NV_SCI_BUF_KEYTYPE_BIT_START) |
           (NvSciBufType_Tensor << NV_SCI_BUF_ATTRKEY_BIT_COUNT);

#else

/**
 * @brief Maximum number of dimensions supported by tensor datatype.
 */
#define NV_SCI_BUF_TENSOR_MAX_DIMS  8u

/**
 * @brief Maximum number of planes supported by image datatype.
 */
#define NV_SCI_BUF_IMAGE_MAX_PLANES 3u

/**
 * @brief Maximum number of levels supported by pyramid datatype.
 */
#define NV_SCI_BUF_PYRAMID_MAX_LEVELS 10u

/**
 * @brief Indicates the size of export descriptor.
 */
#define NVSCIBUF_EXPORT_DESC_SIZE   32u

/**
 * @brief Indicates number of bits used for defining an attribute key.
 * Note: Maximum 16K attribute Keys per datatype.
 */
#define NV_SCI_BUF_ATTRKEY_BIT_COUNT  16u

/**
 * @brief Indicates number of bits used for defining datatype of a key.
 * Note: Maximum 1K datatypes.
 */
#define NV_SCI_BUF_DATATYPE_BIT_COUNT  10u

/**
 * @brief Indicates the attribute key is a public key type.
 */
#define NV_SCI_BUF_ATTR_KEY_TYPE_PUBLIC 0

/**
 * @brief Global constant to specify the start-bit of attribute Keytype.
 */
#define NV_SCI_BUF_KEYTYPE_BIT_START \
        (NV_SCI_BUF_DATATYPE_BIT_COUNT + NV_SCI_BUF_ATTRKEY_BIT_COUNT)

/**
 * @brief Indicates starting value of General attribute keys.
 */
#define NV_SCI_BUF_GENERAL_ATTR_KEY_START \
        (NV_SCI_BUF_ATTR_KEY_TYPE_PUBLIC << NV_SCI_BUF_KEYTYPE_BIT_START) | \
        (NvSciBufType_General << NV_SCI_BUF_ATTRKEY_BIT_COUNT)

/**
 * @brief Indicates the start of Raw-buffer Datatype keys.
 */
#define NV_SCI_BUF_RAW_BUF_ATTR_KEY_START \
          (NV_SCI_BUF_ATTR_KEY_TYPE_PUBLIC << NV_SCI_BUF_KEYTYPE_BIT_START) | \
          (NvSciBufType_RawBuffer << NV_SCI_BUF_ATTRKEY_BIT_COUNT)

/**
 * @brief Indicates the start of Image Datatype keys.
 */
#define NV_SCI_BUF_IMAGE_ATTR_KEY_START \
          (NV_SCI_BUF_ATTR_KEY_TYPE_PUBLIC << NV_SCI_BUF_KEYTYPE_BIT_START) | \
          (NvSciBufType_Image << NV_SCI_BUF_ATTRKEY_BIT_COUNT)

/**
 * @brief Indicates the start of ImagePyramid Datatype keys.
 */
#define NV_SCI_BUF_PYRAMID_ATTR_KEY_START \
          (NV_SCI_BUF_ATTR_KEY_TYPE_PUBLIC << NV_SCI_BUF_KEYTYPE_BIT_START) | \
          (NvSciBufType_Pyramid << NV_SCI_BUF_ATTRKEY_BIT_COUNT)

/**
 * @brief Indicates the start of NvSciBuf Array Datatype keys.
 */
#define NV_SCI_BUF_ARRAY_ATTR_KEY_START \
          (NV_SCI_BUF_ATTR_KEY_TYPE_PUBLIC << NV_SCI_BUF_KEYTYPE_BIT_START) | \
          (NvSciBufType_Array << NV_SCI_BUF_ATTRKEY_BIT_COUNT)

/**
 * @brief Macro to specify the start of Tensor Datatype keys.
 */
#define NV_SCI_BUF_TENSOR_ATTR_KEY_START \
          (NV_SCI_BUF_ATTR_KEY_TYPE_PUBLIC << NV_SCI_BUF_KEYTYPE_BIT_START) | \
          (NvSciBufType_Tensor << NV_SCI_BUF_ATTRKEY_BIT_COUNT)

#endif

/**
 * @}
 */

/**
 * @defgroup nvscibuf_attr_key NvSciBuf Enumerations for Attribute Keys
 * List of all NvSciBuf enumerations for attribute keys.
 * @{
 */

/**
 * Describes the public attribute keys.
 */
typedef enum {
    /**
     * Specifies the lower bound value to check for a valid NvSciBuf attribute
     * key type.
     */
    NvSciBufAttrKey_LowerBound =         NV_SCI_BUF_GENERAL_ATTR_KEY_START,

    /** Specifies the buffer type that the attribute list is expected to have.
     * For each type the buffer has, the associated attributes are valid.
     *
     * Type: Input attribute
     *
     * Datatype: @ref NvSciBufType
     */
    NvSciBufGeneralAttrKey_Types,

    /** Specifies if CPU access is required. If this attribute is set to @c true,
     * then the CPU will be able to obtain a pointer to the buffer
     * from NvSciBufObjGetCpuPtr().
     *
     * Type: Input attribute
     *
     * Datatype: @c bool
     */
    NvSciBufGeneralAttrKey_NeedCpuAccess,

    /** Specifies which access permissions are required.
     * If @ref NvSciBufGeneralAttrKey_NeedCpuAccess is @c true, the pointer
     * returned by NvSciBufObjGetCpuPtr() will provide these
     * permissions.
     * Any hardware accelerators that contribute to this attribute list
     * will be offered at least
     * these permissions.
     *
     * Type: Input attribute
     *
     * Datatype: @ref NvSciBufAttrValAccessPerm
     */
    NvSciBufGeneralAttrKey_RequiredPerm,

    /** Specifies whether to enable/disable CPU caching.
     * If set to @c true:
     *
     *   The CPU must perform write-back caching of the buffer to the greatest
     *   extent possible considering all the CPUs that are sharing the buffer.
     *    Coherency is guaranteed with:
     *         - Other CPU accessors.
     *         - All I/O-Coherent accessors that do not have CPU-invisible
     *           caches.
     *
     * If set to @c false:
     *
     * The CPU must not access the caches at all on read or write accesses
     *  to the buffer from applications.
     *   Coherency is guaranteed with:
     *         - Other CPU accessors.
     *         - All I/O accessors (whether I/O-coherent or not) that do not
     *              have CPU-invisible caches.
     *
     * Type: Input attribute
     *
     * Datatype: @c bool
     */
    NvSciBufGeneralAttrKey_EnableCpuCache,

    /** GpuIDs of the GPUs in the system that will access the buffer.
     *    In a multi GPU System, if multiple GPUs are supposed to access
     *    the buffer, then provide the GPU IDs of all the GPUs that
     *    need to access the buffer. The GPU which is not specified in the
     *    list of GPUIDs may not be able to access the buffer.
     *
     * Type: Input attribute
     *
     * Datatype: @c uint64_t[]
     */
    NvSciBufGeneralAttrKey_GpuId,

    /** Indicates whether the CPU is required to flush before reads and
     * after writes. This can be accomplished using NvSciBufObjFlushRange(),
     * or (if the application prefers) with OS-specific flushing functions.
     *
     * Type: Output attribute
     *
     * Datatype: @c bool
     */
    NvSciBufGeneralAttrKey_CpuNeedSwCacheCoherency,

    /** Specifies the access permissions that are provided.
     *  This key can only be used after reconciliation to find out the
     *  access permissions provided to the ipcEndpoint which has
     *  imported this attribute list.
     * Any hardware accelerators that contribute to this attribute list
     * will be offered at least
     * these permissions.
     *
     * Type: Output attribute
     *
     * Datatype: NvSciBufAttrValAccessPerm
     */
    NvSciBufGeneralAttrKey_ActualPerm,

    /** Specifies the size of the buffer to allocate.
     *
     * Type: Input attribute
     *
     * Datatype: @c uint64_t
     */
    NvSciBufRawBufferAttrKey_Size   =  NV_SCI_BUF_RAW_BUF_ATTR_KEY_START,

    /** Specifies the alignment requirement of raw buffer.
     *
     * Type: Input attribute
     *
     * Datatype: @c uint64_t
     */
    NvSciBufRawBufferAttrKey_Align,

    /** Specifies the layout: Block-linear or Pitch-linear.
     *
     * Type: Input/Output attribute
     *
     * Datatype: @ref NvSciBufAttrValImageLayoutType
     */
    NvSciBufImageAttrKey_Layout   =    NV_SCI_BUF_IMAGE_ATTR_KEY_START,

    /** Specifies the top padding for the image.
     *
     * Type: Input/Output attribute
     *
     * Datatype: @c uint64_t
     */
    NvSciBufImageAttrKey_TopPadding,

    /** Specifies the bottom padding for the image.
     *
     * Type: Input/Output attribute
     *
     * Datatype: uint64_t
     */
    NvSciBufImageAttrKey_BottomPadding,

    /** Specifies the left padding for the image.
     *
     * Type: Input/Output attribute
     *
     * Datatype: @c uint64_t
     */
    NvSciBufImageAttrKey_LeftPadding,

    /** Specifies the right padding for the image.
     *
     * Type: Input/Output attribute
     *
     * Datatype: @c uint64_t
     */
    NvSciBufImageAttrKey_RightPadding,

    /** Specifies the VPR flag for the image.
     *
     * Type: Input attribute
     *
     * Datatype: @c bool
     */
    NvSciBufImageAttrKey_VprFlag,

    /** Indicates the size of the image.
     *
     * Type: Output attribute
     *
     * Datatype: @c uint64_t
     */
    NvSciBufImageAttrKey_Size,

    /** Indicates the alignment for the image.
     *
     * Type: Output attribute
     *
     * Datatype: @c uint64_t
     */
    NvSciBufImageAttrKey_Alignment,

    /** Specifies the number of image planes.
     *
     * Type: Input/Output attribute
     *
     * Datatype: @c uint32_t
     */
    NvSciBufImageAttrKey_PlaneCount,

    /** Specifies the color format of the image plane.
     *
     * Type: Input/Output attribute
     *
     * Dataype: @ref NvSciBufAttrValColorFmt[]
     */
    NvSciBufImageAttrKey_PlaneColorFormat,

    /** Specifies a set of plane color standards.
     *
     * Type: Input/Output attribute
     *
     * Dataype: @ref NvSciBufAttrValColorStd[]
     */
    NvSciBufImageAttrKey_PlaneColorStd,

    /** Specifies the image plane base address alignment.
     *
     * Type: Input/Output attribute
     *
     * Datatype: @c int32_t[]
     */
    NvSciBufImageAttrKey_PlaneBaseAddrAlign,

    /** Specifies the image plane width in pixels.
     *
     * Type: Input/Output attribute
     *
     * Datatype: @c int32_t[]
     */
    NvSciBufImageAttrKey_PlaneWidth,

    /** Specifies the image plane height in pixels.
     *
     * Type: Input/Output attribute
     *
     * Datatype: @c int32_t[]
     */
    NvSciBufImageAttrKey_PlaneHeight,

    /** Specifies the image scan type: Progressive or Interlaced.
     *
     * Type: Input attribute
     *
     * Datatype: @ref NvSciBufAttrValImageScanType
     */
    NvSciBufImageAttrKey_PlaneScanType,
    NvSciBufImageAttrKey_ScanType = NvSciBufImageAttrKey_PlaneScanType,

    /** Indicates the image plane in bits per pixel.
     *
     * Type: Output attribute
     *
     * Datatype: @c uint32_t[]
     */
    NvSciBufImageAttrKey_PlaneBitsPerPixel,

    /** Indicates the starting offset of the image plane.
     *
     * Type: Output attribute
     *
     * Datatype: @c uint64_t[]
     */
    NvSciBufImageAttrKey_PlaneOffset,

    /** Indicates the color format of the image plane.
     *
     * Type: Output attribute
     *
     * Datatype: @ref NvSciBufAttrValDataType
     */
    NvSciBufImageAttrKey_PlaneDatatype,

    /** Indicates the channel count in the image plane.
     *
     * Type: Output attribute
     *
     * Datatype: @c uint8_t
     */
    NvSciBufImageAttrKey_PlaneChannelCount,

    /** Indicates the offset of the start of the second field, 0 for progressive
     * valid for interlaced.
     *
     * Type: Output attribute
     *
     * Datatype: @c int64_t[]
     */
    NvSciBufImageAttrKey_PlaneSecondFieldOffset,

    /** Indicates the pitch of the image plane.
     *
     * Type: Output attribute
     *
     * Datatype: @c int32_t[]
     */
    NvSciBufImageAttrKey_PlanePitch,

    /** Indicates the aligned height of the image plane.
     *
     * Type: Output attribute
     *
     * Datatype: int32_t[]
     */
    NvSciBufImageAttrKey_PlaneAlignedHeight,

    /** Indicates the aligned size of the image plane.
     *
     * Type: Output attribute
     *
     * Datatype: @c int64_t[]
     */
    NvSciBufImageAttrKey_PlaneAlignedSize,

    /** Specifies the tensor data type.
     *
     * Type: Input attribute
     *
     * Datatype: @ref NvSciBufAttrValDataType
     */
    NvSciBufTensorAttrKey_DataType  =  NV_SCI_BUF_TENSOR_ATTR_KEY_START,

    /** Specifies the number of tensor dimensions. A maximum of 8 dimensions are allowed.
     *
     * Type: Input attribute
     *
     * Datatype: @c int32_t
     */
    NvSciBufTensorAttrKey_NumDims,

    /** Specifies the size of each tensor dimension.
     * This attribute takes size value in terms of an array.
     * @note Array indices are not tied to the semantics of the dimension.
     * NvSciBuf is not affected by the dimension order.
     *
     * Type: Input attribute
     *
     * Datatype: @c uint64_t[]
     */
    NvSciBufTensorAttrKey_SizePerDim,

    /** Specifies the alignment constraints per tensor dimension.
     *
     * Type: Input attribute
     *
     * Datatype: @c uint32_t[]
     */
    NvSciBufTensorAttrKey_AlignmentPerDim,

    /** Returns the stride value (in bytes) for each tensor dimension.
     * @note The returned array contains stride values in decreasing order.
     * In other words, the index @em 0 of the array will have the largest
     * stride while [@em number-of-dims - 1] index will have the smallest stride.
     *
     * Type: Output attribute
     *
     * Datatype: @c uint64_t[]
     */
    NvSciBufTensorAttrKey_StridesPerDim,

    /** Specifies the data type of a buffer array.
     *
     * Type: Input attribute
     *
     * Datatype: @ref NvSciBufAttrValDataType
     */
    NvSciBufArrayAttrKey_DataType   =  NV_SCI_BUF_ARRAY_ATTR_KEY_START,

    /** Specifies the stride of each element in the buffer array.
     * Stride must be less than the size of the array data type.
     *
     * Type: Input attribute
     *
     * Datatype: @c uint64_t
     */
    NvSciBufArrayAttrKey_Stride,

    /** Specifies the buffer array capacity.
     *
     * Type: Input attribute
     *
     * Datatype: @c uint64_t
     */
    NvSciBufArrayAttrKey_Capacity,

    /** Indicates the total size of a buffer array.
     *
     * Type: Output attribute
     *
     * Datatype: @c uint64_t
     */
    NvSciBufArrayAttrKey_Size,

    /** Indicates the base alignment of a buffer array.
     *
     * Type: Output attribute
     *
     * Datatype: @c uint64_t
     */
    NvSciBufArrayAttrKey_Alignment,

    /** Specifies the number of levels of images in a pyramid.
     *
     * Type: Input attribute
     *
     * Datatype: @c uint32_t
     */
    NvSciBufPyramidAttrKey_NumLevels  =  NV_SCI_BUF_PYRAMID_ATTR_KEY_START,

    /** Specifies the scaling factor by which each successive image in a
     * pyramid must be scaled. Value must be 0 < @em scale <= 1.
     *
     * Type: Input attribute
     *
     * Datatype: @c float
     */
    NvSciBufPyramidAttrKey_Scale,

    /** NvSciBuf allocates the total buffer size considering all levels in an
     * image pyramid and returns an array of buffer offsets for each level.
     *
     * Type: Output attribute
     *
     * Datatype: @c uint64_t[]
     */
    NvSciBufPyramidAttrKey_LevelOffset,

    /** NvSciBuf allocates the total buffer size considering all levels in an
     * image pyramid and returns an array of buffer sizes for each level.
     *
     * Type: Output attribute
     *
     * Datatype: @c uint64_t[]
     */
    NvSciBufPyramidAttrKey_LevelSize,

    /** Specifies the maximum number of NvSciBuf attribute keys.
     * The total space for keys is 32K.
     *
     * Datatype: None
     */
    NvSciBufAttrKey_UpperBound,

} NvSciBufAttrKey;

/**
 * @}
 */

/**
 * @addtogroup nvscibuf_datatype
 * @{
 */

/**
 * @brief Defines access permissions for NvSciBuf.
 */
typedef enum {
    NvSciBufAccessPerm_Readonly,
    NvSciBufAccessPerm_ReadWrite,
    NvSciBufAccessPerm_Invalid,
} NvSciBufAttrValAccessPerm;

/**
 * @brief Defines the image layout type for NvSciBuf.
 */
typedef enum {
    NvSciBufImage_BlockLinearType,
    NvSciBufImage_PitchLinearType,
} NvSciBufAttrValImageLayoutType;

/**
 * @brief Defines the image scan type for NvSciBuf.
 */
typedef enum {
    NvSciBufScan_ProgressiveType,
    NvSciBufScan_InterlaceType,
} NvSciBufAttrValImageScanType;

/**
 * @brief Defines the image color formats for NvSciBuf.
 */
typedef enum {
    NvSciColor_LowerBound,
    /* RAW PACKED */
    NvSciColor_Bayer8RGGB,
    NvSciColor_Bayer8CCCC,
    NvSciColor_Bayer8BGGR,
    NvSciColor_Bayer8GBRG,
    NvSciColor_Bayer8GRBG,
    NvSciColor_Bayer16BGGR,
    NvSciColor_Bayer16CCCC,
    NvSciColor_Bayer16GBRG,
    NvSciColor_Bayer16GRBG,
    NvSciColor_Bayer16RGGB,
    NvSciColor_X2Bayer14GBRG,
    NvSciColor_X4Bayer12GBRG,
    NvSciColor_X6Bayer10GBRG,
    NvSciColor_X2Bayer14GRBG,
    NvSciColor_X4Bayer12GRBG,
    NvSciColor_X6Bayer10GRBG,
    NvSciColor_X2Bayer14BGGR,
    NvSciColor_X4Bayer12BGGR,
    NvSciColor_X6Bayer10BGGR,
    NvSciColor_X2Bayer14RGGB,
    NvSciColor_X4Bayer12RGGB,
    NvSciColor_X6Bayer10RGGB,
    NvSciColor_X2Bayer14CCCC,
    NvSciColor_X4Bayer12CCCC,
    NvSciColor_X6Bayer10CCCC,
    NvSciColor_Signed_X2Bayer14CCCC,
    NvSciColor_Signed_X4Bayer12CCCC,
    NvSciColor_Signed_X6Bayer10CCCC,
    NvSciColor_Signed_Bayer16CCCC,
    NvSciColor_FloatISP_Bayer16CCCC,
    NvSciColor_FloatISP_Bayer16RGGB,
    NvSciColor_FloatISP_Bayer16BGGR,
    NvSciColor_FloatISP_Bayer16GRBG,
    NvSciColor_FloatISP_Bayer16GBRG,
    NvSciColor_X12Bayer20CCCC,
    NvSciColor_X12Bayer20BGGR,
    NvSciColor_X12Bayer20RGGB,
    NvSciColor_X12Bayer20GRBG,
    NvSciColor_X12Bayer20GBRG,
    NvSciColor_Signed_X12Bayer20CCCC,
    NvSciColor_Signed_X12Bayer20GBRG,

    /* Semiplanar formats */
    NvSciColor_U8V8,
    NvSciColor_U8_V8,
    NvSciColor_V8U8,
    NvSciColor_V8_U8,
    NvSciColor_U10V10,
    NvSciColor_V10U10,
    NvSciColor_U12V12,
    NvSciColor_V12U12,
    NvSciColor_U16V16,
    NvSciColor_V16U16,

    /* PLANAR formats */
    NvSciColor_Y8,
    NvSciColor_Y10,
    NvSciColor_Y12,
    NvSciColor_Y16,
    NvSciColor_U8,
    NvSciColor_V8,
    NvSciColor_U10,
    NvSciColor_V10,
    NvSciColor_U12,
    NvSciColor_V12,
    NvSciColor_U16,
    NvSciColor_V16,

    /* Packed YUV formats */
    NvSciColor_A8Y8U8V8,
    NvSciColor_Y8U8Y8V8,
    NvSciColor_Y8V8Y8U8,
    NvSciColor_U8Y8V8Y8,
    NvSciColor_V8Y8U8Y8,
    NvSciColor_A16Y16U16V16,

    /* RGBA PACKED */
    NvSciColor_A8,
    NvSciColor_Signed_A8,
    NvSciColor_B8G8R8A8,
    NvSciColor_A8R8G8B8,
    NvSciColor_A8B8G8R8,
    NvSciColor_A2R10G10B10,
    NvSciColor_A16,
    NvSciColor_Signed_A16,
    NvSciColor_Signed_R16G16,
    NvSciColor_A16B16G16R16,
    NvSciColor_Signed_A16B16G16R16,
    NvSciColor_Float_A16B16G16R16,
    NvSciColor_A32,
    NvSciColor_Signed_A32,

    NvSciColor_UpperBound
} NvSciBufAttrValColorFmt;

/**
 * @brief Defines the image color standard for NvSciBuf.
 */
typedef enum {
    NvSciColorStd_SRGB,
    NvSciColorStd_REC601_SR,
    NvSciColorStd_REC601_ER,
    NvSciColorStd_REC709_SR,
    NvSciColorStd_REC709_ER,
    NvSciColorStd_REC2020_RGB,
    NvSciColorStd_REC2020_SR,
    NvSciColorStd_REC2020_ER,
    NvSciColorStd_YcCbcCrc_SR,
    NvSciColorStd_YcCbcCrc_ER,
    NvSciColorStd_SENSOR_RGBA,
    NvSciColorStd_REQ2020PQ_ER,
} NvSciBufAttrValColorStd;

/**
 * @brief Defines various numeric datatypes for NvSciBuf.
 */
typedef enum {
    NvSciDataType_Int4,
    NvSciDataType_Uint4,
    NvSciDataType_Int8,
    NvSciDataType_Uint8,
    NvSciDataType_Int16,
    NvSciDataType_Uint16,
    NvSciDataType_Int32,
    NvSciDataType_Uint32,
    NvSciDataType_Float16,
    NvSciDataType_Float32,
    NvSciDataType_FloatISP,
    NvSciDataType_Bool,
    NvSciDataType_UpperBound
} NvSciBufAttrValDataType;


/**
 * @}
 */

/**
 * @defgroup nvscibuf_attr_datastructures NvSciBuf Data Structures
 * Specifies NvSciBuf data structures.
 * @{
 */

/**
 * @brief Holds a pointer to NvSciBufModuleRec.
 * Any @ref NvSciBufAttrList created or imported using a particular @ref NvSciBufModule
 * is bound to that module instance, along with any @ref NvSciBufObj objects created or
 * imported using those @ref NvSciSyncAttrList objects.
 *
 * @note For any NvSciBuf API call that has more than one input of type
 * NvSciBufModule, NvSciBufAttrList, and/or NvSciBufObj, all such inputs
 * must agree on the NvSciBufModule instance.
 */
typedef struct NvSciBufModuleRec* NvSciBufModule;

/**
 * @brief Defines a key/value pair of attribute to be set.
 *
 * @note An array of key/value pairs need to be set
 * in order to allocate a buffer.
 */
typedef struct {
      NvSciBufAttrKey key;
      const void* value;
      size_t len;
} NvSciBufAttrKeyValuePair;

/**
 * @brief Holds a pointer to NvSciBufObjRefRec.
 *
 * @note Every @ref NvSciBufObj that has been created but not freed
 * holds a reference to the @ref NvSciBufModule, preventing the module
 * from being de-initialized.
 */
typedef struct NvSciBufObjRefRec* NvSciBufObj;

/**
 * @brief Holds a pointer to NvSciBufAttrListRec.
 *
 * @note Every @ref NvSciBufAttrList that has been created but not freed
 * holds a reference to the @ref NvSciBufModule, preventing the module
 * from being de-initialized.
 */
typedef struct NvSciBufAttrListRec* NvSciBufAttrList;

/**
 * @brief Defines the export data (blob) for @ref NvSciBufObj.
 */
typedef struct {
      uint64_t data[NVSCIBUF_EXPORT_DESC_SIZE];
} __attribute__((packed)) NvSciBufObjIpcExportDescriptor;

/**
 * @}
 */

/**
 * @defgroup nvscibuf_attr_list_api NvSciBuf Attribute List APIs
 * Methods to perform operations on NvSciBuf attribute lists.
 * @{
 */

/**
 * @brief Creates an attribute list holding the attributes
 * of the NvSciBufObj to be allocated.
 *
 * @param[in] module NvSciBufModule to associate with the newly created NvSciBufAttrList.
 * @param[out] newAttrList A pointer to the NvSciBufAttrList object that this function creates.
 *
 * @return ::NvSciError, the completion status of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a newAttrList is NULL.
 * - ::NvSciError_InsufficientMemory if insufficient system memory to create
 *   a list.
 */
NvSciError NvSciBufAttrListCreate(NvSciBufModule module,
    NvSciBufAttrList* newAttrList);

/**
 * @brief Frees the memory of an attribute list.
 *
 * @param[in] attrList Attribute list to be freed.
 *
 * @return void
 */
void NvSciBufAttrListFree(NvSciBufAttrList attrList);

/**
 * @brief Sets the attribute values in the attribute list.
 * Reads values only during the call, saving copies.
 *
 * @param[in] attrList Unreconciled attribute list where the function
 * will set the attribute key and value.
 * @param[in] pairArray Array of attribute key/value pair structures.
 * @param[in] pairCount Number of elements/entries in @a pairArray.
 *
 * @return ::NvSciError, the completion status of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a attrList is NULL, or @a attrList is a reconciled
 *   and/or imported attribute list.
 */
NvSciError NvSciBufAttrListSetAttrs(NvSciBufAttrList attrList,
                NvSciBufAttrKeyValuePair* pairArray, size_t pairCount);

/**
 * @brief Returns the slot count per key in an attribute list.
 *
 * @param[in] attrList Attribute list to retrieve the slot count from.
 *
 * @return size_t
 * - Number of slots in the attribute list
 * - 0 if attrList is invalid
 */
size_t NvSciBufAttrListGetSlotCount(NvSciBufAttrList attrList);

/**
 * @brief Returns an array of attribute key/value pairs for a given set of keys.
 *
 * This function accepts a set of keys passed in the @ref NvSciBufAttrKeyValuePair
 * structure. The return values, stored back into @ref NvSciBufAttrKeyValuePair, consist of
 * @c const @c void* pointers to the attribute
 * values from the @ref NvSciBufAttrList.
 * The application must not write to this data.
 *
 * @param[in] attrList Attribute list to fetch the attribute key/value pairs from.
 * @param[in,out] pairArray Array of key/value pair structures.
 * @param[in] pairCount Number of elements/entries in @a pairArray.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL, or
 * if @a attrList is an imported unreconciled attribute list.
 */
NvSciError NvSciBufAttrListGetAttrs(NvSciBufAttrList attrList,
                NvSciBufAttrKeyValuePair* pairArray, size_t pairCount);

/**
 * @brief Returns an array of attribute key/value pairs from a multi-slot unreconciled
 * attribute list at the given slot index.
 *
 * @note When exporting an array containing multiple unreconciled attribute
 * lists, the importing endpoint still imports just one unreconciled attribute
 * list. This unreconciled attribute list is referred to as a multi-slot
 * attribute list. It logically represents an array of attribute lists, where
 * each key has an array of values, one per slot.
 *
 * The return values, stored in @ref NvSciBufAttrKeyValuePair, consist of
 * @c const @c void* pointers to the attribute
 * values from the NvSciBufAttrList.
 * The application must not write to this data.
 *
 * @param[in] attrList Attribute list to fetch the attribute key/value pairs from.
 * @param[in] slotIndex Index in the attribute list.
 * @param[in,out] pairArray Array of key/value pair structures. Holds the keys passed
 * into the function and returns an array of NvSciBufAttrKeyValuePair structures.
 * @param[in] pairCount Number of elements/entries in pairArray.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL or invalid.
 */
NvSciError NvSciBufAttrListSlotGetAttrs(NvSciBufAttrList attrList,
    size_t slotIndex, NvSciBufAttrKeyValuePair* pairArray, size_t pairCount);

/**
 * @brief Allocates a buffer and then dumps the contents of the specified
 * attribute list into the buffer.
 *
 * @param[in] attrList Attribute list to fetch contents from.
 * @param[out] buf A pointer to the buffer allocated for the debug dump.
 * @param[out] len The length of the buffer allocated for the debug dump.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a attrList is invalid.
 */
NvSciError NvSciBufAttrListDebugDump(NvSciBufAttrList attrList, void** buf,
    size_t* len);

/**
 * @brief Reconciles the given unreconciled attribute list(s) into a new
 * reconciled attribute list.
 * On success, this API call allocates memory for the reconciled attribute list
 * which has to be freed by the caller using NvSciBufAttrListFree().
 * On reconciliation failure, this API call allocates memory for the conflicting
 * attribute list which has to be freed by the caller using
 * NvSciBufAttrListFree().
 *
 * @param[in] inputArray Array containing unreconciled attribute list(s) to be
 * reconciled.
 * @param[in] inputCount The number of unreconciled attributes lists in
 * @a inputArray.
 *
 * @param[out] newReconciledAttrList Reconciled attribute list. This field
 * is populated only if the reconciliation succeeded.
 * @param[out] newConflictList Unreconciled attribute list consisting of the
 * key/value pairs which caused the reconciliation failure. This field is
 * populated only if the reconciliation failed.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL or invalid.
 * - ::NvSciError_InsufficientMemory if not enough system memory.
 * - ::NvSciError_ReconciliationFailed if reconciliation failed.
 */
NvSciError NvSciBufAttrListReconcile(NvSciBufAttrList inputArray[],
    size_t inputCount, NvSciBufAttrList* newReconciledAttrList,
    NvSciBufAttrList* newConflictList);

/**
 * @brief Clones an unreconciled/reconciled attribute list. The resulting
 * attribute list contains all the values of the original attribute
 * list. If the list is an unreconciled attribute list, then modification
 * will be allowed using setAttrList APIs.
 *
 * @param[in] origAttrList Attribute list to be cloned.
 *
 * @param[out] newAttrList A pointer to the newly cloned attribute list.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL.
 * - ::NvSciError_InsufficientMemory if there is insufficient system memory.
 */
NvSciError NvSciBufAttrListClone(NvSciBufAttrList origAttrList,
    NvSciBufAttrList* newAttrList);

/**
 * @brief Append multiple unreconciled attribute lists together, forming a
 *  single new unreconciled attribute list with a slot count equal to the
 *  sum of all the slot counts in the input list.
 *
 * @param[in] inputUnreconciledAttrListArray[] Array containing the
 *  unreconciled attribute lists to be appended together.
 * @param[in] inputUnreconciledAttrListCount Number of unreconciled attribute
 *  lists to append.
 *
 * @param[out] newUnreconciledAttrList Appended attribute list created out of
 *  the input unreconciled attribute lists.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL or invalid.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 */
NvSciError NvSciBufAttrListAppendUnreconciled(
    const NvSciBufAttrList inputUnreconciledAttrListArray[],
    size_t inputUnreconciledAttrListCount,
    NvSciBufAttrList* newUnreconciledAttrList);

/**
 * @brief Checks if an attribute list is reconciled.
 *
 * @param[in] attrList Attribute list to check.
 *
 * @param[out] isReconciled A pointer to a boolean to store whether the
 * @a attrlist is reconciled or not.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL.
 */
NvSciError NvSciBufAttrListIsReconciled(NvSciBufAttrList attrList,
    bool* isReconciled);

/**
 * @brief Validates a reconciled attribute list against a set of
 *        unreconciled attribute lists.
 *
 * @param[in] reconciledAttrList Reconciled attribute list to be validated.
 * @param[in] unreconciledAttrListArray Set of unreconciled lists that need
 *  to be used for validation.
 * @param[in] unreconciledAttrListCount Number of unreconciled attribute lists.
 * @param[out] isReconcileListValid Flag indicating if the reconciled list
 *  satisfies the parameters of set of unreconciled lists.
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - reconciledAttrList is NULL or
 *         - unreconciledAttrListArray[ ] is NULL or
 *         - unreconciledAttrListCount is zero or
 *         - isReconcileListValid is NULL
 */
NvSciError NvSciBufAttrListValidateReconciled(
    NvSciBufAttrList reconciledAttrList,
    const NvSciBufAttrList unreconciledAttrListArray[],
    size_t unreconciledAttrListCount, bool* isReconcileListValid);

/**
 * @}
 */

/**
 * @defgroup nvscibuf_obj_api NvSciBuf Object APIs
 * List of APIs to create/operate on NvSciBufObj.
 * @{
 */

/**
 * @brief Clones a NvSciBuf object.
 *
 * @param[in] bufObj The NvSciBuf object to clone.
 *
 * @param[out] dupObj A clone of the NvSciBuf object passed into the function.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL.
 */
NvSciError NvSciBufObjDup(NvSciBufObj bufObj, NvSciBufObj* dupObj);

/**
 * @brief Reconcile the input unreconciled attribute list(s) into a new
 * reconciled attribute list and allocate NvSciBuf object that meets all the
 * constraints in the new reconciled attribute list.
 *
 * @param[in] attrListArray Array containing unreconciled attribute list(s) to
 * reconcile.
 * @param[in] attrListCount The number of unreconciled attribute list(s) in
 * @c inputArray.
 *
 * @param[out] bufObj Newly created NvSciBufObj.
 * @param[out] newConflictList Unreconciled attribute list consisting of the
 * key/value pairs which caused the reconciliation failure. This field is
 * populated only if the reconciliation failed.
 *
 * @return ::NvSciError, the completion code of this operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL or invalid.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_ReconciliationFailed if reconciliation failed.
 */
NvSciError NvSciBufAttrListReconcileAndObjAlloc(
    NvSciBufAttrList attrListArray[], size_t attrListCount,
    NvSciBufObj* bufObj, NvSciBufAttrList* newConflictList);

/**
 * @brief Destroys the NvSciBuf object, which frees any memory allocated for it.
 *
 * \param[in] bufObj The NvSciBuf object to deallocate.
 *
 * @return void
 */
void NvSciBufObjFree(NvSciBufObj bufObj);

/**
 * @brief Retrieves the attribute list from an NvSciBuf object. This
 * validated attribute list was used to allocate the object.
 *
 * Note: The retrieved attribute list from an NvSciBuf object is read-only,
 * and the attribute values in the list cannot be modified using
 * set attribute APIs. In
 * addition, the retrieved attribute list must not be freed with
 * NvSciBufAttrListFree.
 *
 * @param[in] bufObj The NvSciBuf object to retrieve the attribute list from.
 *
 * @param[out] bufAttrList A pointer to the retrieved attribute list.
 *
 * @return ::NvSciError, the completion code of this operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL.
 */
NvSciError NvSciBufObjGetAttrList(NvSciBufObj bufObj,
    NvSciBufAttrList* bufAttrList);

/**
 * @brief Gets the CPU virtual address (VA) of the NvSciBufObj.
 *
 * @param[in] bufObj The buffer object to get the address of.
 *
 * @param[out] ptr A pointer to the CPU mapping of the NvSciBufObj.
 *
 * @return ::NvSciError, the completion code of this operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a bufObj is invalid.
 */
NvSciError NvSciBufObjGetCpuPtr(NvSciBufObj bufObj,  void**  ptr);

/**
 * @brief Gets the CPU virtual address (VA) of the NvSciBufObj. The returned value
 * cannot be modified.
 *
 * @param[in] bufObj The buffer object to get the address of.
 *
 * @param[out] ptr A @c const pointer to the CPU mapping of the NvSciBufObj.
 *
 * @return ::NvSciError, the completion code of this operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a bufObj is invalid.
 */
NvSciError NvSciBufObjGetConstCpuPtr(NvSciBufObj bufObj,  const void**  ptr);

/**
 * @brief Flushes the given @c len bytes at starting offset @c offset
 * in NvSciBufObj from all CPU-managed caches.
 *
 * @param[in] bufObj NvSciBuf object to flush the memory of.
 * @param[in] offset The starting offset in memory of the NvSciBuf object.
 * @param[in] len The length (in bytes) to flush.
 *
 * @return ::NvSciError, the completion code of this operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a bufObj is invalid.
 */
NvSciError NvSciBufObjFlushCpuCacheRange(NvSciBufObj bufObj,
    uint64_t offset, uint64_t len);

/**
 * @brief Allocates an NvSciBuf object that meets all the constraints in the
 * specified reconciled attribute list.
 *
 * @param[in] reconciledAttrList The reconciled attribute list generated through
 * NvSciBufAttrListReconcile().
 *
 * @param[out] bufObj Identifier representing NvSciBuf object.
 *
 * @return ::NvSciError, the completion code of this operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if reconciledList is NULL or reconciledList is not a
 *   reconciled attribute list or bufObj is NULL.
 * - ::NvSciError_InsufficientMemory if is insufficient memory to complete the operation.
 */
NvSciError NvSciBufObjAlloc(NvSciBufAttrList reconciledAttrList,
    NvSciBufObj* bufObj);
/**
 * @}
 */

/**
 * @defgroup nvscibuf_transport_api NvSciBuf APIs
 * List of APIs to transport NvSciBuf buffers and attribute list objects across
 * various communication boundaries that interact using NvSciIpc.
 * @{
 */


/**
 * @brief Exports an NvSciBuf attribute list and object into an
 * NvSciIpc-transferable object binary descriptor. The blob can be
 * transferred to the other processes to create a matching NvSciBuf object.
 *
 * @param[in] bufObj NvSciBuf Object to export.
 * @param[in] permissions Flag indicating the expected access permission
 (see @ref NvSciBufAttrValAccessPerm).
 * @param[in] ipcEndpoint NvSciIpcEndpoint to identify the peer process.
 * \param[out] attrListAndObjDesc NvSciBuf allocates and fills in the
 * exported form of NvSciBufObj and its corresponding attribute list
 * to be shared across an NvSciIpc channel.
 * \param[out] attrListAndObjDescSize Size of the exported blob.
  *
 * @return ::NvSciError, the completion code of this operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL or invalid.
 */
NvSciError NvSciBufIpcExportAttrListAndObj(NvSciBufObj bufObj,
    NvSciBufAttrValAccessPerm permissions, NvSciIpcEndpoint ipcEndpoint,
    void** attrListAndObjDesc, size_t* attrListAndObjDescSize);

/**
 * @brief This API is invoked by importing process after it receives the
 * binary blob sent by the other process who has created the binary descriptor.
 * Importing process will create its own NvSciBuf object and return as output.
 *
 * @param[in] module Instance of the @ref NvSciBufModule to be used for importing
 * NvSciBuf Object.
 * @param[in] ipcEndpoint NvSciIpcEndpoint to identify the peer process.
 * @param[in] attrListAndObjDesc The exported form of attribute list and NvSciBufObj.
 * @param[in] attrListAndObjDescSize Size of the imported blob.
 * @param[in] attrList[] Receiver side array of attribute lists against which
 * the imported attrList has to be validated.
 * @param[in] count Number of NvSciBufAttrList objects in the array.
 * @param[in] minPermissions Minimum permissions of the buffer that the process
 * is expecting to import the buffer (see @ref NvSciBufAttrValAccessPerm).
 * @param[in] timeoutUs Maximum delay (in microseconds) before an NvSciBuf object
 * times out.
 * \param[out] bufObj NvSciBuf Object duplicated and exported during the importing process.
 * It contains the attribute list to be shared across an NvSciIpc channel.
 * \param[out] attrListAndObjDescSize Size of the exported blob.
 *
 * @return ::NvSciError, the completion code of this operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL or invalid.
 */
NvSciError NvSciBufIpcImportAttrListAndObj(NvSciBufModule module,
    NvSciIpcEndpoint ipcEndpoint, const void* attrListAndObjDesc,
    size_t attrListAndObjDescSize, const NvSciBufAttrList attrList[],
    size_t count, NvSciBufAttrValAccessPerm minPermissions,
    int64_t timeoutUs, NvSciBufObj* bufObj);

/**
 * @brief Frees the descriptor used for exporting/importing both
 * attrlist and NvSciBuf Object together.
 *
 * @param[in] attrListAndObjDescBuf Descriptor to be freed.
 *
 */
void NvSciiBufAttrListAndObjFreeDesc(void* attrListAndObjDescBuf);

/**
 * @brief Exports a NvSciBuf object into an NvSciIpc-transferable object binary
 * descriptor.
 * Descriptor can be transferred to other end of IPC where matching NvSciBuf
 * object can be created from the descriptor.
 *
 * @param[in] bufObj NvSciBuf object to export.
 * @param[in] accPerm Flag indicating the expected access permission
 (see @ref NvSciBufAttrValAccessPerm).
 * @param[in] ipcEndPoint Handle to IPC endpoint.
 * \param[out] exportData NvSciBuf populates the return value with exported
 * form of NvSciBufObj shared across an NvSciIpc channel.
 *
 * @return ::NvSciError, the completion code of this operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL or invalid.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 */
NvSciError NvSciBufObjIpcExport(NvSciBufObj bufObj,
    NvSciBufAttrValAccessPerm accPerm, NvSciIpcEndpoint ipcEndPoint,
    NvSciBufObjIpcExportDescriptor* exportData);

/**
 * @brief Creates a NvSciBuf object based on supplied binary descriptor and
 * returns the NvSciBuf object.
 *
 * @param[in] ipcEndPoint Identifier of type NvSciIpcEndpoint.
 * @param[in] desc A pointer to an NvSciBufObjIpcExportedDescriptor, an exported
 form of NvSciBufObj received via the NvIpc channel.
 * @param[in] reconciledAttrList Reconciled attribute list returned by
 * NvSciBufAttrListImport().
 * @param[in] minPermissions Flag indicating the expected access permission.
 * @param[in] timeoutUs Import timeout in micro seconds, -1 for infinite
 *            wait.
 * @param[out] bufObj Imported NvSciBufObj created from the descriptor.
 *
 * @return ::NvSciError, the completion code of this operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL.
 * - ::NvSciError_InsufficientMemory if there is insufficient system memory.
 * - ::NvSciError_AttrListValidationFailed if @c reconciledAttrList constraints
     are not met by attributes of the imported NvSciBuf object.
 * - ::NvSciError_InvalidOperation if the importing process does not have permission
to initiate the action to import the buffer.
 * - ::NvSciError_Timeout if the import action did not complete in time.
 */
NvSciError NvSciBufObjIpcImport(NvSciIpcEndpoint ipcEndPoint,
    const NvSciBufObjIpcExportDescriptor* desc,
    NvSciBufAttrList reconciledAttrList,
    NvSciBufAttrValAccessPerm minPermissions, int64_t timeoutUs,
    NvSciBufObj* bufObj);

/**
 * @brief Transforms the input attribute list(s) to an exported
 * attribute list descriptor that can be transported by the application to any
 * remote process as serialized set of bytes over an IPC/IVC/nvIPC channel.
 *
 * @param[in] unreconciledAttrListArray[] Array of unreconciled attribute lists.
 * @param[in] unreconciledAttrListCount Number of unreconciled attribute lists in
 * unreconciledAttrListArray[].
 * @param[in] ipcEndpoint The NvSciIpc endpoint through which the caller may
 * send the exported attribute list descriptor provided by this function.
 * @param[out] descBuf Address at which the pointer to exported attribute list
 * descriptor is to be stored.
 * @param[out] descLen Address at which the size of the exported attribute list
 * descriptor is to be stored.
 *
 * @return ::NvSciError, the completion code of this operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL or invalid.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_InvalidOperation if the import process does not
have permission to initiate the action to import the buffer.
 */
NvSciError NvSciBufAttrListIpcExportUnreconciled(
    NvSciBufAttrList unreconciledAttrListArray[],
    size_t unreconciledAttrListCount, NvSciIpcEndpoint ipcEndpoint,
    void** descBuf, size_t* descLen);

/**
 * @brief Transforms the input attribute list(s) to an exported
 * attribute list descriptor that can be transported by the application to any
 * remote process as serialized set of bytes over an IPC/IVC/nvIPC channel.
 *
 * @param[in] reconciledAttrList Reconciled attribute list.
 * @param[in] ipcEndpoint The NvSciIpc endpoint through which the caller may
 * send the exported attribute list descriptor provided by this function.
 * @param[out] descBuf Address at which pointer to exported attribute list
 * descriptor is to be stored.
 * @param[out] descLen Address at which the size of the exported attribute list
 * descriptor is to be stored.
 *
 * @return ::NvSciError, the completion code of this operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL or invalid.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_InvalidOperation if the import process does not
have permission to initiate the action to import the buffer.
 */
NvSciError NvSciBufAttrListIpcExportReconciled(
    NvSciBufAttrList reconciledAttrList, NvSciIpcEndpoint ipcEndpoint,
    void** descBuf, size_t* descLen);

/**
 * @brief Retrieves the attribute list from the attribute
 * descriptor received over IPC channel. When importing a reconciled attribute
 * list, it will also validate the attribute list to be imported against the
 * provided input unreconciled attribute lists. (This is required while
 * importing a reconciled attribute list to cause NvSciBuf to validate the
 * reconciled attributes against the input attributes so that the process
 * can be sure that a buffer consistent with the imported reconciled list
 * will satisfy the input constraints.)
 *
 * @param[in] module The NvSciBuf module instance with which to associate the
 * imported NvSciBufAttrList.
 * @param[in] ipcEndpoint The NvSciIpc endpoint through which the caller may
 * send the exported attribute list descriptor provided by this function.
 * @param[in] descBuf Address at which pointer to imported attribute list
 * descriptor is to be stored.
 * @param[in] descLen The size of the imported attribute list descriptor.
 * @param[out] importedUnreconciledAttrList The imported unreconciled attribute
 * list.
 *
 * @return ::NvSciError, the completion code of this operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument are NULL or invalid.
 * - ::NvSciError_InsufficientMemory if sufficient system memory.
 * - ::NvSciError_InvalidOperation if the import process does not
have permission to initiate the action to import the buffer.
 */
NvSciError NvSciBufAttrListIpcImportUnreconciled(NvSciBufModule module,
    NvSciIpcEndpoint ipcEndpoint, const void* descBuf, size_t descLen,
    NvSciBufAttrList* importedUnreconciledAttrList);

/**
 * @brief Retrieves the attribute list from the attribute
 * descriptor received over IPC channel. When importing a reconciled attribute
 * list, it will also validate the attribute list to be imported against the
 * provided input unreconciled attribute lists. (This is required while
 * importing a reconciled attribute list to cause NvSciBuf to validate the
 * reconciled attributes against the input attributes so that the process
 * can be sure that a buffer consistent with the imported reconciled list
 * will satisfy the input constraints.)
 *
 * @param[in] module The NvSciBuf module instance with which to associate the
 * imported NvSciBufAttrList.
 * @param[in] ipcEndpoint The NvSciIpc endpoint through which the caller may
 * send the exported attribute list descriptor provided by this function.
 * @param[in] descBuf Address at which pointer to imported attribute list
 * descriptor is to be stored.
 * @param[in] descLen The size of the imported attribute list descriptor.
 * @param[in] inputUnreconciledAttrListArray Array of unreconciled list against
 * imported reconciled list will be verified.
 * @param[in] inputUnreconciledAttrListCount Number of unreconciled attribute list
 * in unreconciledAttrListArray[].
 * @param[out] importedReconciledAttrList The imported reconciled attribute
 * list.
 *
 * @return ::NvSciError, the completion code of this operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL or invalid.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_InvalidOperation if the import process does not
have permission to initiate the action to import the buffer.
 */
NvSciError NvSciBufAttrListIpcImportReconciled(NvSciBufModule module,
    NvSciIpcEndpoint ipcEndpoint, const void* descBuf, size_t descLen,
    NvSciBufAttrList inputUnreconciledAttrListArray[],
    size_t inputUnreconciledAttrListCount,
    NvSciBufAttrList* importedReconciledAttrList);


/**
 * @brief Frees the NvSciBuf exported attribute list descriptor.
 *
 * @param[in] descBuf Attribute list descriptor to be freed.
 *
 * @return void
 */
void NvSciBufAttrListFreeDesc(void* descBuf);

/**
 * @}
 */

/**
 * @defgroup nvscibuf_init_api NvSciBuf Initialization APIs
 * List of APIs to initialize/de-initialize NvSciBuf module.
 * @{
 */

/**
 * @brief Initializes an instance of the NvSciBuf module within the
 * calling process and provides an NvSciBufModule representing the instance.
 * @note A process may call this function multiple times.
 * Each successful invocation will yield a new NvSciBuf module instance.
 *
 * @param[in] newModule The new NvSciBuf module instance.
 *
 * @return ::NvSciError, the completion code of this operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_ResourceError if system drivers not available.
 */
NvSciError NvSciBufModuleOpen(NvSciBufModule* newModule);

/**
 * @brief Releases an instance of the NvSciBuf module obtained through
 * an earlier call to NvSciBufModuleOpen(). Once an NvSciBufModule is closed
 * and all NvSciBufAttrLists and NvSciBufObjs bound to that module instance
 * are freed, the NvSciBuf module instance will be de-initialized in
 * the calling process.
 *
 * @param[in] module The NvSciBufModule instance to close.
 * The calling process must not pass this module to another
 * NvSciBuf API call.
 *
 * @return void
 */
void NvSciBufModuleClose(NvSciBufModule module);

/**
 * @}
 */

/** @} */

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* INCLUDED_NVSCIBUF_H */
