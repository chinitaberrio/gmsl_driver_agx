/*
 * Copyright (c) 2013-2020, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: Image Processing </b>
 *
 * @b Description: This file contains the
 *                 \ref nvmedia_image_top "Image Processing API."
 */

#ifndef NVMEDIA_IMAGE_H
#define NVMEDIA_IMAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"
#include "nvmedia_surface.h"

/**
 * \defgroup nvmedia_image_top Image Handling API
 *
 * The Image Processing API encompasses all NvMedia image-related functionality.
 * \note Not all the functions are supported on safety build and refer to nvmedia safety manual
 * for details of APIs available on safety build.
 *
 * @ingroup nvmedia_top
 * @{
 */

/** \brief Major version number. */
#define NVMEDIA_IMAGE_VERSION_MAJOR   (1u)
/** \brief Minor version number. */
#define NVMEDIA_IMAGE_VERSION_MINOR   (24u)

/**
 * \defgroup image_creation_api Image Creation
 *
 * Defines and manages image objects.
 *
 * \ref NvMediaImage objects are video RAM surfaces
 * storing YUV, RGBA or RAW data. They can store one or more images depending
 * on the class.
 * \ref NvMediaImage objects are created with
 * \ref NvMediaImageCreateNew() or \ref NvMediaImageCreateFromNvSciBuf()
 * and destroyed with \ref NvMediaImageDestroy().
 * @{
 */

/**
 * \hideinitializer
 * \brief Maximum number of images in an image group.
 */
#define NVMEDIA_MAX_IMAGE_GROUP_SIZE   (3u)

/**
 * \hideinitializer
 * \brief Infinite timeout for NvMediaImageGetStatus().
 */
#define NVMEDIA_IMAGE_TIMEOUT_INFINITE  (0xFFFFFFFFu)

/**
 * \brief Holds a handle representing image objects.
 */
struct NvMediaImageRec {
    /*! Holds image surface type. It should be a valid value returned by
        the \ref NvMediaSurfaceFormatGetType API */
    NvMediaSurfaceType type;
    /*! Holds image width. Valid range of width supported is
        [\ref NVM_SURF_ATTR_MIN_WIDTH, \ref NVM_SURF_ATTR_MAX_WIDTH] */
    uint32_t width;
    /*! Holds image height. Valid range of height supported is
        [\ref NVM_SURF_ATTR_MIN_HEIGHT, \ref NVM_SURF_ATTR_MAX_HEIGHT] */
    uint32_t height;
    /*! Holds the size of the top embedded data. Valid range of
        embedded data supported is
        [\ref NVM_SURF_ATTR_MIN_EMB_LINES_TOP, \ref NVM_SURF_ATTR_MAX_EMB_LINES_TOP] */
    uint32_t embeddedDataTopSize;
    /*! Holds the size of the bottom embedded data. Valid range of
        embedded data supported is
        [\ref NVM_SURF_ATTR_MIN_EMB_LINES_TOP, \ref NVM_SURF_ATTR_MAX_EMB_LINES_TOP] */
    uint32_t embeddedDataBottomSize;
    /*! Holds an image color standard type. Valid range of values is
        [\ref NVM_SURF_ATTR_COLOR_STD_SRGB, \ref NVM_SURF_ATTR_COLOR_STD_REC2020PQ_ER] */
    uint32_t colorStd;
    /*! \deprecated  Holds a tag that can be used by the application. NvMedia
        does not change the value of this member. */
    void *tag;
    /*! Holds an image capture global timestamp in microseconds. */
    NvMediaGlobalTime captureGlobalTimeStamp;
#if (NV_IS_SAFETY == 0)
    /*! Holds the number of images stored in this image container. */
    uint32_t imageCount;
    /*! Holds image attributes. */
    uint32_t attributes;
    /*! Holds an image capture timestamp. */
    NvMediaTime captureTimeStamp;
#endif
    /** Holds an opaque pointer for internal use. It should be non-null for
        a valid NvmediaImage*/
    struct NvMediaImagePriv_ *imgPriv;
};

typedef struct NvMediaImageRec NvMediaImage;

/**
 * \brief Holds a handle representing an image group.
 */
typedef struct {
    /*! Holds a list of NvMedia images in the group. */
    NvMediaImage *imageList[NVMEDIA_MAX_IMAGE_GROUP_SIZE];
    /*! Holds the number of NvMedia images in the group. */
    uint32_t numImages;
    /*! Holds a tag for use by the application. NvMedia
        does not change the value of this member. */
    void *tag;
    /*! Holds the VI engine capture timestamp associated with the NvMedia
     images in the group. */
    uint64_t captureTimeStamp[NVMEDIA_MAX_IMAGE_GROUP_SIZE];
} NvMediaImageGroup;

/**
 * \brief Gets the version of the NvMedia Image library.
 * \param[in] version A pointer to an \ref NvMediaVersion structure
 *                    belonging to the client. It should be a valid non-null pointer.
 * \return  A status code; \ref NVMEDIA_STATUS_OK if the call was successful,
 *  or \ref NVMEDIA_STATUS_BAD_PARAMETER if @a version is an invalid pointer.
 */
NvMediaStatus
NvMediaImageGetVersion(
    NvMediaVersion *version
);

#if (NV_IS_SAFETY == 0)
/**
 * \brief Allocates an image object. Upon creation, the contents are undefined.
 *
 * \param[in] device The \ref NvMediaDevice.
 * \param[in] type      Surface format type obtained by
 *                       NvMediaSurfaceFormatGetType().
 * \param[in] attrs An array of surface alloc attributes for surface creation.
 * \param[in] numAttrs Number of attributes in @a attrs.
 * \param[in] flags Flags for module hints (reserved for future use).
 * \return  The new image's handle if the call is successful, or NULL otherwise.
 */
NvMediaImage *
NvMediaImageCreateNew(
    NvMediaDevice *device,
    NvMediaSurfaceType type,
    const NvMediaSurfAllocAttr *attrs,
    uint32_t numAttrs,
    uint32_t flags
);
#endif

/**
 * \brief Destroys an \ref NvMediaImage object
 *
 * \param[in] image The image to destroy.
 */
void
NvMediaImageDestroy(
    const NvMediaImage *image
);

/**@} <!-- Ends image_creation_api Image Creation API --> */

/**
 * \brief Returns embedded data stored in a captured image.
 *
 * Embedded data can be added to the following image types.
 * This type must obtained by a call to NvMediaSurfaceFormatGetType() with:
 * - NVM_SURF_FMT_SET_ATTR_YUV(attr, YUYV, 422, PACKED, UINT, 8, PL)
 * - NVM_SURF_FMT_SET_ATTR_RGBA(attr, RGBA, UINT, 8, [PL])
 * - NVM_SURF_FMT_SET_ATTR_RAW(attr, [RGGB/BGGR/GRBG/GBRG], [UINT/INT], [8/10/12/16/20], PL)
 * - NVM_SURF_FMT_SET_ATTR_RAW(attr, [RCCB/BCCR/CRBC/CBRC], [UINT/INT], 12, PL)
 * - NVM_SURF_FMT_SET_ATTR_RAW(attr, [RCCC/CCCR/CRCC/CCRC], [UINT/INT], 12, PL)
 *
 * To use use this API, while creating an NvMedia image, you must set
 * \ref NvMediaSurfAllocAttrType in \ref NvMediaSurfAllocAttr to
 * \ref NVM_SURF_ATTR_CPU_ACCESS_UNCACHED or
 * \ref NVM_SURF_ATTR_CPU_ACCESS_CACHED.
 *
 * Embedded data may contain 8-bit data or data encoded in the
 * sensor pixel data type, depending on the camera sensor side specification.
 *
 * In case of RAW12 DataType, the bit layout may change without warning,
 * depending on the surface data type specified.
 *
 * The VI hardware changes data layout to:
 * - For RAW12 UINT case: [11][10][9][8][7][6][5][4][3][2][1][0][P][P][P][P]
 * - For RAW12  INT case: [0][11][10][9][8][7][6][5][4][3][2][1][0][P][P][P]
 *
 * Each embedded data pixel has 16 bits. [P] represents a padding bit. Padding
 * bits duplicate the most significant bits: [11][10][9][8] for UINT, or
 * [11][10][9] for INT.
 *
 * Contact NVIDIA for more information on other DataTypes.
 *
 * \param[in] image A pointer to the image from which to get embedded data.
 * \param[in] imageIndex Index of the sub-image in the case of a multi-image
 *                      handle, counting from 0. For a single image handle
 *                      @a imageIndex is ignored. 0 must be passed as this
 *                      parameter's value, else the API will fail.
 * \param[in] embeddedBufTop
 *                      A pointer to a buffer allocated on the caller side to
 *                      store top data.
 * \param[in,out] embeddedBufTopSize
 *                      A pointer to top buffer size (in). Actual size of
 *                      copied data is returned at this location (out).
 *                      Top buffer size is calculated from values like
 *                      frame resolution and \ref NVM_SURF_ATTR_EMB_LINES_TOP.
 * \param[in] embeddedBufBottom
 *                      A pointer to a buffer allocated
 *                      at caller side to store bottom data.
 * \param[in,out] embeddedBufBottomSize
 *                      A pointer to bottom buffer size (in).
 *                      Actual size of copied data is returned at this location
 *                      (out). Bottom buffer size is calculated from values like
 *                      frame resolution and
 *                      \ref NVM_SURF_ATTR_EMB_LINES_BOTTOM.
 * \retval  NVMEDIA_STATUS_OK indicates that that call was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more
 *  pointer parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that the surface type is not
 *  supported.
 */
NvMediaStatus
NvMediaImageGetEmbeddedData(
    const NvMediaImage *image,
    uint32_t imageIndex,
    uint8_t *embeddedBufTop,
    uint32_t *embeddedBufTopSize,
    uint8_t *embeddedBufBottom,
    uint32_t *embeddedBufBottomSize
);

/**
 * \brief Gets status of the current or most recent operation for an image;
 *        optionally waits for the current operation to complete or time out.
 *        It waits only on a write operation to be completed on the image.
 * \param[in] image A pointer to the image. It should be non-null.
 * \param[in] millisecondWait
 *                  Time in milliseconds to wait for the current operation to
 *                  complete before getting status.
 *                  Valid range of input is [0, UINT32_MAX]
 *                  \ref NVMEDIA_IMAGE_TIMEOUT_INFINITE means wait indefinitely.
 * \param[out] taskStatus
 *                  A pointer to the status of the operation. It should be non-null.
 * \retval  NVMEDIA_STATUS_OK indicates that the call was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a image or @a status
 *  was NULL.
 * \retval NVMEDIA_STATUS_TIMED_OUT indicates that wait for the current operation timed out.
 * \retval  NVMEDIA_STATUS_ERROR indicates that another error occurred.
 */
NvMediaStatus
NvMediaImageGetStatus(
    const NvMediaImage *image,
    uint32_t millisecondWait,
    NvMediaTaskStatus *taskStatus
);

/**
 * \brief Sets a tag for an \ref NvMediaImage.
 *
 * Associates a tag (an arbitrary pointer) with an NvMediaImage.
 *
 * \note  The @c NvMediaImage struct's @a tag member is deprecated, and
 *  will be retired in a future release. Use %NvMediaImageSetTag() to set
 *  a tag instead.
 *
 * \param[in] image A pointer to image for which a tag is to be set. It should be a valid non-null
 * pointer
 * \param[in] tag   A pointer to the tag.
 * @return  A status code; \ref NVMEDIA_STATUS_OK if the call was successful, or
 *  \ref NVMEDIA_STATUS_BAD_PARAMETER if @a image was NULL.
 */
NvMediaStatus
NvMediaImageSetTag(
    NvMediaImage *image,
    void *tag
);

/**
 * \brief Gets the tag from an \ref NvMediaImage.
 *
 * \note  The \ref NvMediaImage struct's @a tag member is deprecated, and
 *  will be retired in a future release. Use NvMediaImageSetTag() to set
 * a tag and %NvMediaImageGetTag() to get a tag instead.
 *
 * \param[in]  image A pointer to image for which to get the tag.
 *                   It should be a valid non-null pointer.
 * \param[out] tag   A pointer to a location where the function is to put
 *                    a pointer to the tag. It should be non-null.
 * @return  A status code; \ref NVMEDIA_STATUS_OK if the call was successful,
 *  or \ref NVMEDIA_STATUS_BAD_PARAMETER if @a image or @a tag was NULL.
 */
NvMediaStatus
NvMediaImageGetTag(
    const NvMediaImage *image,
    void **tag
);

#if (NV_IS_SAFETY == 0)
/**
 * \brief Gets the capture timestamp of the image.
 * \param[in] image A pointer to the image for which to get the timestamp.
 * \param[out] timeStamp A pointer the capture timestamp of the image.
 * \retval  NVMEDIA_STATUS_OK indicates that the call was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a image or
 *  @a timeStamp was NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that another error occurred.
 */
NvMediaStatus
NvMediaImageGetTimeStamp(
    const NvMediaImage *image,
    NvMediaTime *timeStamp
);

/**
 * \brief Gets the global capture timestamp of the image.
 * The global timestamp is set by the NvMedia IPP Capture component.
 * \param[in] image             A pointer to the image for which to get the
 *                               timestamp.
 * \param[out] globalTimeStamp  A pointer to the global capture timestamp
 *                               of the image.
 * \retval  NVMEDIA_STATUS_OK indicates that the call was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a image
 *  or @a timeStamp was NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that another error occurred.
 */
NvMediaStatus
NvMediaImageGetGlobalTimeStamp(
    const NvMediaImage *image,
    NvMediaGlobalTime *globalTimeStamp
);

/** \defgroup lock_unlock Image Locking and Unlocking
 *
 * Locking and unlocking controls access to the image surfaces.
 *
 * \warning These functions are for diagnostics purposes only. They
 * are not for use in production code.
 */

/**
 * \brief Specifies image lock access types.
 * \ingroup lock_unlock
 */
typedef enum {
    /** Specifies read access. */
    NVMEDIA_IMAGE_ACCESS_READ       = (1u << 0),
    /** Specifies write access. */
    NVMEDIA_IMAGE_ACCESS_WRITE      = (1u << 1),
    /** Specifies read/write access. */
    NVMEDIA_IMAGE_ACCESS_READ_WRITE = (NVMEDIA_IMAGE_ACCESS_READ | NVMEDIA_IMAGE_ACCESS_WRITE),
} NvMediaImageLockAccess;

/**
 * \brief Holds an image surface descriptor used by NvMediaImageLock().
 * \ingroup lock_unlock
 */
typedef struct {
    /*! Holds pitch of the surface. */
    uint32_t pitch;
    /*! Holds a CPU accessible memory pointer to the surface. */
    void *mapping;
} NvMediaImageSurface;

/**
 * \brief Holds an image surface map descriptor used by
 * NvMediaImageLock(). It contains up to 6 surface descriptors,
 * depending on the image type.
 * \ingroup lock_unlock
 */
typedef struct {
    /*! Holds image surface type. */
    NvMediaSurfaceType type;
    /*! Holds image width. */
    uint32_t width;
    /*! Holds image height. */
    uint32_t height;
    /*! Holds the number of surfaces in the image. */
    uint32_t surfaces;
    /*! Holds surface descriptors. Only the first surfaces number of elements
     is valid. */
    NvMediaImageSurface surface[6];
    /*! Holds a pointer to metadata associated with the surface. */
    void *metaData;
} NvMediaImageSurfaceMap;

/**
 * \brief Locks an image and returns the associated mapped pointers
 * to the image surface data.
 *
 * Only images created without the
 * \ref NVM_SURF_ATTR_CPU_ACCESS_UNMAPPED attribute can be accessed by the CPU.
 * If an image is being used by an internal engine, this function waits until
 * the operation is completed.
 *
 * \param[in] image A pointer to the image object.
 * \param[in] lockAccessType Determines the access type.
 * The following access types are supported, and may be OR'd together:
 * - \ref NVMEDIA_IMAGE_ACCESS_READ specifies read access.
 * - \ref NVMEDIA_IMAGE_ACCESS_WRITE Specifies write access.
 * - \ref NVMEDIA_IMAGE_ACCESS_READ_WRITE specifies read/write access.
 * \param[out] surfaceMap A pointer to surface descriptors.
 * \return A status code; NVMEDIA_STATUS_OK if the call was successful,
 *  NVMEDIA_STATUS_BAD_PARAMETER if the arguments to the function are invalid,
 *  or NVMEDIA_STATUS_ERROR otherwise.
 * \ingroup lock_unlock
 */
NvMediaStatus
NvMediaImageLock(
    const NvMediaImage *image,
    NvMediaImageLockAccess lockAccessType,
    NvMediaImageSurfaceMap *surfaceMap
);

/**
 * \brief  Unlocks an image. Internal engines cannot use a surface
 * until it is locked.
 * \param[in] image A pointer to the image object to be unlocked.
 * \ingroup lock_unlock
 */
void
NvMediaImageUnlock(
    const NvMediaImage *image
);

/** \defgroup image_get_put_bits Image Read and Write by Client
 *
 * Provides image surface read and write by the client application
 * for diagnostic purposes.
 *
 * \warning These functions are for diagnostics purposes only. They
 * are not for use in production code.
 */

/**
 * \brief Reads a client memory buffer and writes the content into an NvMedia
 *  image surface.
 *
 * To use %NvMediaImagePutBits() while creating an NvMedia image, you must set
 * \ref NvMediaSurfAllocAttrType in \ref NvMediaSurfAllocAttr to
 * \ref NVM_SURF_ATTR_CPU_ACCESS_UNCACHED or
 * \ref NVM_SURF_ATTR_CPU_ACCESS_CACHED.
 *
 * \param[in] image
 *       A pointer to the destination \ref NvMediaImage type surface. You must
 *       lock the image with NvMediaImageLock() before calling this function.
 * \param[in] dstRect
 *    @parblock
 *       A pointer to a structure containing co-ordinates of the rectangle in
 *       the destination surface to which the client surface is to be copied.
 *
 *       Set @a dstRect to NULL to specify a rectangle of full surface size.
 *       @a dstRect is supported only for an %NvMediaImage whose
 *       \ref NvMediaSurfaceType was derived with the
 *       \ref NVM_SURF_ATTR_SURF_TYPE attribute set to
 *       \ref NVM_SURF_ATTR_SURF_TYPE_RGBA or \ref NVM_SURF_ATTR_SURF_TYPE_RAW.
 *       For all other surface types, it is ignored.
 *    @endparblock
 * \param[in] srcPntrs
 *    @parblock
 *       A pointer to an array of pointers to the client surface planes.
 *
 *       For an %NvMediaImage whose %NvMediaSurfaceType was derived with
 *       the %NVM_SURF_ATTR_SURF_TYPE attribute set to
 *       %NVM_SURF_ATTR_SURF_TYPE_RGBA or %NVM_SURF_ATTR_SURF_TYPE_RAW,
 *       %NvMediaImagePutBits() expects only one @a srcPntrs and one
 *       @a srcPitches.
 *
 *       For %NvMediaImages whose %NvMediaSurfaceType was derived with
 *       the %NVM_SURF_ATTR_SURF_TYPE attribute set to
 *       \ref NVM_SURF_ATTR_SURF_TYPE_YUV and:
 *          1. The \ref NVM_SURF_ATTR_COMPONENT_ORDER attribute set to
 *             \ref NVM_SURF_ATTR_COMPONENT_ORDER_LUMA,
 *             \ref NVM_SURF_ATTR_COMPONENT_ORDER_XUYV,
 *             \ref NVM_SURF_ATTR_COMPONENT_ORDER_XYUV, or
 *             \ref NVM_SURF_ATTR_COMPONENT_ORDER_VUYX,
 *             %NvMediaImagePutBits() expects only one @a srcPntrs and one
 *             @a srcPitches.
 *          2. For all other YUV surface types, %NvMediaImagePutBits()
 *             expects three @a srcPntrs and three @a srcPitches.
 *
 *             Example: For the %NvMediaSurfaceType derived with attribute
 *             set using
 *             NVM_SURF_FMT_SET_ATTR_YUV(attr, YUV, 420, SEMI_PLANAR, UINT, 8, PL),
 *             %NvMediaImagePutBits() expects three @a srcPntrs and three
 *             @a srcPitches corresponding respectively to the Y, U and V
 *             planes.
 *    @endparblock
 * \param[in] srcPitches
 *      A pointer to an array of pitch values for the client surface planes.
 *      The number of pitch values must be the same as the number of surface
 *      planes on @a srcPntrs.
 * \retval  NVMEDIA_STATUS_OK indicates that the call was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more input
 *  parameters are invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that the image is not locked.
 * \ingroup image_get_put_bits
 */
NvMediaStatus
NvMediaImagePutBits(
    NvMediaImage *image,
    const NvMediaRect *dstRect,
    void **srcPntrs,
    const uint32_t *srcPitches
);

/**
 * \brief Reads an NvMedia image and
 *        writes the content into a client memory buffer.
 *       To use %NvMediaImageGetBits() while creating an NvMedia image,
 *       \ref NvMediaSurfAllocAttrType must be set to
 *       \ref NVM_SURF_ATTR_CPU_ACCESS_UNCACHED or
 *       \ref NVM_SURF_ATTR_CPU_ACCESS_CACHED in \ref NvMediaSurfAllocAttr.
 * \param[in] image
 *       A pointer to the source \ref NvMediaImage type surface. You must lock
 *       the surface with NvMediaImageLock() before calling this function.
 * \param[in] srcRect
 *    @parblock
 *       A pointer to a structure containing coordinates of the rectangle
 *       in the source surface from which the client surface is to be copied.
 *
 *       Set @a srcRect to NULL to specify a rectangle of full surface size.
 *       @a srcRect is supported only for an %NvMediaImage whose
 *       \ref NvMediaSurfaceType was derived with the
 *       \ref NVM_SURF_ATTR_SURF_TYPE attribute set to
 *       \ref NVM_SURF_ATTR_SURF_TYPE_RGBA or \ref NVM_SURF_ATTR_SURF_TYPE_RAW.
 *       For all other surface types, it is ignored.
 *    @endparblock
 * \param[out] dstPntrs
 *    @parblock
 *       A pointer to an array of pointers to the destination surface planes.
 *
 *       For an %NvMediaImage whose %NvMediaSurfaceType was derived with
 *       NVM_SURF_ATTR_SURF_TYPE set to NVM_SURF_ATTR_SURF_TYPE_RGBA
 *       or NVM_SURF_ATTR_SURF_TYPE_RAW, %NvMediaImageGetBits() expects
 *       only one @a dstPntrs and one @a dstPitches.
 *
 *       For an NvMediaImage whose NvMediaSurfaceType was derived with
 *       the NVM_SURF_ATTR_SURF_TYPE attribute set to
 *       \ref NVM_SURF_ATTR_SURF_TYPE_YUV and:
 *          1. The \ref NVM_SURF_ATTR_COMPONENT_ORDER attribute set to
 *             \ref NVM_SURF_ATTR_COMPONENT_ORDER_LUMA,
 *             \ref NVM_SURF_ATTR_COMPONENT_ORDER_XUYV,
 *             \ref NVM_SURF_ATTR_COMPONENT_ORDER_XYUV, or
 *             \ref NVM_SURF_ATTR_COMPONENT_ORDER_VUYX,
 *             this API expects only one @a dstPntrs and one @a dstPitches.
 *          2. For all other YUV surface types, %NvMediaImageGetBits()
 *             expects three @a dstPntrs and three @a dstPitches.
 *
 *             Example: For the NvMediaSurfaceType derived with the
 *             attribute set using
 *             NVM_SURF_FMT_SET_ATTR_YUV(attr, YUV, 420, SEMI_PLANAR, UINT, 8, PL),
 *             %NvMediaImageGetBits() expects three @a dstPntrs and three
 *             @a dstPitches, corresponding respectively to the Y, U and V
 *             planes.
 *    @endparblock
 * \param[in] dstPitches
 *      A pointer to an array of pitch values for the destination surface
 *      planes. There number of pitch values must be the same as the number of
 *      surface planes.
 * \retval  NVMEDIA_STATUS_OK indicates that the call was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more
 *  input parameters are invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that the image is not locked.
 * \ingroup image_get_put_bits
 */
NvMediaStatus
NvMediaImageGetBits(
    const NvMediaImage *image,
    const NvMediaRect *srcRect,
    void **dstPntrs,
    const uint32_t *dstPitches
);
#endif

/**@} <!-- Ends image_api Image Handling API --> */

/*
 * \defgroup history_nvmedia_image History
 * Provides change history for the NvMedia Image API.
 *
 * \section history_nvmedia_image Version History
 *
 * <b> Version 1.1 </b> March 3, 2016
 * - Initial release
 *
 * <b> Version 1.2 </b> May 11, 2016
 * - Added \ref NvMediaImageCheckVersion API
 *
 * <b> Version 1.3 </b> Jan 17, 2017
 * - Added atrribute NVMEDIA_IMAGE_ATTRIBUTE_CAPTURE in \ref NvMediaImageAttributes
 *
 * <b> Version 1.4 </b> Feb 9, 2017
 * - Added new APIs
 *       NvMediaImageCreateNew()
 *
 * <b> Version 1.5 </b> March 16, 2017
 * - Added \ref NVMEDIA_BITS_PER_PIXEL_16 in \ref NvMediaBitsPerPixel
 *
 * <b> Version 1.6 </b> May 9, 2017
 * - Deprecated the following APIs
 *     - NvMediaImageAcquire
 *     - NvMediaImageRelease
 *     - NvMediaImageWaitForIdle
 *     - NvMediaImageWaitForCompletion
 *     - NvMediaImageCheckVersion
 * - NvMediaImageGetVersion() is added to get the version of NvMedia Image library
 * - All NvMedia data types are moved to standard data types
 *
 * <b> Version 1.7 </b> May 14, 2017
 * - Added API to get status of operation on an image. \ref NvMediaImageGetStatus
 *
 * <b> Version 1.8 </b> May 15, 2017
 * - Added deprecated warning message for \ref NvMediaImageCreate,
 *   \ref NvMediaImageSiblingCreate, \ref NvMediaImageSiblingDestroy,
 *   \ref NvMediaVideoCreateImageSibling, \ref NvMediaImageCreateVideoSibling
 *
 * <b> Version 1.9 </b> May 18, 2017
 * - Added colorStd member in \ref NvMediaImage
 *
 * <b> Version 1.10 </b> June 12, 2017
 * - Added \ref NvMediaImageGroup to hold a list of NvMediaImages.
 *
 * <b> Version 1.11 </b> September 12, 2017
 * - Deprecated \ref NvMediaImageClass, \ref NvMediaImageAttributes, \ref NvMediaImageAdvancedConfig
 * - Deprecated \ref NvMediaImageCreate, \ref NvMediaImageSiblingCreate,
 *   \ref NvMediaVideoCreateImageSibling, \ref NvMediaImageSiblingDestroy,
 *   \ref NvMediaImageCreateVideoSibling, \ref NvMediaVideoSurfaceSiblingDestroy
 *
 * <b> Version 1.12 </b> Dec 06, 2018
 * - Fixed MISRA-C rule 10.4, 20.7 and 21.1 violations
 *   resulting from this header.
 *
 * <b> Version 1.13 </b> January 16, 2019
 * Updated desciption for \ref NvMediaImageGetBits and \ref NvMediaImagePutBits
 *
 * <b> Version 1.14 </b> March 13, 2019
 * - Data type of lockAcessType in NvMediaImageLock Api
     is changed from unit32_t to NvMediaImageLockAccess.
 *
 * <b> Version 1.15 </b> March 16, 2019
 * - Add required header include nvmedia_surface.h
 *
 * <b> Version 1.16 </b> March 20, 2019
 * - Added \ref NvMediaImageSetTag and \ref NvMediaImageGetTag APIs
 * - Tag member of NvMedia Image will be deprecated in later release.
 *
 * <b> Version 1.17 </b> March 22, 2019
 * - Added \ref NVMEDIA_RAW_PIXEL_ORDER_RCCB, \ref NVMEDIA_RAW_PIXEL_ORDER_BCCR,
 *   \ref NVMEDIA_RAW_PIXEL_ORDER_CRBC, \ref NVMEDIA_RAW_PIXEL_ORDER_CBRC, and
 *   \ref NVMEDIA_RAW_PIXEL_ORDER_COUNT
 *   in \ref NvMediaRawPixelOrder
 *
 * <b> Version 1.18 </b> March 25, 2019
 * - Added opaque pointer for image structure
 * - Fixed MISRA-C rule 8.13 violations resulting from this header.
 *
 *<b> Version 1.19 </b> March 28, 2019
 * - Unnecessary header include nvmedia_video.h has been removed
 *
 * <b> Version 1.20 </b> March 29, 2019
 * - Added captureTimeStamp to \ref NvMediaImageGroup to hold timestamps
 *
 *<b> Version 1.21 </b> March 28, 2019
 * - Move NvMediaBitsPerPixel and NVMEDIA_MAX_AGGREGATE_IMAGES to nvmedia_icp.h
 * - Move NvMediaRawPixelOrder to nvmedia_ipp.h
 *
 *<b> Version 1.22 </b> Dec 24, 2019
 * - Bypassed non-safety functions and non-safety NvMediaImage struct members
 *   using NV_IS_SAFETY macro to resolve MISRA-C rule 8.6 violations
 * - Fixed MISRA 2.4 for unused NvMediaImageRec tag.
 * - Changed arg type of NvMediaImageGetEmbeddedData from void*  to unit8_t*.
 *
 *<b> Version 1.23 </b> Jan 23, 2020
 *    Modify comments: Add warning for lock/unlock APIs and format support info in
 *    GetEmbeddedData
 *
 *<b> Version 1.24 </b> Feb 7, 2020
 *    Removed non-safety debug APIs on safety build
 *
 */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_IMAGE_H */
