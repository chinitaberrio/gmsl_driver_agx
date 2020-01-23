/*
 * Copyright (c) 2013-2019, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: 2D Processing Control </b>
 *
 * @b Description: This file contains the \ref image_2d_api "Image 2D
 *  Processing API."
 */

#ifndef NVMEDIA_2D_H
#define NVMEDIA_2D_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"
#include "nvmedia_image.h"
/**
 * \defgroup image_2d_api Image 2D Processing
 *
 * The Image 2D Processing API encompasses all NvMedia 2D image processing
 * related functionality.
 *
 * @ingroup nvmedia_image_top
 * @{
 */

/** \brief Major version number. */
#define NVMEDIA_2D_VERSION_MAJOR   3
/** \brief Minor version number. */
#define NVMEDIA_2D_VERSION_MINOR   7


/** \defgroup filter Surface Filtering */
/**
 * \brief Specifies filtering mode used for stretched blits.
 * \ingroup filter
 */
typedef enum
{
    /** Specifies disable the horizontal and vertical filtering. */
    NVMEDIA_2D_STRETCH_FILTER_OFF = 0x1,
    /** Specifies enable low quality filtering. */
    NVMEDIA_2D_STRETCH_FILTER_LOW,
    /** Specifies enable media quality filtering. */
    NVMEDIA_2D_STRETCH_FILTER_MEDIUM,
    /** Specifies enable the best quality filtering. */
    NVMEDIA_2D_STRETCH_FILTER_HIGH
} NvMedia2DStretchFilter;

/** \defgroup transform Transformations
 *
 * Transformations are used to rotate and mirror the source surface of
 * a blit operation.  The destination rectangle is not affected by any
 * transformation settings.
 *
 * @ref NvMediaTransform identifies the transformations that can be applied
 * as a combination of rotation and mirroring
 *
 * Specifically, given a hypothetical 2x2 image, applying these operations
 * would yield the following results
 * @code
 *
 *       INPUT                      OUTPUT
 *
 *      +---+---+                  +---+---+
 *      | 0 | 1 |     IDENTITY     | 0 | 1 |
 *      +---+---+ ---------------> +---+---+
 *      | 2 | 3 |                  | 2 | 3 |
 *      +---+---+                  +---+---+
 *
 *      +---+---+                  +---+---+
 *      | 0 | 1 |    ROTATE_90     | 1 | 3 |
 *      +---+---+ ---------------> +---+---+
 *      | 2 | 3 |                  | 0 | 2 |
 *      +---+---+                  +---+---+
 *
 *      +---+---+                  +---+---+
 *      | 0 | 1 |    ROTATE_180    | 3 | 2 |
 *      +---+---+ ---------------> +---+---+
 *      | 2 | 3 |                  | 1 | 0 |
 *      +---+---+                  +---+---+
 *
 *      +---+---+                  +---+---+
 *      | 0 | 1 |    ROTATE_270    | 2 | 0 |
 *      +---+---+ ---------------> +---+---+
 *      | 2 | 3 |                  | 3 | 1 |
 *      +---+---+                  +---+---+
 *
 *      +---+---+                  +---+---+
 *      | 0 | 1 |  FLIP_HORIZONTAL | 1 | 0 |
 *      +---+---+ ---------------> +---+---+
 *      | 2 | 3 |                  | 3 | 2 |
 *      +---+---+                  +---+---+
 *
 *      +---+---+                  +---+---+
 *      | 0 | 1 |  INVTRANSPOSE    | 3 | 1 |
 *      +---+---+ ---------------> +---+---+
 *      | 2 | 3 |                  | 2 | 0 |
 *      +---+---+                  +---+---+
 *
 *      +---+---+                  +---+---+
 *      | 0 | 1 |  FLIP_VERTICAL   | 2 | 3 |
 *      +---+---+ ---------------> +---+---+
 *      | 2 | 3 |                  | 0 | 1 |
 *      +---+---+                  +---+---+
 *
 *      +---+---+                  +---+---+
 *      | 0 | 1 |    TRANSPOSE     | 0 | 2 |
 *      +---+---+ ---------------> +---+---+
 *      | 2 | 3 |                  | 1 | 3 |
 *      +---+---+                  +---+---+
 * @endcode
 */

/**
 * \brief Defines transformations.
 * \ingroup transform
 **/
typedef enum
{
    /** Specifies no transformation. */
    NVMEDIA_TRANSFORM_NONE = 0x0,
    /** Specifies rotation by 90 degrees. */
    NVMEDIA_TRANSFORM_ROTATE_90,
    /** Specifies rotation by 180 degrees. */
    NVMEDIA_TRANSFORM_ROTATE_180,
    /** Specifies rotation by 270 degrees. */
    NVMEDIA_TRANSFORM_ROTATE_270,
    /** Specifies horizontal mirroring. */
    NVMEDIA_TRANSFORM_FLIP_HORIZONTAL,
    /** Specifies mirroring along a diagonal axis from top right to bottom left
     of the rectangular region. */
    NVMEDIA_TRANSFORM_INV_TRANSPOSE,
    /** Specifies vertical mirroring. */
    NVMEDIA_TRANSFORM_FLIP_VERTICAL,
    /** Specifies mirroring along a diagonal axis from top left to bottom right
     of the rectangular region. */
    NVMEDIA_TRANSFORM_TRANSPOSE
} NvMediaTransform;

/*---------------------------------------------------------*/
/** \defgroup blit Blits
 *
 * Blit functions define valid parameters for a blit.
 */

/**
 * \brief Defines bit masks for \ref NvMedia2DBlitParameters::validFields.
 * \ingroup blit
 */
typedef enum
{
    /** Specifies enable use of stretch filter. */
    NVMEDIA_2D_BLIT_PARAMS_FILTER               = (1u << 0),
    /** Specifies enabling use of blit flags. */
    NVMEDIA_2D_BLIT_PARAMS_FLAGS                = (1u << 1),
    /** Specifies enabling use of destination transform. */
    NVMEDIA_2D_BLIT_PARAMS_DST_TRANSFORM        = (1u << 2),
    /** Specifies enabling use of color space conversion standard. */
    NVMEDIA_2D_BLIT_PARAMS_COLOR_STD            = (1u << 3)
} NvMedia2DBlitParamField;

/**
 * Holds the additional parameters for a blit.
 * \a validFields is a mask which indicates which fields of the struct to read.
 *
 * \ingroup blit
 */
typedef struct
{
    /*! \brief  Holds flags which determine which fields in this structure
     are used.

     The following bit masks are valid, and may be ORed:
     - \ref NVMEDIA_2D_BLIT_PARAMS_FILTER
     - \ref NVMEDIA_2D_BLIT_PARAMS_FLAGS
     - \ref NVMEDIA_2D_BLIT_PARAMS_DST_TRANSFORM
    */
    uint32_t                        validFields;
    /*! Holds the filter mode. */
    NvMedia2DStretchFilter          filter;
    /*! Holds flags use to set \ref NVMEDIA_2D_BLIT_PARAMS_FLAGS. */
    uint32_t                        flags;
    /*! Holds destination transformation when
     \ref NVMEDIA_2D_BLIT_PARAMS_DST_TRANSFORM is set. */
    NvMediaTransform                dstTransform;
    /*! Holds the color space conversion standard when
     \ref NVMEDIA_2D_BLIT_PARAMS_COLOR_STD is set. */
    NvMediaColorStandard            colorStandard;
} NvMedia2DBlitParameters;

/**
 * Holds additional values returned from a blit.
 *
 * \ingroup blit
 */
typedef struct
{
    /* Reserved for future use. */
    uint32_t reserved[8];
} NvMedia2DBlitParametersOut;

/**
 * \brief Returns the version information for the NvMedia 2D library.
 * \param[out] version A pointer to an \ref NvMediaVersion structure
 *                    filled by the 2D library.
 * \return  A status code; NVMEDIA_STATUS_OK if successful, or
 *  NVMEDIA_STATUS_BAD_PARAMETER if @a version was invalid.
 */
NvMediaStatus
NvMedia2DGetVersion(
    NvMediaVersion *version
);

/**
 * \brief  An opaque handle representing an NvMedia2D object.
 */
typedef struct NvMedia2D NvMedia2D;

/**
 * \brief Creates a 2D object.
 * \param[in] device A pointer to the \ref NvMediaDevice device this 2D object
 *  is to use.
 * \return The new 2D object's handle if successful, or NULL otherwise.
 * \ingroup blit
 */
NvMedia2D *
NvMedia2DCreate(
    NvMediaDevice *device
);

/**
 * \brief Destroys a 2D object.
 * \param[in] i2d A pointer to the 2D object to be destroyed.
 * \ingroup blit
 */
void
NvMedia2DDestroy(
    NvMedia2D *i2d
);

/**
 * \brief Performs a 2D blit operation with supplementary return values.
 *
 * A blit transfers pixels from a source surface to a destination surface,
 * applying a variety of transformations to the pixel values on the way.
 *
 * \note  For a YUV surface type with 16-bit depth, only scale and crop are
 * supported. Format conversion is not supported.
 *
 * The interface is designed to simplify the typical uses of normal pixel copy
 * by not requiring advanced blit parameters to be set unless they are
 * actually required.
 *
 * Set \a params to NULL to invoke a standard pixel copy blit without
 * additional transformations. If the dimensions of the source rectangle do
 * not match the dimensions of the destination rectangle, the blit scales the
 * pixels to fit the destination rectangle. The filtering mode for scale
 * defaults to \ref NVMEDIA_2D_STRETCH_FILTER_LOW. You can use additional
 * filtering modes by setting the corresponding parameter in
 * \ref NvMedia2DBlitParameters.
 *
 * Set \a srcRect or \a dstRect to NULL to default to a source or destination
 * rectangle the size of the full source or destination surface.
 *
 * This example performs a straight pixel copy between surfaces of the same
 * dimensions (but not necessarily the same bit depth or color format):
 *
 * @code
 *      NvMedia2DBlitEx(i2d, dst, NULL, src, NULL, NULL, NULL);
 * @endcode
 *
 * \param[in]  i2d         A pointer to a i2d Image 2D object.
 * \param[in]  dstSurface  A pointer to a destination surface.
 * \param[in]  dstRect     A pointer to a destination rectangle. For a YUV
 *                          surface type with 16-bit depth, the x0 and y0
 *                          of \a dstRect must be 0.
 * \param[in]  srcSurface  A pointer to a source surface.
 * \param[in]  srcRect     A pointer to a source rectangle.
 * \param[in]  params      A pointer to parameters.
 * \param[out] paramsOut   Reserved for future use. Currently, set @a paramsOut
 *                          to NULL.
 * \return  NVMEDIA_STATUS_OK if succcessful, or
 *  NVMEDIA_STATUS_BAD_PARAMETER if any required pointer parameter was invalid.
 * \ingroup blit
 */
NvMediaStatus
NvMedia2DBlitEx(
    const NvMedia2D                 *i2d,
    NvMediaImage                    *dstSurface,
    const NvMediaRect               *dstRect,
    NvMediaImage                    *srcSurface,
    const NvMediaRect               *srcRect,
    const NvMedia2DBlitParameters   *params,
    NvMedia2DBlitParametersOut      *paramsOut
);

/**
 * \brief  Copies a plane of a YUV image to another YUV image.
 *
 * The source and the destination must have the same format.
 *
 * \param[in] i2d           An NvMedia 2D device handle.
 * \param[in] dstSurface    A pointer to the destination surface.
 * \param[in] dstPlane      The destination plane.
 * \param[in] srcSurface    A pointer to the source surface.
 * \param[in] srcPlane      The source plane.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the requested operation
 *  was not supported.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 * \ingroup blit
 */
NvMediaStatus
NvMedia2DCopyPlaneNew(
    const NvMedia2D     *i2d,
    NvMediaImage        *dstSurface,
    uint32_t            dstPlane,
    NvMediaImage        *srcSurface,
    uint32_t            srcPlane
);

/**
 * \brief  Performs an NvMedia2D weave operation on NvMedia images.
 *
 * The 2D weave operation takes frames containing odd and even lines
 * as input. The weave operation forms a destination frame by interleaving the
 * odd and even lines.
 *
 * The input frames must have the same format, which must be RAW8 or RGBA.
 * The output frame format must be RGBA; the output frame has the same
 * width as each input frame, and twice the height.
 *
 * \param[in]  i2d       An NvMedia 2D device handle.
 * \param[in]  imageOdd  A pointer to an NvMedia image representing odd lines.
 * \param[in]  imageEven A pointer to an NvMedia image representing even lines.
 * \param[out] outImage  A pointer to an NvMedia image which receivese weave
 *                        output.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_OUT_OF_MEMORY indicates that the operation ran out
 *  of memory.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the input surface types
 *  were not the same.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more
 *  pointer parameters were invalid.
 **/
NvMediaStatus
NvMedia2DWeaveNew(
    const NvMedia2D  *i2d,
    NvMediaImage     *imageOdd,
    NvMediaImage     *imageEven,
    NvMediaImage     *outImage
);

/**
 * \brief Registers an \ref NvMediaImage for use with an NvMedia2D handle.
 *
 * \note  This function is not supported in this release, and currently
 *  returns only the status code NVMEDIA_STATUS_NOT_SUPPORTED.)
 *
 * The NvMedia2D handle maintains a record of all the images registered by
 * this function.
 *
 * This is an optional function call. Skipping it results in nondeterministic
 * NvMedia2DBlitEx() execution time.
 * To ensure deterministic execution time for %NvMedia2DBlitEx():
 * - You must call NvMedia2DImageRegister() for every input and output
 *   \ref NvMediaImage to be used with NvMedia2D.
 * - You must make all calls to %NvMedia2DImageRegister() before the first
 *   call to %NvMedia2DBlitEx().
 *
 * You can register a maximum of 32 \ref NvMediaImage handles per access mode.
 *
 * \param[in] i2d        An NvMedia 2D device handle.
 * \param[in] image      A pointer to an NvMedia image.
 * \param[in] accessMode The access mode required for the image.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more of the
 *  parameters were invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that the application tried to
 *  register more than 32 images, register the same image with more than one
 *  @a accessMode, or register the same image more than once.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that this function currently
 *  is not supported.
 **/
NvMediaStatus
NvMedia2DImageRegister(
    const NvMedia2D  *i2d,
    NvMediaImage     *image,
    NvMediaAccessMode accessMode
);

/**
 * \brief  Unregisters an \ref NvMediaImage registered with NvMedia2D by a
 * call to NvMedia2DImageRegister().
 *
 * \note  This function currently is not fully implemented. It returns only
 *  the status code NVMEDIA_STATUS_NOT_SUPPORTED.
 *
 * This function must be called for all \ref NvMediaImage handles registered
 * with NvMedia2D before NvMedia2DDestroy() is called.
 *
 * To ensure deterministic execution of NvMedia2DBlitEx(), you must call
 * this function only after the last call to %NvMedia2DBlitEx().
 *
 * \param[in] i2d       An NvMedia 2D device handle.
 * \param[in] image     A pointer to an NvMedia image
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a i2d was invalid,
 *  the image had already been unregistered, or the image never was registered.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that this function currently
 *  is not supported.
 **/
NvMediaStatus
NvMedia2DImageUnRegister(
    const NvMedia2D  *i2d,
    NvMediaImage     *image
);

/*
 * \defgroup history_nvmedia_2d History
 * Provides change history for the NvMedia 2D API.
 *
 * \section history_nvmedia_2d Version History
 *
 * <b> Version 1.1 </b> February 1, 2016
 * - Initial release
 *
 * <b> Version 1.2 </b> May 11, 2016
 * - Added \ref NvMedia2DCheckVersion API
 *
 * <b> Version 1.3 </b> May 5, 2017
 * - Removed compositing, blending and alpha related defines and structures
 *
 * <b> Version 2.0 </b> May 11, 2017
 * - Deprecated NvMedia2DBlit API
 * - Deprecated NvMedia2DCheckVersion API
 * - Deprecated NvMedia2DColorStandard, NvMedia2DColorRange and
 *   NvMedia2DColorMatrix types
 * - Added \ref NvMedia2DGetVersion API
 *
 * <b> Version 2.1 </b> May 17, 2017
 * - Moved transformation to nvmedia_common.h
 * - Renamed NvMedia2DTransform to \ref NvMediaTransform
 *
 * <b> Version 2.2 </b> September 4, 2018
 * - Added deprecated warning message for \ref NvMedia2DCopyPlane,
 *   NvMedia2DWeave
 * - Added APIs \ref NvMedia2DCopyPlaneNew, \ref NvMedia2DWeaveNew
 *
 * <b> Version 3.0 </b> October 30, 2018
 * - Deprecated \ref NvMedia2DCopyPlane API
 * - Deprecated \ref NvMedia2DWeave API
 *
 * <b> Version 3.1 </b> December 11, 2018
 * - Fixed MISRA-C Rule 21.1 and 21.2 Violations
 *
 * <b> Version 3.2 </b> January 21, 2019
 * - Moved \ref NvMediaTransform from nvmedia_common.h to this header
 *
 * <b> Version 3.3 </b> Feb 21, 2019
 * - Changed \ref NvMedia2D type from void to struct
 *
 * <b> Version 3.4 </b> March 5, 2019
 * - Fixed MISRA-C Rule 8.13 Violations
 *
 * <b> Version 3.5 </b> March 14, 2019
 * - Removing NvMedia2DBlitFlags enum definition
 * - updated \ref NvMedia2DBlitParametersOut structure definition
 *
 * <b> Version 3.6 </b> March 18, 2019
 * - Added APIs \ref NvMedia2DImageRegister, \ref NvMedia2DImageUnRegister
 *
 * <b> Version 3.7 </b> March 22, 2019
 * - Unnecessary header include nvmedia_common.h has been removed
 */
/** @} */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_2D_H */
