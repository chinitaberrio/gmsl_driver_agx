/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved. All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 *
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: Lens Distortion Correction (LDC) </b>
 *
 * @b Description: This file contains the NvMedia LDC API.
 */

#ifndef NVMEDIA_LDC_H
#define NVMEDIA_LDC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_common.h"
#include "nvmedia_core.h"
#include "nvmedia_image.h"
#include "nvmedia_surface.h"

/**
 * \defgroup ldc_api Lens Distortion Correction
 *
 * The NvMedia Lens Distortion Correction (LDC) API contains NvMedia
 * functions for accessing the LDC hardware engine
 * for geometric transform and temporal noise reduction (TNR3).
 *
 * @ingroup nvmedia_top
 * @{
 */

/** \brief Major version number. */
#define NVMEDIA_LDC_VERSION_MAJOR   2
/** \brief Minor version number. */
#define NVMEDIA_LDC_VERSION_MINOR   1

/**
 * \brief Defines the NvMedia LDC handle as an opaque struct.
 */
typedef struct NvMediaLDC NvMediaLDC;

/** \brief Maximum number of horizontal regions. */
#define NVMEDIA_LDC_MAX_HOR_REGION    4u

/** \brief Maximum number of vertical regions. */
#define NVMEDIA_LDC_MAX_VER_REGION    4u

/** \brief Minimum width of horizontal and vertical regions. */
#define NVMEDIA_LDC_MIN_REGION_WIDTH  64u

/** \brief Minimum height of horizontal and vertical regions. */
#define NVMEDIA_LDC_MIN_REGION_HEIGHT 16u

/**
 * \brief Defines the NvMedia LDC operating modes.
 */
typedef enum {
    /*! Specifies geometric transform mode. */
    NVMEDIA_LDC_MODE_GEOTRANS,

    /*! Specifies temporal noise reduction version 3 mode. */
    NVMEDIA_LDC_MODE_TNR3,

    /*! Specifies both geometric transform mode and temporal noise reduction
     version 3 mode. */
    NVMEDIA_LDC_MODE_GEOTRANS_TNR3,

    /*! Specifies temporal noise reduction version 2 mode. */
    NVMEDIA_LDC_MODE_TNR2
} NvMediaLDCMode;

/**
 * \brief Defines geometric transform operation modes.
 */
typedef enum {
    /*! Specifies generating a sparse warp map for geometric transform. */
    NVMEDIA_GEOTRANS_MODE_GEN_MAPPING = 0,

    /*! Specifies taking a sparse warp map from client via
     * NvMediaLDCFeedSparseWarpMap(). */
    NVMEDIA_GEOTRANS_MODE_FEED_MAPPING,

    /*! Specifies performing an affine transform. */
    NVMEDIA_GEOTRANS_MODE_AFFINE_TRANSFORM,

    /*! Specifies performing a perspective transform. */
    NVMEDIA_GEOTRANS_MODE_PERSPECTIVE_TRANSFORM
} NvMediaGeoTransMode;

/**
 * \brief Defines supported lens models.
 */
typedef enum {
    /*! Specifies a polynomial distortion model. */
    NVMEDIA_LDC_MODEL_POLYNOMIAL_DISTORTION,

    /*! Specifies a fisheye model: r = 2ftan(theta/2), where theta is the angle
     from the optical axis, f is the focal length, and r is the distance of a
     pixel from the image center. */
    NVMEDIA_LDC_MODEL_FISHEYE_EQUIDISTANT,

    /*! Specifies a fisheye model: r = f*theta, where theta is the angle
     from the optical axis, f is the focal length, and r is the distance of a
     pixel from the image center. */
    NVMEDIA_LDC_MODEL_FISHEYE_EQUISOLID,

    /*! Specifies a fisheye model: r = 2fsin(theta/2), where theta is the angle
     from the optical axis, f is the focal length, and r is the distance of a
     pixel from the image center. */
    NVMEDIA_LDC_MODEL_FISHEYE_ORTHOGRAPHIC,

    /*! Specifies a fisheye model: r = fsin(theta), where theta is the angle
     from the optical axis, f is the focal length, and r is the distance of a
     pixel from the image center. */
    NVMEDIA_LDC_MODEL_FISHEYE_STEREOGRAPHIC
} NvMediaLensModel;

/**
 * \brief Holds NvMedia camera intrinsic parameters.
 */
typedef struct {
    /*! Holds the camera focal length in the X axis, in pixels. */
    float_t fx;

    /*! Holds the camera focal length in the Y axis, in pixels. */
    float_t fy;

    /*! Holds the optical center in the X axis, in pixels. */
    float_t cx;

    /*! Holds the optical center in the Y axis, in pixels. */
    float_t cy;
} NvMediaCamIntriParams;

/**
 * \brief Holds distortion coefficients for the lens model.
 *
 * Distortion coefficients are defined in the following way:
 * - k1, k2, k3, k4, k5, k6 are radial distortion coeffcients.
 * - p1 and p2 are tangential distortion coeffcients.
 *
 * Setting any coefficient to 0 implies that it is not used in computation.
 *
 * If we denote a point without distortion as [x, y, 1] and the corresponding
 * point with distortion as [xd, yd, 1], then the distortion model is defined
 * as follows:
 *
 * When \ref NvMediaLensModel is \ref NVMEDIA_LDC_MODEL_POLYNOMIAL_DISTORTION,
 * the control parameters are k1, k2, k3, k4, k5, k6, p1, and p2.
 * - \f$r = sqrt (x ^ 2 + y ^ 2)\f$
 * - \f$kr = (1 + k1 * r^2 + k2 * r^4 + k3 * r^6) / (1 + k4 * r^2 + k5 * r^4 + k6 * r^6)\f$
 * - \f$xd = x * kr + p1 * (2 * x * y) + p2 * (r^2 + 2 * x^2)\f$
 * - \f$yd = y * kr + p1 * (r^2 + 2 * y^2) + p2 * (2 * x * y)\f$
 *
 * When NvMediaLensModel is \ref NVMEDIA_LDC_MODEL_FISHEYE_EQUIDISTANT,
 * \ref NVMEDIA_LDC_MODEL_FISHEYE_EQUISOLID,
 * \ref NVMEDIA_LDC_MODEL_FISHEYE_ORTHOGRAPHIC, or
 * \ref NVMEDIA_LDC_MODEL_FISHEYE_STEREOGRAPHIC,
 *  the control parameters are k1, k2, k3, and k4.
 * - \f$r = sqrt (x ^ 2 + y ^ 2)\f$
 * - \f$theta = atan(r)\f$
 * - \f$theta_d = theta * (1 + k1 * theta^2 + k2 * theta^4 + k3 * theta^6 + k4 * theta^8)\f$
 */
typedef struct {
    /*! Holds the radial distortion coefficient. */
    float_t k1;

    /*! Holds the radial distortion coefficient. */
    float_t k2;

    /*! Holds the radial distortion coefficient. */
    float_t k3;

    /*! Holds the radial distortion coefficient. */
    float_t k4;

    /*! Holds the radial distortion coefficient. */
    float_t k5;

    /*! Holds the radial distortion coefficient. */
    float_t k6;

    /*! Holds the tangential distortion coefficient. */
    float_t p1;

    /*! Holds the tangential distortion coefficient. */
    float_t p2;
} NvMediaLensDistortion;

/**
 * \brief Holds NvMedia LDC camera parameter info.
 */
typedef struct {
    /*! Holds the camera model. */
    NvMediaLensModel model;

    /*! Holds the distortion coefficients. */
    NvMediaLensDistortion distCoeffs;

    /*! Holds the camera intrinsic matrix. */
    NvMediaCamIntriParams K;

    /*! Holds the rotation matrix. The matrix is defined as follows:
    \n           |r11  r12  r13|
    \n           |r21  r22  r23|
    \n           |r31  r32  r33|
    */
    float_t R[3][3];

    /*! Holds the translation vector. The vector is defined as follows:
     * \n         |t1 t2 t3|
     */
    float_t T[3];

    /*! Holds the target camera intrinsic matrix. */
    NvMediaCamIntriParams targetK;
} NvMediaCameraModel;

/**
 * \brief Holds the NvMedia LDC region configuration.
 *
 * This structure defines the layout of the control points in the destination
 * image.
 *
 * The control points are used as the basis for geometric transformation from
 * source image to destination image. The remaining points are transformed
 * based on the interpolation. Thus the density of the control points controls
 * the quality of the geometric transformation.
 *
 * This is an example of defining regions in the image:
 * \code
 *
 *
 *
 *            (dstRect->x1 - dstRect->x0)
 *         /                              \
 *        /                                \
 *       /                                  \
 *
 *   horRegionWidth[0]         horRegionWidth[numHorRegion -1]
 *       /     \                     /      \
 *      |-------|                   |--------|
 *      --------------------------------------                                      \
 *      |******** ******** ...      ******** |--                                     \
 *      |* +  + * *      * ...      *      * |   \                                    \
 *      |* +  + * *      *          *      * | verRegionHeight[0]                      \
 *      |* +  + * *      *          *      * |   /                                      \
 *      |******** ********          ******** |--                                         \
 *      |..                         ..       |                                            \
 *      |..                         ..       |
 *      |..                         ..       |                           (dstRect->y1 - dstRect->y0)
 *      |                                    |
 *      |                                    |                                            /
 *      |******** ********...       ******** |--                                         /
 *      |*      * *      *...       *      * |  \                                       /
 *      |*      * *      *          *      * | verRegionHeight[numVerRegion -1]        /
 *      |*      * *      *          *      * |  /                                     /
 *      |******** ********          ******** |--                                     /
 *      --------------------------------------
 *
 * \endcode
 *
 * This is an example of defining control points in one region:
 * \code
 *      *********
 *      *  +  + *-- \
 *      *       *     (1 << log2verSpace)
 *      *  +  + *-- /
 *      *       *
 *      *********
 *         |--|
 *       (1 << log2horSpace)
 *
 * \endcode
 *
 * ### Restrictions
 *
 * * numHorRegion cannot exceed \ref NVMEDIA_LDC_MAX_HOR_REGION.
 * * numVerRegion cannot exceed \ref NVMEDIA_LDC_MAX_VER_REGION.
 * * Alignment restrictions:
 *   - horRegionWidth[0] to horRegionWidth[numHorRegion-2] must be greater
 *     than and aligned to \ref NVMEDIA_LDC_MIN_REGION_WIDTH.
 *     horRegionWidth[numHorRegion-1] must be greater than
 *     \ref NVMEDIA_LDC_MIN_REGION_WIDTH.
 *   - verRegionHeight[0] to verRegionHeight[numVerRegion-2] must be greater
 *     than and aligned to \ref NVMEDIA_LDC_MIN_REGION_HEIGHT.
 *     verRegionHeight[numVerRegion-1] must be greater than
 *     \ref NVMEDIA_LDC_MIN_REGION_HEIGHT.
 * * Region width/height restrictions:
 *  - The sum of horRegionWidth[] must be equal to the width of the \a dstRect
 *    argument passed to NvMediaLDCCreateNew().
 *  - The sum of verRegionHeight[] must be equal to the height of \a dstRect
 *    argument passed to %NvMediaLDCCreateNew().
 */
typedef struct {
    /*! Holds the number of horizontal regions.
        Allowed values are [1, 4], inclusive. */
    uint8_t numHorRegion;

    /*! Holds the number of vertical regions.
        Allowed values are [1, 4], inclusive. */
    uint8_t numVerRegion;

    /*! Holds the width of regions. */
    uint16_t horRegionWidth[NVMEDIA_LDC_MAX_HOR_REGION];

    /*! Holds the height of regions. */
    uint16_t verRegionHeight[NVMEDIA_LDC_MAX_VER_REGION];

    /*! Holds the horizontal interval between the control points in each region
        in log2 space. */
    uint16_t log2horSpace[NVMEDIA_LDC_MAX_HOR_REGION];

    /*! Holds the vertical interval between the control points in each region
        in log2 space. */
    uint16_t log2verSpace[NVMEDIA_LDC_MAX_VER_REGION];
} NvMediaLDCRegionConfig;

/**
 * \brief Holds the NvMedia mask map surface.
 *
 * With the destination rectangle, this mask map surface defines the region of
 * interest in the destination image.
 * The \a dstRect argument passed to NvMediaLDCCreateNew() defines the
 * destination rectangle.
 */
typedef struct {
    /*! Holds the width in bytes of the mask map surface, which must be equal to
        the width of the destination rectangle. */
    uint16_t width;

    /*! Holds the height in bytes of the mask map surface, which must be equal
     * to the height of the destination rectangle. */
    uint16_t height;

    /*! Holds the value for the mask map surface. The mask map surface is stored
     * row by row. Each byte is used to indicate whether this pixel has been
     * masked or not. A non-zero pixel value means that the pixel is not to be
     * masked. The size of this buffer in bytes must be width * height.
     */
    void *mapPtr;

    /*! Holds a flag indicating whether to fill the masked pixel with the
     specified color. */
    NvMediaBool maskedPixelFillColor;
} NvMediaLDCBitMaskMap;

/**
 * \brief Holds geometric transform initialization paramters.
 */
typedef struct {
    /*! Holds  the geometric transform mode. */
    NvMediaGeoTransMode geoTransMode;

    /*! Holds the filter quality. */
    NvMediaFilterQuality filter;

    /*!
     * Holds the camera model parameters. Must be set when @a geoTransMode is
     * \ref NVMEDIA_GEOTRANS_MODE_GEN_MAPPING.
     */
    NvMediaCameraModel cameraModel;

    /*!
     * Holds the perspective matrix. Must be set when @a geoTransMode is
     *  \ref NVMEDIA_GEOTRANS_MODE_AFFINE_TRANSFORM or
     *  \ref NVMEDIA_GEOTRANS_MODE_PERSPECTIVE_TRANSFORM.
     *  Defined in the following way:
     *  \n        |p[0][0]  p[0][1]  p[0][2]|
     *  \n        |p[1][0]  p[1][1]  p[1][2]|
     *  \n        |p[2][0]  p[2][1]  p[2][2]|
     */
    float_t ptMatrix[3][3];

    /*! Specifies the region configure paramters. */
    NvMediaLDCRegionConfig regionConfig;

    /*! Holds the Y channel value of the default color. This value is clamped
     between 0.0 and 1.0. */
    float_t maskY;

    /*! Holds the U channel value of the default color. This value is clamped
     between 0.0 and 1.0. */
    float_t maskU;

    /*! Holds the V channel value of the default color. This value is clamped
     between 0.0 and 1.0. */
    float_t maskV;

    /*! Indicates whether bit mask map is enabled. */
    NvMediaBool bitMaskEnable;

    /*! Holds the bit mask map information when @a bitMaskEnable is NV_TRUE.
     * The width and height of this bit mask map must match the destination
     * rectangle.
     */
    NvMediaLDCBitMaskMap bitMaskMap;
} NvMediaGeoTransParams;

/**
 * \brief Holds the TNR3 initialization paramters.
 */
typedef struct {
    /*! Holds the sigma of the luma for spatial filter. */
    uint16_t spatialSigmaLuma;

    /*! Holds the sigma of the chroma for spatial filter. */
    uint16_t spatialSigmaChroma;

    /*! Holds the sigma of the luma for range filter. */
    uint16_t rangeSigmaLuma;

    /*! Holds the sigma of the chroma for range filter. */
    uint16_t rangeSigmaChroma;

    /*! Holds the SAD multiplier parameter.
     * This value is clamped between 0.0 and 1.0.
     */
    float_t sadMultiplier;

    /*! Holds the weight of luma when calculating SAD.
     *  This value is clamped between 0.0 and 1.0.
     */
    float_t sadWeightLuma;

    /*! Holds a flag which enables or disables the spatial alpha smooth. */
    NvMediaBool alphaSmoothEnable;

    /*! Holds the temporal alpha restrict increase capablility.
     *  This value is clamped between 0.0 and 1.0.
     */
    float_t alphaIncreaseCap;

    /*! Holds the alpha scale IIR for strength.
     *  This value is clamped between 0.0 and 1.0.
     */
    float_t alphaScaleIIR;

    /*! Holds the max luma value in Alpha Clip Calculation.
     *  This value is clamped between 0.0 and 1.0.
     */
    float_t alphaMaxLuma;

    /*! Holds the min luma value in Alpha Clip Calculation.
     *  This value is clamped between 0.0 and 1.0.
     */
    float_t alphaMinLuma;

    /*! Holds the max chroma value in Alpha Clip Calculation.
     *  This value is clamped between 0.0 and 1.0.
     */
    float_t alphaMaxChroma;

    /*! Holds the min chroma value in Alpha Clip Calculation.
     *  This value is clamped between 0.0 and 1.0.
     */
    float_t alphaMinChroma;

    /*! Holds parameter BetaX1 in Beta Calculation.
     *  This value is clamped between 0.0 and 1.0.
     */
    float_t betaX1;

    /*! Holds parameter BetaX2 in Beta Calculation.
     *  This value is clamped between 0.0 and 1.0.
     */
    float_t betaX2;

    /*! Holds parameter MinBeta threshold in Beta Calculation.
     *  This value is clamped between 0.0 and 1.0.
     */
    float_t minBeta;

    /*! Holds parameter MaxBeta threshold in Beta Calculation.
     *  This value is clamped between 0.0 and 1.0.
     */
    float_t maxBeta;
} NvMediaTNR3Params;

/**
 * \brief Holds the TNR2 initialization paramters.
 */
typedef struct {
    /*! Holds the noise reduction strength. This value is clamped between 0.0
     and 1.0. */
    float_t noiseReduction;
    /*! Holds the noise reduction algorithm. See
     \ref NvMediaNoiseReductionAlgorithm. */
    NvMediaNoiseReductionAlgorithm noiseReductionAlgorithm;
} NvMediaTNR2Params;

/**
 * \brief Holds the NvMedia LDC initialization paramters.
 */
typedef struct {
    /*! Holds the LDC mode. */
    NvMediaLDCMode ldcMode;

    /*! Holds geometric transform initialization paramters.
     *  Must be set when @a ldcMode is
     *  \ref NVMEDIA_LDC_MODE_GEOTRANS or \ref NVMEDIA_LDC_MODE_GEOTRANS_TNR3.
     */
    NvMediaGeoTransParams geoTransParams;

    /*! Holds TNR3 initialization parameters.
     *  Must be set when @a ldcMode is
     *  \ref NVMEDIA_LDC_MODE_TNR3 or \ref NVMEDIA_LDC_MODE_GEOTRANS_TNR3.
     */
    NvMediaTNR3Params tnr3Params;

    /*! Holds TNR2 initialization paramters.
     *  Must be set when @a ldcMode is
     *  \ref NVMEDIA_LDC_MODE_TNR2.
     */
    NvMediaTNR2Params tnr2Params;

    /*! Holds source format type for LDC or TNR operation. Must be set. */
    NvMediaSurfaceType srcSurfaceType;
} NvMediaLDCInitParams;

/**
 * \brief Defines the NvMedia LDC Data Format.
 */
typedef enum {
    /*! Specifies standard float_t format. */
    NVMEDIA_LDC_DATAFORMAT_FLOAT_T,

    /*! Specifies S15.5 fixed floating format. Data is stored only
        in the most significant 20 bits of a 32-bit (four-byte) value. */
    NVMEDIA_LDC_DATAFORMAT_FIXEDFLOAT_S15_5
} NvMediaLDCDataFormat;

/**
 * \brief Holds the NvMedia LDC defintion of a sparse warp map.
 *
 * A sparse warp map stores the mappings of each control point from destination
 * image to input image. The coordinates of control points in a destination
 * image are defined by \ref NvMediaLDCRegionConfig.
 */
typedef struct {
    /*! Holds the number of control points in each row. */
    uint16_t numHorPoints;

    /*! Holds the number of control points in each column. */
    uint16_t numVerPoints;

    /*! Holds the number of control points in one stride. */
    uint16_t mapStride;

    /*! Holds the mapping value for each of the control points. While the
     *  control points
     *  are stored in row order, the coordinates for each control point are
     *  stored in order of (x,y) where x is the horizontal coordinate and y is
     *  the vertical coordinate.
     *  The size of this buffer in bytes must be:
     *  mapStride * numVerPoints * sizeof(float_t) * 2.
     */
    void *mapPtr;

    /*! Holds the data format used in the buffer which @a mapPtr points to.
     *  Supported formats are as specified by \ref NvMediaLDCDataFormat.
     */
    NvMediaLDCDataFormat dataFormat;
} NvMediaLDCSparseWarpMap;

/**
 * \brief Defines the xSobel working modes.
 */
typedef enum {
    /*! Specifies disabling both Sobel and downsample output. */
    NVMEDIA_GEOTRANS_DISABLE_XSOBEL_DISABLE_DS = 0,

    /*! Specifies producing luma Sobel and luma downsample output. */
    NVMEDIA_GEOTRANS_DISABLE_XSOBEL_ENABLE_DS,

    /*! Specifies producing gradient Sobel and disable downsample output. */
    NVMEDIA_GEOTRANS_ENABLE_XSOBEL_DISABLE_DS,

    /*! Specifies producing gradient Sobel and gradient downsample output. */
    NVMEDIA_GEOTRANS_ENABLE_XSOBEL_ENABLE_DS
} NvMediaLDCSobelMode;

/**
 * \brief Holds runtime control parameters for NvMediaLDCProcess().
 */
typedef struct {
    /*! Holds the xSobel working mode.
     *  Supported modes are specified by \ref NvMediaLDCSobelMode.
     *  \n Used when @a NvMediaLDCInitParams::ldcMode is
     *  \ref NVMEDIA_LDC_MODE_GEOTRANS. In other cases, set this field to NULL.
     */
    NvMediaLDCSobelMode xSobelMode;
} NvMediaLDCCtrlParams;

/**
 * \brief Returns the version information of NvMedia LDC.
 *
 * \param[in] version A pointer to a structure to be  filled by the NvMedia LDC.
 * \return A status code; NVMEDIA_STATUS_OK if successful, or
 *  NVMEDIA_STATUS_BAD_PARAMETER if @a version is invalid.
 */
NvMediaStatus
NvMediaLDCGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Creates an NvMedia LDC handle.
 *
 * The function creates and initializes the appropriate internal infrastructure
 * for LDC or TNR3, depending on the input arguments. It returns a valid
 * NvMedia LDC handle if successful.
 *
 * \param[in]  device     An NvMediaDevice handle.
 * \param[out] pldc       A pointer to an LDC handle.
 * \param[in]  srcWidth   Width of the source image.
 * \param[in]  srcHeight  Height of the source image.
 * \param[in]  srcRect    A pointer to coordinates of the rectangle in the
 *                         source image. Set to NULL to specify a rectangle of
 *                         full source image size.
 * \param[in]  dstWidth   Width of the destination image.
 * \param[in]  dstHeight  Height of the destination image.
 * \param[in]  dstRect    A pointer to coordinates of the rectangle in the
 *                         destination image. Set to NULL to specify a rectangle
 *                         of full destination image size.
                           DestRect's top and left corner is limited to (0,0).
 * \param[in]  initParams A pointer to initialization parameters for creating
 *                         an instance of LDC. See \ref NvMediaLDCInitParams.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 *  @a pldc points to a valid LDC handle.
 * \retval  NVMEDIA_STATUS_NOT_INITIALIZED indicates that the device handle
 *  was in an invalid state.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the operation is not
 *  supported in the current configuration.
 * \retval  NVMEDIA_STATUS_OUT_OF_MEMORY indicates that there was insufficient
 *  memory to perform the operation.
 */
NvMediaStatus
NvMediaLDCCreateNew(
    const NvMediaDevice *device,
    NvMediaLDC **pldc,
    const uint16_t srcWidth,
    const uint16_t srcHeight,
    const NvMediaRect *srcRect,
    const uint16_t dstWidth,
    const uint16_t dstHeight,
    const NvMediaRect *dstRect,
    const NvMediaLDCInitParams *initParams);

/**
 * \brief Destroys an NvMedia LDC handle that was created by a call to
 *  NvMediaLDCCreateNew().
 *
 * This function frees internal resources and handles allocated by
 * %NvMediaLDCCreateNew().
 *
 * \param[in] ldc A handle to the current context, returned by
 *  %NvMediaLDCCreateNew().
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful. The
 *  handle @a ldc was destroyed.
 * \retval  NVMEDIA_STATUS_NOT_INITIALIZED indicates that the device handle
 *  was in an invalid state.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a ldc is not a valid
 *  LDC handle.
 * \retval  NVMEDIA_STATUS_ERROR indicates that one or more objects were
 *  unregistered.
 */
NvMediaStatus
NvMediaLDCDestroy(
    NvMediaLDC *ldc
);

/**
 * \brief  Feeds a sparse warp map to NvMedia LDC.
 *
 * \ref NvMediaGeoTransMode must be \ref NVMEDIA_GEOTRANS_MODE_FEED_MAPPING
 * when this function is called.
 * The sparse warp map must match the \ref NvMediaLDCRegionConfig.
 *
 * \param[in] ldc   A handle to the current context, returned by
 *                   NvMediaLDCCreateNew().
 * \param[in] map   A pointer to a client-specified sparse warp map.
 *
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_NOT_INITIALIZED indicates that the current context
 *  was in an invalid state.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the operation is not
 *  supported in the current configuration.
 * \retval  NVMEDIA_STATUS_OUT_OF_MEMORY indicates that there was insufficient
 *  memory to perform the operation.
 */
NvMediaStatus
NvMediaLDCFeedSparseWarpMap(
    NvMediaLDC *ldc,
    const NvMediaLDCSparseWarpMap *map);

/**
 * \brief Generates a sparse warp mapping based on \ref NvMediaCameraModel.
 *
 * \ref NvMediaGeoTransMode must be \ref NVMEDIA_GEOTRANS_MODE_GEN_MAPPING
 * when this function is called.
 *
 * This is a CPU-based operation.
 *
 * \param[in] ldc   A handle to the current context, returned by
 *                   NvMediaLDCCreateNew().
 *
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_NOT_INITIALIZED indicates that the current context
 *  was in an invalid state.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a ldc was invalid.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the operation is not
 *  supported in the current configuration.
 */
NvMediaStatus
NvMediaLDCMappingGen(
    NvMediaLDC *ldc);

/**
 * \brief Updates TNR3 parameters after a call to NvMediaLDCCreateNew().
 *
 * %NvMediaLDCCreateNew() initializes \ref NvMediaLDC with TNR3 parameters
 * specified as part of \ref NvMediaLDCInitParams.
 *
 * This function can be used to update the TNR3 parameters if necessary.
 *
 * \param[in] ldc           A handle to the current context, returned by
 *                           %NvMediaLDCCreateNew().
 * \param[in] tnr3Params    A pointer to an object which holds TNR3 parameters.
 * \return \ref NvMediaStatus. Status indicator.
 *
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_NOT_INITIALIZED indicates that the current context
 *  was in an invalid state.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the operation is not
 *  supported in the current configuration.
 */
NvMediaStatus
NvMediaLDCUpdateTNR3Params(
    const NvMediaLDC *ldc,
    const NvMediaTNR3Params *tnr3Params);

/**
 * \brief Performs a specified LDC operation.
 *
 * LDC performs transformations depending on the GEO mode:
 *
 * 1. LDC performs a geometric transformation if NvMediaLDCInitParams::ldcMode is
 *    \ref NVMEDIA_LDC_MODE_GEOTRANS or \ref NVMEDIA_LDC_MODE_GEOTRANS_TNR3.
 *    It uses \ref NvMediaLDCInitParams::geoTransParams to fetch the pixel defined
 *    by @a srcRect in the source image and renders onto the destination
 *    rectangle of the destination image.
 *
 *    The source image and the destination image must have the same format.
 *
 *    LDC bypasses the transform stage if geometric transform is disabled.
 *
 *    The region of interest in the destination image is defined by:
 *    - Destination rectangle and
 *    - NvMediaLDCBitMaskMap surface
 *      (if \ref NvMediaGeoTransParams::bitMaskEnable is set).
 * \n\n
 * 2. LDC outputs \a xSobel if:
 *    - \ref NvMediaLDCInitParams::ldcMode is \ref NVMEDIA_LDC_MODE_GEOTRANS and
 *    - \ref NvMediaLDCCtrlParams::xSobelMode is
 *      \ref NVMEDIA_GEOTRANS_ENABLE_XSOBEL_ENABLE_DS or
 *      \ref NVMEDIA_GEOTRANS_ENABLE_XSOBEL_DISABLE_DS.
 *    \n The xSobel image must have the same bit-depth as the source image.
 *    NvMediaLDCCtrlParams::xSobelMode must be NULL if
 *    NvMediaLDCInitParams::ldcMode is not NVMEDIA_LDC_MODE_GEOTRANS.
 * \n\n
 * 3. LDC outputs 4x4 downsampled \a xSobel output if:
 *    - NvMediaLDCInitParams::ldcMode is NVMEDIA_LDC_MODE_GEOTRANS and
 *    - NvMediaLDCCtrlParams::xSobelMode is
 *      NVMEDIA_GEOTRANS_ENABLE_XSOBEL_ENABLE_DS or
 *      \ref NVMEDIA_GEOTRANS_DISABLE_XSOBEL_ENABLE_DS.
 *    \n The downSample image must have the same bit depth as the source image.
 *    Set \a downSample to NULL if no downsample output is wanted.
 *
 * If NvMediaLDCInitParams::ldcMode is \ref NVMEDIA_LDC_MODE_TNR3 or
 * NVMEDIA_LDC_MODE_GEOTRANS_TNR3, LDC performs temporal noise reduction.
 *
 * %NvMediaLDCProcess() is a non-blocking call.
 *
 * Use NvMediaImageGetStatus() to check the status of input/output images.
 *
 * \param[in]    ldc            An LDC handle, obtained by a call to
 *                               NvMediaLDCCreateNew().
 * \param[in]     prevSurface   A pointer to the previous image output. Used
 *                               only when NvMediaLDCInitParams::ldcMode is
 *                               NVMEDIA_LDC_MODE_GEOTRANS_TNR3 or
 *                               NVMEDIA_LDC_MODE_TNR3.
 * \param[in]     curSurface    A pointer to the source image.
 * \param[in,out] outputSurface A pointer to the output image. The application
 *                               must create this surface.
 * \param[in,out] xSobel         A pointer to the xSobel image output. The
 *                               application must create this surface.
 * \param[in,out] downSample    A pointer to the downsampled image output. The
 *                               application must create this surface.
 * \param[in]     ldcCtrlParams A pointer to run-time control parameters.
 *
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_NOT_INITIALIZED indicates that the current context
 *  was in an invalid state.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the operation is not
 *  supported in the current configuration.
 * \retval  NVMEDIA_STATUS_OUT_OF_MEMORY indicates that there was insufficient
 *  memory to perform the operation.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaLDCProcess(
    const NvMediaLDC *ldc,
    NvMediaImage *prevSurface,
    NvMediaImage *curSurface,
    NvMediaImage *outputSurface,
    NvMediaImage *xSobel,
    NvMediaImage *downSample,
    const NvMediaLDCCtrlParams *ldcCtrlParams
);

/** @} <!-- Ends ldc_api Lens Distortion Correction sub-group -> */

/*
 * \defgroup history_nvmedia_ldc History
 * Provides change history for the NvMedia LDC API.
 *
 * \section history_nvmedia_ldc Version History
 *
 * <b> Version 1.0 </b> May 1, 2017
 * - Initial release.
 *
 * <b> Version 1.1 </b> March 16, 2018
 * - Add support of TNR2
 *
 * <b> version 1.2 </b> September 4, 2018
 * - New member variable srcSurfaceType is added to NvMediaLDCInitParams.
 *
 * <b> version 1.3 </b> December 26, 2018
 * - Adding unsigned tag to macro constants to fix MISRA 10.4 violation.
 * - Fixing MISRA 21.1 violations
 *
 * <b> version 1.4 </b> January 2, 2019
 * - Added deprecated warning message for \ref NvMediaLDCCreate.
 * - Added API \ref NvMediaLDCCreateNew
 *
 * <b> version 1.5 </b> March 6, 2019
 * - Fixing MISRA 8.13 violations.
 *
 * <b> Version 1.6 </b> March 12, 2019
 * - Added required header includes nvmedia_core.h and nvmedia_surface.h
 *
 * <b> version 2.0 </b> March 29, 2019
 * - Deprecated NvMediaLDCCreate API.
 *
 * <b> version 2.1 </b> January 23, 2020
 * - Limited destination surface rectangle's top, left corner to (0, 0).
 */

#ifdef __cplusplus
}
#endif

#endif // NvMedia_LDC_H
