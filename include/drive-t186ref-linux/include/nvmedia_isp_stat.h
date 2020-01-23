/*
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NvMedia ISP stat struct </b>
 */

#ifndef NVMEDIA_ISP_STAT_H
#define NVMEDIA_ISP_STAT_H

#include <stdint.h>

#include "nvmedia_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup nvmedia_isp_stat Image Signal Processing (ISP) Statistics
 *
 * NvMedia ISP Defines NvMedia ISP Stat structures.
 *
 * @ingroup nvmedia_image_top
 * @{
 */

/**
 * \brief Maximum number of input planes.
 */
#define NVM_ISP_MAX_INPUT_PLANES            (3U)

/**
 * \brief Maximum number of color components.
 */
#define NVM_ISP_MAX_COLOR_COMPONENT         (4U)

/**
 * \brief Maximum matrix dimension.
 */
#define NVM_ISP_MAX_COLORMATRIX_DIM         (3U)

/**
 * \brief Number of radial transfer function control points.
 */
#define NVM_ISP_RADTF_POINTS                (6U)

/**
 * \brief Number of histogram knee points.
 */
#define NVM_ISP_HIST_KNEE_POINTS            (8U)

/**
 * \brief Maximum number of local average and clip statistic block
 * regions of interest.
 */
#define NVM_ISP_MAX_LAC_ROI                 (4U)

/**
 * \brief Maximum number of windows for local average and clip in a region of
 * interest.
 */
#define NVM_ISP_MAX_LAC_ROI_WINDOWS         (32U * 32U)

/**
 * \brief Number of histogram bins.
 */
#define NVM_ISP_HIST_BINS                   (256U)

/**
 * \brief Number of histogram bins for local tone map statistics block.
 */
#define NVM_ISP_LTM_HIST_BINS               (128U)

/**
 * \brief Number of averaging windows for local tone map statistics block.
 */
#define NVM_ISP_LTM_AVG_WINDOWS             (8U)

/**
 * \brief Maximum number of windows for local average and clip in a region of
 * interest.
 */
#define NVM_ISP_MAX_FB_BANDS                (256U)

/**
 * \brief Defines a spline control point.
 */
typedef struct {
    /**
     * Holds X coordinate of the control point.
     */
    float_t x;
    /**
     * Holds Y coordinate of the control point.
     */
    float_t y;
    /**
     * Holds slope of the spline curve at the control point.
     */
    double_t slope;
} NvMediaISPSplineControlPoint;

/**
 * \brief Defines an ellipse.
 */
typedef struct {
    /**
     * Holds center of the ellipse.
     */
    NvMediaPoint center;
    /**
     * Holds horizontal axis of the ellipse.
     */
    uint32_t horizontalAxis;
    /**
     * Holds vertical axis of the ellipse.
     */
    uint32_t verticalAxis;
    /**
     * Holds angle of the ellipse horizontal axis from X axis in degrees in
     * clockwise direction.
     */
    float_t angle;
} NvMediaISPEllipse;

/**
 * \brief Defines a radial transform.
 */
typedef struct {
    /**
     * Holds ellipse for radial transform.
     *
     * Coordinates of image top left and bottom right points are (0, 0) &
     * (width, height) respectively.
     *
     * @li Supported values for X coordinate of the center: [0, input width]
     * @li Supported values for Y coordinate of the center: [0, input height]
     * @li Supported values for hortizontal axis: [16, 2 x input width]
     * @li Supported values for vertical axis: [16, 2 x input height]
     * @li Supported values for angle: [0.0, 360.0]
     */
    NvMediaISPEllipse radialTransform;
    /**
     * Defines spline control point for radial transfer function.
     * @li Supported values for X coordinate of spline control point : [0.0, 2.0]
     * @li Supported values for Y coordinate of spline control point : [0.0, 2.0]
     * @li Supported values for slope of spline control point : \f$[-2^{16}, 2^{16}]\f$
     */
    NvMediaISPSplineControlPoint controlPoints[NVM_ISP_RADTF_POINTS];
} NvMediaISPRadialTF;

/**
 * \brief Holds controls for histogram statistics (HIST Stats).
 */
typedef struct {
    /**
     * Holds a Boolean to enable histogram statistics block.
     */
    NvMediaBool enable;
    /**
     * Holds offset to be applied to input data prior to bin mapping.
     * @li Supported values: [-2.0, 2.0]
     */
    float_t offset;
    /**
     * Holds bin index specifying different zones in the histogram. Each zone
     * can have a different number of bins.
     * @li Supported values: [1, 255]
     */
    uint8_t knees[NVM_ISP_HIST_KNEE_POINTS];
    /**
     * Holds \f$log_2\f$ range of the pixel values to be considered for each
     * zone. The whole pixel range is divided into NVM_ISP_HIST_KNEE_POINTS
     * zones.
     * @li Supported values: [0, 21]
     */
    uint8_t ranges[NVM_ISP_HIST_KNEE_POINTS];
    /**
     * Holds a rectangular mask for excluding pixels outside a specified area.
     *
     * The coordinates of image top left and bottom right points are (0, 0) and
     * (width, height), respectively.
     *
     * The rectangle must be within the input image, and must be a valid
     * rectangle ((right &gt; left) && (bottom &gt; top)), or must be
     * ((0,0), (0,0)) to disable the rectangular mask.
     */
    NvMediaRect rectangularMask;
    /**
     * Holds a Boolean to enable an elliptical mask for excluding pixels
     * outside a specified area.
     */
    NvMediaBool ellipticalMaskEnable;
    /**
     * Holds an elliptical mask for excluding pixels outside a specified area.
     *
     * Coordinates of the image top left and bottom right points are (0, 0) and
     * (width, height), respectively.
     *
     * @li Supported values for X coordinate of the center: [0, input width]
     * @li Supported values for Y coordinate of the center: [0, input height]
     * @li Supported values for hortizontal axis: [16, 2 x input width]
     * @li Supported values for vertical axis: [16, 2 x input height]
     * @li Supported values for angle: [0.0, 360.0]
     */
    NvMediaISPEllipse ellipticalMask;
    /**
     * Holds a Boolean to enable elliptical weighting of pixels based on spatial
     * location. This can be used to compensate for lens shading when the
     * histogram is measured before lens shading correction.
     */
    NvMediaBool ellipticalWeightEnable;
    /**
     * Holds a radial transfer function for elliptical weight.
     * @li Supported values: Check the declaration of \ref NvMediaISPRadialTF.
     */
    NvMediaISPRadialTF radialTF;
} NvMediaISPHistogramStats;

/**
 * \brief Defines the windows used in ISP stats calculations.
 *
 * \code
 * ------------------------------------------------------------------------------
 * |         startOffset    horizontalInterval                                  |
 * |                    \  |--------------|                                     |
 * |                     - *******        *******        *******                |
 * |                     | *     *        *     *        *     *                |
 * |                     | *     *        *     *        *     *                |
 * |                     | *     *        *     *        *     *                |
 * |                     | *******        *******        *******                |
 * |  verticalInterval-->|                                        \             |
 * |                     |                                          numWindowsV |
 * |                     |                                        /             |
 * |                     - *******        *******        *******                |
 * |                     | *     *        *     *        *     *                |
 * |            height-->| *     *        *     *        *     *                |
 * |                     | *     *        *     *        *     *                |
 * |                     - *******        *******        *******                |
 * |                       |-----|                                              |
 * |                        width     \      |     /                            |
 * |                                    numWindowsH                             |
 * ------------------------------------------------------------------------------
 * \endcode
 */
typedef struct {
    /**
     * Holds width of the window in pixels.
     */
    uint32_t width;
    /**
     * Holds height of the window in pixels.
     */
    uint32_t height;
    /**
     * Holds number of windows horizontally.
     */
    uint32_t numWindowsH;
    /**
     * Holds number of windows vertically.
     */
    uint32_t numWindowsV;
    /**
     * Holds the distance between the left edge of one window and a horizontally
     * adjacent window.
     */
    uint32_t horizontalInterval;
    /**
     * Holds the distance between the top edge of one window and a vertically
     * adjacent window.
     */
    uint32_t verticalInterval;
    /**
     * Holds the position of the top left pixel in the top left window.
     */
    NvMediaPoint startOffset;
} NvMediaISPStatisticsWindows;

/**
 * \brief Holds controls for local average and clip statistics (LAC Stats).
 */
typedef struct {
    /**
     * Holds a Boolean to enable the local average and clip statistics block.
     */
    NvMediaBool enable;
    /**
     * Holds minimum value of pixels in RGGB/RCCB/RCCC order.
     * @li Supported values: [0.0, 1.0]
     */
    float_t min[NVM_ISP_MAX_COLOR_COMPONENT];
    /**
     * Holds maximum value of pixels in RGGB/RCCB/RCCC order.
     * @li Supported values: [0.0, 1.0], max >= min
     */
    float_t max[NVM_ISP_MAX_COLOR_COMPONENT];
    /**
     * Holds a Boolean to enable an indiviual region of interest.
     */
    NvMediaBool roiEnable[NVM_ISP_MAX_LAC_ROI];
    /**
     * Holds local average and clip windows for each region of interest.
     * @li Supported values for width of the window: [2, 256];
     *  must be an even number
     * @li Supported values for height of the window: [2, 256]
     * @li Supported values for number of the windows horizontally: [1, 32]
     * @li Supported values for number of the windows vertically: [1, 32]
     * @li Supported values for horizontal interval between windows:
     *  \f$[4, 2^{16})\f$; must be an even number
     * @li Supported values for vertical interval between windows:
     *  \f$[2, 2^{16})\f$
     * @li Supported values for X coordinate of start offset:
     *  \f$[0, 2^{16})\f$; must be an even number
     * @li Supported values for Y coordinate of start offset: \f$[0, 2^{16})\f$
     */
    NvMediaISPStatisticsWindows windows[NVM_ISP_MAX_LAC_ROI];
    /**
     * Holds a Boolean to enable an elliptical mask for excluding pixels
     * outside a specified area for each region of interest.
     */
    NvMediaBool ellipticalMaskEnable[NVM_ISP_MAX_LAC_ROI];
    /**
     * Holds an elliptical mask for excluding pixels outside specified area.
     *
     * Coordinates of the image's top left and bottom right points are (0, 0)
     *  and (width, height), respectively.
     *
     * @li Supported values for X coordinate of the center: [0, input width]
     * @li Supported values for Y coordinate of the center: [0, input height]
     * @li Supported values for hortizontal axis: [16, 2 x input width]
     * @li Supported values for vertical axis: [16, 2 x input height]
     * @li Supported values for angle: [0.0, 360.0]
     */
    NvMediaISPEllipse ellipticalMask;
} NvMediaISPLocalAvgClipStats;

/**
 * \brief Holds controls for bad pixel statistics (BP Stats).
 */
typedef struct {
    /**
     * Holds a Boolean to enable the bad pixel statistics block.
     * \note Bad Pixel Correction must also be enabled to get bad pixel
     *  statistics.
     */
    NvMediaBool enable;
    /**
     * Holds rectangular mask for excluding pixel outside a specified area.
     *
     * Coordinates of the image's top left and bottom right points are (0, 0)
     * and(width, height), respectively. Either set the rectangle's dimensions
     ( to 0 or set the rectangle to include the full image with no rectangular
     * mask.
     *
     * @li Supported values: Rectangle must be within the input image and must
     *  be a valid rectangle ((right > left) && (bottom > top)). The minimum
     *  supported rectangular mask size is 4x4.
     * Constraints: All left, top, bottom, and right coordinates must be even.
     */
    NvMediaRect rectangularMask;
} NvMediaISPBadPixelStats;

/**
 * \brief Holds controls for local tone map statistics (LTM Stats).
 */
typedef struct {
    /**
     * Holds a Boolean to enable the local tonemap statistics block.
     */
    NvMediaBool enable;
    /**
     * Holds a rectangular mask for excluding pixels outside a specified area.
     *
     * Coordinates of the image's top left and bottom right points are (0, 0)
     * and (width, height), respectively. Either set the rectangle's dimensions
     * to 0 or set the rectangle to include the full image with no rectangular
     * mask.
     *
     * @li Supported values: Rectangle must be within the input image and must
     *  be a valid rectangle ((right > left) && (bottom > top)).
     */
    NvMediaRect rectangularMask;
    /**
     * Holds a Boolean to enable an elliptical mask for excluding pixels
     * outside a specified area.
     */
    NvMediaBool ellipticalMaskEnable;
    /**
     * Holds an elliptical mask for excluding pixels outside a specified area.
     *
     * Coordinates of the image's top left and bottom right points are (0, 0)
     * and (width, height), respectively.
     *
     * @li Supported values for X coordinate of the center: [0, input width]
     * @li Supported values for Y coordinate of the center: [0, input height]
     * @li Supported values for hortizontal axis: [16, 2 x input width]
     * @li Supported values for vertical axis: [16, 2 x input height]
     * @li Supported values for angle: [0.0, 360.0]
     */
    NvMediaISPEllipse ellipticalMask;
} NvMediaISPLocalToneMapStats;

/**
 * \brief Holds controls for flicker band statistics (FB Stats).
 */
typedef struct {
    /**
     * Holds a Boolean to enable flicker band statistics block.
     */
    NvMediaBool enable;
    /**
     * Holds the offset of the first band top line.
     * @li Supported values for X coordinate of start offset: [0, input width]
     * @li Supported values for Y coordinate of start offset: [0, input height]
     */
    NvMediaPoint startOffset;
    /**
     * Holds count of flicker band samples to collect per frame.
     * @li Supported values: [1, 256]
     * @li Constraints: If bandCount == 256, bottom of last band
     * must align with bottom of the image.
     */
    uint16_t bandCount;
    /**
     * Holds width of single band.
     * @li Supported values: [2, input width - startOffset.x];
     *  must be an even number
     * @li Constrains: Total number of accumulated pixels must be <= 2^18
     */
    uint32_t bandWidth;
    /**
     * Holds height of single band.
     * @li Supported values: [2, input height - startOffset.y]
     * @li Constrains: Total number of accumulated pixels must be <= 2^18
     * @li Constrains: If bandCount == 256, bottom of last band
     * must align with bottom of the image.
     */
    uint32_t bandHeight;
    /**
     * Holds minimum value of pixel to include for flicker band stats.
     * @li Supported values: [0.0, 1.0]
     */
    float_t min;
    /**
     * Holds maximum value of pixel to include for flicker band stats.
     * @li Supported values: [0.0, 1.0], max >= min
     */
    float_t max;
    /**
     * Holds a Boolean to enable an elliptical mask for excluding pixels
     * outside a specified area.
     */
    NvMediaBool ellipticalMaskEnable;
    /**
     * Holds an elliptical mask to exclude pixels outside a specified area.
     *
     * Coordinates of the image's top left and bottom right points are (0, 0)
     * and(width, height), respectively.
     *
     * @li Supported values for X coordinate of the center: [0, input width]
     * @li Supported values for Y coordinate of the center: [0, input height]
     * @li Supported values for hortizontal axis: [16, 2 x input width]
     * @li Supported values for vertical axis: [16, 2 x input height]
     * @li Supported values for angle: [0.0, 360.0]
     */
    NvMediaISPEllipse ellipticalMask;
} NvMediaISPFlickerBandStats;

/**
 * \brief Holds histogram statistics (HIST Stats).
 */
typedef struct {
    /**
     * Holds histogram data for each color component in RGGB/RCCB/RCCC order.
     */
    uint32_t data[NVM_ISP_HIST_BINS][NVM_ISP_MAX_COLOR_COMPONENT];
    /**
     * Holds the number of pixels excluded by the elliptical mask for each
     * color component.
     */
    uint32_t excludedCount[NVM_ISP_MAX_COLOR_COMPONENT];
} NvMediaISPHistogramStatsData;

/**
 * \brief Holds local average and clip statistics data for a region of interest.
 */
typedef struct {
    /**
     * Holds number of windows horizontally in one region of interest.
     */
    uint32_t numWindowsH;
    /**
     * Holds number of windows vertically in one region of interest.
     */
    uint32_t numWindowsV;
    /**
     * Holds average pixel value for each color component in each window in
     * RGGB/RCCB/RCCC order.
     */
    float_t average[NVM_ISP_MAX_LAC_ROI_WINDOWS][NVM_ISP_MAX_COLOR_COMPONENT];
    /**
     * Holds the number of pixels excluded by the elliptical mask for each
     * color component in each window
     * in RGGB/RCCB/RCCC order.
     */
    uint32_t maskedOffCount[NVM_ISP_MAX_LAC_ROI_WINDOWS][NVM_ISP_MAX_COLOR_COMPONENT];
    /**
     * Holds number of clipped pixels for each color component in each window in
     * RGGB/RCCB/RCCC order.
     */
    uint32_t clippedCount[NVM_ISP_MAX_LAC_ROI_WINDOWS][NVM_ISP_MAX_COLOR_COMPONENT];
} NvMediaISPLocalAvgClipStatsROIData;

/**
 * \brief Holds local average and clip statistics block (LAC Stats).
 */
typedef struct {
    /**
     * Holds statistics data for each region of interest.
     */
    NvMediaISPLocalAvgClipStatsROIData data[NVM_ISP_MAX_LAC_ROI];
} NvMediaISPLocalAvgClipStatsData;

/**
 * \brief Holds local tone map statistics block (LTM Stats).
 */
typedef struct {
    /**
     * Holds count of pixels for each histogram bin.
     */
    uint32_t histogram[NVM_ISP_LTM_HIST_BINS];
    /**
     * Holds average tone in local average window. The tone values are computed
     * by converting YUV input to tone, taking the logarithm of the value, and
     * normalizing it so that dynamic range is mapped to [0.0, 1.0].
     * If no pixels contributed to a windows (either because the windows was
     * out of image boundaries, or it was completely excluded by rectangular
     * or elliptical masks), this value is set to zero.
     */
    float_t localAverageTone[NVM_ISP_LTM_AVG_WINDOWS][NVM_ISP_LTM_AVG_WINDOWS];
    /**
     * Holds number of pixels for each local average window that contributed to
     * statistics.
     */
    uint32_t nonMaskedCount[NVM_ISP_LTM_AVG_WINDOWS][NVM_ISP_LTM_AVG_WINDOWS];
} NvMediaISPLocalToneMapStatsData;

/**
 * \brief Holds bad pixel statistics (BP Stats).
 */
typedef struct {
    /**
     * Holds bad pixel count for pixels corrected upward within the window.
     */
    uint32_t highInWin;
    /**
     * Holds bad pixel count for pixels corrected downward within the window.
     */
    uint32_t lowInWin;
    /**
     * Holds accumulated pixel adjustment for pixels corrected upward within the
     * window.
     */
    uint32_t highMagInWin;
    /**
     * Holds accumulatd pixel adjustment for pixels corrected downward within
     * the window.
     */
    uint32_t lowMagInWin;
    /**
     * Holds bad pixel count for pixels corrected upward outside the window.
     */
    uint32_t highOutWin;
    /**
     * Holds bad pixel count for pixels corrected downward outside the window.
     */
    uint32_t lowOutWin;
    /**
     * Holds accumulatd pixel adjustment for pixels corrected upward outside
     * the window. */
    uint32_t highMagOutWin;
    /**
     * Holds accumulatd pixel adjustment for pixels corrected downward outside
     * the window.
     */
    uint32_t lowMagOutWin;
} NvMediaISPBadPixelStatsData;

/**
 * \brief Holds flicker band statistics (FB Stats).
 */
typedef struct {
    /**
     * Holds band count.
     */
    uint32_t bandCount;
    /**
     * Holds average luminance value for each band.
     */
    float_t luminance[NVM_ISP_MAX_FB_BANDS];
} NvMediaISPFlickerBandStatsData;

/**
 * \defgroup history_nvmedia_isp_stat History
 * Provides change history for the NvMedia ISP Stats.
 *
 * \section history_nvmedia_isp_stat Version History
 *
 * <b> Version 1.0 </b> March 25, 2019
 * - Initial release
 */
/*@} */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_ISP_STAT_H */
