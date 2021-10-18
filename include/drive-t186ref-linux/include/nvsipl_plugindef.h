/*
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */


/**
 * \file
 * \brief <b> NVIDIA SIPL Plugin API struct definitions </b>
 *
 */

/*
 *  This header file is a draft and recommended implementation
 *  of SIPL plugin API struct definitions
 */

#ifndef NVSIPL_PLUGINDEF_H
#define NVSIPL_PLUGINDEF_H

#include <stdio.h>
#include <stdint.h>

#include "nvmedia_core.h"
#include "nvmedia_isp_stat.h"
#include "devblk_isc_framework.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * \brief Exposure settings for sensor
 */
typedef struct {
    /**
     * Holds the number of sensor contexts to activate.
     * Sensor context is a mode of operation, supported by some sensors,
     * in which multiple set of setttings(contexts) are programmed and
     * the sensor toggles between them at runtime.
     * For sensors not supporting this mode of operation, it
     * should be set to ‘1’.
     * Supported values: [1, DEVBLK_ISC_MAX_SENSOR_CONTEXTS]
     */
    uint8_t                           numSensorContexts;
    /**
     * Holds the sensor exposure settings to set for each context.
     */
    DevBlkISCExposure                 exposureControl[DEVBLK_ISC_MAX_SENSOR_CONTEXTS];
    /**
     * Holds the sensor white balance settings to set for each context.
     */
    DevBlkISCWhiteBalance             wbControl[DEVBLK_ISC_MAX_SENSOR_CONTEXTS];
    /** Exposure profile value is valid or not */
    NvMediaBool                       expProfileValid;
    /** Exposure profile value, such as day and night settings */
    uint32_t                          expProfile;
} NvSIPLAutoControlSensorSetting;

/**
 * \brief Parsed embedded info
 */
typedef struct {
    /** Holds the parsed embedded data frame number of exposures info for the captured frame. */
    uint32_t                          numExposures;
    /** Holds the parsed embedded data sensor exposure info for the captured frame. */
    DevBlkISCExposure                 sensorExpInfo;
    /** Holds the parsed embedded data sensor white balance info for the captured frame. */
    DevBlkISCWhiteBalance             sensorWBInfo;
    /** Exposure profile value is valid or not */
    NvMediaBool                       expProfileValid;
    /** Exposure profile value, such as day and night settings */
    uint32_t                          expProfile;
    /** Holds the parsed embedded data sensor temperature info for the captured frame. */
    DevBlkISCTemperature              sensorTempInfo;
} NvSIPLAutoControlEmbedInfo;

/**
 * \brief embedded data and parsed info
 */
typedef struct {
    /** Holds the parsed embedded info for the captured frame. */
    NvSIPLAutoControlEmbedInfo        embedInfo;
    /** Holds frame sequence number for the captured frame. */
    DevBlkISCFrameSeqNum              frameSeqNum;
    /** Embedded buffer at the beginning of the frame */
    DevBlkISCEmbeddedDataChunk        topEmbeddedData;
    /** Embedded buffer at the end of the frame */
    DevBlkISCEmbeddedDataChunk        bottomEmbeddedData;
} NvSIPLAutoControlEmbedData;


/**
 * \brief Input image attributes
 */
typedef struct {
    /** Number of input images for ISP to process */
    uint32_t                          numPlane;
    /** Input image width */
    uint32_t                          width;
    /** Input image height */
    uint32_t                          height;
    /** Input image crop location */
    NvMediaRect                       cropRect;
    /** Input image CFA, define as NVM_SURF_ATTR_COMPONENT_ORDER_XXXX in nvmedia_surface.h */
    uint32_t                          cfaFormat;
    /** camera ID in the car config, left, right, etc. */
    uint32_t                          id;
} NvSIPLAutoControlInputImageInfo;

/**
 * \brief Gains for each color assuming order RGGB, RCCB, RCCC
 */
typedef struct {
    /** White balance gains are valid */
    NvMediaBool                       valid;
    /** Gains that applies to indvidiual color channels */
    float_t                           gain[NVM_ISP_MAX_COLOR_COMPONENT];
} NvSIPLAutoControlWhiteBalanceGain;

/**
 * \brief Output image attributes
 */
typedef struct {
    /** Total white balance gains, including both senor channel gains and ISP gains */
    NvSIPLAutoControlWhiteBalanceGain wbGainTotal[NVM_ISP_MAX_INPUT_PLANES];
    /** Correlated Color Temperature */
    float_t                           cct;
    /** Color Correlation Matrix */
    float_t                           ccmMatrix[NVM_ISP_MAX_COLORMATRIX_DIM][NVM_ISP_MAX_COLORMATRIX_DIM];
} NvSIPLAutoControlAwbSetting;

/** @brief Holds Stats information */
typedef struct
{
    /** Holds const pointers to 2 LAC stats data */
    const NvMediaISPLocalAvgClipStatsData   *lacData[2];
    /** Holds const pointers to 2 LAC stats settings */
    const NvMediaISPLocalAvgClipStats       *lacSettings[2];
    /** Holds const pointers to 2 Histogram stats data */
    const NvMediaISPHistogramStatsData      *histData[2];
    /** Holds const pointers to 2 Histogram stats settings */
    const NvMediaISPHistogramStats          *histSettings[2];
    /** Holds const pointer to Flicker Band stats data */
    const NvMediaISPFlickerBandStatsData    *fbStatsData;
    /** Holds const pointer to Flicker Band stats settings */
    const NvMediaISPFlickerBandStats        *fbStatsSettings;
} NvSIPLAutoControlStatsInfo;

/**
 * \brief Settings to config stats
 */
typedef struct {
    /** Settings to control ISP stats blocks are valid or not */
    NvMediaBool                             valid;
    /** Settings for 2 LAC stats ISP blocks */
    NvMediaISPLocalAvgClipStats             lac[2];
    /** Settings for Histogram 1 stats blocks */
    NvMediaISPHistogramStats                hist1;
    /** Settings for Flicker Band stats block */
    NvMediaISPFlickerBandStats              fbStats;
} NvSIPLAutoControlStatsSetting;

/**
 * \brief Initial parameters for processing AE/AWB
 */
typedef struct {
    /** Input image attributes */
    NvSIPLAutoControlInputImageInfo   inputImageInfo;
} NvSIPLAutoControlInitParam;

/**
 * \brief Input parameters for processing AE/AWB
 */
typedef struct {
    /** Embedded Settings */
    NvSIPLAutoControlEmbedData        embedData;
    /** Sensor Attributes */
    DevBlkISCSensorAttributes         sensorAttr;
    /** Stats buffers and settings */
    NvSIPLAutoControlStatsInfo        statsInfo;
} NvSIPLAutoControlInputParam;

/**
 * \brief AE/AWB Output parameters
 */
typedef struct {
    /** Sensor exposure and gain settings */
    NvSIPLAutoControlSensorSetting    sensorSetting;
    /** AWB settings */
    NvSIPLAutoControlAwbSetting       awbSetting;
    /** Settings to config stats */
    NvSIPLAutoControlStatsSetting     newStatsSetting;
    /** Digital gain to be applied in ISP */
    float_t                           ispDigitalGain;
} NvSIPLAutoControlOutputParam;



#ifdef __cplusplus
}     /* extern "C" */
#endif

#endif /* NVSIPL_PLUGINDEF_H */
