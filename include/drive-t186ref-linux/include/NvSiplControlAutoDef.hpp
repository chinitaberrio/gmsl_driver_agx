/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/*  NVIDIA SIPL Control Auto Definitions */

#ifndef NVSIPLCONTROLAUTODEF_HPP
#define NVSIPLCONTROLAUTODEF_HPP

#include "nvmedia_isp_stat.h"
#include "devblk_cdi.h"

/**
 * @file
 *
 * @brief <b> NVIDIA SIPL: Auto Control Settings - @ref NvSIPLAutoControl </b>
 *
 */

namespace nvsipl{

/** @addtogroup NvSIPLAutoControl
 * @{
 */

/**
 * \brief Sensor Settings
 */
struct SiplControlAutoSensorSetting {
    /**
     * Holds the number of sensor contexts to activate.
     * Sensor context is a mode of operation, supported by some sensors,
     * in which multiple set of settings(contexts) are programmed and
     * the sensor toggles between them at runtime.
     * For sensors not supporting this mode of operation, it
     * should be set to ‘1’.
     * Supported values: [1, DEVBLK_CDI_MAX_SENSOR_CONTEXTS]
     */
    uint8_t                           numSensorContexts;
    /**
     * Holds the sensor exposure settings to set for each context.
     */
    DevBlkCDIExposure                exposureControl[DEVBLK_CDI_MAX_SENSOR_CONTEXTS];
    /**
     * Holds the sensor white balance settings to set for each context.
     */
    DevBlkCDIWhiteBalance            wbControl[DEVBLK_CDI_MAX_SENSOR_CONTEXTS];
    /** Exposure profile value is valid or not */
    bool                              expProfileValid;
    /** Exposure profile value, such as day and night settings */
    uint32_t                          expProfile;
};

/**
 * \brief Parsed Frame Embedded Info
 */
struct SiplControlEmbedInfo {
    /** Holds the parsed embedded data frame number of exposures info for the captured frame. */
    uint32_t                          numExposures;
    /** Holds the parsed embedded data sensor exposure info for the captured frame. */
    DevBlkCDIExposure                sensorExpInfo;
    /** Holds the parsed embedded data sensor white balance info for the captured frame. */
    DevBlkCDIWhiteBalance            sensorWBInfo;
    /** Exposure profile value is valid or not */
    bool                              expProfileValid;
    /** Exposure profile value, such as day and night settings */
    uint32_t                          expProfile;
    /** Holds the parsed embedded data sensor temperature info for the captured frame. */
    DevBlkCDITemperature             sensorTempInfo;
};


/**
 * \brief Embedded data and parsed info
 */
struct SiplControlEmbedData {
    /** Holds the parsed embedded info for the captured frame. */
    SiplControlEmbedInfo              embedInfo;
    /** Holds frame sequence number for the captured frame. */
    DevBlkCDIFrameSeqNum             frameSeqNum;
    /** Embedded buffer at the beginning of the frame */
    DevBlkCDIEmbeddedDataChunk       topEmbeddedData;
    /** Embedded buffer at the end of the frame */
    DevBlkCDIEmbeddedDataChunk       bottomEmbeddedData;
};

/**
 * \brief Color Gains assuming order RGGB, RCCB, RCCC
 */
struct SiplControlAutoAwbGain {
    /** White balance gains are valid */
    bool                              valid;
    /** Gains that applies to individual color channels */
    float_t                           gain[NVM_ISP_MAX_COLOR_COMPONENT];
};

/**
 * \brief Output image attributes
 */
struct SiplControlAutoAwbSetting {
    /** Total white balance gains, including both senor channel gains and ISP gains */
    SiplControlAutoAwbGain  wbGainTotal[NVM_ISP_MAX_INPUT_PLANES];
    /** Correlated Color Temperature */
    float_t                 cct;
    /** Color Correlation Matrix */
    float_t                 ccmMatrix[NVM_ISP_MAX_COLORMATRIX_DIM][NVM_ISP_MAX_COLORMATRIX_DIM];
};

/** @brief Holds Stats information */
struct SiplControlIspStatsInfo {

    /** Holds const pointers to 2 LAC stats data */
    const NvMediaISPLocalAvgClipStatsData* lacData[2];
    /** Holds const pointers to 2 LAC stats settings */
    const NvMediaISPLocalAvgClipStats* lacSettings[2];
    /** Holds const pointers to 2 Histogram stats data */
    const NvMediaISPHistogramStatsData* histData[2];
    /** Holds const pointers to 2 Histogram stats settings */
    const NvMediaISPHistogramStats* histSettings[2];
    /** Holds const pointer to Flicker Band stats data */
    const NvMediaISPFlickerBandStatsData* fbStatsData;
    /** Holds const pointer to Flicker Band stats settings */
    const NvMediaISPFlickerBandStats*  fbStatsSettings;
};

/**
 * \brief Settings to config stats
 */
struct SiplControlIspStatsSetting {
    /** Settings to control ISP stats blocks are valid or not */
    bool                                    valid;
    /** Settings for 2 LAC stats ISP blocks */
    NvMediaISPLocalAvgClipStats             lac[2];
    /** Settings for Histogram 1 stats blocks */
    NvMediaISPHistogramStats                hist1;
    /** Settings for Flicker Band stats block */
    NvMediaISPFlickerBandStats              fbStats;
};

/**
 * \brief Input parameters for processing AE/AWB
 */
struct SiplControlAutoInputParam {
    /** Embedded Settings */
    SiplControlEmbedData        embedData;
    /** Sensor Attributes */
    DevBlkCDISensorAttributes  sensorAttr;
    /** Stats buffers and settings */
    SiplControlIspStatsInfo     statsInfo;
};

/**
 * \brief AE/AWB Output parameters
 */
struct SiplControlAutoOutputParam {
    /** Sensor exposure and gain settings */
    SiplControlAutoSensorSetting    sensorSetting;
    /** AWB settings */
    SiplControlAutoAwbSetting       awbSetting;
    /** Settings to config stats */
    SiplControlIspStatsSetting      newStatsSetting;
    /** Digital gain to be applied in ISP */
    float_t                         ispDigitalGain;
};

/** @} */

}  // namespace nvsipl

#endif /* NVSIPLCONTROLAUTODEF_HPP */
