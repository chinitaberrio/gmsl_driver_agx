/*
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: Image Signal Processing (ISP)</b>
 *
 * This file contains the Image Signal Processing API.
 */

#ifndef NVMEDIA_ISP_H
#define NVMEDIA_ISP_H

#include <stdint.h>

#include "nvmedia_core.h"
#include "nvmedia_image.h"
#include "nvmedia_isp_stat.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup nvmedia_isp_api Image Signal Processing (ISP)
 *
 * The Image Signal Processing API encompasses all NvMedia image processing
 * functions that are necessary to produce a processed image from image data
 * captured from an image sensor.
 *
 * @ingroup nvmedia_image_top
 * @{
 */

/** \brief Major version number. */
#define NVM_ISP_VERSION_MAJOR   1

/** \brief Minor version number. */
#define NVM_ISP_VERSION_MINOR   5

/**
 * \brief Maximum supported simultaneous outputs.
 */
#define NVM_ISP_MAX_OUTPUTS                 (3U)

/**
 * \brief Maximum number of queued requests.
 */
#define NVM_ISP_MAX_QUEUED_REQUESTS         (32U)

/**
 * \brief Maximum number of linearization knee points.
 */
#define NVM_ISP_MAX_LIN_KNEE_POINTS         (10U)

/**
 * \brief Number of transfer function control points for level adjusted
 * saturation block.
 */
#define NVM_ISP_LAS_TF_POINTS               (9U)

/**
 * \brief Number of transfer function control points for global tone map.
 */
#define NVM_ISP_GTM_TF_POINTS               (18U)

/**
 * \brief Local tone map soft key width.
 */
#define NVM_ISP_LTM_SOFT_KEY_WIDTH          (8U)

/**
 * \brief Local tone map soft key height.
 */
#define NVM_ISP_LTM_SOFT_KEY_HEIGHT         (8U)

/**
 * \brief Local tone map gain points.
 */
#define NVM_ISP_LTM_GAIN_POINTS             (9U)

/**
 * \brief Maximum number of windows for local average & clip in a region of
 * interest.
 */
#define NVM_ISP_MAX_FB_BANDS                (256U)

/**
 * \brief A handle representing ISP object.
 */
typedef struct _NvMediaISP NvMediaISP;

/**
 * \brief A handle representing ISP stats surface object.
 */
typedef struct _NvMediaISPStatsSurface NvMediaISPStatsSurface;

/**
 * \brief A handle representing ISP settings object.
 */
typedef struct _NvMediaISPSettings NvMediaISPSettings;

/**
 * \brief Defines supported ISP pipelines.
 */
typedef enum {
    /**
     * \brief Xavier ISP pipeline.
     */
    NVM_ISP_PIPELINE_X1 = (('X' << 24) | 0x01),
} NvMediaISPPipelineEnum;

/**
 * \brief Gives the version information for the NvMedia ISP library.
 * \param[in,out] version \ref NvMediaVersion structure which will be populated.
 * \return  NVMEDIA_STATUS_OK if successful, or NVMEDIA_STATUS_BAD_PARAMETER
 *  if @a version was invalid.
 */
NvMediaStatus
NvMediaISPGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Allocates an Image Signal Processing object.
 * \param[in] instanceId ISP instance ID. Valid values: 0
 * \param[in] pipelineEnum ISP pipeline configuration.
 * \param[in] maxQueuedRequests
        Determines number of processing requests can be pending for an ISP
        instance at a time.
        Supported values: [1, NVM_ISP_MAX_QUEUED_REQUESTS]
 * \return  An NvMediaISP handle to the specified ISP engine instance if
 *  successful, or NULL otherwise.
 */
NvMediaISP *
NvMediaISPCreate(
    uint32_t instanceId,
    NvMediaISPPipelineEnum pipelineEnum,
    uint32_t maxQueuedRequests
);

/**
 * \brief Stops the image processing.
 * \param[in] isp ISP object to destroy.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a isp was NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPStop(
    NvMediaISP *isp
);

/**
 * \brief Destroys an Image Signal Processing object.
 * \param[in] isp ISP object.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a isp was NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPDestroy(
    NvMediaISP *isp
);

/**
 * \brief Allocates an ISP statistics surface object.
 * \param[in] isp ISP object.
 * \return  A handle to the ISP statistics surface if successful, or NULL
 *  otherwise.
 */
NvMediaISPStatsSurface *
NvMediaISPStatsSurfaceCreate(
    NvMediaISP *isp
);

/**
 * \brief Destroys an ISP statistics surface object.
 * \param[in] statsSurface ISP statistics surface object to destroy.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a statsSurface was
 *  invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPStatsSurfaceDestroy(
    NvMediaISPStatsSurface *statsSurface
);

/**
 * \brief Allocates an ISP settings object.
 * \param[in] isp ISP object.
 * \param[in] pipelineEnum
 *      ISP pipeline configuration, reserved for future extension.
        For now this must be same as value given in \ref NvMediaISPCreate
 * \return  A handle to the ISP settings object if successful, or NULL
 *  otherwise.
 */
NvMediaISPSettings *
NvMediaISPSettingsCreate(
    NvMediaISP *isp,
    NvMediaISPPipelineEnum pipelineEnum
);

/**
 * \brief Loads konb data from binary blob in memory.
 * \param[in] settings ISP settings object.
 * \param[in] blob Pointer to the binary blob.
 * \param[in] blobSize Size of binary blob memory.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSettingsLoadConfig(
    NvMediaISPSettings* settings,
    const uint8_t* blob,
    size_t blobSize
);

/**
 * \brief Destroys an ISP settings object.
 * \param[in] settings ISP settings object to destroy.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameter
 *  was invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSettingsDestroy(
    NvMediaISPSettings *settings
);

/**
 * \brief Holds controls for ISP linearization (LIN) block.
 */
typedef struct {
    /**
     * Holds boolean to enable linearization block.
     */
    NvMediaBool enable;
    /**
     * Holds count of piecewise linear transfer function.
     * @li Supported values: [1, NVM_ISP_MAX_INPUT_PLANES]
     */
    uint32_t numPlanes;
    /**
     * Holds count of knee points for each plane.
     * @li Supported values: [2, NVM_ISP_MAX_LIN_KNEE_POINTS]
     */
    uint32_t numKneePoints[NVM_ISP_MAX_INPUT_PLANES];
    /**
     * Holds knee points for piecewise linear transfer function for each plane.
     * @li Supported values for X coordinate of knee point: [0.0, 1.0]
     * @li Supported values for Y coordinate of knee point: [0.0, 1.0]
     * @li Constrains: Either X or Y coordinate of the 1st knee point must be 0.0
     * @li Constrains: Knee points must be monotonically non-decreasing
     */
    NvMediaPointDouble kneePoints[NVM_ISP_MAX_INPUT_PLANES][NVM_ISP_MAX_LIN_KNEE_POINTS];
} NvMediaISPLinearization;

/**
 * \brief Programs linearizartion block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a linearization
 *                      block.
 * \param[in] size      Size of the linearization block control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetLinearization(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPLinearization *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP black level correction (BLC) block.
 */
typedef struct {
    /**
     * Holds boolean to enable black level correction block.
     */
    NvMediaBool enable;
    /**
     * Holds black level correction values in RGGB/RCCB/RCCC component order
     * for each plane & each color component.
     * @li Supported values: [0.0, 1.0]
     */
    double_t pedestal[NVM_ISP_MAX_INPUT_PLANES][NVM_ISP_MAX_COLOR_COMPONENT];
} NvMediaISPBlackLevelCorrection;

/**
 * \brief Programs black level correction block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a black level
 *                      correction block.
 * \param[in] size      Size of the black level correction block control
 *                      structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetBlackLevelCorrection(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPBlackLevelCorrection *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP white balance correction (WBC) block.
 */
typedef struct {
    /**
     * Holds boolean to enable white balance correction block.
     */
    NvMediaBool enable;
    /**
     * Holds white balance gains in RGGB/RCCB/RCCC component order for each
     * plane & each color component.
     * @li Supported values: [0.0, 8.0]
     */
    float_t wbGain[NVM_ISP_MAX_INPUT_PLANES][NVM_ISP_MAX_COLOR_COMPONENT];
} NvMediaISPWhiteBalanceCorrection;

/**
 * \brief Programs white balance correction block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a white balance
 *                      correction block.
 * \param[in] size      Size of the white balance correction block control
 *                      structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetWhiteBalanceCorrection(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPWhiteBalanceCorrection *controls,
    size_t size
);

/**
 * \brief Defines supported exposure fusion modes.
 */
typedef enum {
    /**
     * Specifies use plane 0.
     */
    NVM_ISP_EXP_FUSION_USE_0 = 0,
    /**
     * Specifies use plane 1.
     */
    NVM_ISP_EXP_FUSION_USE_1 = 1,
    /**
     * Specifies use plane 2.
     */
    NVM_ISP_EXP_FUSION_USE_2 = 2,
    /**
     * Specifies blend plane 0 & 1.
     */
    NVM_ISP_EXP_FUSION_BLEND_01 = 3,
    /**
     * Specifies blend all planes.
     */
    NVM_ISP_EXP_FUSION_BLEND_ALL = 4,
} NvMediaISPExposureFusionMode;

/**
 * \brief Holds controls for ISP exposure fusion (Fusion) block.
 */
typedef struct {
    /**
     * Holds boolean to enable exposure fusion block.
     */
    NvMediaBool enable;
    /**
     * Holds ratio of shortest exposure gain vs exposure gain for each plane.
     * Index 0 & 2 are for longest & shortest exposure respectively.
     * @li Supported values: [0.0, 1.0]
     */
    float_t scaleFactors[NVM_ISP_MAX_INPUT_PLANES];
    /**
     * Holds exposure fusion mode.
     */
    NvMediaISPExposureFusionMode fusionMode;
} NvMediaISPExposureFusion;

/**
 * \brief Programs exposure fusion block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for an exposure
 *                      fusion block.
 * \param[in] size      Size of the exposure fusion block control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetExposureFusion(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPExposureFusion *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP bad pixel correction (BPC) block.
 */
typedef struct {
    /**
     * Holds boolean to enable bad pixel correction block.
     */
    NvMediaBool enable;
    /**
     * Holds strength of bad pixel correction.
     * @li Supported values: [0.0, 1.0],
     * 0.0 & 1.0 means minimum & maximum strength respectively
     */
    float_t strength;
    /**
     * Holds boolean to enable noise estimation.
     */
    NvMediaBool noiseProfileEnable;
    /**
     * Holds noise profile nunber.
     * @li Supported values: [0, 31]
     * \note Actual supported value depends on binary blob file.
     */
    uint32_t profileNum;
    /**
     * Holds gain affecting noise profile.
     * @li Supported values: [0.0, 10000.0]
     */
    float_t gain;
} NvMediaISPBadPixelCorrection;

/**
 * \brief Programs bad pixel correction block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a
 *                      control structure for a bad pixel correction block.
 * \param[in] size      Size of the bad pixel correction block control
 *                      structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetBadPixelCorrection(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPBadPixelCorrection *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP lens shading correction (LSC) block.
 */
typedef struct {
    /**
     * Holds boolean to enable CCT based lens shading correction.
     */
    NvMediaBool cctBasedLSCEnable;
    /**
     * Holds color correlated temperature.
     * @li Supported values: [1000.0, 20000.0]
     */
    float_t cct;
    /**
     * Holds ratio between corner & center lens shading correction.
     * @li Supported values: [0.0, 1.0]
     */
    float_t fallOff;
    /**
     * Holds boolean to enable adjustment of shading profile for compressed data.
     */
    NvMediaBool applyAlpha;
    /**
     * Holds compression factor from N to 20-bit depth, where N >= 20.
     * @li Supported values: [0.75, 1.0]
     * \note Recommened alpha = 20 / (sensor-caputred data bit N).
     */
    float_t alpha;
} NvMediaISPLensShadingCorrection;

/**
 * \brief Programs lens shading correction block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a lens shading
 *                      correction block.
 * \param[in] size      Size of the lens shading correction block
 *                      control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetLensShadingCorrection(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPLensShadingCorrection *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP demosaic (DM) block.
 */
typedef struct {
    /**
     * Holds strength of demosaicing.
     * @li Supported values: [0.0, 1.0],
     * 0.0 & 1.0 means minimum & maximum strength respectively
     */
    float_t strength;
} NvMediaISPDemosaic;

/**
 * \brief Programs demosaic block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a demosaic block.
 * \param[in] size      Size of the demosaic block control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetDemosaic(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPDemosaic *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP level adjusted saturation (LAS) block.
 */
typedef struct {
    /**
     * Holds boolean to enable level adjusted saturation block.
     */
    NvMediaBool enable;
    /**
     * Holds spline control points for transfer function.
     * @li Supported values for X coordinate of spline control point : [0.0, 2.0]
     * @li Supported values for Y coordinate of spline control point : [0.0, 2.0]
     * @li Supported values for slope of spline control point : \f$[-2^{16}, 2^{16}]\f$
     */
    NvMediaISPSplineControlPoint controlPoints[NVM_ISP_LAS_TF_POINTS];
} NvMediaISPLevelAdjSat;

/**
 * \brief Programs level adjusted saturation block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a level
 *                      adjusted saturation block.
 * \param[in] size      Size of the level adjusted saturation block
 *                      control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetLevelAdjSat(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPLevelAdjSat *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP noise reduction (NR) block.
 */
typedef struct {
    /**
     * Holds boolean to enable noise reduction block.
     */
    NvMediaBool enable;
    /**
     * Holds strength of noise reduction.
     * @li Supported values: [0.0, 1.0],
     * 0.0 & 1.0 means minimum & maximum strength respectively
     */
    float_t strength;
    /**
     * Holds boolean to enable noise estimation.
     */
    NvMediaBool noiseProfileEnable;
    /**
     * Holds noise profile number.
     * @li Supported values: [0, 31],
     * \note Actual supported value depends on binary blob file.
     */
    uint32_t profileNum;
    /**
     * Holds gain affecting noise profile.
     * @li Supported values: [0.0, 10000.0]
     */
    float_t gain;
    /**
     * Holds normalized white balance gain affecting noise profile.
     * @li Supported values: [0.0, 128.0]
     */
    float_t wbGain[NVM_ISP_MAX_COLOR_COMPONENT];
    /**
     * Holds boolean to enable radial transform.
     */
    NvMediaBool radialTFEnable;
    /**
     * Holds a radial transfer function.
     *
	 * Supported values are described in the declaration of
	 * \ref NvMediaISPRadialTF.
     */
    NvMediaISPRadialTF radialTF;
} NvMediaISPNoiseReduction;

/**
 * \brief Programs noise reduction block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a
 *                      noise reduction block.
 * \param[in] size      Size of the noise reduction block control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetNoiseReduction(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPNoiseReduction *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP color correction matrix (CCM) block.
 */
typedef struct {
    /**
     * Holds boolean to enable color correction matrix block.
     */
    NvMediaBool enable;
    /**
     * Holds color correction matrix.
     * @li Supported values: [-8.0, 8.0]
     */
    float_t matrix[NVM_ISP_MAX_COLORMATRIX_DIM][NVM_ISP_MAX_COLORMATRIX_DIM];
} NvMediaISPColorCorrectionMatrix;

/**
 * \brief Programs color correction matrix block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a
 *                      color correction matrix block.
 * \param[in] size      Size of the color correction matrix block
 *                      control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetColorCorrectionMatrix(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPColorCorrectionMatrix *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP global tone map (GTM) block.
 */
typedef struct {
    /**
     * Holds boolean to enable global tone map block.
     */
    NvMediaBool enable;
    /**
     * Holds spline control points for global tone map transfer function.
     * @li Supported values for X coordinate of spline control point: [0.0, 1.0]
     * @li Supported values for Y coordinate of spline control point: [0.0, 1.0]
     * @li Supported values for slope of spline control point: \f$[0.0, 2^{16}]\f$
     */
    NvMediaISPSplineControlPoint controlPoints[NVM_ISP_GTM_TF_POINTS];
} NvMediaISPGlobalToneMap;

/**
 * \brief Programs global tone map block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a
 *                      global tone map block.
 * \param[in] size      Size of the global tone map block control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetGlobalToneMap(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPGlobalToneMap *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP local tone map (LTM) block.
 */
typedef struct {
    /**
     * Holds boolean to enable local tone map block.
     */
    NvMediaBool enable;
    /**
     * Holds boolean to enable soft key based local contrast enhancement.
     */
    NvMediaBool softKeyEnable;
    /**
     * Holds strength of local tone map.
     * @li Supported values: [0.0, 1.0],
     * 0.0 & 1.0 means minimum & maximum strength respectively
     */
    float_t strength;
    /**
     * Holds piecewise linear transfer function to adjust saturation based on
     * tone value.
     * @li Supported values: [-1.0, 1.0]
     */
    float_t saturation[NVM_ISP_LTM_GAIN_POINTS];
    /**
     * Holds softkey signal. Softkey signal is a low resolution estimate of the
     * image used to determine the local average tone.
     * @li Supported values: [0.0, 1.0]
     */
    float_t softKey[NVM_ISP_LTM_SOFT_KEY_HEIGHT][NVM_ISP_LTM_SOFT_KEY_WIDTH];
} NvMediaISPLocalToneMap;

/**
 * \brief Programs local tone map block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a
 *                      local tone map block.
 * \param[in] size      Size of the local tone map block control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetLocalToneMap(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPLocalToneMap *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP color space conversion (CSC) block.
 */
typedef struct {
    /**
     * Holds boolean to enable color space conversion block.
     */
    NvMediaBool enable;
    /**
     * Holds color space conversion matrix.
     * @li Supported values: [-8.0, 8.0]
     */
    float_t matrix[NVM_ISP_MAX_COLORMATRIX_DIM][NVM_ISP_MAX_COLORMATRIX_DIM];
} NvMediaISPColorSpaceConversion;

/**
 * \brief Programs color space conversion block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a
 *                      color space conversion block.
 * \param[in] size      Size of the color space conversion block
 *                      control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetColorSpaceConversion(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPColorSpaceConversion *controls,
    size_t size
);

/**
 * \brief Programs inverse color space conversion block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for an
 *                      inverse color space conversion block.
 * \param[in] size      Size of the inverse color space conversion block
 *                      control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetInvColorSpaceConversion(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPColorSpaceConversion *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP saturation (SAT) block.
 */
typedef struct {
    /**
     * Holds boolean to enable saturation block.
     */
    NvMediaBool enable;
    /**
     * Holds saturation value.
     * @li Supported values: [-1.0, 1.0],
     * -1.0, 0.0 & 1.0 means 0%, 100% & 200% saturation respectively
     */
    float_t saturation;
} NvMediaISPSaturation;

/**
 * \brief Programs saturation block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a saturation block.
 * \param[in] size      Size of the saturation block control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetSaturation(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPSaturation *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP sharpness (Sharp) block.
 */
typedef struct {
    /**
     * Holds boolean to enable sharpness block.
     */
    NvMediaBool enable;
    /**
     * Holds strength of sharpness.
     * @li Supported values: [0.0, 1.0],
     * 0.0 & 1.0 means minimum & maximum strength respectively
     */
    float_t strength;
} NvMediaISPSharpness;

/**
 * \brief Programs sharpness block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a sharpness block.
 * \param[in] size      Size of the sharpness block control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetSharpness(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPSharpness *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP downscale (DS) block.
 */
typedef struct {
    /**
     * Holds boolean to enable downscale block.
     */
    NvMediaBool enable;
    /**
     * Holds image width after downscaling.
     * @li Supported values: [ceil(input width / 32.0), input width]
     */
    uint32_t downscaledWidth;
    /**
     * Holds image height after downscaling.
     * @li Supported values: [ceil(input height / 32.0), input height]
     */
    uint32_t downscaledHeight;
} NvMediaISPDownscale;

/**
 * \brief Programs downscale block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a downscale block.
 * \param[in] size      Size of the downscale block control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetDownscale(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPDownscale *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP clip (Clip) block.
 */
typedef struct {
    /**
     * Holds boolean to enable clip block.
     */
    NvMediaBool enable;
    /**
     * Holds minimum clip value for each color component, component order is
     * RGGB/RCCB/RCCC for bayer data & RGB/YUV for RGB/YUV data.
     * @li Supported values for non-chroma data: [-0.125, 2.0]
     * @li Supported values for chroma data: [-0.5, 0.5]
     */
    float_t min[NVM_ISP_MAX_COLOR_COMPONENT];
    /**
     * Holds minimum clip value for each color component, component order is
     * RGGB/RCCB/RCCC for bayer data & RGB/YUV for RGB/YUV data.
     * @li Supported values for non-chroma data: [-0.125, 2.0], max >= min
     * @li Supported values for chroma data: [-0.5, 0.5], max >= min
     */
    float_t max[NVM_ISP_MAX_COLOR_COMPONENT];
} NvMediaISPClip;

/**
 * \brief Programs clip block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a clip block.
 * \param[in] size      Size of the clip block control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetClip(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPClip *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP offset (Offset) block.
 */
typedef struct {
    /**
     * Holds boolean to enable offset block.
     */
    NvMediaBool enable;
    /**
     * Holds offset value for each color component.
     * @li Supported values: [-2.0, 2.0]
     */
    float_t offset[NVM_ISP_MAX_COLOR_COMPONENT];
} NvMediaISPOffset;

/**
 * \brief Programs offset block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for an offset block.
 * \param[in] size      Size of the offset block control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetOffset(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPOffset *controls,
    size_t size
);

/**
 * \brief Holds controls for ISP digital gain (DG) block.
 */
typedef struct {
    /**
     * Holds boolean to enable digital gain block.
     */
    NvMediaBool enable;
    /**
     * Holds digital gain value.
     * @li Supported values: [0.0, 8.0]
     */
    float_t digitalGain;
} NvMediaISPDigitalGain;

/**
 * \brief Programs digital gain block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a
 *                      digital gain block.
 * \param[in] size      Size of the digital gain block control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetDigitalGain(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPDigitalGain *controls,
    size_t size
);

/**
 * \brief Programs histogram statistics block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a
 *                      histogram statistics block.
 * \param[in] size      Size of the histogram statistics block
 *                      control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetHistogramStats(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPHistogramStats *controls,
    size_t size
);

/**
 * \brief Programs local average and clip statistics block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a
 *                      local average and clip statistics block.
 * \param[in] size      Size of the local average and clip statistics block
 *                      control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetLocalAvgClipStats(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPLocalAvgClipStats *controls,
    size_t size
);

/**
 * \brief Programs bad pixel statistics block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a
 *                      control structure for a bad pixel statistics block.
 * \param[in] size      Size of the bad pixel statistics block control
 *                      structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetBadPixelStats(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPBadPixelStats *controls,
    size_t size
);

/**
 * \brief Programs local tone map statistics block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a
 *                      local tone map statistics block.
 * \param[in] size      Size of the local tone map statistics block
 *                      control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetLocalToneMapStats(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPLocalToneMapStats *controls,
    size_t size
);

/**
 * \brief Programs flicker band statistics block controls.
 * \param[in] settings  Handle representing the ISP settings object.
 * \param[in] instance  Instance of the block to be programmed.
 * \param[in] controls  A pointer to a control structure for a
 *                      flicker band statistics block.
 * \param[in] size      Size of the flicker band statistics block
 *                      control structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more pointer
 *  parameters were NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPSetFlickerBandStats(
    NvMediaISPSettings *settings,
    uint32_t instance,
    const NvMediaISPFlickerBandStats *controls,
    size_t size
);

/**
 * \brief Registers an image group as input to isp engine
 * \param[in] isp ISP object.
 * \param[in] imageGrp Image group to be registered to isp engine
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPRegisterInputImageGroup(
    NvMediaISP *isp,
    NvMediaImageGroup *imageGrp
);

/**
 * \brief Unregisters an image group from input to isp engine
 * \param[in] isp ISP object.
 * \param[in] imageGrp Image group to be unregistered from isp engine
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPUnregisterInputImageGroup(
    NvMediaISP *isp,
    NvMediaImageGroup *imageGrp
);

/**
 * \brief Registers an image as output to isp engine
 * \param[in] isp ISP object.
 * \param[in] image Image to be registered to isp engine
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPRegisterOutputImage(
    NvMediaISP *isp,
    NvMediaImage *image
);

/**
 * \brief Unregisters an image from output to isp engine
 * \param[in] isp ISP object.
 * \param[in] image Image to be unregistered from isp engine
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPUnregisterOutputImage(
    NvMediaISP *isp,
    NvMediaImage *image
);

/**
 * \brief Holds ISP processing parameters.
 */
typedef struct {
    /**
     * Holds input image(s).
     */
    NvMediaImageGroup input;
    /**
     * Holds cropping rectangle for input image.
     *
     * Coordinates of image top-left & bottom-right points are (0, 0) &
     * (width, height) respectively. Either memset the rectangle to 0 or set it
     * to include full image for no cropping.
     *
     * Input crop only supports cropping in vertical direction, meaning
     * left & bottom cordinated must be 0 & input width respectively.
     */
    NvMediaRect inputCropRect;
    /**
     * Holds output image for each ISP output.
     * output \#i is enabled if @c output[i] is a valid pointer, and disabled
     * if it is NULL.
     */
    NvMediaImage *output[NVM_ISP_MAX_OUTPUTS];
    /**
     * Holds cropping rectangle for each output image.
     *
     * Coordinates of image top-left & bottom-right points are (0, 0) &
     * (width, height) respectively. Either memset the rectangle to 0 or set it
     * to include full image for no cropping.
     *
     * Rectangle must be within input image or downscaled image if downscaling
     * is enabled. Cropped width & height must be same as output width & height
     * respectively and cropped width must be even.
     *
     */
    NvMediaRect outputCropRect[NVM_ISP_MAX_OUTPUTS];
    /**
     * Holds output statistics surface.
     * Must not be NULL, if any of the statistics blocks are enabled.
     */
    NvMediaISPStatsSurface *statsSurface;
    /**
     * Holds ISP settings.
     */
    const NvMediaISPSettings *settings;
} NvMediaISPProcessParams;

/**
 * \brief ISP processing function.
 * \param[in] isp ISP object.
 * \param[in] params  ISP processing parameters.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPProcess(
    NvMediaISP *isp,
    const NvMediaISPProcessParams *params
);

/**
 * \brief Gets the histogram statistics data.
 * \param[in] isp           ISP object.
 * \param[in] statsSurface  Statistics surface.
 * \param[in] instance      Instance of the block to get statistics.
 * \param[in,out] statsData A pointer to histogram statistics data.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPGetHistogramStatsData(
    NvMediaISP *isp,
    NvMediaISPStatsSurface *statsSurface,
    uint32_t instance,
    NvMediaISPHistogramStatsData *statsData
);

/**
 * \brief Gets the local average & clip statistics data.
 * \param[in] isp           ISP object.
 * \param[in] statsSurface  A pointer to a statistics surface.
 * \param[in] instance      Instance of the block to get statistics.
 * \param[in,out] statsData A pointer to local average & clip statistics data.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPGetLocalAvgClipStatsData(
    NvMediaISP *isp,
    NvMediaISPStatsSurface *statsSurface,
    uint32_t instance,
    NvMediaISPLocalAvgClipStatsData *statsData
);

/**
 * \brief Gets the local tone map statistics data.
 * \param[in] isp           ISP object.
 * \param[in] statsSurface  A pointer to a statistics surface.
 * \param[in] instance      Instance of the block to get statistics.
 * \param[in,out] statsData A pointer to local tone map statistics data.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPGetLocalToneMapStatsData(
    NvMediaISP *isp,
    NvMediaISPStatsSurface *statsSurface,
    uint32_t instance,
    NvMediaISPLocalToneMapStatsData *statsData
);

/**
 * \brief Gets the bad pixel statistics data.
 * \param[in] isp           ISP object.
 * \param[in] statsSurface  A pointer to a statistics surface.
 * \param[in] instance      Instance of the block to get statistics.
 * \param[in,out] statsData A pointer to bad pixel statistics data.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISPGetBadPixelStatsData(
    NvMediaISP *isp,
    NvMediaISPStatsSurface *statsSurface,
    uint32_t instance,
    NvMediaISPBadPixelStatsData *statsData
);

/**
 * \brief Gets the flicker band statistics data.
 * \param[in] isp           ISP object.
 * \param[in] statsSurface  A pointer to a statistics surface.
 * \param[in] instance      Instance of the block to get statistics.
 * \param[in,out] statsData A pointer to flicker band statistics data.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more parameters
 *  were invalid.
 * \retval  NVMEDIA_STATUS_ERROR indicates that there was some other error.
 */
NvMediaStatus
NvMediaISPGetFlickerBandStatsData(
    NvMediaISP *isp,
    NvMediaISPStatsSurface *statsSurface,
    uint32_t instance,
    NvMediaISPFlickerBandStatsData *statsData
);

/*@} <!-- Ends nvmedia_isp_api Image Signal Processing --> */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_ISP_H */
