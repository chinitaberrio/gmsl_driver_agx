/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION. All rights reserved. All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */


/**
 * \file
 * \brief <b> NVIDIA Media VPI Interface </b>
 *
 * @b Description: This file contains the NvMedia VPI API.
 */

#ifndef _NvMediaVPI_H
#define _NvMediaVPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"
#include "nvmedia_common.h"
#include "nvmedia_image.h"
#include "nvmedia_array.h"
#include "nvmedia_image_pyramid.h"

/**
 * \defgroup NvMediaVPI Vision Programming Interface
 *
 * The NvMedia Vision Programming Interface (VPI) API contains NvMedia
 * functions for accessing the Computer Vision (CV) hardware accelerated
 * algorithms.
 *
 * @ingroup nvmedia_top
 * @{
 */

/** \brief Major Version number */
#define NVMEDIA_VPI_VERSION_MAJOR   2
/** \brief Minor Version number */
#define NVMEDIA_VPI_VERSION_MINOR   2

/** \brief Maximum tasks that can be queued - used by \ref NvMediaVPICreate. */
#define NVMEDIA_VPI_MAX_QUEUED_TASKS 64

/**
 * \brief Holds an opaque object representing NvMediaVPI.
 */
typedef struct NvMediaVPI NvMediaVPI;

/** \brief Defines the different types of VPI-specific Arrays.
 */
typedef enum {
    /*! Keypoint type. */
    NVMEDIA_VPI_ARRAY_TYPE_KEYPOINT = 0,
    /*! Bounding Box with Transform type. */
    NVMEDIA_VPI_ARRAY_TYPE_BBOX_XFORM = 1,
    /*! Transform type. */
    NVMEDIA_VPI_ARRAY_TYPE_TRANSFORM = 2,
    /*! Number of types - If more types are needed, add above this and update this.*/
    NVMEDIA_VPI_ARRAY_NUM_TYPES = 2,
} NvMediaVPIArrayType;

/**
* \brief Holds 2D floating point definition.
*/
typedef struct {
    /*! x co-ordinate. */
    float x;
    /*! y co-ordinate. */
    float y;
} NvMediaVPIPoint2Df;

/**
 * \brief Axis Aligned Bounding Box.
 *  Holds coordinates, scale factor delta portion and status.
 */
typedef struct {
    /*! x coordinate. */
    float x;
    /*! y coordinate. */
    float y;
    /*! width of bounding box. */
    float width;
    /*! height of bounding box. */
    float height;
    /*! 1 indicates tracking is invalid, 0 indicates valid. */
    uint8_t trackingStatus;
    /*! 1 indicatestemplate needs updating, 0 indicates template is valid. */
    uint8_t updateTemplate;
    /*! Reserved for future. Do not read/write this field. */
    uint8_t reserved1;
    /*! Reserved for future. Do not read/write this field. */
    uint8_t reserved2;
} NvMediaVPIAABB;

/**
 * \brief 3x3 transform matrix
 */
typedef struct {
    float mat3[3][3];
} NvMediaVPI2DTransform;

/**
 * \brief Translation with Scale parameters.
 */
typedef struct {
    /*! Scale. */
    float deltaScale;
    /*! Need description. */
    float deltaX;
    /*! Need description. */
    float deltaY;
} NvMediaVPITranslationWithScale;

/**
* \brief Holds bounding box definition.
*/
typedef struct {
    /*! Transform array */
    NvMediaVPI2DTransform transform;
    /*! width of bounding box. */
    float width;
    /*! height of bounding box. */
    float height;
    /*! 1 indicates tracking is invalid, 0 indicates valid. */
    uint8_t trackingStatus;
    /*! 1 indicates template needs updating, 0 indicates template is valid. */
    uint8_t templateStatus;
    /*! Reserved for future. Do not read/write this field. */
    uint8_t reserved1;
    /*! Reserved for future. Do not read/write this field. */
    uint8_t reserved2;
} NvMediaVPIBoundingBoxWithTransform;

/**
 * \brief Returns the number of HW engines accessible.
 * \param[in] numEngines A pointer to the number of engines.
 * \return \ref NvMediaStatus The status of the operation.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if the pointer is invalid.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIGetNumEngines(
    uint32_t *numEngines
);

/**
 * \brief Returns the version information for the NvMediaVPI library.
 * \param[in] version A pointer to a \ref NvMediaVersion structure
 *                    filled by the NvMediaVPI library.
 * \return \ref NvMediaStatus The status of the operation.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if the pointer is invalid.
 */
NvMediaStatus
NvMediaVPIGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Creates an NvMediaVPI object.
 * \param[in] Id Engine ID. This value must be < the result of \ref NvMediaVPIGetNumEngines.
 * \param[in] maxQueuedTasks
 *      Number of simultaneous tasks that can be submitted
 *      to an instance at a time, before \ref NvMediaVPIFlush must be called.
 *      maxQueuedTasks will be clamped between 1 and
 *      \ref NVMEDIA_VPI_MAX_QUEUED_TASKS.
 * \return A pointer to \ref NvMediaVPI object if successful; or NULL if unsuccessful.
 */
NvMediaVPI *
NvMediaVPICreate(
    const uint32_t Id,
    const uint32_t maxQueuedTasks
);

/**
 * \brief Destroys an NvMediaVPI object created by \ref NvMediaVPICreate.
 * \param[in] vpi A pointer to the object to be destroyed.
 * \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if input argument is invalid.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIDestroy(
     NvMediaVPI *vpi
);

/**
 * \brief Flushes all queued operations.
 * \param[in] vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if input argument is invalid.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIFlush(
     NvMediaVPI *vpi
);

/**
 * \brief Registers an image for use with an NvMediaVPI function.
 * \param[in] vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in] image A pointer to image to be registered.
 * \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if input argument is invalid.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIImageRegister(
    NvMediaVPI *vpi,
    NvMediaImage *image
);

/**
 * \brief Unregisters an image after use.
 * \param[in] vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in] image A pointer to the image to be unregistered.
 * \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if input argument is invalid.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIImageUnregister(
    NvMediaVPI *vpi,
    NvMediaImage *image
);

/**
 * \brief Registers an array for use with an NvMediaVPI function.
 * \param[in] vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in] array A pointer to array to be registered.
 * \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if input argument is invalid.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIArrayRegister(
    NvMediaVPI *vpi,
    NvMediaArray *array
);

/**
 * \brief Unregisters an array after use.
 * \param[in] vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in] array A pointer to array to be unregistered.
 * \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if input argument is invalid.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIArrayUnregister(
    NvMediaVPI *vpi,
    NvMediaArray *array
);

/**
 * \brief Creates an \ref NvMediaArray for a VPI-specific element type.
 * \param[in] device Handle to the NvMedia device obtained by calling \ref NvMediaDeviceCreate.
 * \param[in] type \ref NvMediaArrayType Type of Array to be created.
 * \param[in] numElements number of Elements in the array.
 * \param[in] attrs An array of \ref NvMediaArrayAllocAttr Allocation attributes.
 * \param[in] numAttrs Number of allocation objects @a attrs.
 * \return    \ref NvMediaArray Handle to array. Null if unsuccessful.
 */
NvMediaArray *
NvMediaVPIArrayCreate(
    NvMediaDevice *device,
    NvMediaVPIArrayType type,
    uint32_t numElements,
    const NvMediaArrayAllocAttr *attrs,
    uint32_t numAttrs
);

/**
 * \brief Registers a pyramid for use with an NvMediaVPI function.
 * \param[in] vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in] pyramid A pointer to the pyramid to be registered.
 * \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if input argument is invalid.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIPyramidRegister(
    NvMediaVPI *vpi,
    NvMediaImagePyramid *pyramid
);

/**
 * \brief Unregisters a pyramid after use.
 * \param[in] vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in] pyramid A pointer to the pyramid to be unregistered.
 * \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if input argument is invalid.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIPyramidUnregister(
    NvMediaVPI *vpi,
    NvMediaImagePyramid *pyramid
);

/**
 * \brief NvMediaVPIProcessStereo descriptor.
 */
typedef struct NvMediaVPIProcessStereoDescriptor NvMediaVPIProcessStereoDescriptor;

/**
 * \brief Enumerates the types of output for Optical Flow/Stereo Disparity
 * processing.
 */
typedef enum {
    /*! Disparity relative to reference */
    NVMEDIA_VPI_STEREO_DISPARITY = 0,
    /*! Absolute Motion Vector output type. */
    NVMEDIA_VPI_MV,
    /*! Motion Vector hints for the NVENC hardware engine, which performs
     *  video encoding.
     *  Specifically, when this flag is used with \c processStereoPair(), the
     *  output holds data of type \ref NvMediaEncodeExternalMEHint. */
    NVMEDIA_VPI_MV_HINTS,
} NvMediaVPIOFSTOutputType;

/**
 * \brief Creates \ref NvMediaVPIProcessStereoDescriptor.
 *
 * \param[in]  vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in]  width Width of the input images.
 * \param[in]  height Height of the input images.
 * \param[in]  outputType \ref NvMediaVPIOFSTOutputType for type of output.
 * \return     A pointer to the \ref NvMediaVPIProcessStereoDescriptor.
 */
NvMediaVPIProcessStereoDescriptor *
NvMediaVPICreateProcessStereoPairDescriptor(
    NvMediaVPI *vpi,
    const uint32_t width,
    const uint32_t height,
    const NvMediaVPIOFSTOutputType outputType
);

/**
 * \brief Queues an operation to process a stereo image pair.
 * \note \ref NvMediaVPIFlush() must be called to actually flush the queue to the engine.
 *
 * \param[in]     vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in,out] descriptor Pointer returned by \ref NvMediaVPICreateProcessStereoPairDescriptor.
 * \param[in]     left A pointer to the left \ref NvMediaImage image.
 * \param[in]     right A pointer to the right \ref NvMediaImage image.
 * \param[out]    output A pointer to the output \ref NvMediaImage image.
 * \return        \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED for unsupported operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING for full queue. Call \ref NvMediaVPIFlush .
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIProcessStereoPairDesc(
    NvMediaVPI *vpi,
    NvMediaVPIProcessStereoDescriptor *descriptor,
    NvMediaImage *left,
    NvMediaImage *right,
    NvMediaImage *output
);

/**
 * \brief NvMediaVPIConvertMV descriptor.
 */
typedef struct NvMediaVPIConvertMVDescriptor NvMediaVPIConvertMVDescriptor;

/**
 * \brief Creates an \ref NvMediaVPIConvertMVDescriptor.
 *
 * \param[in]  vpi A pointer to the \ref NvMediaVPI object that NvMediaVPICreate() returns.
 * \param[in]  width Width of the input depth image.
 * \param[in]  height Height of the input depth image.
 * \param[in]  type \ref NvMediaSurfaceType of the input depth image.
 * \param[in]  strength Smoothing strength: (0 - 1).
 * \param[in]  scale Scale Factor.
 * \return     A pointer to the \ref NvMediaVPIConvertMVDescriptor.
 */
NvMediaVPIConvertMVDescriptor *
NvMediaVPICreateConvertMVDescriptor(
    NvMediaVPI *vpi,
    const uint32_t width,
    const uint32_t height,
    const NvMediaSurfaceType type,
    const float strength,
    const float scale
);

/**
 * \brief Converts or refines a motion vector image.
 * The processing done  on the image depends on the \a outputType parameter:
 * - If set to \c NVMEDIA_VPI_STEREO_DISPARITY, this function converts the
 *   image to a stereo disparity.
 * - If set to \c NVMEDIA_VPI_MV this function refines it.
 * \note \ref NvMediaVPIFlush() must be called to actually flush the queue to
 * the engine.
 *
 * \param[in]     vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in,out] descriptor A Pointer returned by \ref NvMediaVPICreateConvertMVDescriptor.
 * \param[in]     inputMVImage Input Motion Vectors image which must be converted/refined.
 * \param[in]     inputColor Input color \ref NvMediaImage image.
 * \param[out]    output Output \ref NvMediaImage image which refined.
 * \param[in]     outputType \ref NvMediaVPIOFSTOutputType.
 * \return        \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED for unsupported operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING for full queue. Call \ref NvMediaVPIFlush .
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIConvertMVDesc(
    NvMediaVPI *vpi,
    NvMediaVPIConvertMVDescriptor *descriptor,
    NvMediaImage *inputMVImage,
    NvMediaImage *inputColor,
    NvMediaImage *output,
    const NvMediaVPIOFSTOutputType outputType
);

/**
 * \brief Enumerates the types of post-process non-maximum suppression.
 */
typedef enum {
    /* Use a 2D grid of 8x8 cells with no minimal distance. */
    NVMEDIA_VPI_2D_GRID_8x8_CELL_NO_MIN_DISTANCE = 0
} NvMediaVPINonMaxSuppressionType;

/**
 * \brief Enumerates the types of filters used in the Harris corner detector.
 */
typedef enum {
    /*! Built-in normalized separable Sobel filter. */
    NVMEDIA_VPI_HARRIS_FILTER_TYPE_DEFAULT = 0,
    /*! User-supplied separable gradient filter. The filter is automatically normalized. */
    NVMEDIA_VPI_HARRIS_FILTER_TYPE_USER
} NvMediaVPIHarrisGradientFilterType;

/*
 * \brief NvMediaVPIGetKeyPointsHarrisDescriptor.
 */
typedef struct NvMediaVPIGetKeyPointsHarrisDescriptor NvMediaVPIGetKeyPointsHarrisDescriptor;

/**
 * \brief Creates a pointer to the \ref NvMediaVPIGetKeyPointsHarrisDescriptor.
 *
 * \param[in]  vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in]  width Input image width.
 * \param[in]  height Input image height.
 * \param[in]  nonMaxSuppressionType Post-process non-maximum suppression type.
 * \return     A pointer to the \ref NvMediaVPIGetKeyPointsHarrisDescriptor.
 */
NvMediaVPIGetKeyPointsHarrisDescriptor *
NvMediaVPICreateGetKeyPointsHarrisDescriptor(
    NvMediaVPI *vpi,
    const uint32_t width,
    const uint32_t height,
    const NvMediaVPINonMaxSuppressionType nonMaxSuppressionType);

/**
 * \brief Holds the Harris key-point parameters.
 */
typedef struct {
    /*! Gradient window size. Must be 3, 5 or 7. */
    const uint32_t gradientSize;
    /*! Filter type. */
    const NvMediaVPIHarrisGradientFilterType gradientFilterType;
    /*! Horizotal filter of gradientSize coefficients */
    const int16_t *horizontalFilterCoefficients;
    /*! Vertical filter of gradientSize coefficients*/
    const uint16_t *verticalFilterCoefficients;
    /*! Block window size used to compute the Harris Corner score. Must be 3, 5 or 7. */
    const uint32_t blockSize;
    /*! Specifies the minimum threshold with which to eliminate Harris Corner scores */
    const float strengthThresh;
    /*! Specifies sensitivity threshold from the Harris-Stephens equation. */
    const float sensitivity;
    /*! Specifies the post-process non-maximum suppression type. */
    const NvMediaVPINonMaxSuppressionType nonMaxSuppressionType;
    /*! Specifies the radial Euclidean distance for non-maximum suppression */
    const float minDistance;
} NvMediaVPIGetKeyPointsHarrisParams;

/**
 * \brief Queues an operation to get Harris keypoints throughout an image.
 *        If the tracked keypoints array is not empty, then those keypoints are inserted
 *        at the beginning of the output \a keypoints array.  \a An optional \a mask
 *        can be supplied to suppress detected and tracked keypoints.
 *        Suppressed tracked keypoints are removed from the output array.
 *        Nonmaximum suppression is applied only to newly detected keypoints.
 * \note \ref NvMediaVPIFlush() must be called to actually flush the queue to
 * the engine.
 *
 * \param[in]     vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in,out] descriptor Pointer returned by
 *                \ref NvMediaVPICreateGetKeyPointsHarrisDescriptor.
 * \param[in]     params A pointer to \ref NvMediaVPIGetKeyPointsHarrisParams.
 * \param[in]     input \ref NvMediaImage image.
 * \param[in]     mask \ref NvMediaImage mask image. Optional.
 *                Mask value: 0 - suppressed, 0xFF - not suppressed.
 * \param[in]     trackedKeypoints \ref NvMediaArray containing keypoints to track.
 *                Optional. If NULL then trackedScores must be NULL.
 *                \a array must be of \ref NVMEDIA_VPI_ARRAY_TYPE_KEYPOINT type
 *                created using \ref NvMediaVPIArrayCreate.
 * \param[in]     trackedScores \ref NvMediaArray containing scores of keypoints to track.
 *                Optional. If NULL then trackedKeypoints must be NULL.
 * \param[out]    keypoints \ref NvMediaArray containing keypoints.
 *                \a array must be of \ref NVMEDIA_VPI_ARRAY_TYPE_KEYPOINT type
 *                created using \ref NvMediaVPIArrayCreate.
 * \param[out]    scores \ref NvMediaArray containing scores.
 * \return        \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED for unsupported operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING for full queue. Call \ref NvMediaVPIFlush.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIGetKeyPointsHarrisDesc(
    NvMediaVPI *vpi,
    NvMediaVPIGetKeyPointsHarrisDescriptor *descriptor,
    const NvMediaVPIGetKeyPointsHarrisParams *params,
    NvMediaImage *input,
    NvMediaImage *mask,
    NvMediaArray *trackedKeypoints,
    NvMediaArray *trackedScores,
    NvMediaArray *keypoints,
    NvMediaArray *scores
);

/*
 * \brief NvMediaVPIGetKeyPointsFastDescriptor.
 */
typedef struct NvMediaVPIGetKeyPointsFastDescriptor NvMediaVPIGetKeyPointsFastDescriptor;

/**
 * \brief Creates a pointer to the \ref NvMediaVPIGetKeyPointsFastDescriptor.
 *
 * \param[in]  vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in]  width Width of the input image.
 * \param[in]  height Height of the input image.
 * \param[in]  type \ref NvMediaSurfaceType of the input image.
 * \return     A pointer to the \ref NvMediaVPIGetKeyPointsFastDescriptor./
 */
NvMediaVPIGetKeyPointsFastDescriptor *
NvMediaVPICreateGetKeyPointsFastDescriptor(
    NvMediaVPI *vpi,
    const uint32_t width,
    const uint32_t height,
    const NvMediaSurfaceType type
);

/**
 * \brief Queues an operation to get FAST keypoints throughout an image.
 *        If the key points array is not empty, then it tracks the key points as well.
 * \note \ref NvMediaVPIFlush() must be called to actually flush the queue to the engine.
 *
 * \param[in]     vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in,out] descriptor Pointer returned by \ref NvMediaVPICreateGetKeyPointsFastDescriptor.
 * \param[in]     input \ref NvMediaImage image.
 * \param[in]     strengthThresh Threshold on difference between intensity of the central pixel
 *                and pixels on Bresenham's circle of radius 3.
 * \param[in]     nonmaxSupression Boolean flag that indicates whether to apply
 *                non-maximum suppression.
 * \param[in,out] keypoints A pointer to an \ref NvMediaArray containing keypoints.
 * \param[out]    scores A pointer to an \ref NvMediaArray containing scores.
 * \return        \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED for unsupported operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING for full queue. Call \ref NvMediaVPIFlush .
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIGetKeyPointsFastDesc(
    NvMediaVPI *vpi,
    NvMediaVPIGetKeyPointsFastDescriptor *descriptor,
    NvMediaImage *input,
    const uint32_t strengthThresh,
    const NvMediaBool nonmaxSupression,
    NvMediaArray *keypoints,
    NvMediaArray *scores
);

/** Defines the conditions that cause the Lucas-Kanade Feature Tracker
 *  to be terminated.
 */
typedef enum {
    /** Specifies termination by the number of iterations. */
    NVMEDIA_VPI_TERMINATION_CRITERION_ITERATIONS = 0,
    /** Specifies termination by matching the value against epsilon. */
    NVMEDIA_VPI_TERMINATION_CRITERION_EPSILON,
    /** Specifies termination by iterations or epsilon, depending
      * on which happens first. */
    NVMEDIA_VPI_TERMINATION_CRITERION_BOTH
} NvMediaVPITerminationCriterion;

/**
 * \brief NvMediaVPIGetSparseFlowPyrLKParams.
 */
typedef struct {
    /*! Specifies the termination criteria. */
    const NvMediaVPITerminationCriterion termination;
    /*! Specifies the error for terminating the algorithm. */
    float epsilon;
    /*! Number of Points. */
    uint32_t numPoints;
    /*! Specifies the number of iterations. */
    uint32_t numIterations;
    /*! Specifies the size of the window on which to perform the algorithm.
     *  This field must be greater than
     *  or equal to 3 and less than or equal to 32. */
    uint32_t windowDimension;
    /*! Specifies the boolean flag that indicates whether to calculate tracking error. */
    int32_t calcError;
    /*! Number of levels in the pyramid. */
    uint32_t numLevels;
} NvMediaVPIGetSparseFlowPyrLKParams;

/**
 * \brief NvMediaVPISparseFlowPyr descriptor.
 */
typedef struct NvMediaVPIGetSparseFlowPyrLKDescriptor NvMediaVPIGetSparseFlowPyrLKDescriptor;

/**
 * \brief Creates a pointer to the \ref NvMediaVPIGetSparseFlowPyrLKDescriptor.
 *
 * \param[in]  vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in]  params A pointer to \ref NvMediaVPIGetSparseFlowPyrLKParams.
 * \return     A pointer to the \ref NvMediaVPIGetSparseFlowPyrLKDescriptor.
 */
NvMediaVPIGetSparseFlowPyrLKDescriptor *
NvMediaVPICreateGetSparseFlowPyrLKDescriptor(
    NvMediaVPI *vpi,
    const NvMediaVPIGetSparseFlowPyrLKParams *params
);

/**
 * \brief Queues an operation to track features in pyramid.
 * \note \ref NvMediaVPIFlush() must be called to actually flush the queue to  the engine.
 *
 * \param[in]     vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in,out] descriptor Pointer returned by \ref NvMediaVPICreateGetSparseFlowPyrLKDescriptor.
 * \param[in]     params A pointer to \ref NvMediaVPIGetSparseFlowPyrLKParams.
 * \param[in]     oldImages Input \ref NvMediaImagePyramid of first (old) images.
 * \param[in]     newImages Input \ref NvMediaImagePyramid of destination (new) images.
 * \param[in]     oldPoints A pointer to an array of keypoints in the \a oldImages
 *                high resolution pyramid.
 * \param[in,out] newPoints A pointer to an array of keypoints in the \a newImages
 *                high resolution pyramid. It is used for starting
 *                search in \a newImages.
 * \param[out]    newStatus pointer to array for tracking status for each input in
 *                \a oldPoints.
 * \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED for unsupported operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING for full queue. Call \ref NvMediaVPIFlush .
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus NvMediaVPIGetSparseFlowPyrLKDesc(
    NvMediaVPI *vpi,
    NvMediaVPIGetSparseFlowPyrLKDescriptor *descriptor,
    const NvMediaVPIGetSparseFlowPyrLKParams *params,
    NvMediaImagePyramid *oldImages,
    NvMediaImagePyramid *newImages,
    NvMediaArray *oldPoints,
    NvMediaArray *newPoints,
    NvMediaArray *newStatus
);

/**
 * \brief NvMediaVPIKLTDescriptor descriptor.
 */
typedef struct NvMediaVPIKLTDescriptor NvMediaVPIKLTDescriptor;

/**
 * \brief Creates a pointer to the \ref NvMediaVPIKLTDescriptor.
 *
 * \param[in]  vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in]  width Width of the input image.
 * \param[in]  height Height of the input image.
 * \param[in]  type \ref NvMediaSurfaceType of the input image.
 * \return     A pointer to the \ref NvMediaVPIKLTDescriptor.
 */
NvMediaVPIKLTDescriptor *
NvMediaVPICreateKLTDescriptor(
    NvMediaVPI *vpi,
    const uint32_t width,
    const uint32_t height,
    const NvMediaSurfaceType type
);

/** Defines the types of KLT Tracking. */
typedef enum NvMediaVPIKltTrackType
{
    /** Inverse Compositional Tracking Type. */
    NVMEDIA_VPI_KLT_INVERSE_COMPOSITIONAL = 0,
} NvMediaVPIKltTrackType;

/**
 * \brief Holds the KLT parameters.
 */
typedef struct {
    /*! Num Iterations Scaling. */
    uint32_t numIterationsScaling;
    /*! Num Iterations Translation. */
    uint32_t numIterationsTranslation;
    /*! Num Iterations Coarse. */
    uint32_t numIterationsCoarse;
    /*! Num Iterations Fine. */
    uint32_t numIterationsFine;
    /*! Max Pixel Tolerance. */
    float maxPixelTolerance;
    /*! Threshold Update. */
    float thresholdUpdate;
    /*! Threshold Kill. */
    float thresholdKill;
    /*! Threshold Stop. */
    float thresholdStop;
    /*! Max Scale Change. */
    float maxScaleChange;
    /*! Tracking Type. */
    NvMediaVPIKltTrackType trackingType;
} NvMediaVPIKLTParams;

/**
 * \brief Queues an operation to run Kanade Lucas Tomasi (KLT) tracking algorithm.
 * \note \ref NvMediaVPIFlush() must be called to actually flush the queue
 * to the engine.
 *
 * \param[in]     vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in,out] descriptor Pointer returned by \ref NvMediaVPIKLTDescriptor.
 * \param[in]     params A pointer to \ref NvMediaVPIKLTParams.
 * \param[in]     referenceImage \ref NvMediaImage reference image.
 * \param[in]     templateImage \ref NvMediaImage template image.
 * \param[in]     inputBoxList \ref NvMediaArray input bounding box list.
 *                \a array must be of \ref NVMEDIA_VPI_ARRAY_TYPE_BBOX_XFORM type
 *                created using \ref NvMediaVPIArrayCreate.
 * \param[in]     predictedBoxList \ref NvMediaArray predicted bounding box list.
 *                \a array must be of \ref NVMEDIA_VPI_ARRAY_TYPE_TRANSFORM type
 *                created using \ref NvMediaVPIArrayCreate.
 * \param[out]    outputBoxList \ref NvMediaArray output bounding box list.
 *                \a array must be of \ref NVMEDIA_VPI_ARRAY_TYPE_BBOX_XFORM type
 *                created using \ref NvMediaVPIArrayCreate.
 * \param[out]    estimationList \ref NvMediaArray output estimated motion.
 *                \a array must be of \ref NVMEDIA_VPI_ARRAY_TYPE_TRANSFORM type
 *                created using \ref NvMediaVPIArrayCreate.
 * \return        \retval NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED for unsupported operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING for full queue. Call \ref NvMediaVPIFlush.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIKLTDesc(
    NvMediaVPI *vpi,
    NvMediaVPIKLTDescriptor *descriptor,
    const NvMediaVPIKLTParams *params,
    NvMediaImage *referenceImage,
    NvMediaImage *templateImage,
    NvMediaArray *inputBoxList,
    NvMediaArray *predictedBoxList,
    NvMediaArray *outputBoxList,
    NvMediaArray *estimationList
);

/**
 * \brief Queues an operation to generate a Gaussian pyramid from an input image.
 * \note \ref NvMediaVPIFlush() must be called to actually flush the queue
 * to the engine.
 *
 * \param[in]  vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in]  input \ref NvMediaImage image.
 *             The input image may be the same NvMediaImage as is returned by
 *             \c  NvMediaImagePyramidGetImageForLevel(output, 0), thereby saving a
 *             copy operation of the input to the output.
 * \param[out] output \ref NvMediaImagePyramid.
 * \return     \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED for unsupported operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING for full queue. Call \ref NvMediaVPIFlush .
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIGetImagePyramid(
    NvMediaVPI *vpi,
    NvMediaImage *input,
    NvMediaImagePyramid *output
);

/**
 * \brief Queues an operation to convolve an input image with client supplied
 * convolution matrix.
 * \note \ref NvMediaVPIFlush() must be called to actually flush the queue
 * to the engine.
 *
 * \param[in]  vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in]  input \ref NvMediaImage image.
 * \param[in]  kernelData A pointer to the convolution kernel coefficients.
 * \param[in]  kernelWidth Convolution kernel width.
 * \param[in]  kernelHeight Convolution kernel height.
 * \param[out] output \ref NvMediaImage image.
 * \return     \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED for unsupported operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING for full queue. Call \ref NvMediaVPIFlush .
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIConvolveImage(
    NvMediaVPI *vpi,
    NvMediaImage *input,
    const float *kernelData,
    const uint32_t kernelWidth,
    const uint32_t kernelHeight,
    NvMediaImage *output
);

/**
 * \brief NvMediaVPIConvolveImageSeparableDescriptor.
 */
typedef struct NvMediaVPIConvolveImageSeparableDescriptor \
               NvMediaVPIConvolveImageSeparableDescriptor;

/**
 * \brief Creates a pointer to the \ref NvMediaVPIConvolveImageSeparableDescriptor.
 *
 * \param[in]  vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in]  type \ref NvMediaSurfaceType of the input image.
 * \param[in]  kernelX A pointer to the x convolution kernel coefficients.
 * \param[in]  kernelXSize Size of the x convolution kernel.
 * \param[in]  kernelY A pointer to the y convolution kernel coefficients.
 * \param[in]  kernelYSize Size of the y convolution kernel.
 * \return     A pointer to the \ref NvMediaVPIConvolveImageSeparableDescriptor.
 */
NvMediaVPIConvolveImageSeparableDescriptor *
NvMediaVPICreateConvolveImageSeparableDescriptor(
    NvMediaVPI *vpi,
    NvMediaSurfaceType type,
    const float *kernelX,
    const uint32_t kernelXSize,
    const float *kernelY,
    const uint32_t kernelYSize
);

/**
 * \brief Queues an operation to convolve an image with a 2D kernel, using separable
 * convolution.
 * \note \ref NvMediaVPIFlush() must be called to actually flush the queue to the engine.
 *
 * \param[in]  vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in]  input \ref NvMediaImage image.
 * \param[in]  kernelX A pointer to the x convolution kernel coefficients.
 * \param[in]  kernelXSize Size of the x convolution kernel.
 * \param[in]  kernelY A pointer to the y convolution kernel coefficients.
 * \param[in]  kernelYSize Size of the y convolution kernel.
 * \param[in]  output A pointer to an \ref NvMediaImage image.
 * \return    \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED for unsupported operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING for full queue. Call \ref NvMediaVPIFlush .
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIConvolveImageSeparable(
    NvMediaVPI *vpi,
    NvMediaImage *input,
    const float *kernelX,
    const uint32_t kernelXSize,
    const float *kernelY,
    const uint32_t kernelYSize,
    NvMediaImage *output
);

/**
 * \brief Queues an operation to compute a generic Box filter window of the input image.
 * \note \ref NvMediaVPIFlush() must be called to actually flush the queue to the engine.
 *
 * \param[in]     vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in]     input \ref NvMediaImage image.
 * \param[in]     windowWidth Window width.
 * \param[in]     windowHeight Window height.
 * \param[in]     output \ref NvMediaImage image.
 * \return        \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED for unsupported operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING for full queue. Call \ref NvMediaVPIFlush .
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIFilterImageBox(
    NvMediaVPI *vpi,
    NvMediaImage *input,
    const uint32_t windowWidth,
    const uint32_t windowHeight,
    NvMediaImage *output
);

/**
* Helper API to convert an \ref NvMediaArray of \ref NVMEDIA_VPI_ARRAY_TYPE_KEYPOINT type to an array of
* \ref NvMediaVPIPoint2Df.
*
* \param[in]  array A pointer to the input \ref NvMediaArray.
*                   \a array must be created with \ref NvMediaVPIArrayCreate.
* \param[in]  startIndex Index offset (number of elements) into the array. Currently, only 0 is supported.
* \param[in]  numElements Number of elements to write to the destination.
* \param[out] destPtr A pointer to the memory location where the output array is to be written.
* \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
*/
NvMediaStatus
NvMediaVPIArrayGetPoint2Df(
    NvMediaArray *array,
    const uint32_t startIndex,
    const uint32_t numElements,
    NvMediaVPIPoint2Df *destPtr
);

/**
* Helper API to convert an array of \ref NvMediaVPIPoint2Df to \ref NvMediaArray
* of \ref NVMEDIA_VPI_ARRAY_TYPE_KEYPOINT type.
* \param[in]  inputPtr A pointer to the input array.
* \param[in]  startIndex Index offset (number of elements) into the destination array.
* \param[in]  numElements Number of elements to write to the destination.
* \param[out] destArray A pointer to the output \ref NvMediaArray.
*                   \a destArray must be created by calling \ref NvMediaVPIArrayCreate
*                   with type set to \ref NVMEDIA_VPI_ARRAY_TYPE_KEYPOINT.
* \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
*/
NvMediaStatus
NvMediaVPIArrayUpdatePoint2Df(
    NvMediaVPIPoint2Df *inputPtr,
    const uint32_t startIndex,
    const uint32_t numElements,
    NvMediaArray *destArray
);

/**
* Helper API to convert an \ref NvMediaArray of \ref NVMEDIA_VPI_ARRAY_TYPE_BBOX_XFORM type to an array of
* \ref NvMediaVPIAABB.
*
* \param[in]  array A pointer to input \ref NvMediaArray.
*                   \a array must be created with \ref NvMediaVPIArrayCreate.
* \param[in]  startIndex index offset (number of elements) into array.
* \param[in]  numElements number of elements to write to destination.
* \param[out] destPtr pointer to location in memory to write output array.
* \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
*/
NvMediaStatus
NvMediaVPIArrayGetAABB(
    NvMediaArray *array,
    const uint32_t startIndex,
    const uint32_t numElements,
    NvMediaVPIAABB *destPtr
);

/**
* Helper API to convert an array of \ref NvMediaVPIAABB to \ref NvMediaArray of \ref NVMEDIA_VPI_ARRAY_TYPE_BBOX_XFORM type.
*
* \param[in]  inputPtr pointer to the input array.
* \param[in]  startIndex index offset (number of elements) into the destination array.
* \param[in]  numElements Number of elements to write to the destination.
* \param[out] destArray A pointer to the output \ref NvMediaArray.
*                   \a destArray must be created using \ref NvMediaVPIArrayCreate.
* \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
*/
NvMediaStatus
NvMediaVPIArrayUpdateAABB(
    NvMediaVPIAABB *inputPtr,
    const uint32_t startIndex,
    const uint32_t numElements,
    NvMediaArray *destArray
);

/**
* Helper API to convert an \ref NvMediaArray of \ref NVMEDIA_VPI_ARRAY_TYPE_BBOX_XFORM type to an array of
* \ref NvMediaVPIBoundingBoxWithTransform.
*
* \param[in]  array A pointer to input \ref NvMediaArray.
*                   \a array must be created using \ref NvMediaVPIArrayCreate.
* \param[in]  startIndex Index offset (number of elements) into array.
* \param[in]  numElements Number of elements to write to destination.
* \param[out] destPtr A pointer to location in memory to write output array.
*/
NvMediaStatus
NvMediaVPIArrayGetBoundingBoxWithTransform(
    NvMediaArray *array,
    const uint32_t startIndex,
    const uint32_t numElements,
    NvMediaVPIBoundingBoxWithTransform *destPtr
);

/**
* Helper API to convert an array of \ref NvMediaVPIBoundingBoxWithTransform to \ref NvMediaArray of \ref NVMEDIA_VPI_ARRAY_TYPE_BBOX_XFORM type.
*
* \param[in]  inputPtr A pointer to location in memory for input array.
* \param[in]  startIndex Index offset (number of elements) into destination array.
* \param[in]  numElements Number of elements to write to the destination.
* \param[out] destArray A pointer to output \ref NvMediaArray.
*                   \a destArray must be created using \ref NvMediaVPIArrayCreate.
* \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
*/
NvMediaStatus
NvMediaVPIArrayUpdateBoundingBoxWithTransform(
    NvMediaVPIBoundingBoxWithTransform *inputPtr,
    const uint32_t startIndex,
    const uint32_t numElements,
    NvMediaArray *destArray
);

/**
* Helper API to convert an \ref NvMediaArray of \ref NVMEDIA_VPI_ARRAY_TYPE_TRANSFORM type to an array of
* \ref NvMediaVPITranslationWithScale.
*
* \param[in]  array A pointer to input \ref NvMediaArray.
*                   \a array must be created using \ref NvMediaVPIArrayCreate.
* \param[in]  startIndex Index offset (number of elements) into array.
* \param[in]  numElements Number of elements to write to destination.
* \param[out] destPtr A pointer to location in memory to write output array.
*/

NvMediaStatus
NvMediaVPIArrayGetTranslationWithScale(
    NvMediaArray *array,
    const uint32_t startIndex,
    const uint32_t numElements,
    NvMediaVPITranslationWithScale *destPtr
);

/**
* Helper API to convert an array of \ref NvMediaVPITranslationWithScale to
* \ref NvMediaArray of \ref NVMEDIA_VPI_ARRAY_TYPE_TRANSFORM type.
*
* \param[in]  inputPtr A pointer to location in memory for input array.
* \param[in]  startIndex Index offset (number of elements) into destination array.
* \param[in]  numElements Number of elements to write to destination.
* \param[out] destArray A pointer to output \ref NvMediaArray.
*                   \a destArray must be created using \ref NvMediaVPIArrayCreate.
* \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
*/
NvMediaStatus
NvMediaVPIArrayUpdateTranslationWithScale(
    NvMediaVPITranslationWithScale *inputPtr,
    const uint32_t startIndex,
    const uint32_t numElements,
    NvMediaArray *destArray
);

/**
* Helper API to convert an \ref NvMediaArray of \ref NVMEDIA_VPI_ARRAY_TYPE_TRANSFORM type to an array of
* \ref NvMediaVPI2DTransform.
*
* \param[in]  array A pointer to input \ref NvMediaArray.
*                   \a array must be created using \ref NvMediaVPIArrayCreate.
* \param[in]  startIndex index offset (number of elements) into array.
* \param[in]  numElements number of elements to write to destination.
* \param[out] destPtr pointer to location in memory to write output array.
*/
NvMediaStatus
NvMediaVPIArrayGet2DTransform(
    NvMediaArray *array,
    const uint32_t startIndex,
    const uint32_t numElements,
    NvMediaVPI2DTransform *destPtr
);

/**
* Helper API to convert an array of \ref NvMediaVPI2DTransform to
* \ref NvMediaArray of \ref NVMEDIA_VPI_ARRAY_TYPE_TRANSFORM type.
*
* \param[in]  inputPtr A pointer to location in memory for input array.
* \param[in]  startIndex Index offset (number of elements) into array.
* \param[in]  numElements Number of elements to write to destination.
* \param[out] destArray A pointer to output \ref NvMediaArray.
*                   \a destArray must be created using \ref NvMediaVPIArrayCreate.
* \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK for successful queuing of operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER for incorrect arguments.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
*/
NvMediaStatus
NvMediaVPIArrayUpdate2DTransform(
    NvMediaVPI2DTransform *inputPtr,
    const uint32_t startIndex,
    const uint32_t numElements,
    NvMediaArray *destArray
);

/**
 * \brief Destroy NvMediaVPI function descriptor.
 *
 * \param[in] vpi A pointer to the \ref NvMediaVPI object returned by \ref NvMediaVPICreate.
 * \param[in] descriptor A pointer to NvMediaVPI function descriptor.
 * \return \ref NvMediaStatus The status of the API.
 * Possible values are:
 * - \ref NVMEDIA_STATUS_OK - successful queuing of operation.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER - incorrect arguments.
 * - \ref NVMEDIA_STATUS_NOT_INITIALIZED - uninitialized
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaVPIDestroyDescriptor(
    NvMediaVPI *vpi,
    void *descriptor
);

/*
 * \defgroup history_NvMediaVPI History
 * Provides change history for the NVIDIA VPI API.
 *
 * \section history_NvMediaVPI Version History
 *
 * <b> Version 1.0 </b> March 2, 2018
 * - Initial API definitions.
 *
 * <b> Version 2.0 </b> June 5, 2018
 * - Update Harris tracking API.
 *
 * <b> Version 2.1 </b> July 12, 2018
 * - Add KLT API, and array accessor API's.
 *
 * <b> Version 2.2 </b> July 12, 2018
 * - Add API to query number of available engines.
 *
 */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* _NVMEDIA_VPI_H */
