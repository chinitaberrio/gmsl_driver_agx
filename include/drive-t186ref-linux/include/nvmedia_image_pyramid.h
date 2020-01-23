/*
 * Copyright (c) 2018 - 2019, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: Image Pyramid Processing </b>
 *
 * @b Description: This file contains the
 *    \ref nvmedia_image_pyramid_top "Image Pyramid Processing API."
 */

#ifndef _NVMEDIA_IMAGE_PYRAMID_H
#define _NVMEDIA_IMAGE_PYRAMID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"
#include "nvmedia_image.h"
#include "nvmedia_surface.h"

/**
 * \defgroup nvmedia_image_pyramid_top Image Pyramid Handling API
 *
 * The Image Pyramid Processing API encompasses all NvMedia-related
 * functionality for handling pyramids of \ref NvMediaImage objects.
 *
 * @ingroup nvmedia_top
 * @{
 */

/** \brief Major version number. */
#define NVMEDIA_IMAGE_PYRAMID_VERSION_MAJOR   1
/** \brief Minor version number. */
#define NVMEDIA_IMAGE_PYRAMID_VERSION_MINOR   1

/** \brief Maximum number of levels allowed in a pyramid. */
#define MAX_PYRAMID_LEVELS (10)

/**
 * \brief A handle representing an image pyramid object.
 */
typedef struct NvMediaImagePyramid NvMediaImagePyramid;

/**
 * \brief Allocates an image pyramid.
 *
 * \param[in] device    A handle representing the associated \ref NvMediaDevice.
 * \param[in] type      Surface format type, obtained by calling
 *                       NvMediaSurfaceFormatGetType().
 * \param[in] attrs     A pointer to an array of surface alloc attributes for
 *                       surface creation. Resolution is for the base level of
 *                       the pyramid.
 * \param[in] numLevels Number of levels in the pyramid.
 * \param[in] scale     Scale factor for the pyramid. Must be in the range
 *                       (0.0, 1.0].
 * \param[in] numAttrs  Number of attributes in @a attrs.
 * \param[in] flags     Flags for module hints (reserved for future use).
 * \return  The new image pyramid's handle if successful, or NULL otherwise.
 */
NvMediaImagePyramid *
NvMediaImagePyramidCreate(
    NvMediaDevice *device,
    NvMediaSurfaceType type,
    NvMediaSurfAllocAttr *attrs,
    uint32_t numLevels,
    float scale,
    uint32_t numAttrs,
    uint32_t flags
);

/**
 * \brief Destroys an image pyramid created by NvMediaImagePyramidCreate().
 * \param[in] pyramid A handle to the image pyramid to be destroyed.
 */
void
NvMediaImagePyramidDestroy(
    NvMediaImagePyramid *pyramid
);

/**
 * Locks an image pyramid and returns the associated mapped pointers
 * to the image pyramid surface data. The CPU can only access images created
 * without the \ref NVM_SURF_ATTR_CPU_ACCESS_UNMAPPED attribute.
 * If an image is being used by an internal engine. this function waits until
 * the operation is completed.
 *
 * \param[in] pyramid A handle to the image pyramid object
 * \param[in] lockAccessType Determines the access type.
 * The following access types are supported, and may be OR'd together:
 * - \ref NVMEDIA_IMAGE_ACCESS_READ for read access.
 * - \ref NVMEDIA_IMAGE_ACCESS_WRITE for write access.
 * - \ref NVMEDIA_IMAGE_ACCESS_READ_WRITE for read/write access.
 * \param[out] surfaceMap Pointer to an array of surface descriptors per level.
 * \return  A status code; \ref NVMEDIA_STATUS_OK if the call was successful,
 *  or \ref NVMEDIA_STATUS_ERROR otherwise.
 * \ingroup lock_unlock
 */
NvMediaStatus
NvMediaImagePyramidLock(
    NvMediaImagePyramid *pyramid,
    uint32_t lockAccessType,
    NvMediaImageSurfaceMap *surfaceMap
);

/**
 * Unlocks an image pyramid. Internal engines cannot use a surface
 * until it is locked.
 *
 * \param[in] pyramid A handle to the pyramid object to be unlocked.
 * \ingroup lock_unlock
 */
void
NvMediaImagePyramidUnlock(
    NvMediaImagePyramid *pyramid
);

/**
 * \brief Gets the status of the current or most recent operation for the image
 *  pyramid; optionally waits for the current operation to complete or time out.
 *
 * \param[in]  pyramid  A pointer to the image.
 * \param[in]  millisecondWait
 *                      Time in milliseconds to wait for the current operation
 *                      to complete before getting status.
 *                      \ref NVMEDIA_IMAGE_TIMEOUT_INFINITE means wait
 *                      indefinitely.
 * \param[out] status   A pointer to the status of the operation.
 * \retval  NVMEDIA_STATUS_OK indates that the operation is successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a pyramid or
 *           @a status is NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates another error.
 */
NvMediaStatus
NvMediaImagePyramidGetStatus(
    NvMediaImagePyramid *pyramid,
    uint32_t millisecondWait,
    NvMediaTaskStatus *status
);

/**
 * \brief Gets a pointer to the image for a level.
 * \param[in] pyramid A handle to the image pyramid.
 * \param[in] level The level for which to obtain an image pointer.
 * \return A pointer to the image if successful, or NULL otherwise.
 */
NvMediaImage *
NvMediaImagePyramidGetImageForLevel(
    NvMediaImagePyramid * pyramid,
    uint32_t level
);

/**
 * \brief Returns the number of levels in a pyramid.
 * \param[in] pyramid A handle to the image pyramid.
 * \return The number of levels in the pyramid if successful, or 0 otherwise.
 */
uint32_t
NvMediaImagePyramidGetNumLevels(
    NvMediaImagePyramid * pyramid
);

/**
 * \brief Returns the scale factor of a pyramid.
 * \param[in] pyramid A handle to the image pyramid.
 * \return The scale factor if successful, or 0 otherwise.
 */
float
NvMediaImagePyramidGetScale(
    NvMediaImagePyramid * pyramid
);

/*
 * \defgroup history_nvmedia_image_pyramid History
 * Provides change history for the NvMedia ImagePyramid API.
 *
 * \section history_nvmedia_image_pyramid Version History
 *
 * <b> Version 1.0 </b> December 4, 2017
 * - Initial Release.
 *
 * <b> Version 1.1 </b> February 6, 2019
 * - Added required header includes nvmedia_core.h and nvmedia_surface.h
 *
 */

/** @} */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* _NVMEDIA_IMAGE_PYRAMID_H */
