/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: Image Pyramid Processing with Buffer Allocation</b>
 *
 * @b Description: This file contains the NvMediaImagePyramid and NvSciBuf related APIs.
 *
 */

#ifndef NVMEDIA_IMAGE_PYRAMID_NVSCIBUF_H
#define NVMEDIA_IMAGE_PYRAMID_NVSCIBUF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvscibuf.h"
#include "nvmedia_core.h"
#include "nvmedia_surface.h"
#include "nvmedia_image.h"
#include "nvmedia_image_pyramid.h"

/**
 * \defgroup nvmedia_image_pyramid_buffer Image Pyramid with Buffer Allocation API
 *
 * The Image Pyramid with Buffer Allocation API encompasses all NvMedia-related
 * functionality for handling pyramids of \ref NvMediaImage objects with buffer
 * allocation.
 *
 * @ingroup nvmedia_image_pyramid_top
 * @{
 */
/** \brief Major version number. */
#define NVMEDIA_IMAGE_PYRAMID_NVSCIBUF_VERSION_MAJOR   (1u)
/** \brief Minor version number. */
#define NVMEDIA_IMAGE_PYRAMID_NVSCIBUF_VERSION_MINOR   (1u)

/**
 * \brief Initializes the \ref NvMediaImagePyramid NvSciBuf APIs.
 *
 * This function must be called before calling
 * NvMediaImagePyramidFillNvSciBufAttrs() and
 * NvMediaImagePyramidCreateFromNvSciBuf().
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the initialization is successful.
 * - \ref NVMEDIA_STATUS_ERROR if there is an error in the initialization.
 */
NvMediaStatus
NvMediaImagePyramidNvSciBufInit(void);

/**
 * \brief De-initializes the \ref NvMediaImagePyramid NvSciBuf APIs.
 *
 * NvMediaImagePyramidFillNvSciBufAttrs() and
 * NvMediaImagePyramidCreateFromNvSciBuf() cannot be called after calling
 * this function.
 */
void
NvMediaImagePyramidNvSciBufDeinit(void);

/**
 * \brief Fills the NvSciBuf attributes to use to allocate an ImagePyramid.
 *
 * This function assumes that @a attr_h is a valid NvSciBufAttrList created
 * by the application using NvSciBufAttrListCreate(). This function maps the
 * information in @a type, @a attrs and @a flags to NvSciBuf attributes and
 * fills them in @a attr_h. The following list of NvSciBuf
 * input attributes are set by NvMedia. The application must not set these values.
 *  - \ref NvSciBufGeneralAttrKey_Types
 *  - \ref NvSciBufGeneralAttrKey_NeedCpuAccess
 *  - \ref NvSciBufGeneralAttrKey_EnableCpuCache
 *  - \ref NvSciBufImageAttrKey_Layout
 *  - \ref NvSciBufImageAttrKey_PlaneCount
 *  - \ref NvSciBufImageAttrKey_TopPadding
 *  - \ref NvSciBufImageAttrKey_BottomPadding
 *  -  \ref NvSciBufImageAttrKey_LeftPadding
 *  - \ref NvSciBufImageAttrKey_RightPadding
 *  - \ref NvSciBufImageAttrKey_PlaneColorFormat
 *  - \ref NvSciBufImageAttrKey_PlaneColorStd
 *  - \ref NvSciBufImageAttrKey_PlaneBaseAddrAlign
 *  - \ref NvSciBufImageAttrKey_PlaneWidth
 *  - \ref NvSciBufImageAttrKey_PlaneHeight
 *  - \ref NvSciBufImageAttrKey_PlaneScanType
 *  - \ref NvSciBufImageAttrKey_VprFlag
 *  - \ref NvSciBufPyramidAttrKey_NumLevels
 *  - \ref NvSciBufPyramidAttrKey_Scale
 *  - \ref NvSciBufPyramidAttrKey_LevelOffset
 *  - \ref NvSciBufPyramidAttrKey_LevelSize
 *  - \ref NvSciBufPyramidAttrKey_Alignment
 *
 * The following image attribute is not set by NvMedia and they must be set
 * by the application:
 *  - \ref NvSciBufGeneralAttrKey_RequiredPerm
 *
 * After calling NvMediaImagePyramidFillNvSciBufAttrs(), the application can use
 * @a attr_h in the NvSciBuf attribute list reconcile and object allocation APIs
 * to allocate an NvSciBufObj.
 * Then the application can call NvMediaImagePyramidCreateFromNvSciBuf() to
 * create an \ref NvMediaImagePyramid from the NvSciBufObj.
 *
 * \param[in] device A pointer to the \ref NvMediaDevice.
 * \param[in] type NvMediaSurfaceType type obtained from NvMediaSurfaceFormatGetType().
 * \param[in] attrs An array of surface allocation attributes for surface creation.
 * \param[in] numAttrs Number of attributes in the array.
 * \param[in] numLevels Number of levels for pyramid creation.
 * \param[in] scale Scaling factor for pyramid creation.
 * \param[in] flags Flags for module hint (used in future).
 * \param[in,out] attr_h NvSciBufAttrlist where the NvSciBuf attributes for the
 *  requested NvMediaImagePyramid are filled.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a type is invalid or @a attrs has
 *        invalid attributes set.
 */
NvMediaStatus
NvMediaImagePyramidFillNvSciBufAttrs(
    const NvMediaDevice *device,
    NvMediaSurfaceType type,
    const NvMediaSurfAllocAttr *attrs,
    uint32_t numLevels,
    float_t scale,
    uint32_t numAttrs,
    uint64_t flags,
    NvSciBufAttrList attr_h
);


/**
 * \brief Creates an \ref NvMediaImagePyramid from an NvSciBufObj.
 *
 * NvMediaImagePyramidFillNvSciBufAttrs() initializes @a nvscibufObjInstance using the
 * attributes populated by NvMediaImagePyramidFillNvSciBufAttrs().
 * When the application has finished using @a nvmImagePyramid, it is expected to call
 * NvMediaImagePyramidDestroy() with the @a nvmImagePyramid.
 *
 * \param[in] device A pointer to the \ref NvMediaDevice.
 * \param[in] nvscibufObjInstance NvSciBufObj for which an NvMediaImagePyramid needs to
 *            be imported.
 * \param[in,out] nvmImagePyramid A pointer to the imported NvMediaImagePyramid.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if @a nvmImagePyramid is created successfully from
          @a nvscibufObjInstance.
 * - \ref NVMEDIA_STATUS_OUT_OF_MEMORY if memory allocation fails.
 * - \ref NVMEDIA_STATUS_ERROR if @a nvmImagePyramid creation from @a nvscibufObj
          fails or any other reason not listed in other status.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if any of the parameters are NULL,or if
          @a nvscibufObj was not allocated by using the attributes filled by
          NvMediaImagePyramidFillNvSciBufAttrs().
 */
NvMediaStatus
NvMediaImagePyramidCreateFromNvSciBuf(
    NvMediaDevice *device,
    NvSciBufObj nvscibufObjInstance,
    NvMediaImagePyramid **nvmImagePyramid
);

/**
 * \brief Returns the version information for the \ref NvMediaImagePyramid
 * NvSciBuf APIs.
 *
 * \param[in] version A pointer to a \ref NvMediaVersion structure
 *                    of the client.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if the pointer is invalid.
 */
NvMediaStatus
NvMediaImagePyramidNvSciBufGetVersion(
    NvMediaVersion *version
);

/*
 * \defgroup history_nvmedia_image_pyramid_nvscibuf History
 * Provides change history for the NvMedia NvSciBuf API.
 *
 * \section history_nvmedia_image_pyramid_nvscibuf Version History
 *
 * <b> Version 1.0 </b> July 16, 2019
 * - Initial release
 *
 * <b> Version 1.1 </b> April 22, 2020
 * - Fixed minor MISRA violations
 */

/** @} */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_IMAGE_PYRAMID_NVSCIBUF_H */
