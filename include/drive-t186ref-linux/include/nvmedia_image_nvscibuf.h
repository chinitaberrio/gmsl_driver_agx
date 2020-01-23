/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: Image Processing </b>
 *
 * @b Description: This file contains the \ref NvMediaImage and NvSciBuf
 *                  related APIs.
 */

#ifndef NVMEDIA_IMAGE_NVSCIBUF_H
#define NVMEDIA_IMAGE_NVSCIBUF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvscibuf.h"
#include "nvmedia_core.h"
#include "nvmedia_surface.h"
#include "nvmedia_image.h"

/**
 * @defgroup nvmedia_image_nvscibuf_api Image Handling with Buffer Allocation API
 *
 * The NvMedia Image NvSciBuf API encompasses all NvMediaImage
 * NvSciBuf handling functions.
 *
 * @ingroup nvmedia_top
 * @{
 */

/** \brief  Major version number. */
#define NVMEDIA_IMAGENVSCIBUF_VERSION_MAJOR   (1u)
/** \brief  Minor version number. */
#define NVMEDIA_IMAGENVSCIBUF_VERSION_MINOR   (0u)

/**
 * \brief  Initializes the NvMediaImage NvSciBuf API.
 *
 * You must call this function before you call NvMediaImageFillNvSciBufAttrs()
 * and NvMediaImageCreateFromNvSciBuf().
 *
 * \return  A status code; ::NVMEDIA_STATUS_OK if the API was successfully
 *  initialized, or ::NVMEDIA_STATUS_ERROR otherwise.
 */
NvMediaStatus
NvMediaImageNvSciBufInit(void);

/**
 * \brief  Deinitializes the NvMediaImage NvSciBuf API.
 *
 * You cannot call NvMediaImageFillNvSciBufAttrs() or
 * NvMediaImageCreateFromNvSciBuf() after you call this function.
 */
void
NvMediaImageNvSciBufDeinit(void);

/**
 * \brief  Fills the NvSciBuf (see nvscibuf.h) attributes which can be used to
 * allocate an image.
 *
 * Assumes that @a attr_h is a valid \ref NvSciBufAttrList created by the
 * application by calling NvSciBufAttrListCreate(). This function maps the
 * information in @a type, @a attrs, and @a flags to NvSciBuf attributes and
 * fills them in @a attr_h.
 *
 * The following NvSciBuf input attributes are set by NvMedia, and so may not
 * be set by the application:
 *  - \ref NvSciBufGeneralAttrKey_Types
 *  - \ref NvSciBufGeneralAttrKey_NeedCpuAccess
 *  - \ref NvSciBufGeneralAttrKey_EnableCpuCache
 *  - \ref NvSciBufImageAttrKey_Layout
 *  - \ref NvSciBufImageAttrKey_PlaneCount
 *  - \ref NvSciBufImageAttrKey_TopPadding
 *  - \ref NvSciBufImageAttrKey_BottomPadding
 *  - \ref NvSciBufImageAttrKey_LeftPadding
 *  - \ref NvSciBufImageAttrKey_RightPadding
 *  - \ref NvSciBufImageAttrKey_PlaneColorFormat
 *  - \ref NvSciBufImageAttrKey_PlaneColorStd
 *  - \ref NvSciBufImageAttrKey_PlaneBaseAddrAlign
 *  - \ref NvSciBufImageAttrKey_PlaneWidth
 *  - \ref NvSciBufImageAttrKey_PlaneHeight
 *  - \ref NvSciBufImageAttrKey_PlaneScanType
 *  - \ref NvSciBufImageAttrKey_VprFlag
 *
 * The following image attribute is not set by NvMedia, and so must be set
 * by the application:
 *  - \ref NvSciBufGeneralAttrKey_RequiredPerm
 *
 * After calling this function, the application can use @a attr_h in %NvSciBuf
 * attribute list reconcile and object allocation functions to allocate an
 * \ref NvSciBufObj. Then it can call NvMediaImageCreateFromNvSciBuf()
 * to create an \ref NvMediaImage from the %NvSciBufObj.
 *
 * \param[in] device    A handle for the \ref NvMediaDevice.
 * \param[in] type      A surface type obtained by calling
 *                       NvMediaSurfaceFormatGetType().
 * \param[in] attrs     A pointer to an array of surface allocation attributes
 *                       for surface creation.
 * \param[in] numAttrs  Number of attributes in @a attrs.
 * \param[in] flags     Flags for module hint (for future use).
 * \param[in,out] attr_h A structure where the %NvSciBuf attributes for the
 *                       requested %NvMediaImage are put.
 * \return  A status code; ::NVMEDIA_STATUS_OK if the function call is
 *  successful, or ::NVMEDIA_STATUS_BAD_PARAMETER if @a type is invalid or
 *  @a attrs contains invalid attributes.
 */
NvMediaStatus
NvMediaImageFillNvSciBufAttrs(
    NvMediaDevice *device,
    NvMediaSurfaceType type,
    const NvMediaSurfAllocAttr *attrs,
    uint32_t numAttrs,
    uint64_t flags,
    NvSciBufAttrList attr_h
);


/**
 * \brief Creates an \ref NvMediaImage from an \ref NvSciBufObj.
 *
 * You must allocate the @a nvscibufObj before you call this function, using
 * the \ref NvSciBufAttrList filled by NvMediaImageFillNvSciBufAttrs().
 *
 * When the application is done using @a nvmImage,
 * it must call NvMediaImageDestroy() with @a nvmImage.
 *
 * \param[in] device        A handle for the \ref NvMediaDevice.
 * \param[in] nvscibufObj   An %NvSciBufObj for which an %NvMediaImage is to be
 *                           imported.
 * \param[in,out] nvmImage  A pointer to a location in which a pointer to an
 *                           imported %NvMediaImage in stored.
 * \retval  NVMEDIA_STATUS_OK indicates that @a nvmImage was successfully
 *           created from @a nvscibufObj.
 * \retval  NVMEDIA_STATUS_OUT_OF_MEMORY indicates that memory allocation
 *           failed.
 * \retval  NVMEDIA_STATUS_ERROR indicates that another error occurred, such as
 *           failure to create @a nvmImage from @a nvscibufObj.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one of the pointer
 *           parameters is NULL, or %nvscibufObj was not allocated using the
 *           attributes filled by %NvMediaImageFillNvSciBufAttrs().
 */
NvMediaStatus
NvMediaImageCreateFromNvSciBuf(
    NvMediaDevice *device,
    NvSciBufObj nvscibufObj,
    NvMediaImage **nvmImage
);

/**
 * \brief  Returns version information for the %NvMediaImage NvSciBuf API.
 *
 * \param[out] version  A pointer to a structure in which the function may
 *                       store version information.
 * \return  A status code; ::NVMEDIA_STATUS_OK if the function call was
 *  successful, or ::NVMEDIA_STATUS_BAD_PARAMETER if @a version was invalid.
 */
NvMediaStatus
NvMediaImageNvSciBufGetVersion(
    NvMediaVersion *version
);

/*
 * \defgroup history_nvmedia_image_nvscibuf History
 * Provides change history for the NvMedia NvSciBuf API.
 *
 * \section history_nvmedia_image_nvscibuf Version History
 *
 * <b> Version 1.0 </b> March 08, 2019
 * - Initial release
 */
/** @} <!-- Ends nvmedia_image_nvscibuf_api NvMedia Image Handling NvSciBuf --> */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_IMAGE_NVSCIBUF_H */
