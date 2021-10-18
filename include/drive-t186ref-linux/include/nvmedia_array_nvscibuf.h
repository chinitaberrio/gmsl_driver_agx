/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NvMedia Interfaces for Array for NvSciBuf </b>
 *
 * @b Description: This file contains APIs for allocation of NvMediaArray using NvSciBuf.
 *
 */

#ifndef NVMEDIA_ARRAY_NVSCIBUF_H
#define NVMEDIA_ARRAY_NVSCIBUF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvscibuf.h"
#include "nvmedia_core.h"
#include "nvmedia_array.h"

/**
 * \defgroup array_nvscibuf_api NvMedia Array for NvSciBuf
 *
 * The NvMedia Array NvSciBuf API encompasses all NvMediaArray
 * NvSciBuf handling functions.
 *
 * @ingroup nvmedia_array_top
 * @{
 */
/** \brief Major version number. */
#define NVMEDIA_ARRAY_NVSCIBUF_VERSION_MAJOR   (1u)
/** \brief Minor version number. */
#define NVMEDIA_ARRAY_NVSCIBUF_VERSION_MINOR   (1u)

/**
 * \brief Initializes the NvMediaArray NvSciBuf APIs.
 *
 * This function must be called before calling
 * NvMediaArrayFillNvSciBufAttrs() and NvMediaArrayCreateFromNvSciBuf().
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the initialization is successful.
 * - \ref NVMEDIA_STATUS_ERROR if there is an error in the initialization.
 */
NvMediaStatus
NvMediaArrayNvSciBufInit(void);

/**
 * \brief De-initializes the NvMediaArray NvSciBuf APIs.
 *
 * NvMediaArrayFillNvSciBufAttrs() and NvMediaArrayCreateFromNvSciBuf()
 *  cannot be called after calling this function.
 */
void
NvMediaArrayNvSciBufDeinit(void);

/**
 * \brief Fills the NvSciBuf attributes used to allocate an array.
 *
 * This function assumes that @a attr_h is a valid \ref NvSciBufAttrList
 * created by the application.
 *
 * This API maps the information in @a type and @a attrs to
 * NvSciBuf attributes and fills them into NvSciBufAttrList
 * referenced by @a attr_h.
 *
 * The following list of NvSciBuf input attributes are set by NvMedia.
 * The application must not set these values.
 *  - \ref NvSciBufGeneralAttrKey_Types
 *  - \ref NvSciBufGeneralAttrKey_NeedCpuAccess
 *  - \ref NvSciBufGeneralAttrKey_EnableCpuCache
 *  - \ref NvSciBufArrayAttrKey_DataType
 *  - \ref NvSciBufArrayAttrKey_Stride
 *  - \ref NvSciBufArrayAttrKey_Capacity
 *
 * The following image attribute is not set by NvMedia and it must be set
 * by the application:
 *  - \ref NvSciBufGeneralAttrKey_RequiredPerm
 *
 * After calling this function, the application can call NvSciBufAllocate() with
 * @a attr_h as input and get an \ref NvSciBufObj as output.
 * Then it can call NvMediaArrayCreateFromNvSciBuf()
 * to create an \ref NvMediaArray from the %NvSciBufObj.
 *
 * \param[in] device      A pointer to NvMediaDevice.
 * \param[in] type        The datatype for elements in the array.
 * \param[in] stride      Number of strides.
 * \param[in] numElements Number of elements.
 * \param[in] attrs       A pointer to an array of array alloc attributes
 *                        for array creation.
 * \param[in] numAttrs    The number of attributes in the array.
 * \param[in, out] attr_h A handle to NvSciBufAttrlist to hold the NvSciBuf
 *                        attributes for the requested NvMediaArray.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if any argument is NULL or invalid.
 */

NvMediaStatus
NvMediaArrayFillNvSciBufAttrs(
    NvMediaDevice *device,
    NvMediaArrayType type,
    uint32_t stride,
    uint32_t numElements,
    const NvMediaArrayAllocAttr *attrs,
    uint32_t numAttrs,
    NvSciBufAttrList attr_h
);

/**
 * \brief Creates NvMediaArray from an NvSciBuf handle.
 *
 * This API assumes that @a nvSciBufObjInstance is a pointer to a valid NvSciBufObj.
 *
 * You must allocate the @a nvSciBufObjInstance before you call this function, using
 * the \ref NvSciBufAttrList filled by NvMediaArrayGetNvSciBufAttrs().
 *
 * When the application is done using @a nvmArray,
 * it must call NvMediaArrayDestroy() with @a nvmArray.
 *
 * \param[in] device         A pointer to the NvMediaDevice.
 * \param[in] nvSciBufObjInstance    An %NvSciBufObj for which an %NvMediaArray is to be
 *                           imported.
 * \param[in, out] nvmArray A pointer to a location in which a pointer to an
 *                           imported %NvMediaArray in stored.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK indicates that @a nvmArray was successfully
 *           created from @a nvSciBufObjInstance.
 * - \ref NVMEDIA_STATUS_ERROR indicates that another error occurred, such as
 *           failure to create @a nvmArray from @a nvSciBufObjInstance.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER indicates that one of the pointer
 *           parameters is NULL, or @a nvSciBufObjInstance was not allocated using the
 *           attributes filled by %NvMediaArrayGetNvSciBufAttrs().
 */

NvMediaStatus
NvMediaArrayCreateFromNvSciBuf(
    NvMediaDevice *device,
    NvSciBufObj nvSciBufObjInstance,
    NvMediaArray **nvmArray
);

/**
 * \brief  Returns version information for the %NvMediaArray NvSciBuf API.
 *
 * \param[out] version  A pointer to a \ref NvMediaVersion in which
 *                      the function may store version information.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function call is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a version is invalid.
 */
NvMediaStatus
NvMediaArrayNvSciBufGetVersion(
    NvMediaVersion *version
);

/*
 * \defgroup history_nvmedia_array_nvscibuf History
 * Provides change history for the NvMedia NvSciBuf API.
 *
 * \section history_nvmedia_array_nvscibuf Version History
 *
 * <b> Version 1.0 </b> June 18, 2019
 * - Initial release
 *
 * <b> Version 1.1 </b> April 22, 2020
 * - Renamed parameter in NvMediaArrayCreateFromNvSciBuf()
 *
 */

/**@} <!-- Ends nvmedia_array_nvscibuf --> */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_ARRAY_NVSCIBUF_H */