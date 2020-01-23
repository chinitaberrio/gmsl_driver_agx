/*
 * Copyright (c) 2018-2019, NVIDIA CORPORATION. All rights reserved. All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */


/**
 * \file
 * \brief <b> NVIDIA Media Interface: Arrays </b>
 *
 * @b Description: This file contains the API to access one-dimensional arrays
 *    managed by NvMedia for use in multimedia applications.
 */

#ifndef NVMEDIA_ARRAY_H
#define NVMEDIA_ARRAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"

/**
 * \defgroup array_api NvMedia Array
 *
 * The NvMedia Array API encompasses all NvMedia
 * functions that create, destroy, access and update arrays
 * used in media processing and computer vision applications.
 *
 * @ingroup nvmedia_top
 * @{
 */

/** \brief Major version number. */
#define NVMEDIA_ARRAY_VERSION_MAJOR   2
/** \brief Minor version number. */
#define NVMEDIA_ARRAY_VERSION_MINOR   1

/**
 * \hideinitializer
 * \brief Infinite time-out for \ref NvMediaArrayGetStatus.
 */
#define NVMEDIA_ARRAY_TIMEOUT_INFINITE  0xFFFFFFFFu

/** \brief Defines the different types of arrays.
 */
typedef enum {
    /*! Specifies an undefined array type. */
    NVMEDIA_ARRAY_TYPE_UNDEFINED = 0,
    /*! Specifies a signed 8-bit array.*/
    NVMEDIA_ARRAY_TYPE_INT8 = 1,
    /*! Specifies an unsigned 8-bit array.*/
    NVMEDIA_ARRAY_TYPE_UINT8 = 2,
    /*! Specifies a signed 16-bit array.*/
    NVMEDIA_ARRAY_TYPE_INT16 = 3,
    /*! Specifies an unsigned 16-bit array.*/
    NVMEDIA_ARRAY_TYPE_UINT16 = 4,
    /*! Specifies a float 16-bit array.*/
    NVMEDIA_ARRAY_TYPE_FLOAT16 = 5,
    /*! Specifies a unsigned 32-bit array.*/
    NVMEDIA_ARRAY_TYPE_UINT32 = 6,
    /*! Specifies a signed 32-bit array.*/
    NVMEDIA_ARRAY_TYPE_INT32 = 7,
    /*! Specifies a float 32-bit array.*/
    NVMEDIA_ARRAY_TYPE_FLOAT32 = 8,
    /*! Number of types - If more types are needed, add above this pointer and
     update this constant.*/
    NVMEDIA_ARRAY_NUM_TYPES = 8,
} NvMediaArrayType;

/**
 * \brief Holds a descriptor for an array.
 * Note: Create and destroy an array with the corresponding
 *       NvMediaArrayCreate() and NvMediaArrayDestroy() functions.
 */
typedef struct NvMediaArray NvMediaArray;

/**
 * \brief Defines NvMedia array allocation attribute types.
 */
typedef enum {
    /**! CPU access to surface flags (default: uncached). */
    NVM_ARRAY_ATTR_CPU_ACCESS,
} NvMediaArrayAllocAttrType;

/** \brief \ref NVM_SURF_ATTR_CPU_ACCESS flags
  */
/** Uncached (mapped) access type flag. */
#define NVM_ARRAY_ATTR_CPU_ACCESS_UNCACHED 0x00000000u
/** NVM_SURF_ATTR_CPU_ACCESS flag: Cached (mapped) access type flag */
#define NVM_ARRAY_ATTR_CPU_ACCESS_CACHED 0x00000001u

/**
 * \brief Holds array allocation attributes.
 */
typedef struct {
    /** \brief Array allocation attribute type. */
    NvMediaArrayAllocAttrType type;
    /** \brief Array allocation attribute value. */
    uint32_t value;
} NvMediaArrayAllocAttr;

/**
 * \brief Defines array-lock access types.
 * \ingroup lock_unlock
 */
typedef enum {
    /** Specifies read access. */
    NVMEDIA_ARRAY_ACCESS_READ       = 1,
    /** Specifies write access. */
    NVMEDIA_ARRAY_ACCESS_WRITE      = 2,
    /** Specifies read/write access. */
    NVMEDIA_ARRAY_ACCESS_READ_WRITE = 3,
} NvMediaArrayLockAccess;

/**
 * \brief Returns version information for the NvMediaArray library.
 * \param[in] version A pointer to an \ref NvMediaVersion structure
 *                    filled by the function.
 * \return A status code; \ref NVMEDIA_STATUS_OK if the call is successful, or
 *  \ref NVMEDIA_STATUS_BAD_PARAMETER if the pointer is invalid.
 */
NvMediaStatus
NvMediaArrayGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Creates an NvMedia Array.
 * \param[in] device        Handle to the NvMedia device obtained by calling
 *                           NvMediaDeviceCreate().
 * \param[in] type          Type of array to be created.
 * \param[in] stride        Stride in bytes of each element.
 * \param[in] numElements   Number of elements in the array.
 * \param[in] attrs         An array of \ref NvMediaArrayAllocAttr allocation
 *                           attributes to be applied to the created array.
 * \param[in] numAttrs      Number of allocation attributes in @a attrs.
 * \return  A handle to the array if the call is successful, or NULL otherwise.
 */
NvMediaArray *
NvMediaArrayCreate(
    NvMediaDevice *device,
    NvMediaArrayType type,
    uint32_t stride,
    uint32_t numElements,
    const NvMediaArrayAllocAttr *attrs,
    uint32_t numAttrs
);

/**
 * \brief Destroys an array created by NvMediaArrayCreate().
 * \param[in] handle The handle to the array to be destroyed.
 * \retval  NVMEDIA_STATUS_OK indicates that the call is successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a handle is NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates any other error.
 */
NvMediaStatus
NvMediaArrayDestroy(
    NvMediaArray* handle
);

/**
 * \brief Gets the size of a specified type of array element.
 * \param[in]  type         Type of element.
 * \param[out] elementSize  A pointer to the size of an element in bytes.
 * \retval  NVMEDIA_STATUS_OK indicates that the call is successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a type is invalid or
 *  @a elementSize is NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates any other error.
 */
NvMediaStatus
NvMediaArrayGetElemSizeForType(
    NvMediaArrayType type,
    uint32_t *elementSize
);

/**
 * \brief Gets the size of an array.
 *
 * An array's size is the number of populated elements in the array. If the
 * array has not been written to, the function returns a size of 0.
 *
 * \param[in]  handle           The handle to the array.
 * \param[out] numElementsPtr   A pointer to the number of elements.
 * \retval  NVMEDIA_STATUS_OK indicates that the call is successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a handle or
 *  @c numElementsPtr is NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that the array is not locked.
 */
NvMediaStatus
NvMediaArrayGetSize(
    NvMediaArray* handle,
    uint32_t *numElementsPtr
);

/**
 * \brief Sets the size of an array (the number of populated elements in the
 * array).
 *
 * Call this function before writing to the array. The specified size
 * must be less than or equal to the number of elements with which the array
 * was created.
 *
 * \sa NvMediaArrayCreate().
 * \param[in] handle        The handle to the array.
 * \param[in] numElements   Size of the array.
 * \retval  NVMEDIA_STATUS_OK indicates that the call is successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a handle is NULL.
 * \retval  NVMEDIA_STATUS_INVALID_SIZE indicates that numElements is greater
 *  than the array's initial size.
 * \retval  NVMEDIA_STATUS_ERROR indicates that the array is not locked.
 */
NvMediaStatus
NvMediaArraySetSize(
    NvMediaArray* handle,
    uint32_t numElements
);

/**
 * \brief  Gets the array properties with which array was created.
 * \param[in]  handle       The handle to the array.
 * \param[out] elementType  Type of elements in the array.
 * \param[out] capacity     A pointer to the array's capacity (the number of
 *                           elements).
 * \param[out] stride       A pointer to the array's stride (the offset in
 *                           bytes between one elelment and the next).
 * \retval  NVMEDIA_STATUS_OK indicates that the call is successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more arguments
 *  are NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates any other error.
 */
NvMediaStatus
NvMediaArrayGetProperties(
    NvMediaArray* handle,
    NvMediaArrayType* elementType,
    uint32_t* capacity,
    uint32_t* stride
);

/**
 * \brief Gets the status of the current or most recent operation on the array;
 *        optionally waits for the operation to complete or time out.
 *
 * \param[in]  handle   The handle to the array.
 * \param[in]  millisecondWait
 *                      Time in milliseconds to wait for the current operation
 *                      to complete. \ref NVMEDIA_ARRAY_TIMEOUT_INFINITE
 *                      means wait indefinitely.
 * \param[out] status   Status of the operation.
 * \retval  NVMEDIA_STATUS_OK indicates that the call is successful.
 * \retval  NVMEDIA_STATUS_TIMED_OUT indicates that the operation did not
 *  complete within the specified wait time.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that a parameter has an
 *  invalid value.
 * \retval  NVMEDIA_STATUS_ERROR indicates any other error.
 */
NvMediaStatus
NvMediaArrayGetStatus(
    NvMediaArray *handle,
    uint32_t millisecondWait,
    NvMediaTaskStatus *status
);

/**
 * \brief  Locks an array. A client can read or write a locked array
 *  without interference from another thread or process.
 *
 * If the array is being used by an internal engine, this function waits until
 * the operation is completed.
 *
 * \param[in]  handle   Handle to the array to be locked.
 * \param[in]  lockAccessType
 *                      Specifies the access type. The following access types
 *                       are supported, and may be OR'd together:
 *                      - \ref NVMEDIA_ARRAY_ACCESS_READ specifies read access.
 *                      - \ref NVMEDIA_ARRAY_ACCESS_WRITE specifies write
 *                          access.
 *                      - \ref NVMEDIA_ARRAY_ACCESS_READ_WRITE Specifies
 *                          read/write access.
 * \param[out] ptr      A CPU mapped pointer which may be used to read or write
 *                       data.
 * \return  A status code; \ref NVMEDIA_STATUS_OK if the call is successful, or
 *  \ref NVMEDIA_STATUS_ERROR otherwise.
 * \ingroup lock_unlock
 */
NvMediaStatus
NvMediaArrayLock(
    NvMediaArray *handle,
    NvMediaArrayLockAccess lockAccessType,
    void **ptr
);

/**
 * Unlocks an array.
 *
 * Call this function when the client has finished writing to or reading from
 * the array.
 *
 * \param[in] handle Handle to the array to be unlocked.
 * \ingroup lock_unlock
 */
void
NvMediaArrayUnlock(
    NvMediaArray *handle
);

/*
 * \defgroup history_nvmedia_array History
 * Provides change history for the NvMedia Array API.
 *
 * \section history_nvmedia_arrays Version History
 *
 * <b> Version 1.0 </b> December 4, 2017
 * - Initial Release.
 *
 * <b> Version 2.0 </b> June 1, 2018
 * - Remove keypoint type. VPI-specific types are in VPI headers.
 *
 * <b> Version 2.1 </b> July 9, 2018
 * - Add version query API.
 */

/** @} */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif // NVMEDIA_ARRAY_H
