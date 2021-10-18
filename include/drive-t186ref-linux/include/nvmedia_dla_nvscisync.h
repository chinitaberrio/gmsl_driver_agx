/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved. All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 *
 * \brief <b> NVIDIA Media Interface: DLA NvSciSync </b>
 *
 * @b Description: This file contains the NvMediaDla and NvSciSync related APIs.
 *
 */

#ifndef NVMEDIA_DLA_NVSCISYNC_H
#define NVMEDIA_DLA_NVSCISYNC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"
#include "nvscisync.h"
#include "nvmedia_dla.h"

/**
 * @defgroup nvmedia_dla_nvscisync_api Deep Learning Accelerator Synchronization
 *
 * The NvMedia DLA NvSciSync API encompasses all NvMediaDla
 * NvSciSync handling functions.
 *
 * @ingroup nvmedia_dla_top
 * @{
 */

/** \brief Major version number. */
#define NVMEDIA_DLA_NVSCISYNC_VERSION_MAJOR   1
/** \brief Minor version number. */
#define NVMEDIA_DLA_NVSCISYNC_VERSION_MINOR   2

/**
 * NvMediaDlaInsertPreNvSciSyncFence API can be called at most
 * NVMEDIA_DLA_MAX_PRENVSCISYNCFENCES times before each Dla submit call.
*/
#define NVMEDIA_DLA_MAX_PRENVSCISYNCFENCES  (8U)

/**
 * \brief Returns the version information for the NvMedia DLA NvSciSync library.
 *
 * \param[in, out] version A pointer to an NvMediaVersion structure
 *                 filled by the DLA NvSciSync library.
 *                 @inputrange A non-null pointer to an %NvMediaVersion.
 *                 @outputrange A non-null pointer to an %NvMediaVersion if
 *                 successful, otherwise the value pointed to by @a version
 *                 remains unchanged.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a version is NULL.
 */
NvMediaStatus
NvMediaDlaNvSciSyncGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Fills the NvMediaDla specific NvSciSync attributes.
 *
 * This API assumes that @a attrlist is a valid \ref NvSciSyncAttrList.
 *
 * This function sets the public attribute:
 * - \ref NvSciSyncAttrKey_RequiredPerm
 *
 * The application must not set this attribute.
 *
 * \param[in] dla           An NvMedia DLA device handle.
 *                          @inputrange A non-null pointer to an \ref NvMediaDla
 *                          created with NvMediaDlaCreate().
 * \param[in, out] attrlist A pointer to an \ref NvSciSyncAttrList structure
 *                          where NvMedia places NvSciSync attributes.
 *                          @inputrange A non-null pointer created by
 *                          NvSciSyncAttrListCreate().
 *                          @outputrange A non-null pointer to an NvSciSyncAttrList.
 * \param[in] clienttype    Indicates whether the @a attrlist is requested for
 *                          an %NvMediaDla signaler or an %NvMediaDla waiter.
 *                          @inputrange Any enum value defined by
 *                          \ref NvMediaNvSciSyncClientType.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the call is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a attrlist is NULL,
 *         or any of the above listed public attributes are already set,
 *         or if client type is invalid.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaDlaFillNvSciSyncAttrList(
    const NvMediaDla                *dla,
    NvSciSyncAttrList          attrlist,
    NvMediaNvSciSyncClientType clienttype
);

/**
 * \brief Registers an \ref NvSciSyncObj with \ref NvMediaDla.
 *
 * Every NvSciSyncObj(even duplicate objects) used by %NvMediaDla
 * must be registered by a call to this function before it is used.
 * Only the exact same registered \ref NvSciSyncObj can be passed to the run time APIs.
 *
 * For a given NvMediaDla handle, one NvSciSyncObj can be registered
 * as one \ref NvMediaNvSciSyncObjType only.
 * For each NvMediaNvSciSyncObjType, a maximum of 16 NvSciSyncObj objects can
 * be registered.
 *
 * \param[in] dla         An NvMedia DLA device handle.
 *                        @inputrange A non-null pointer to an %NvMediaDla
 *                        created with NvMediaDlaCreate().
 * \param[in] syncobjtype Determines how @a nvscisync is used by @a dla.
 *                        @inputrange Any enum value defined by
 *                        \ref NvMediaNvSciSyncObjType.
 * \param[in] nvscisync   The %NvSciSyncObj to be registered
 *                        with @a dla.
 *                        @inputrange A non-null pointer created by
 *                        NvSciSyncObjAlloc().
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a dla is NULL or
 *        @a syncobjtype is not a valid NvMediaNvSciSyncObjType.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED if @a nvscisync is not a
 *        compatible %NvSciSyncObj which %NvMediaDla can support.
 * - \ref NVMEDIA_STATUS_ERROR if the maximum number of NvSciScynObj objects
 *        are already registered for the given @a syncobjtype, OR
 *        if @a nvscisync is already registered with the same @a dla
 *        handle for a different @a syncobjtype.
 */
NvMediaStatus
NvMediaDlaRegisterNvSciSyncObj(
    NvMediaDla                *dla,
    NvMediaNvSciSyncObjType    syncobjtype,
    NvSciSyncObj               nvscisync
);

/**
 * \brief Unregisters an \ref NvSciSyncObj with NvMediaDla.
 *
 * During teardown, every %NvSciSyncObj registered with NvMediaDla must be
 * unregistered before calling NvMediaDlaDestroy().
 *
 * Before the application calls this function, it must ensure that the
 * application is in teardown mode, and any NvMediaDla operation using
 * this @a nvscisync has completed.
 * If the function is called while @a nvscisync is still in use by any
 * NvMediaDla operations, the behavior is undefined.
 *
 * \param[in] dla         An NvMedia DLA device handle.
 *                        @inputrange A non-null pointer to an \ref NvMediaDla
 *                        created with NvMediaDlaCreate().
 * \param[in] nvscisync   An \ref NvSciSyncObj to be unregistered with @a dla.
 *                        @inputrange A non-null pointer created by
 *                        NvSciSyncObjAlloc().
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a dla is NULL, or
 *        @a nvscisync is not registered with @a dla.
 * - \ref NVMEDIA_STATUS_ERROR if @a dla is destroyed before this function is
 *        called.
 */
NvMediaStatus
NvMediaDlaUnregisterNvSciSyncObj(
    NvMediaDla                *dla,
    NvSciSyncObj               nvscisync
);

#if (NV_IS_SAFETY == 0)
/**
 * \brief Sets the \ref NvSciSyncObj to be used for a Start of Frame (SOF)
 * \ref NvSciSyncFence.
 *
 * \note This API is currently not supported.
 *
 * To use NvMediaDlaGetSOFNvSciSyncFence(), the application must call
 * this function before the first DLA submit API.
 * NvMedia DlA currently accepts only one SOF \ref NvSciSyncObj.
 *
 * \param[in] dla          An NvMedia DLA device handle.
 *                         @inputrange A non-null pointer to an \ref NvMediaDla
 *                         created with NvMediaDlaCreate().
 * \param[in] nvscisyncSOF A registered NvSciSyncObj to be
 *                         associated with SOF %NvSciSyncFence.
 *                         @inputrange A non-null pointer created by
 *                         NvSciSyncObjAlloc().
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a dla is NULL, or if @a nvscisyncSOF
 *        is not registered with @a dla as either type
 *        \ref NVMEDIA_SOFSYNCOBJ or type \ref NVMEDIA_SOF_PRESYNCOBJ.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaDlaSetNvSciSyncObjforSOF(
    NvMediaDla                *dla,
    NvSciSyncObj               nvscisyncSOF
);
#endif // (NV_IS_SAFETY == 0)

/**
 * \brief Sets an \ref NvSciSyncObj to be used for a End of Frame (EOF)
 * \ref NvSciSyncFence.
 *
 * To use NvMediaDlaGetEOFNvSciSyncFence(), the application must call
 * this function before the calling the first DLA submit API.
 * NvMedia DLA currently accepts only one EOF %NvSciSyncObj.
 *
 * \param[in] dla          An NvMedia DLA device handle.
 *                         @inputrange A non-null pointer to an \ref NvMediaDla
 *                         created with NvMediaDlaCreate().
 * \param[in] nvscisyncEOF A registered NvSciSyncObj which is to be
 *                         associated with EOF %NvSciSyncFence.
 *                         @inputrange A non-null pointer created by
 *                         NvSciSyncObjAlloc().
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a dla is NULL, OR if @a nvscisyncEOF
 *        is not registered with @a dla as either type \ref NVMEDIA_EOFSYNCOBJ
 *        or type \ref NVMEDIA_EOF_PRESYNCOBJ.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaDlaSetNvSciSyncObjforEOF(
    NvMediaDla                *dla,
    NvSciSyncObj               nvscisyncEOF
);

/**
 * \brief Sets an \ref NvSciSyncFence as a prefence for a DLA submit operation.
 *
 * If you use %NvMediaDlaInsertPreNvSciSyncFence(), the application must call it
 * before calling a DLA submit API. The following DLA submit operation
 * is started only after the expiry of the @a prenvscisyncfence.
 *
 * You can set a maximum of \ref NVMEDIA_DLA_MAX_PRENVSCISYNCFENCES prefences
 * by calling %NvMediaDlaInsertPreNvSciSyncFence() before calling a DLA
 * submit function.
 *
 * After a call to the DLA submit function, all NvSciSyncFences previously inserted
 * by %NvMediaDlaInsertPreNvSciSyncFence() are cleared, and they are not
 * reused for subsequent DLA submit calls.
 *
 * \param[in] dla               An NvMedia DLA device handle.
 *                              @inputrange A non-null pointer to an
 *                              \ref NvMediaDla created with NvMediaDlaCreate().
 * \param[in] prenvscisyncfence A pointer to NvSciSyncFence.
 *                              @inputrange A non-null pointer to
 *                              %NvSciSyncFence.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if any of the following occurs:
 *        - @a dla is not a valid NvMediaDla handle, or
 *        - @a prenvscisyncfence is NULL, or
 *        - @a prenvscisyncfence is not generated with an \ref NvSciSyncObj
 *          that was registered with @a dla as either type
 *          \ref NVMEDIA_PRESYNCOBJ or type \ref NVMEDIA_EOF_PRESYNCOBJ.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED if %NvMediaDlaInsertPreNvSciSyncFence is
 *        already called at least %NVMEDIA_DLA_MAX_PRENVSCISYNCFENCES times with
 *        the same @a dla NvMediaDla handle before a DLA submit call.
 * - \ref NVMEDIA_STATUS_ERROR for any other error.
 */
NvMediaStatus
NvMediaDlaInsertPreNvSciSyncFence(
    NvMediaDla                *dla,
    const NvSciSyncFence      *prenvscisyncfence
);

#if (NV_IS_SAFETY == 0)
/**
 * \brief Gets an SOF \ref NvSciSyncFence for a DLA submit operation.
 *
 * \note This API is currently not supported.
 *
 * An SOF %NvSciSyncFence is associated with a DLA submit operation, and its
 * expiry indicates that the corresponding DLA submit operation has started.
 * NvMediaDlaGetSOFNvSciSyncFence() returns the SOF %NvSciSyncFence associated
 * with the last DLA submit call.
 *
 * If you use %NvMediaDlaGetSOFNvSciSyncFence(),
 * you must call it after calling the DLA submit function.
 *
 * \param[in] dla                An NvMedia DLA device handle.
 *                               @inputrange A non-null pointer to an
 *                               \ref NvMediaDla created with NvMediaDlaCreate().
 * \param[in] sofnvscisyncobj    The SOF \ref NvSciSyncObj associated with
 *                               the \ref NvSciSyncFence being requested.
 *                               This structure will be modified by this function.
 *                               @inputrange A non-null pointer created by
 *                               NvSciSyncObjAlloc().
 * \param[in, out] sofnvscisyncfence A pointer to the SOF %NvSciSyncFence.
 *                               @inputrange A non-null pointer to
 *                               %NvSciSyncFence.
 *                               @outputrange A non-null pointer to
 *                               SOF %NvSciSyncFence.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if any of the following occurs:
 *        - @a dla is not a valid NvMediaDla handle, or
 *        - @a sofnvscisyncfence is NULL, or
 *        - @a sofnvscisyncobj is not registered with @a dla as type
 *          \ref NVMEDIA_SOFSYNCOBJ or type \ref NVMEDIA_SOF_PRESYNCOBJ.
 * - \ref NVMEDIA_STATUS_ERROR indicates that this function was called before
 *        calling the DLA submit API.
 */
NvMediaStatus
NvMediaDlaGetSOFNvSciSyncFence(
    const NvMediaDla                *dla,
    NvSciSyncObj               sofnvscisyncobj,
    NvSciSyncFence            *sofnvscisyncfence
);
#endif // (NV_IS_SAFETY == 0)

/**
 * \brief Gets an EOF \ref NvSciSyncFence for a DLA submit operation.
 *
 * An EOF %NvSciSyncFence is associated with a DLA submit operation and its expiry
 * indicates that the corresponding DLA submit operation has finished.
 * \ref NvMediaDlaGetEOFNvSciSyncFence returns the EOF %NvSciSyncFence associated
 * with the last DLA submit call.
 *
 * If you use %NvMediaDlaGetEOFNvSciSyncFence(),
 * you must call it after calling a DLA submit function.
 *
 * \param[in] dla                An NvMedia DLA device handle.
 *                               @inputrange A non-null pointer to an
 *                               \ref NvMediaDla created with NvMediaDlaCreate().
 * \param[in] eofnvscisyncobj    An EOF \ref NvSciSyncObj associated with
 *                               the \ref NvSciSyncFence being requested.
 *                               This structure will be modified by this function.
 *                               @inputrange A non-null pointer created by
 *                               NvSciSyncObjAlloc().
 * \param[in, out] eofnvscisyncfence A pointer to the EOF %NvSciSyncFence.
 *                               @inputrange A non-null pointer to
 *                               %NvSciSyncFence.
 *                               @outputrange A non-null pointer to
 *                               %NvSciSyncFence.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if any of the following occurs:
 *        - @a dla is not a valid NvMediaDla handle, or
 *        - @a eofnvscisyncfence is NULL, or
 *        - @a eofnvscisyncobj is not registered with @a dla as type
 *          \ref NVMEDIA_EOFSYNCOBJ or type \ref NVMEDIA_EOF_PRESYNCOBJ.
 * - \ref NVMEDIA_STATUS_ERROR indicates that this function was called before
 *        calling the DLA submit API.
 */
NvMediaStatus
NvMediaDlaGetEOFNvSciSyncFence(
    const NvMediaDla                *dla,
    NvSciSyncObj               eofnvscisyncobj,
    NvSciSyncFence            *eofnvscisyncfence
);

/*
 * \defgroup history_nvmedia_dla_nvscisync History
 * Provides change history for the NvMedia Dla NvSciSync API
 *
 * \section history_nvmedia_dla_nvscisync Version History
 *
 * <b> Version 1.0 </b> March 14, 2019
 * - Initial release
 *
 * <b> Version 1.1 </b> April 11, 2019
 * - Add new API NvMediaDlaSetNvSciSyncObjforSOF and NvMediaDlaGetEOFNvSciSyncFence
 * - Rename NvMediaDlaUnRegisterNvSciSyncObj to NvMediaDlaUnregisterNvSciSyncObj
 *
 * <b> Version 1.2 </b> Jan 22, 2020
 * - Disable NvMediaDlaSetNvSciSyncObjforSOF and NvMediaDlaGetSOFNvSciSyncFence in
 *   safety build as they are currently unsupported.
 *
 */

/** @} <!-- Ends nvmedia_dla_nvscisync_api DLA Synchronization --> */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_DLA_NVSCISYNC_H */
