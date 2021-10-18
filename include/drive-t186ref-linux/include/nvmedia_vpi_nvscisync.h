/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved. All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 *
 * \brief <b> NVIDIA Media Interface: VPI NvSciSync </b>
 *
 * @b Description: This file contains the NvMediaVPI and NvSciSync related APIs.
 *
 */

#ifndef NVMEDIA_VPI_NVSCISYNC_H
#define NVMEDIA_VPI_NVSCISYNC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"
#include "nvscisync.h"
#include "nvmedia_vpi.h"

/**
 * \defgroup nvmedia_vpi_nvscisync NvMedia VPI Synchronization
 *
 * The NvMedia VPI NvSciSync API encompasses NvMedia VPI functions using
 * synchronization.
 *
 * @ingroup nvmedia_vpi_top
 * @{
 */

/** \brief Major version number. */
#define NVMEDIA_VPI_NVSCISYNC_VERSION_MAJOR   (1u)
/** \brief Minor version number. */
#define NVMEDIA_VPI_NVSCISYNC_VERSION_MINOR   (1u)

/**
 * Maximum number of times NvMediaVPIInsertPreNvSciSyncFence() can be called
 * before each VPI queue call.
*/
#define NVMEDIA_VPI_MAX_PRENVSCISYNCFENCES  (3u)

/** \brief Index of last queued task. */
#define LAST_QUEUED_TASK_IDX (0xFFFFFFFFu)

/**
 * \brief Returns the version information for the NvMedia VPI NvSciSync library.
 * \param[out] version A pointer to an \ref NvMediaVersion structure
 *                     filled by the VPI NvSciSync library.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the call is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if the pointer is invalid.
 */
NvMediaStatus
NvMediaVPINvSciSyncGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Fills the NvMediaVPI specific NvSciSync attributes.
 * This API assumes that @a attrlist is a valid \ref NvSciSyncAttrList.
 *
 * This function sets the public attributes:
 * - \ref NvSciSyncAttrKey_RequiredPerm
 *
 * The application must not set this attribute.
 *
 * \param[in] vpi          An NvMedia VPI device handle.
 * \param[in,out] attrlist A pointer to an \ref NvSciSyncAttrList structure
 *                         where NvMedia fills NvSciSync attributes.
 * \param[in] clienttype   Indicates whether the attributes in @a attrlist are
 *                         requested for an NvMediaVPI signaler or waiter.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the call is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a attrlist is NULL
 *         or any of the above listed public attributes are already set.
 */
NvMediaStatus
NvMediaVPIFillNvSciSyncAttrList(
    const NvMediaVPI          *vpi,
    NvSciSyncAttrList          attrlist,
    NvMediaNvSciSyncClientType clienttype
);

/**
 * \brief Registers an \ref NvSciSyncObj with NvMediaVPI.
 *
 * Every NvSciSyncObj(even duplicate objects) used by %NvMediaVPI
 * must be registered by a call to this function before it is used.
 * Only the exact same registered NvSciSyncObj can be passed to the run time APIs.
 *
 * For a given %NvMediaVPI handle, one NvSciSyncObj can be registered
 * as one \ref NvMediaNvSciSyncObjType only.
 * For each NvMediaNvSciSyncObjType, a maximum of 16 NvSciSyncObjs can
 * be registered.
 *
 * \param[in] vpi         An NvMedia VPI device handle.
 * \param[in] syncobjtype Determines how @a nvscisync is used by @a vpi.
 * \param[in] nvscisync   The %NvSciSyncObj to be registered with @a vpi.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a vpi is NULL or
 *        @a syncobjtype is not a valid NvMediaNvSciSyncObjType.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED if @a nvscisync is not a
 *        compatible %NvSciSyncObj which %NvMediaVPI can support.
 * - \ref NVMEDIA_STATUS_ERROR if the maximum number of NvSciScynObjs
 *        are already registered for the given @a syncobjtype, or
 *        if @a nvscisync is already registered with the same @a vpi
 *        handle for a different @a syncobjtype.
 */
NvMediaStatus
NvMediaVPIRegisterNvSciSyncObj(
    NvMediaVPI                *vpi,
    NvMediaNvSciSyncObjType    syncobjtype,
    NvSciSyncObj               nvscisync
);

/**
 * \brief Unregisters an \ref NvSciSyncObj with %NvMediaVPI.
 *
 * During teardown, every %NvSciSyncObj registered with NvMediaVPI must be
 * unregistered before calling NvMediaVPIDestroy().
 *
 * Before the application calls this function, it must ensure that the
 * application is in teardown mode, and any NvMediaVPI operation using
 * this NvSciSyncObj has completed.
 * If the function is called while NvSciSyncObj is still in use by any
 * NvMediaVPI operations, the behavior is undefined.
 *
 * \param[in] vpi       An NvMedia VPI device handle.
 * \param[in] nvscisync An NvSciSyncObj to be unregistered
 *                        with @a vpi.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a vpi is NULL, or
 *        @a nvscisync is not registered with @a vpi.
 * - \ref NVMEDIA_STATUS_ERROR if @a vpi is destroyed before this function is called.
 */
NvMediaStatus
NvMediaVPIUnRegisterNvSciSyncObj(
    NvMediaVPI                *vpi,
    NvSciSyncObj               nvscisync
);

/**
 * \brief Sets the \ref NvSciSyncObj to be used for a
 * Start of Frame (SOF) \ref NvSciSyncFence.
 *
 * To use NvMediaVPIGetSOFNvSciSyncFence(), the application must call this
 * function before calling a VPI task queue function.
 * NvMediaVPI currently accepts only one SOF \ref NvSciSyncObj. Therefore,
 * NvMediaVPISetNvSciSyncObjforSOF may be called only once for an
 * \ref NvMediaVPI handle before a call to a VPI task queue function.
 * Currently @a nvscisyncSOF is signaled before the start of the first queued task.
 *
 * \param[in] vpi          An NvMedia VPI device handle.
 * \param[in] nvscisyncSOF A registered NvSciSyncObj to be associated with
 *                         SOF %NvSciSyncFence.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a vpi is NULL, or if @a nvscisyncSOF
 *        is not registered with @a vpi as either type \ref NVMEDIA_SOFSYNCOBJ or
 *        type \ref NVMEDIA_SOF_PRESYNCOBJ.
 */
NvMediaStatus
NvMediaVPISetNvSciSyncObjforSOF(
    NvMediaVPI                *vpi,
    NvSciSyncObj               nvscisyncSOF
);

/**
 * \brief Sets the \ref NvSciSyncObj to be used for an
 *  End of Frame (EOF) \ref NvSciSyncFence.
 *
 * To use NvMediaVPIGetEOFNvSciSyncFence(), the application must call
 * this function before calling a VPI task queue function.
 *
 * NvMediaVPI currently accepts only one EOF \ref NvSciSyncObj. Therefore,
 * %NvMediaVPISetNvSciSyncObjforEOF() can be called only once for an
 * \ref NvMediaVPI handle.
 * Currently, EOF is signaled only upon the completion of the last queued task.
 *
 * \param[in] vpi           An NvMedia VPI device handle.
 * \param[in] nvscisyncEOF  A registered NvSciSyncObj which is to be
 *                            associated with EOF %NvSciSyncFence.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 *
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a vpi is NULL, or if @a nvscisyncEOF
 *        is not registered with @a vpi as either type \ref NVMEDIA_EOFSYNCOBJ
 *        or type \ref NVMEDIA_EOF_PRESYNCOBJ.
 */
NvMediaStatus
NvMediaVPISetNvSciSyncObjforEOF(
    NvMediaVPI                *vpi,
    NvSciSyncObj               nvscisyncEOF
);

/**
 * \brief Sets an \ref NvSciSyncFence as a prefence for a VPI submit operation.
 *
 * If you use %NvMediaVPIInsertPreNvSciSyncFence(), the application must call it
 * before calling a VPI algorithm queue API. The following queued VPI task
 * is started only after the expiry of the @a prenvscisyncfence.
 *
 * Currently, before calling \ref NvMediaVPIFlush() for a sequence of
 * queued VPI tasks, you may call \ref NvMediaVPIInsertPreNvSciSyncFence() only
 * once.
 *
 * You can set a maximum of \ref NVMEDIA_VPI_MAX_PRENVSCISYNCFENCES prefences
 * by calling %NvMediaVPIInsertPreNvSciSyncFence() before calling a VPI
 * queue function.
 * After a call to the VPI queue function, all NvSciSyncFences previously inserted
 * by %NvMediaVPIInsertPreNvSciSyncFence() are cleared, and they are not
 * reused for the subsequent VPI queue calls.
 *
 * \param[in] vpi               An NvMedia VPI device handle.
 * This structure is modified internally.
 * \param[in] prenvscisyncfence A pointer to NvSciSyncFence.
 *
 * \return \ref NvMediaStatus, the completion status of the operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a vpi is not a valid NvMediaVPI
 *    handle or @a prenvscisyncfence is NULL, or if @a prenvscisyncfence
 *    was not generated with an \ref NvSciSyncObj that was registered with
 *    @a vpi as either type \ref NVMEDIA_PRESYNCOBJ or type
 *    \ref NVMEDIA_EOF_PRESYNCOBJ.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED if %NvMediaVPIInsertPreNvSciSyncFence()
 *    was already called at least %NVMEDIA_VPI_MAX_PRENVSCISYNCFENCES times with
 *    the same @a vpi NvMediaVPI handle before a VPI queue call.
 */
NvMediaStatus
NvMediaVPIInsertPreNvSciSyncFence(
    NvMediaVPI                *vpi,
    const NvSciSyncFence      *prenvscisyncfence
);

/**
 * \brief Gets an SOF \ref NvSciSyncFence for a queued VPI task.
 *
 * The expiry of an SOF NvSciFence associated with a queued VPI task
 * indicates that the corresponding submitted VPI task has started.
 * If you use %NvMediaVPIGetSOFNvSciSyncFence(), the application must call it after
 * calling \ref NvMediaVPIFlush().
 *
 * \param[in] vpi                An NvMedia VPI device handle.
 * \param[in] sofnvscisyncobj   SOF %NvSciSyncObj associated with
 *                               the requested NvSciSyncFence.
 * \param[out] sofnvscisyncfence A pointer to the SOF NvSciSyncFence.
 * \param[in] opIdx              An index of the queued operation for this
 *                               object's SOF.
 *            This value is between 0 and \ref NVMEDIA_VPI_MAX_QUEUED_TASKS.
 *            This release supports only 0 (SOF for first queued task).
 *
 * \return \ref NvMediaStatus, return status of operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a vpi is not a valid NvMediaVPI
 *        handle, or @a sofnvscisyncfence is NULL, or @a sofnvscisyncobj
 *        was not registered with @a vpi of type \ref NVMEDIA_SOFSYNCOBJ or
 *        type \ref NVMEDIA_SOF_PRESYNCOBJ.
 * - \ref NVMEDIA_STATUS_ERROR if this API is called before calling VPI submit API.
 */
NvMediaStatus
NvMediaVPIGetSOFNvSciSyncFence(
    const NvMediaVPI          *vpi,
    NvSciSyncObj               sofnvscisyncobj,
    NvSciSyncFence            *sofnvscisyncfence,
    uint32_t                   opIdx
);

/**
 * \brief Gets an EOF \ref NvSciSyncFence for a queued VPI task.
 *
 * The expiry of an EOF NvSciSyncFence associated with a queued VPI task
 * indicates that the corresponding submitted VPI task has finished.
 * If you use %NvMediaVPIGetEOFNvSciSyncFence(), you must call it after
 * calling \ref NvMediaVPIFlush().
 *
 * \param[in] vpi                An NvMedia VPI device handle.
 * \param[in] eofnvscisyncobj    The EOF \ref NvSciSyncObj associated with
 *                               the %NvSciSyncFence being requested.
 * \param[out] eofnvscisyncfence A pointer to the EOF %NvSciSyncFence.
 * \param[in] opIdx              An index of the queued operation for this
 *                               object's EOF.
 *            This value can be:
 *            - Between 0 and \ref NVMEDIA_VPI_MAX_QUEUED_TASKS or
 *            - \ref LAST_QUEUED_TASK_IDX to get the EOF for the last queued task.
 *            This release supports only %LAST_QUEUED_TASK_IDX.
 *
 * \return \ref NvMediaStatus, return status of operation:
 * - \ref NVMEDIA_STATUS_OK if the function is successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER if @a vpi is not a valid NvMediaVPI
 *        handle or @a eofnvscisyncfence is NULL or @a eofnvscisyncobj was not
 *        registered with @a vpi type \ref NVMEDIA_EOFSYNCOBJ or
 *        type \ref NVMEDIA_EOF_PRESYNCOBJ.
 * - \ref NVMEDIA_STATUS_ERROR if this API is called before calling VPI queue API.
 */
NvMediaStatus
NvMediaVPIGetEOFNvSciSyncFence(
    const NvMediaVPI          *vpi,
    NvSciSyncObj               eofnvscisyncobj,
    NvSciSyncFence            *eofnvscisyncfence,
    uint32_t                   opIdx
);
/*
 * \defgroup history_nvmedia_vpi_nvscisync History
 * Provides change history for the NvMedia VPI NvSciSync API
 *
 * \section history_nvmedia_vpi_nvscisync Version History
 *
 * <b> Version 1.0 </b> March 14, 2019
 * - Initial release
 *
 * <b> Version 1.1 </b> April 13, 2020
 * - Fix minor MISRA violations
 * - Update NvMediaVPIFillNvSciSyncAttrList, NvMediaVPIGetSOFNvSciSyncFence,
 *   NvMediaVPIGetEOFNvSciSyncFence function parameters to be const
 *
 */

#ifdef __cplusplus
};     /* extern "C" */
#endif
/** @} */
#endif /* NVMEDIA_VPI_NVSCISYNC_H */
