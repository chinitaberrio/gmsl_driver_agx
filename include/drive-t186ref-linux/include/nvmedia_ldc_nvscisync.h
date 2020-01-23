/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: LDC NvSciSync </b>
 *
 * @b Description: This file contains the NvMediaLDC and NvSciSync related APIs.
 *
 */

#ifndef NVMEDIA_LDC_NVSCISYNC_H
#define NVMEDIA_LDC_NVSCISYNC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"
#include "nvscisync.h"
#include "nvmedia_ldc.h"

/**
 * @defgroup nvmedia_ldc_nvscisync_api Lens Distortion Correction Synchronization
 *
 * The NvMedia LDC NvSciSync API encompasses all
 * NvMediaLDC NvSciSync handling functions.
 *
 * @ingroup nvmedia_top
 * @{
 */

/** \brief Major version number. */
#define NVMEDIA_LDC_NVSCISYNC_VERSION_MAJOR   1
/** \brief Minor version number. */
#define NVMEDIA_LDC_NVSCISYNC_VERSION_MINOR   0

/**
 * Maximum number of times NvMediaLDCInsertPreNvSciSyncFence() can be called
 * before each NvMediaLDCProcess() call.
 */
#define NVMEDIA_LDC_MAX_PRENVSCISYNCFENCES  (3)

/**
 * \brief  Gets version information for the NvMedia LDC NvSciSync library.
 * \param[out] version A pointer to a structure where version information is to
 *                      be put.
 * \return  A status code; NVMEDIA_STATUS_OK if successful, or
 *  NVMEDIA_STATUS_BAD_PARAMETER if @a version is invalid.
 */
NvMediaStatus
NvMediaLDCNvSciSyncGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Gets \ref NvMediaLDC specific NvSciSync attributes.
 *
 * This function sets the following public attributes:
 * - \ref NvSciSyncAttrKey_RequiredPerm
 *
 * The application must not set this attribute.
 *
 * \param[in]     ldc           An NvMedia LDC device handle.
 * \param[out]    attrlist      A pointer to a structure where NvSciSync
 *                               attributes are to be placed.
 * \param[in]     clienttype    Indicates whether the attributes in @a attrlist
 *                               are requested for an NvMediaLDC signaler or
 *                               a waiter.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a attrlist was NULL,
 *  or any of the public attributes listed above were already set.
 * \retval  NVMEDIA_STATUS_OUT_OF_MEMORY indicates that there is not enough
 *  memory to perform the requested operation.
 */
NvMediaStatus
NvMediaLDCFillNvSciSyncAttrList(
    NvMediaLDC         *ldc,
    NvSciSyncAttrList attrlist,
    NvMediaNvSciSyncClientType clienttype
);


/**
 * \brief  Registers an \ref NvSciSyncObj with \ref NvMediaLDC.
 *
 * Every NvSciSyncObj (even duplicate objects) used by NvMediaLDC
 * must be registered by a call to this function before it is used.
 * Only the exact same registered NvSciSyncObj may be passed in
 * NvMediaLDCSetNvSciSyncObjforEOF(), NvMediaLDCGetEOFNvSciSyncFence(), and
 * NvMediaLDCUnregisterNvSciSyncObj().
 *
 * For a given NvMediaLDC handle,
 * one NvSciSyncObj can be registered as one \ref NvMediaNvSciSyncObjType only.
 * For each NvMediaNvSciSyncObjType a maximum of 16 NvSciSyncObjs can
 * be registered.
 *
 * \param[in] ldc           An NvMedia LDC device handle.
 * \param[in] syncobjtype   Determines how @a nvscisync is to be used by LDC.
 * \param[in] nvscisync     An NvSciSyncObj to be registered with LDC.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a ldc was NULL
 *  or @a syncobjtype was not a valid NvMediaNvSciSyncObjType.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that @a nvscisync is not a
 *         compatible NvSciSyncObj which NvMediaLDC can support.
 * \retval  NVMEDIA_STATUS_ERROR indicates that the maximum number of
 *  NvSciSyncObj objects are already registered for the given @a syncobjtype,
 *  or @a nvscisync is already registered for a different @a syncobjtype with
 *  the same LDC handle.
 */
NvMediaStatus
NvMediaLDCRegisterNvSciSyncObj(
    NvMediaLDC             *ldc,
    NvMediaNvSciSyncObjType    syncobjtype,
    NvSciSyncObj          nvscisync
);

/**
 * \brief Unregisters an \ref NvSciSyncObj with \ref NvMediaLDC.
 *
 * Every NvSciSyncObj registered with NvMediaLDC through
 * NvMediaLDCRegisterNvSciSyncObj() must be unregistered before it may be
 * destroyed by a call to NvMediaLDCDestroy().
 * Before the application calls this function it must ensure that any
 * NvMediaLDCProcess() operation which uses this NvSciSyncObj has completed.
 * The function's behavior is undefined if it is called while the NvSciSyncObj
 * is in use by an #NvMediaLDCProcess() operation.
 *
 * \param[in] ldc       An NvMedia LDC device handle.
 * \param[in] nvscisync An NvSciSyncObj to be unregistered with LDC.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a ldc was NULL
 *  or @a nvscisync was not registered with LDC.
 * \retval  NVMEDIA_STATUS_ERROR indicates that @a ldc has been destroyed.
 */
NvMediaStatus
NvMediaLDCUnregisterNvSciSyncObj(
    NvMediaLDC         *ldc,
    NvSciSyncObj      nvscisync
);

/**
 * \brief Sets an \ref NvSciSyncObj to be used for a EOF \ref NvSciSyncFence.
 *
 * To use NvMediaLDCGetEOFNvSciSyncFence(), the application must call
 * this function before it calls NvMediaLDCProcess().
 *
 * This function currently may be called only once before each call to
 * %NvMediaLDCProcess(). The application may choose to call this function only
 * once before the first #NvMediaLDCProcess().
 *
 * \param[in] ldc           An NvMedia LDC device handle.
 * \param[in] nvscisyncEOF  A registered NvSciSyncObj which is to be
 *                           associated with EOF NvSciSyncFence.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a ldc was NULL
 *  or that @a nvscisyncEOF was not registered with LDC as either type
 *  \ref NVMEDIA_EOFSYNCOBJ or type \ref NVMEDIA_EOF_PRESYNCOBJ.
 */
NvMediaStatus
NvMediaLDCSetNvSciSyncObjforEOF(
    NvMediaLDC       *ldc,
    NvSciSyncObj    nvscisyncEOF
);

/**
 * \brief  Sets an \ref NvSciSyncFence as a prefence for an NvMediaLDCProcess()
 * operation.
 *
 * The application must call this function before it
 * calls %NvMediaLDCProcess(). The following %NvMediaLDCProcess() operation is
 * assured to be started only after the expiry of the @a prenvscisyncfence.
 * For example, in the below sequence of code,
 * \code
 * nvmstatus = NvMediaLDCInsertPreNvSciSyncFence(nvmldchdl, prenvscisyncfence);
 * nvmstatus = NvMediaLDCProcess(nvmldchdl, ...);
 * \endcode
 * The NvMediaLDCProcess() operation is assured to start only after the expiry
 * of @a prenvscisyncfence.
 *
 * You can set a maximum of \ref NVMEDIA_LDC_MAX_PRENVSCISYNCFENCES prefences
 * by calling %NvMediaLDCInsertPreNvSciSyncFence() before calling
 * %NvMediaLDCProcess(). After the %NvMediaLDCProcess() call, all
 * NvSciSyncFence objects previously inserted by
 * %NvMediaLDCInsertPreNvSciSyncFence() are removed, and are not reused for
 * subsequent %NvMediaLDCProcess() calls.
 *
 * \param[in] ldc               And NvMedia LDC device handle.
 * \param[in] prenvscisyncfence A pointer to an NvSciSyncFence.
 *
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a ldc was not a valid
 *  LDC handle, @a prenvscisyncfence was NULL, or
 *  @a prenvscisyncfence was not generated with an \ref NvSciSyncObj which was
 *  registered with LDC as either type \ref NVMEDIA_PRESYNCOBJ or type
 *  \ref NVMEDIA_EOF_PRESYNCOBJ.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that this function
 *  was already called \ref NVMEDIA_LDC_MAX_PRENVSCISYNCFENCES or more times
 *  with the same LDC handle before an %NvMediaLDCProcess() call.
 */
NvMediaStatus
NvMediaLDCInsertPreNvSciSyncFence(
    NvMediaLDC                *ldc,
    const NvSciSyncFence     *prenvscisyncfence
);

/**
 * \brief  Get EOF \ref NvSciSyncFence for an NvMediaLDCProcess() operation.
 *
 * The EOF NvSciSyncFence associated with an NvMediaLDCBlit() operation is an
 * NvSciSyncFence, and its expiry indicates that the corresponding
 * %NvMediaLDCBlit() operation has finished.
 * NvMediaLDCGetEOFNvSciSyncFence() returns the EOF NvSciSyncFence associated
 * withthe last %NvMediaLDCProcess() call.
 *
 * %NvMediaLDCGetEOFNvSciSyncFence() must be called after %NvMediaLDCProcess()
 * is called. For example, in the below sequence of code:
 * \code
 * nvmstatus = NvMediaLDCProcess(nvmldchdl, ...);
 * nvmstatus = NvMediaLDCGetEOFNvSciSyncFence(nvmldchdl, nvscisyncEOF, eofnvscisyncfence);
 * \endcode
 * expiry of @a eofnvscisyncfence indicates that the preceding %NvMediaLDCBlit()
 * operation has finished.
 *
 * \param[in] ldc                   An NvMedia LDC device handle.
 * \param[in] eofnvscisyncobj       An EOF \ref NvSciSyncObj associated with
 *                                   the NvSciSyncFence being requested.
 * \param[out] eofnvscisyncfence    A pointer to an EOF NvSciSyncFence.
 *
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a ldc was not a valid
 *  LDC handle, @a eofnvscisyncfence was NULL, or @a eofnvscisyncobj was not
 *  registered with LDC as type \ref NVMEDIA_EOFSYNCOBJ or type
 *  \ref NVMEDIA_EOF_PRESYNCOBJ type.
 * \retval  NVMEDIA_STATUS_ERROR indicates that this function was called before
 *  %NvMediaLDCProcess() was called.
 */
NvMediaStatus
NvMediaLDCGetEOFNvSciSyncFence(
    NvMediaLDC         *ldc,
    NvSciSyncObj      eofnvscisyncobj,
    NvSciSyncFence    *eofnvscisyncfence
);

/*
 * \defgroup history_nvmedia_ldc_nvscisync History
 * Provides change history for the NvMedia LDC NvSciSync API
 *
 * \section history_nvmedia_ldc_nvscisync Version History
 *
 * <b> Version 1.0 </b> March 13, 2019
 * - Initial release
 *
 */
/** @} <!-- Ends nvmedia_ldc_nvscisync_api NvMedia LDC NvSciSync --> */
#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_LDC_NVSCISYNC_H */
