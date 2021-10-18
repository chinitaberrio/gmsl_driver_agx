/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: IOFST NvSciSync </b>
 *
 * @b Description: This file contains NvSciSync related APIs for NvMediaIOFST.
 *
 */

#ifndef NVMEDIA_IOFST_NVSCISYNC_H
#define NVMEDIA_IOFST_NVSCISYNC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"
#include "nvscisync.h"
#include "nvmedia_iofst.h"

/**
 * \defgroup nvmedia_iofst_nvscisync_api Image OFST Synchronization
 *
 * The NvMedia IOFST NvSciSync API encompasses all NvMediaIOFST
 * NvSciSync handling functions.
 *
 * @ingroup nvmedia_image_top
 * @{
 */

/** \brief Major version number. */
#define NVMEDIA_IOFST_NVSCISYNC_VERSION_MAJOR   1
/** \brief Minor version number. */
#define NVMEDIA_IOFST_NVSCISYNC_VERSION_MINOR   2

/**
 * Specifies the maximum number of times NvMediaIOFSTInsertPreNvSciSyncFence()
 * can be called before each call to NvMediaIOFSTProcessFrame().
 */
#define NVMEDIA_IOFST_MAX_PRENVSCISYNCFENCES  (3U)

/**
 * \brief Returns the version information for the NvMediaIOFST
 * NvSciSync library.
 *
 * \pre  None
 * \post None
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Init
 *
 * \param[out] version A pointer to an NvMediaVersion structure
 *                       filled by the IOFST NvSciSync library.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if the \a version pointer is NULL.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIOFSTNvSciSyncGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Fills the NvMediaIOFST specific NvSciSync attributes.
 *
 * This function assumes that @a attrlist is a valid \ref NvSciSyncAttrList.
 *
 * This function sets the public attribute:
 * - \ref NvSciSyncAttrKey_RequiredPerm
 *
 * The application must not set this attribute.
 *
 * \pre NvMediaIOFSTCreate()
 * \post NvSciSyncAttrList populated with NvMediaIOFST specific NvSciSync
 *        attributes
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Init
 *
 * \param[in] iofst An %NvMediaIOFST device handle.
 * \param[out] attrlist A pointer to an %NvSciSyncAttrList structure where
 *                NvMedia places NvSciSync attributes.
 * \param[in] clienttype Indicates whether the @a attrlist is requested for
 *                an %NvMediaIOFST signaler or an %NvMediaIOFST waiter.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if @a attrlist is NULL,
 *         or any of the  public attributes listed above are already set.
 * - ::NVMEDIA_STATUS_OUT_OF_MEMORY if there is not enough
 *         memory for the requested operation.
 * - ::NVMEDIA_STATUS_ERROR if there is an internal error while setting
 *         the attributes.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIOFSTFillNvSciSyncAttrList(
    const NvMediaIOFST  *iofst,
    NvSciSyncAttrList   attrlist,
    NvMediaNvSciSyncClientType clienttype
);

/**
 * \brief Registers an \ref NvSciSyncObj with NvMediaIOFST.
 *
 * Every NvSciSyncObj (even duplicate objects) used by %NvMediaIOFST
 * must be registered by a call to this function before it is used.
 * Only the exact same registered NvSciSyncObj can be passed to
 * NvMediaIOFSTSetNvSciSyncObjforEOF(), NvMediaIOFSTGetEOFNvSciSyncFence(), or
 * NvMediaIOFSTUnregisterNvSciSyncObj().
 *
 * For a given %NvMediaIOFST handle,
 * one NvSciSyncObj can be registered as one \ref NvMediaNvSciSyncObjType only.
 * For each NvMediaNvSciSyncObjType, a maximum of 16 NvSciSyncObjs can
 * be registered.
 *
 * \pre NvMediaIOFSTFillNvSciSyncAttrList()
 * \post NvSciSyncObj registered with NvMediaIOFST
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Init
 *
 * \param[in] iofst       An %NvMediaIOFST device handle.
 * \param[in] syncobjtype Determines how @a nvscisync is used by @a iofst.
 * \param[in] nvscisync   The NvSciSyncObj to be registered with @a iofst.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if @a iofst is NULL or
 *         @a syncobjtype is not a valid NvMediaNvSciSyncObjType.
 * - ::NVMEDIA_STATUS_NOT_SUPPORTED if @a nvscisync is not a
 *         compatible NvSciSyncObj which %NvMediaIOFST can support.
 * - ::NVMEDIA_STATUS_ERROR if the maximum number of NvSciScynObjs
 *         are already registered for the given @a syncobjtype, or
 *         if @a nvscisync is already registered with the same @a iofst
 *         handle for a different @a syncobjtype.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIOFSTRegisterNvSciSyncObj(
    const NvMediaIOFST         *iofst,
    NvMediaNvSciSyncObjType    syncobjtype,
    NvSciSyncObj               nvscisync
);

/**
 * \brief Unregisters an \ref NvSciSyncObj with NvMediaIOFST.
 *
 * Every %NvSciSyncObj registered with %NvMediaIOFST by
 * NvMediaIOFSTRegisterNvSciSyncObj() must be unregistered
 * before calling NvMediaIOFSTDestroy().
 *
 * Before the application calls this function, it must ensure that any
 * NvMediaIOFSTProcessFrame() operation that uses the NvSciSyncObj has completed.
 * If this function is called while NvSciSyncObj is still in use by any
 * %NvMediaIOFSTProcessFrame() operation, the behavior is undefined.
 *
 * \pre NvMediaIOFSTProcessFrame()
 * \pre NvMediaImageGetStatus() [verify that processing is complete]
 * \post NvSciSyncObj un-registered with NvMediaIOFST
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: De-init
 *
 * \param[in] iofst     An %NvMediaIOFST device handle.
 * \param[in] nvscisync An NvSciSyncObj to be unregistered with @a iofst.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if @a iofst is NULL, or
 *         @a nvscisync is not registered with @a iofst.
 * - ::NVMEDIA_STATUS_ERROR if @a iofst was destroyed before this function is
 *         called.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIOFSTUnregisterNvSciSyncObj(
    const NvMediaIOFST      *iofst,
    NvSciSyncObj            nvscisync
);

/**
 * \brief Specifies the \ref NvSciSyncObj to be used for an EOF
 * \ref NvSciSyncFence.
 *
 * To use NvMediaIOFSTGetEOFNvSciSyncFence(), the application must call
 * NvMediaIOFSTSetNvSciSyncObjforEOF() before it calls
 * NvMediaIOFSTProcessFrame().
 *
 * %NvMediaIOFSTSetNvSciSyncObjforEOF() currently may be called only once before
 * each call to %NvMediaIOFSTProcessFrame(). The application may choose to call
 * this function only once before the first call to %NvMediaIOFSTProcessFrame().
 *
 * \pre NvMediaIOFSTRegisterNvSciSyncObj()
 * \post NvSciSyncObj to be used as EOF NvSciSyncFence is set
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Runtime
 *
 * \param[in] iofst        An \ref NvMediaIOFST device handle.
 * \param[in] nvscisyncEOF A registered NvSciSyncObj which is to be
 *                           associated with EOF \ref NvSciSyncFence.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if @a iofst is NULL, or if @a nvscisyncEOF
 *         is not registered with @a iofst as either type
 *         \ref NVMEDIA_EOFSYNCOBJ or \ref NVMEDIA_EOF_PRESYNCOBJ.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIOFSTSetNvSciSyncObjforEOF(
    const NvMediaIOFST    *iofst,
    NvSciSyncObj          nvscisyncEOF
);

/**
 * \brief Sets an \ref NvSciSyncFence as a prefence for an
 * NvMediaIOFSTProcessFrame() operation.
 *
 * You must call %NvMediaIOFSTInsertPreNvSciSyncFence() before you call
 * %NvMediaIOFSTProcessFrame(). The %NvMediaIOFSTProcessFrame() operation is
 * started only after the expiry of the @a prenvscisyncfence.
 *
 * For example, in this sequence of code:
 * \code
 * nvmstatus = NvMediaIOFSTInsertPreNvSciSyncFence(handle, prenvscisyncfence);
 * nvmstatus = NvMediaIOFSTProcessFrame(handle, srcsurf, refsurf, mvsurf,
 *                      hintparams, instanceId);
 * \endcode
 * the %NvMediaIOFSTProcessFrame() operation is assured to start only after
 * the expiry of @a prenvscisyncfence.
 *
 * You can set a maximum of \ref NVMEDIA_IOFST_MAX_PRENVSCISYNCFENCES prefences
 * by calling %NvMediaIOFSTInsertPreNvSciSyncFence() before
 * %NvMediaIOFSTProcessFrame(). After the call to %NvMediaIOFSTProcessFrame(),
 * all NvSciSyncFences previously inserted by
 * %NvMediaIOFSTInsertPreNvSciSyncFence() are removed, and they are not
 * reused for the subsequent %NvMediaIOFSTProcessFrame() calls.
 *
 * \pre Pre-NvSciSync fence obtained from previous engine in the pipeline
 * \post Pre-NvSciSync fence is set
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Runtime
 *
 * \param[in] iofst             An \ref NvMediaIOFST device handle.
 * \param[in] prenvscisyncfence A pointer to %NvSciSyncFence.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if @a iofst is not a valid %NvMediaIOFST
 *     handle, or @a prenvscisyncfence is NULL, or if @a prenvscisyncfence was
 *     not generated with an \ref NvSciSyncObj that was registered with @a iofst
 *     as either \ref NVMEDIA_PRESYNCOBJ or \ref NVMEDIA_EOF_PRESYNCOBJ type.
 * - ::NVMEDIA_STATUS_NOT_SUPPORTED if %NvMediaIOFSTInsertPreNvSciSyncFence()
 *     has already been called at least %NVMEDIA_IOFST_MAX_PRENVSCISYNCFENCES
 *     times with the same @a iofst handle before an %NvMediaIOFSTProcessFrame()
 *     call.
 */

// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIOFSTInsertPreNvSciSyncFence(
    const NvMediaIOFST       *iofst,
    const NvSciSyncFence     *prenvscisyncfence
);

/**
 * \brief Gets EOF \ref NvSciSyncFence for an NvMediaIOFSTProcessFrame()
 * operation.
 *
 * The EOF %NvSciSyncFence associated with an %NvMediaIOFSTProcessFrame()
 * operation is an NvSciSyncFence. Its expiry indicates that the corresponding
 * %NvMediaIOFSTProcessFrame() operation has finished.
 *
 * This function returns the EOF %NvSciSyncFence associated with the last
 * %NvMediaIOFSTProcessFrame() call. %NvMediaIOFSTGetEOFNvSciSyncFence() must be
 * called after an %NvMediaIOFSTProcessFrame() call.
 *
 * For example, in this sequence of code:
 * \code
 * nvmstatus = NvMediaIOFSTProcessFrame(handle, srcsurf, refsurf, mvsurf,
 *                      hintparams, instanceId);
 * nvmstatus = NvMediaIOFSTGetEOFNvSciSyncFence(handle, nvscisyncEOF,
 *                      eofnvscisyncfence);
 * \endcode
 * expiry of @a eofnvscisyncfence indicates that the preceding
 * %NvMediaIOFSTProcessFrame() operation has finished.
 *
 * \pre NvMediaIOFSTSetNvSciSyncObjforEOF()
 * \pre NvMediaIOFSTProcessFrame()
 * \post EOF NvSciSync fence for a submitted task is obtained
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Runtime
 *
 * \param[in] iofst            An \ref NvMediaIOFST device handle.
 * \param[in] eofnvscisyncobj  An EOF \ref NvSciSyncObj associated with
 *                               the %NvSciSyncFence which is being
 *                               requested.
 * \param[out] eofnvscisyncfence A pointer to EOF %NvSciSyncFence.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if @a iofst is not a valid %NvMediaIOFST
 *         handle or @a eofnvscisyncfence is NULL, or @a eofnvscisyncobj is not
 *         registered with @a iofst as type \ref NVMEDIA_EOFSYNCOBJ or
 *         \ref NVMEDIA_EOF_PRESYNCOBJ.
 * - ::NVMEDIA_STATUS_ERROR if the function was called before
 *          %NvMediaIOFSTProcessFrame() was called.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIOFSTGetEOFNvSciSyncFence(
    const NvMediaIOFST      *iofst,
    NvSciSyncObj            eofnvscisyncobj,
    NvSciSyncFence          *eofnvscisyncfence
);

/**
 * \brief Specifies the \ref NvSciSyncObj to be used for an SOF
 * \ref NvSciSyncFence.
 *
 * <b> This function is not supported </b>
 *
 * To use NvMediaIOFSTGetSOFNvSciSyncFence(), the application must call
 * NvMediaIOFSTSetNvSciSyncObjforSOF() before it calls
 * NvMediaIOFSTProcessFrame().
 *
 * %NvMediaIOFSTSetNvSciSyncObjforSOF() currently may be called only once before
 * each call to %NvMediaIOFSTProcessFrame(). The application may choose to call
 * this function only once before the first call to %NvMediaIOFSTProcessFrame().
 *
 * \pre N/A
 * \post N/A
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: N/A
 *
 * \param[in] iofst        An \ref NvMediaIOFST device handle.
 * \param[in] nvscisyncSOF A registered NvSciSyncObj which is to be
 *                           associated with SOF \ref NvSciSyncFence.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if @a iofst is NULL, or if @a nvscisyncSOF
 *         is not registered with @a iofst as either type
 *         \ref NVMEDIA_SOFSYNCOBJ or \ref NVMEDIA_SOF_PRESYNCOBJ.
 * - ::NVMEDIA_STATUS_NOT_SUPPORTED on platforms where this functionality is not
 *          supported.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIOFSTSetNvSciSyncObjforSOF(
    const NvMediaIOFST      *iofst,
    const NvSciSyncObj      nvscisyncSOF
);

/**
 * \brief Gets SOF \ref NvSciSyncFence for an NvMediaIOFSTProcessFrame()
 * operation.
 *
 * <b> This function is not supported </b>
 *
 * The SOF %NvSciSyncFence associated with an %NvMediaIOFSTProcessFrame()
 * operation is an NvSciSyncFence. Its expiry indicates that the corresponding
 * %NvMediaIOFSTProcessFrame() operation has started.
 *
 * This function returns the SOF %NvSciSyncFence associated with the last
 * %NvMediaIOFSTProcessFrame() call. %NvMediaIOFSTGetSOFNvSciSyncFence() must be
 * called after an %NvMediaIOFSTProcessFrame() call.
 *
 * \pre N/A
 * \post N/A
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: N/A
 *
 * \param[in] iofst            An \ref NvMediaIOFST device handle.
 * \param[in] sofnvscisyncobj  An SOF \ref NvSciSyncObj associated with
 *                               the %NvSciSyncFence which is being
 *                               requested.
 * \param[out] sofnvscisyncfence A pointer to SOF %NvSciSyncFence.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if @a iofst is not a valid %NvMediaIOFST
 *         handle or @a sofnvscisyncfence is NULL, or @a sofnvscisyncobj is not
 *         registered with @a iofst as type \ref NVMEDIA_SOFSYNCOBJ or
 *         \ref NVMEDIA_SOF_PRESYNCOBJ.
 * - ::NVMEDIA_STATUS_ERROR if the function was called before
 *          %NvMediaIOFSTProcessFrame() was called.
 * - ::NVMEDIA_STATUS_NOT_SUPPORTED on platforms where this functionality is not
 *          supported.
 */
// coverity[misra_c_2012_rule_8_7_violation : FALSE]
NvMediaStatus
NvMediaIOFSTGetSOFNvSciSyncFence(
    const NvMediaIOFST      *iofst,
    const NvSciSyncObj      sofnvscisyncobj,
    NvSciSyncFence          *sofnvscisyncfence
);

/*
 * \defgroup history_nvmedia_iofst_nvscisync History
 * Provides change history for the NvMedia IOFST NvSciSync API
 *
 * \section history_nvmedia_iofst_nvscisync Version History
 *
 * <b> Version 1.0 </b> April 03, 2019
 * - Initial release
 *
 * <b> Version 1.1 </b> July 15, 2019
 * - Add new API NvMediaIOFSTSetNvSciSyncObjforSOF and NvMediaIOFSTGetSOFNvSciSyncFence
 *
 * <b> Version 1.2 </b> October 5, 2019
 * - Fix MISRA violations 8.13
 * - Add Coverity Code annotations to suppress partial 8.7 violations
 */
/** @} <!-- Ends nvmedia_iofst_nvscisync_api NvMediaIOFST NvSciSync --> */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_IOFST_NVSCISYNC_H */
