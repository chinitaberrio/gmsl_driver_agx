/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: IEP NvSciSync </b>
 *
 * @b Description: This file contains NvSciSync related APIs for NvMediaIEP.
 *
 */

#ifndef NVMEDIA_IEP_NVSCISYNC_H
#define NVMEDIA_IEP_NVSCISYNC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"
#include "nvscisync.h"
#include "nvmedia_iep.h"

/**
 * \defgroup nvmedia_iep_nvscisync_api Image Encoder Synchronization
 *
 * The NvMedia IEP NvSciSync API encompasses all NvMediaIEP
 * NvSciSync handling functions.
 *
 * @ingroup nvmedia_image_top
 * @{
 */

/** \brief Major version number. */
#define NVMEDIA_IEP_NVSCISYNC_VERSION_MAJOR   1
/** \brief Minor version number. */
#define NVMEDIA_IEP_NVSCISYNC_VERSION_MINOR   1

/**
 * Specifies the maximum number of times NvMediaIEPInsertPreNvSciSyncFence()
 * can be called before each call to NvMediaIEPFeedFrame().
 */
#define NVMEDIA_IEP_MAX_PRENVSCISYNCFENCES  (3U)

/**
 * \brief Returns the version information for the NvMedia IEP NvSciSync library.
 *
 * \pre  None
 * \post None
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Init
 *
 * \param[out] version A pointer to an NvMediaVersion structure
 *                    filled by the IEP NvSciSync library.
 *
 * \return ::NvMediaStatus The completion status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if the pointer is invalid.
 */
NvMediaStatus
NvMediaIEPNvSciSyncGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Fills the NvMediaIEP specific NvSciSync attributes.
 *
 * This function assumes that @a attrlist is a valid \ref NvSciSyncAttrList.
 *
 * This function sets the public attribute:
 * - \ref NvSciSyncAttrKey_RequiredPerm
 *
 * The application must not set this attribute.
 *
 * \pre NvMediaIEPCreate()
 * \post NvSciSyncAttrList populated with NvMediaIEP specific NvSciSync
 *        attributes
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Init
 *
 * \param[in] encoder An %NvMediaIEP device handle.
 * \param[out] attrlist A pointer to an %NvSciSyncAttrList structure where
 *                NvMedia places NvSciSync attributes.
 * \param[in] clienttype Indicates whether the NvSciSyncAttrList requested for
 *                an %NvMediaIEP signaler or an %NvMediaIEP waiter.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if @a attrlist is NULL,
 *         or any of the public attributes listed above are already set.
 * - ::NVMEDIA_STATUS_OUT_OF_MEMORY if there is not enough
 *         memory for the requested operation.
 */
NvMediaStatus
NvMediaIEPFillNvSciSyncAttrList(
    const NvMediaIEP           *encoder,
    NvSciSyncAttrList          attrlist,
    NvMediaNvSciSyncClientType clienttype
);


/**
 * \brief Registers an \ref NvSciSyncObj with NvMediaIEP.
 *
 * Every NvSciSyncObj(even duplicate objects) used by %NvMediaIEP
 * must be registered by a call to this function before it is used.
 * Only the exact same registered NvSciSyncObj can be passed to
 * NvMediaIEPSetNvSciSyncObjforEOF(), NvMediaIEPGetEOFNvSciSyncFence(), or
 * NvMediaIEPUnregisterNvSciSyncObj().
 *
 * For a given %NvMediaIEP handle,
 * one NvSciSyncObj can be registered as one \ref NvMediaNvSciSyncObjType only.
 * For each NvMediaNvSciSyncObjType, a maximum of 16 NvSciSyncObjs can
 * be registered.
 *
 * \pre NvMediaIEPFillNvSciSyncAttrList()
 * \post NvSciSyncObj registered with NvMediaIEP
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Init
 *
 * \param[in] encoder     An %NvMediaIEP device handle.
 * \param[in] syncobjtype Determines how @a nvscisync is used by @a encoder.
 * \param[in] nvscisync   The NvSciSyncObj to be registered with @a encoder.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if @a encoder is NULL or
 *         @a syncobjtype is not a valid NvMediaNvSciSyncObjType.
 * - ::NVMEDIA_STATUS_NOT_SUPPORTED if @a nvscisync is not a
 *         compatible NvSciSyncObj which %NvMediaIEP can support.
 * - ::NVMEDIA_STATUS_ERROR if the maximum number of NvSciScynObjs
 *         are already registered for the given @a syncobjtype, or
 *         if @a nvscisync is already registered with the same @a encoder
 *         handle for a different @a syncobjtype.
 */
NvMediaStatus
NvMediaIEPRegisterNvSciSyncObj(
    const NvMediaIEP           *encoder,
    NvMediaNvSciSyncObjType    syncobjtype,
    NvSciSyncObj               nvscisync
);

/**
 * \brief Unregisters an \ref NvSciSyncObj with NvMediaIEP.
 *
 * Every %NvSciSyncObj registered with %NvMediaIEP by NvMediaIEPRegisterNvSciSyncObj()
 * must be unregistered before calling NvMediaIEPDestroy().
 *
 * Before the application calls this function, it must ensure that any
 * \ref NvMediaIEPFeedFrame() operation that uses the NvSciSyncObj has completed.
 * If this function is called while NvSciSyncObj is still
 * in use by any %NvMediaIEPFeedFrame() operation, the behavior is undefined.
 *
 * \pre NvMediaIEPFeedFrame()
 * \pre NvMediaIEPGetBitsEx() [verify that processing is complete]
 * \post NvSciSyncObj un-registered with NvMediaIEP
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: De-init
 *
 * \param[in] encoder An %NvMediaIEP device handle.
 * \param[in] nvscisync An NvSciSyncObj to be unregistered with @a encoder.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if encoder is NULL, or
 *         @a nvscisync is not registered with @a encoder.
 * - ::NVMEDIA_STATUS_ERROR if @a encoder was destroyed before this function
 *         was called.
 */
NvMediaStatus
NvMediaIEPUnregisterNvSciSyncObj(
    const NvMediaIEP  *encoder,
    NvSciSyncObj      nvscisync
);

/**
 * \brief Specifies the \ref NvSciSyncObj to be used for an EOF \ref NvSciSyncFence.
 *
 * To use NvMediaIEPGetEOFNvSciSyncFence(), the application must call
 * %NvMediaIEPSetNvSciSyncObjforEOF() before it calls NvMediaIEPFeedFrame().
 *
 * %NvMediaIEPSetNvSciSyncObjforEOF() currently may be called only once before
 * each call to %NvMediaIEPFeedFrame(). The application may choose to call this
 * function only once before the first call to %NvMediaIEPFeedFrame().
 *
 * \pre NvMediaIEPRegisterNvSciSyncObj()
 * \post NvSciSyncObj to be used as EOF NvSciSyncFence is set
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Runtime
 *
 * \param[in] encoder An %NvMediaIEP device handle.
 * \param[in] nvscisyncEOF A registered NvSciSyncObj which is to be
 *                           associated with EOF \ref NvSciSyncFence.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if @a encoder is NULL, or if @a nvscisyncEOF
 *         is not registered with @a encoder as either type
 *         \ref NVMEDIA_EOFSYNCOBJ or \ref NVMEDIA_EOF_PRESYNCOBJ.
 */
NvMediaStatus
NvMediaIEPSetNvSciSyncObjforEOF(
    const NvMediaIEP      *encoder,
    NvSciSyncObj          nvscisyncEOF
);

/**
 * \brief Sets an \ref NvSciSyncFence as a prefence for an
 * NvMediaIEPFeedFrame() %NvSciSyncFence operation.
 *
 * You must call %NvMediaIEPInsertPreNvSciSyncFence() before you call
 * %NvMediaIEPFeedFrame(). The %NvMediaIEPFeedFrame() operation is started only
 * after the expiry of the @a prenvscisyncfence.
 *
 * For example, in this sequence of code:
 * \code
 * nvmstatus = NvMediaIEPInsertPreNvSciSyncFence(handle, prenvscisyncfence);
 * nvmstatus = NvMediaIEPFeedFrame(handle, srcsurf, srcrect, picparams, instanceid);
 * \endcode
 * the %NvMediaIEPFeedFrame() operation is assured to start only after the
 * expiry of @a prenvscisyncfence.
 *
 * You can set a maximum of \ref NVMEDIA_IEP_MAX_PRENVSCISYNCFENCES prefences
 * by calling %NvMediaIEPInsertPreNvSciSyncFence() before %NvMediaIEPFeedFrame().
 * After the call to %NvMediaIEPFeedFrame(), all NvSciSyncFences previously
 * inserted by %NvMediaIEPInsertPreNvSciSyncFence() are removed, and they are not
 * reused for the subsequent %NvMediaIEPFeedFrame() calls.
 *
 * \pre Pre-NvSciSync fence obtained from previous engine in the pipeline
 * \post Pre-NvSciSync fence is set
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Runtime
 *
 * \param[in] encoder An %NvMediaIEP device handle.
 * \param[in] prenvscisyncfence A pointer to %NvSciSyncFence.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if @a encoder is not a valid %NvMediaIEP
 *     handle, or @a prenvscisyncfence is NULL, or if @a prenvscisyncfence was not
 *     generated with an \ref NvSciSyncObj that was registered with @a encoder as
 *     either \ref NVMEDIA_PRESYNCOBJ or \ref NVMEDIA_EOF_PRESYNCOBJ type.
 * - ::NVMEDIA_STATUS_NOT_SUPPORTED if %NvMediaIEPInsertPreNvSciSyncFence()
 *     has already been called at least %NVMEDIA_IEP_MAX_PRENVSCISYNCFENCES times
 *     with the same @a encoder handle before an %NvMediaIEPFeedFrame() call.
 */
NvMediaStatus
NvMediaIEPInsertPreNvSciSyncFence(
    const NvMediaIEP         *encoder,
    const NvSciSyncFence     *prenvscisyncfence
);

/**
 * \brief Gets EOF \ref NvSciSyncFence for an NvMediaIEPFeedFrame() operation.
 *
 * The EOF %NvSciSyncFence associated with an %NvMediaIEPFeedFrame() operation
 * is an %NvSciSyncFence. Its expiry indicates that the corresponding
 * %NvMediaIEPFeedFrame() operation has finished.
 *
 * This function returns the EOF %NvSciSyncFence associated
 * with the last %NvMediaIEPFeedFrame() call. %NvMediaIEPGetEOFNvSciSyncFence()
 * must be called after an %NvMediaIEPFeedFrame() call.
 *
 * For example, in this sequence of code:
 * \code
 * nvmstatus = NvMediaIEPFeedFrame(handle, srcsurf, srcrect, picparams, instanceid);
 * nvmstatus = NvMediaIEPGetEOFNvSciSyncFence(handle, nvscisyncEOF, eofnvscisyncfence);
 * \endcode
 * expiry of @a eofnvscisyncfence indicates that the preceding
 * %NvMediaIEPFeedFrame() operation has finished.
 *
 * \pre NvMediaIEPSetNvSciSyncObjforEOF()
 * \pre NvMediaIEPFeedFrame()
 * \post EOF NvSciSync fence for a submitted task is obtained
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: Runtime
 *
 * \param[in] encoder            An %NvMediaIEP device handle.
 * \param[in] eofnvscisyncobj    An EOF NvSciSyncObj associated with the
 *                                 NvSciSyncFence which is being requested.
 * \param[out] eofnvscisyncfence A pointer to the EOF NvSciSyncFence.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if @a encoder is not a valid %NvMediaIEP
 *         handle, @a eofnvscisyncfence is NULL, or @a eofnvscisyncobj is not
 *         registered with @a encoder as type \ref NVMEDIA_EOFSYNCOBJ or
 *         \ref NVMEDIA_EOF_PRESYNCOBJ.
 * - ::NVMEDIA_STATUS_ERROR if the function was called before
 *         %NvMediaIEPFeedFrame() was called.
 */
NvMediaStatus
NvMediaIEPGetEOFNvSciSyncFence(
    const NvMediaIEP        *encoder,
    NvSciSyncObj      eofnvscisyncobj,
    NvSciSyncFence    *eofnvscisyncfence
);

/**
 * \brief Specifies the \ref NvSciSyncObj to be used for an SOF \ref NvSciSyncFence.
 *
 * This function is not supported.
 *
 * To use NvMediaIEPGetSOFNvSciSyncFence(), the application must call
 * %NvMediaIEPSetNvSciSyncObjforSOF() before it calls NvMediaIEPFeedFrame().
 *
 * %NvMediaIEPSetNvSciSyncObjforSOF() currently may be called only once before
 * each call to %NvMediaIEPFeedFrame(). The application may choose to call this
 * function only once before the first call to %NvMediaIEPFeedFrame().
 *
 * \pre N/A
 * \post N/A
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: N/A
 *
 * \param[in] encoder An %NvMediaIEP device handle.
 * \param[in] nvscisyncSOF A registered NvSciSyncObj which is to be
 *                           associated with SOF \ref NvSciSyncFence.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if @a encoder is NULL, or if @a nvscisyncSOF
 *         is not registered with @a encoder as either type
 *         \ref NVMEDIA_SOFSYNCOBJ or \ref NVMEDIA_SOF_PRESYNCOBJ.
 */
NvMediaStatus
NvMediaIEPSetNvSciSyncObjforSOF(
    const NvMediaIEP        *encoder,
    NvSciSyncObj            nvscisyncSOF
);

/**
 * \brief Gets SOF \ref NvSciSyncFence for an NvMediaIEPFeedFrame() operation.
 *
 * This function is not supported.
 *
 * The SOF %NvSciSyncFence associated with an %NvMediaIEPFeedFrame() operation
 * is an %NvSciSyncFence. Its expiry indicates that the corresponding
 * %NvMediaIEPFeedFrame() operation has started.
 *
 * This function returns the SOF %NvSciSyncFence associated
 * with the last %NvMediaIEPFeedFrame() call. %NvMediaIEPGetSOFNvSciSyncFence()
 * must be called after an %NvMediaIEPFeedFrame() call.
 *
 * \pre N/A
 * \post N/A
 *
 * <b>Considerations for Safety</b>:
 * - Operation Mode: N/A
 *
 * \param[in] encoder            An %NvMediaIEP device handle.
 * \param[in] sofnvscisyncobj    An SOF NvSciSyncObj associated with the
 *                                 NvSciSyncFence which is being requested.
 * \param[out] sofnvscisyncfence A pointer to the SOF NvSciSyncFence.
 *
 * \return ::NvMediaStatus The status of the operation.
 * Possible values are:
 * - ::NVMEDIA_STATUS_OK if the function is successful.
 * - ::NVMEDIA_STATUS_BAD_PARAMETER if @a encoder is not a valid %NvMediaIEP
 *         handle, @a sofnvscisyncfence is NULL, or @a sofnvscisyncobj is not
 *         registered with @a encoder as type \ref NVMEDIA_SOFSYNCOBJ or
 *         \ref NVMEDIA_SOF_PRESYNCOBJ.
 * - ::NVMEDIA_STATUS_ERROR if the function was called before
 *         %NvMediaIEPFeedFrame() was called.
 */
NvMediaStatus
NvMediaIEPGetSOFNvSciSyncFence(
    const NvMediaIEP        *encoder,
    NvSciSyncObj            sofnvscisyncobj,
    NvSciSyncFence          *sofnvscisyncfence
);


/*
 * \defgroup history_nvmedia_iep_nvscisync History
 * Provides change history for the NvMedia IEP NvSciSync API
 *
 * \section history_nvmedia_iep_nvscisync Version History
 *
 * <b> Version 1.0 </b> April 03, 2019
 * - Initial release
 *
 * <b> Version 1.1 </b> July 15, 2019
 * - Add new API NvMediaIEPSetNvSciSyncObjforSOF and NvMediaIEPGetSOFNvSciSyncFence
 *
 */
/** @} <!-- Ends nvmedia_iep_nvscisync_api NvMediaIEP NvSciSync --> */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_IEP_NVSCISYNC_H */
