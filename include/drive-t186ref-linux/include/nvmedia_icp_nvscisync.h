/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: ICP NvSciSync </b>
 *
 * @b Description: This file contains the NvMediaICP and NvSciSync related APIs.
 *
 */

#ifndef NVMEDIA_ICP_NVSCISYNC_H
#define NVMEDIA_ICP_NVSCISYNC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"
#include "nvscisync.h"
#include "nvmedia_icp.h"

/**
 * @defgroup nvmedia_icp_nvscisync_api Image Capture Synchronization
 *
 * The NvMedia ICP NvSciSync API encompasses all NvMediaICP
 * NvSciSync handling functions.
 *
 * @ingroup nvmedia_image_top
 * @{
 */

/** \brief Major version number. */
#define NVMEDIA_ICP_NVSCISYNC_VERSION_MAJOR   1
/** \brief Minor version number. */
#define NVMEDIA_ICP_NVSCISYNC_VERSION_MINOR   0

/** Maximum number of times NvMediaICPInsertPreNvSciSyncFence() may be called
 before each NvMediaICPFeedImageGroup call. */
#define NVMEDIA_ICP_MAX_PRENVSCISYNCFENCES  (3)

/**
 * \brief Returns version information for the NvMedia ICP NvSciSync library.
 * \param[out] version  A pointer to a structure to be filled with version
 *                       information.
 * \return  A status code; \ref NVMEDIA_STATUS_OK if the call was successful, or
 * \ref NVMEDIA_STATUS_BAD_PARAMETER if @a version was invalid.
 */
NvMediaStatus
NvMediaICPNvSciSyncGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Fills the \ref NvMediaICP specific NvSciSync attributes.
 *
 * This function assumes that @a attrlist is a valid \ref NvSciSyncAttrList.
 *
 * This function sets the public attributes:
 * - \ref NvSciSyncAttrKey_RequiredPerm
 *
 * The application must not set this attribute.
 *
 * \param[in]     icp           An NvMedia ICP device handle.
 * \param[out]    attrlist      A pointer to a structure to be filled with
 *                               the NvSciSync attributes.
 * \param[in]     clienttype    Indicates whether @a attrlist is
 *                               requested for an NvMediaICP signaler or
 *                               an NvMediaICP waiter.
 * \retval  NVMEDIA_STATUS_OK indicates that the call was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a attrlist was NULL, or
 *  any of the public attributes listed above were already set.
 * \retval  NVMEDIA_STATUS_OUT_OF_MEMORY indicates that there was not enough
 *  memory for the requested operation.
 */
NvMediaStatus
NvMediaICPFillNvSciSyncAttrList(
    NvMediaICP                 *icp,
    NvSciSyncAttrList          attrlist,
    NvMediaNvSciSyncClientType clienttype
);

/**
 * \brief Registers an \ref NvSciSyncObj with \ref NvMediaICP.
 *
 * Every \ref NvSciSyncObj used by NvMediaICP (including duplicate objects)
 * must be registered in advance by a call to this function.
 * Only the exact same registered NvSciSyncObj can be used as an argument in
 * NvMediaICPSetNvSciSyncObjforEOF(), NvMediaICPGetEOFNvSciSyncFence(),
 * and NvMediaICPUnregisterNvSciSyncObj().
 * One NvSciSyncObj can be registered as only one NvMediaNvSciSyncObjType
 * (see nvmedia_core.h) for a given NvMediaICP handle. A maximum of 16
 * NvSciSyncObj objects can be registered for each NvMediaSciSyncObjType.
 *
 * \param[in] icp         An NvMedia ICP device handle.
 * \param[in] syncobjtype Specifies how @a syncobj is to be used by ICP.
 * \param[in] syncobj     An NvSciSyncObj to be registered with ICP.
 * \param[in] channelId   Channel ID for HDR capture.
 *
 * \retval  NVMEDIA_STATUS_OK indicates that the call was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a icp was NULL, or
 *  @a syncobjtype was not a valid NvMediaSciSyncObjType.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that @a syncobj was not a
 *  compatible NvSciSyncObj which NvMediaICP could support.
 * \retval  NVMEDIA_STATUS_ERROR indicates that the maximum allowed number of
 *  NvSciScynObj objects were already registered for the given @a syncobjtype,
 *  or that @a syncobj was already registered with the same ICP handle for a
 *  different @a syncobjtype.
 */
NvMediaStatus
NvMediaICPRegisterNvSciSyncObj(
    NvMediaICP              *icp,
    NvMediaNvSciSyncObjType syncobjtype,
    NvSciSyncObj            syncobj,
    uint32_t                channelId
);

/**
 * \brief Unregisters an \ref NvSciSyncObj with \ref NvMediaICP.
 *
 * Every NvSciSyncObj registered with NvMediaICP
 * must be unregistered before you call NvMediaICPDestroy().
 * Before the application calls this function, it must ensure that the
 * \ref NvMediaICPGetImageGroup operation which uses the NvSciSyncObj being
 * unregistered has completed. If this function is called while the
 * NvSciSyncObj is still in use by an ICP operation, the result is undefined.
 *
 * \param[in] icp       An NvMedia ICP device handle.
 * \param[in] syncobj   An NvSciSyncObj to be unregistered with ICP.
 * \param[in] channelId A channel ID for HDR capture.
 *
 * \retval  NVMEDIA_STATUS_OK indicates that the call was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a icp was NULL, or that
 *  @a syncobj was not registered.
 * \retval  NVMEDIA_STATUS_ERROR indicates that @a icp has been destroyed.
 */
NvMediaStatus
NvMediaICPUnregisterNvSciSyncObj(
    NvMediaICP        *icp,
    NvSciSyncObj      syncobj,
    uint32_t          channelId
);

/**
 * \brief Set \ref NvSciSyncObj to be used for a EOF \ref NvSciSyncFence.
 *
 * To use NvMediaICPGetEOFNvSciSyncFence(), the application must call
 * NvMediaICPSetNvSciSyncObjforEOF() before NvMediaICPFeedImageGroup().
 *
 * This function currently may be called only once before each call to
 * %NvMediaICPFeedImageGroup(). The application may choose to call this
 * function only once before the first call to %NvMediaICPFeedImageGroup().
 *
 * \param[in] icp          An NvMediaICP device handle.
 * \param[in] nvscisyncEOF A preregistered NvSciSyncObj to be
 *                           associated with EOF NvSciSyncFence.
 * \param[in] channelId    A channel ID for HDR capture.
 *
 * \retval  NVMEDIA_STATUS_OK indicates that the call was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a icp was NULL, or that
 *  @a nvscisyncEOF was not registered with ICP as either type
 *  \ref NVMEDIA_EOFSYNCOBJ or type \ref NVMEDIA_EOF_PRESYNCOBJ.
 */
NvMediaStatus
NvMediaICPSetNvSciSyncObjforEOF(
    NvMediaICP     *icp,
    NvSciSyncObj    nvscisyncEOF,
    uint32_t        channelId
);

/**
 * \brief Gets EOFNvSciSyncFence for a NvMediaICPFeedImageGroup()
 * operation.
 *
 * The EOFNvSciSyncFence associated with an %NvMediaICPFeedImageGroup()
 * operation is an \ref NvSciSyncFence. Its expiry indicates that the
 * corresponding frame capture operation has finished.
 * NvMediaICPGetEOFNvSciSyncFence() returns the EOFNvSciSyncFence associated
 * with the last %NvMediaICPFeedImageGroup() call.
 * %NvMediaICPGetEOFNvSciSyncFence() must be called after
 * %NvMediaICPFeedImageGroup(). For example, in the below sequence of code:
 * the last %NvMediaICPFeedImageGroup() call. %NvMediaICPGetEOFNvSciSyncFence()
 * must be called after an %NvMediaICPFeedImageGroup() call. For example,
 * in the below sequence of code,
 *
 * \code
 * nvmstatus = NvMediaICPFeedImageGroup(icp, imageGrp, timeout);
 *
 * nvmstatus = NvMediaICPGetEOFNvSciSyncFence(icp, nvscisyncEOF, eofnvscisyncfence);
 * \endcode
 *
 * expiry of @a eofnvscisyncfence indicates that the preceding frame capture
 * has finished.
 *
 * \param[in]  icp                  An NvMedia ICP device handle.
 * \param[in]  eofnvscisyncobj      EOF \ref NvSciSyncObj associated with the
 *                                   NvSciSyncFence being requested.
 * \param[in]  channelId            Channel ID for HDR capture.
 * \param[out] eofnvscisyncfence    A pointer to eof NvSciSyncFence.
 *
 * \retval  NVMEDIA_STATUS_OK indicates that the call was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a icp was not a valid
 *  \ref NvMediaICP handle, or that @a eofnvscisyncfence was NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that the function was called before
 *  %NvMediaICPFeedImageGroup().
 */
NvMediaStatus
NvMediaICPGetEOFNvSciSyncFence(
    NvMediaICP          *icp,
    const NvSciSyncObj  eofnvscisyncobj,
    uint32_t            channelId,
    NvSciSyncFence      *eofnvscisyncfence
);

/**
 * \brief Sets the \ref NvSciSyncObj to be used for a SOF \ref NvSciSyncFence.
 *
 * To use NvMediaICPGetSOFNvSciSyncFence(), the application must call
 * this function before calling NvMediaICPFeedImageGroup().
 *
 * This function currently may be called only once before a call to
 * %NvMediaICPFeedImageGroup(). The application may call this function only once
 * before the first call to %NvMediaICPFeedImageGroup().
 *
 * \param[in] icp          An NvMedia ICP device handle.
 * \param[in] nvscisyncSOF A preregistered NvSciSyncObj to be associated with
 *                          SOFfence.
 * \param[in] channelId    Channel ID for HDR capture.
 * \retval  NVMEDIA_STATUS_OK indicates that the call was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a icp was NULL, or that
 *  @a nvscisyncSOF was not registered with ICP as either type
 *  NVMEDIA_SOFSYNCOBJ or type NVMEDIA_SOF_PRESYNCOBJ.
 */
NvMediaStatus
NvMediaICPSetNvSciSyncObjforSOF(
    NvMediaICP      *icp,
    NvSciSyncObj    nvscisyncSOF,
    uint32_t        channelId
);


/**
 * \brief Gets SOF \ref NvSciSyncFence for an NvMediaFeedImageGroup()
 * operation.
 *
 * The SOF \ref NvSciSyncFence associated with an NvMediaICPFeedImageGroup()
 * operation is an NvSciSyncFence. Its expiry indicates that the corresponding
 * frame capture operation has started.
 * NvMediaICPGetSOFNvSciSyncFence() returns the SOFNvSciSyncFence associated
 * with the last %NvMediaICPFeedImageGroup() call.
 * You must call %NvMediaICPGetSOFNvSciSyncFence() after
 * %NvMediaICPFeedImageGroup(). For example, in the below sequence of code:
 *
 * \code
 * nvmstatus = NvMediaICPFeedImageGroup(icp, imageGrp, timeout);
 * nvmstatus = NvMediaICPGetSOFNvSciSyncFence(icp, nvscisyncSOF, sofnvscisyncfence);
 * \endcode
 *
 * expiry of @a sofnvscisyncfence indicates that the preceding camera capture
 * operation has started.
 *
 * \param[in] icp                   An NvMedia ICP device handle.
 * \param[in] sofnvscisyncobj       The SOF NvSciSyncObj associated with
 *                                   the NvSciSyncFence that is being requested.
 * \param[in] channelId             The channel ID for HDR capture.
 * \param[out] sofnvscisyncfence    A pointer to an NvSciSyncFence where
 *                                   SOFfence will be stored.
 *
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a icp is not a valid
 *           NvMediaICP handle, @a sofnvscisyncfence was NULL, or
 *           @a sofnvscisyncobj was not registered with icp as type
 *           \ref NVMEDIA_SOFSYNCOBJ or type \ref NVMEDIA_SOF_PRESYNCOBJ.
 * \retval  NVMEDIA_STATUS_ERROR indicates that this function was called before
 *           %NvMediaICPFeedImageGroup().
 */
NvMediaStatus
NvMediaICPGetSOFNvSciSyncFence(
    NvMediaICP          *icp,
    const NvSciSyncObj  sofnvscisyncobj,
    uint32_t            channelId,
    NvSciSyncFence      *sofnvscisyncfence
);


/**
 * \brief Sets an \ref NvSciSyncFence as a prefence for an
 * NvMediaICPFeedImageGroup() operation.
 *
 * NvMediaICPInsertPreNvSciSyncFence() must be called before
 * %NvMediaICPFeedImageGroup() is called. The following capture operation
 * is assured to be started only after the expiry of
 * the @a prenvscisyncfence. For example, in the below sequence of code:
 *
 * \code
 * nvmstatus = NvMediaICPInsertPreNvSciSyncFence(icp, prenvscisyncfence);
 * nvmstatus = NvMediaICPFeedImageGroup(icp, imageGrp, timeout);
 * \endcode
 *
 * The image capture operation may start only after the expiry of
 * @a prenvscisyncfence.
 *
 * You can set a maximum of \ref NVMEDIA_ICP_MAX_PRENVSCISYNCFENCES prefences
 * by calling %NvMediaICPInsertPreNvSciSyncFence() before calling
 * %NvMediaICPFeedImageGroup(). After the call to %NvMediaICPFeedImageGroup(),
 * all of the NvSciSyncFence objects previously inserted
 * by %NvMediaICPInsertPreNvSciSyncFence() are removed, and are not
 * reused for the subsequent %NvMediaICPFeedImageGroup() calls.
 *
 * \param[in] icp               NvMedia ICP device handle.
 * \param[in] prenvscisyncfence A pointer to \ref NvSciSyncFence.
 * \param[in] channelId         Channel ID for HDR capture.
 *
 * \retval  NVMEDIA_STATUS_OK indicates that the call was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a icp was not a valid
 *  NvMediaICP handle, or that @a prenvscisyncfence was NULL.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that
 *  %NvMediaICPInsertPreNvSciSyncFence() has already been called
 *  \ref NVMEDIA_ICP_MAX_PRENVSCISYNCFENCES or more times with the same
 *  NvMediaICP handle before a call to %NvMediaICPFeedImageGroup().
 */
NvMediaStatus
NvMediaICPInsertPreNvSciSyncFence(
    NvMediaICP               *icp,
    const NvSciSyncFence     *prenvscisyncfence,
    uint32_t                 channelId
);


/*
 * \defgroup history_nvmedia_icp_nvscisync History
 * Provides change history for the NvMedia ICP NvSciSync API
 *
 * \section history_nvmedia_icp_nvscisync Version History
 *
 * <b> Version 1.0 </b> March 11, 2019
 * - Initial release
 *
 */
/** @} <!-- Ends nvmedia_icp_nvscisync_api NvMedia ICP NvSciSync --> */
#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_ICP_NVSCISYNC_H */
