/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: ISP NvSciSync </b>
 *
 * @b Description: This file contains the NvSciSync related APIs for NvMediaISP.
 *
 */

#ifndef NVMEDIA_ISP_NVSCISYNC_H
#define NVMEDIA_ISP_NVSCISYNC_H

#include "nvmedia_core.h"
#include "nvscisync.h"
#include "nvmedia_isp.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup nvmedia_isp_nvscisync_api Image Signal Processing (ISP) Synchronization
 *
 * The NvMedia ISP NvSciSync API encompasses all NvMediaISP
 * NvSciSync handling functions.
 *
 * @ingroup nvmedia_image_top
 * @{
 */

/** \brief Major version number. */
#define NVM_ISP_NVSCISYNC_VERSION_MAJOR   1
/** \brief Minor version number. */
#define NVM_ISP_NVSCISYNC_VERSION_MINOR   0

/**
 * \brief Defines max number of ISP NvSciSync prefences.
 */
#define NVM_ISP_MAX_NVSCISYNC_PREFENCES  (8u)

/**
 * \brief Defines max number of ISP NvSciSync EOF fences.
 */
#define NVM_ISP_MAX_NVSCISYNC_EOFFENCES  (2u)

/**
 * \brief Defines supported ISP EOF fence types.
 */
typedef enum {
    /**
     * \brief ISP Process EOF fence.
     */
    NVM_ISP_NVSCISYNC_PROCESS_EOF_FENCE = 0U,
    /**
     * \brief ISP Stats EOF fence.
     */
    NVM_ISP_NVSCISYNC_STATS_EOF_FENCE
} NvMediaISPEOFfenceType;

/**
 * \brief Returns the version information for the NvMedia ISP NvSciSync APIs.
 *
 * \param[out] version A pointer to an NvMediaVersion structure
 *                     filled by the ISP NvSciSync library.
 *
 * \return NvMediaStatus The completion status of the operation.
 * Possible values are:
 * - NVMEDIA_STATUS_OK if successful.
 * - NVMEDIA_STATUS_BAD_PARAMETER if the pointer is invalid.
 */
NvMediaStatus
NvMediaISPNvSciSyncGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Fills the NvMediaISP specific NvSciSync attributes.
 *
 * This function sets the public attribute:
 * - NvSciSyncAttrKey_RequiredPerm
 *
 * The application must not set this attribute.
 *
 * \param[in] nvmisp An NvMedia ISP %NvMediaISP handle.
 * \param[out] attrlist A pointer to an NvSciSyncAttrList structure where
 *                NvMedia places NvSciSync attributes.
 * \param[in] clientType Indicates whether the @a attrlist is requested for
 *                an %NvMediaISP signaler or an %NvMediaISP waiter.
 *
 * \return NvMediaStatus The status of the operation.
 * - NVMEDIA_STATUS_OK if the function is successful.
 * - NVMEDIA_STATUS_BAD_PARAMETER if any of the input arguments were
 *         invalid.
 * - NVMEDIA_STATUS_ERROR if any other error happens.
 */
NvMediaStatus
NvMediaISPFillNvSciSyncAttrList(
    const NvMediaISP        *nvmisp,
    NvSciSyncAttrList attrlist,
    NvMediaNvSciSyncClientType clientType
);

/**
 * \brief Registers an NvSciSyncObj with NvMediaISP.
 *
 * Every NvSciSyncObj(even duplicate objects) used by %NvMediaISP
 * must be registered by a call to this function before it is used.
 * Only the exact same registered NvSciSyncObj can be passed to
 * NvMediaISPSetNvSciSyncObjforEOF(), NvMediaISPGetEOFNvSciSyncFence(), or
 * NvMediaISPUnregisterNvSciSyncObj().
 *
 * For a given %NvMediaISP handle,
 * one NvSciSyncObj can be registered  as one NvMediaNvSciSyncObjType only.
 * For each NvMediaNvSciSyncObjType, a maximum of 16 NvSciSyncObjs can
 * be registered.
 *
 * \param[in] nvmisp         An NvMedia ISP %NvMediaISP handle.
 * \param[in] syncObjType Determines how @a nvsciSync is used by @a nvmisp.
 * \param[in] nvsciSync   The %NvSciSyncObj to be registered with @a nvmisp.
 * \param[in] fenceType   A handle to \ref NvMediaISPEOFfenceType if
 *                        @a syncObjType is of type NVMEDIA_EOFSYNCOBJ or
 *                        NVMEDIA_EOF_PRESYNCOBJ. For
 *                        other types of %NvMediaNvSciSyncObjType, the value
 *                        is set to 0.
 *
 * \return NvMediaStatus The status of the operation.
 * Possible values are:
 * - NVMEDIA_STATUS_OK if the function is successful.
 * - NVMEDIA_STATUS_BAD_PARAMETER if @a nvmisp is NULL or
 *         @a syncObjType is not a valid %NvMediaNvSciSyncObjType.
 * - NVMEDIA_STATUS_NOT_SUPPORTED if @a nvsciSync is not a
 *         compatible %NvSciSyncObj which %NvMediaISP can support.
 * - NVMEDIA_STATUS_ERROR if any other error happens.
 */
NvMediaStatus
NvMediaISPRegisterNvSciSyncObj(
    NvMediaISP              *nvmisp,
    NvMediaNvSciSyncObjType syncObjType,
    NvSciSyncObj            nvsciSync,
    NvMediaISPEOFfenceType  fenceType
);

/**
 * \brief Unregisters an NvSciSyncObj with %NvMediaISP.
 *
 * Every %NvSciSyncObj registered with %NvMediaISP by
 * NvMediaISPRegisterNvSciSyncObj() must be unregistered before calling
 * NvMediaISPDestroy().
 *
 * Before the application calls this function, it must ensure that any
 * NvMediaISPProcess() operation that uses NvSciSyncObj has completed.
 * If this function is called while NvSciSyncObj is still in use by any
 * NvMediaISPProcess() operation, the behavior is undefined.
 *
 * \param[in] nvmisp       An NvMedia ISP %NvMediaISP handle.
 * \param[in] nvsciSync An NvSciSyncObj to be unregistered with @a nvmisp.
 *
 * \return NvMediaStatus The status of the operation.
 * Possible values are:
 * - NVMEDIA_STATUS_OK if the function is successful.
 * - NVMEDIA_STATUS_BAD_PARAMETER if @a nvmisp is NULL, or
 *         @a nvsciSync is not registered with @a nvmisp.
 * - NVMEDIA_STATUS_ERROR if any other error happens.
 */
NvMediaStatus
NvMediaISPUnregisterNvSciSyncObj(
    NvMediaISP   *nvmisp,
    NvSciSyncObj nvsciSync
);

/**
 * \brief Specifies the NvSciSyncObj to be used for an EOF NvSciSyncFence.
 *
 * To use NvMediaISPGetEOFNvSciSyncFence(), the application must call
 * %NvMediaISPSetNvSciSyncObjforEOF() before it calls NvMediaISPProcess().
 *
 * %NvMediaISPSetNvSciSyncObjforEOF() currently may be called only once before
 * each call to %NvMediaISPProcess(). The application may choose to call this
 * function only once before the first call to %NvMediaISPProcess().
 *
 * \param[in] nvmisp          An NvMedia ISP %NvMediaISP handle.
 * \param[in] nvsciSyncEOF A registered NvSciSyncObj which is to be
 *                           associated with EOF %NvSciSyncFence.
 * \param[in] eofType      A handle to \ref NvMediaISPEOFfenceType to be used
 *                           with @a nvsciSyncEOF.
 *
 * \return NvMediaStatus The status of the operation.
 * Possible values are:
 * - NVMEDIA_STATUS_OK if the function is successful.
 * - NVMEDIA_STATUS_BAD_PARAMETER if @a nvmisp is NULL, or if @a nvsciSyncEOF
 *         is not registered with @a nvmisp as either type
 *         NVMEDIA_EOFSYNCOBJ or NVMEDIA_EOF_PRESYNCOBJ.
 */
NvMediaStatus
NvMediaISPSetNvSciSyncObjforEOF(
    NvMediaISP             *nvmisp,
    NvSciSyncObj           nvsciSyncEOF,
    NvMediaISPEOFfenceType eofType
);

/**
 * \brief Sets an NvSciSyncFence as a prefence for an
 * NvMediaISPProcess() operation.
 *
 * You must call %NvMediaISPInsertPreNvSciSyncFence() before you call
 * %NvMediaISPProcess(). The %NvMediaISPProcess() operation is started only
 * after the expiry of the @a preNvsciSyncFence.
 *
 * For example, in this sequence of code:
 * \code
 * nvmstatus = NvMediaISPInsertPreNvSciSyncFence(nvmisp, preNvsciSyncFence);
 * nvmstatus = NvMediaISPProcess(nvmisp, params);
 * \endcode
 * the %NvMediaISPProcess() operation is assured to start only after the
 * expiry of @a preNvsciSyncFence.
 *
 * You can set a maximum of \ref NVM_ISP_MAX_NVSCISYNC_PREFENCES prefences
 * by calling %NvMediaISPInsertPreNvSciSyncFence() before calling
 * %NvMediaISPProcess(). After the call to %NvMediaISPProcess(), all
 * NvSciSyncFences previously inserted by %NvMediaISPInsertPreNvSciSyncFence()
 * are removed, and they are not reused for the subsequent %NvMediaISPProcess()
 * calls.
 *
 * \param[in] nvmisp               An NvMedia ISP %NvMediaISP handle.
 * \param[in] preNvsciSyncFence A pointer to %NvSciSyncFence.
 *
 * \return NvMediaStatus The status of the operation.
 * Possible values are:
 * - NVMEDIA_STATUS_OK if the function is successful.
 * - NVMEDIA_STATUS_BAD_PARAMETER if @a nvmisp is not a valid %NvMediaISP
 *     handle, or @a preNvsciSyncFence is NULL, or if @a preNvsciSyncFence was not
 *     generated with an NvSciSyncObj that was registered with @a nvmisp as
 *     either NVMEDIA_PRESYNCOBJ or NVMEDIA_EOF_PRESYNCOBJ type.
 * - NVMEDIA_STATUS_ERROR if any other error happens.
 */
NvMediaStatus
NvMediaISPInsertPreNvSciSyncFence(
    NvMediaISP           *nvmisp,
    const NvSciSyncFence *preNvsciSyncFence
);

/**
 * \brief Gets EOF NvSciSyncFence for an NvMediaISPProcess() operation.
 *
 * The EOF %NvSciSyncFence associated with an %NvMediaISPProcess() operation is
 * an NvSciSyncFence. Its expiry indicates that the corresponding
 * %NvMediaISPProcess() operation has finished.
 *
 * This function returns the EOF %NvSciSyncFence associated with the last
 * %NvMediaISPProcess() call. %NvMediaISPGetEOFNvSciSyncFence() must be
 * called after an %NvMediaISPProcess() call.
 *
 * For example, in this sequence of code:
 * \code
 * nvmstatus = NvMediaISPProcess(nvmisp, params);
 * nvmstatus = NvMediaISPGetEOFNvSciSyncFence(nvmisp, eofNvsciSyncObj, eofType,
 *                                           eofNvsciSyncFence);
 * \endcode
 * - expiry of @a eofNvsciSyncFence for the @a eoftype
 *      \ref NVM_ISP_NVSCISYNC_PROCESS_EOF_FENCE indicates that the preceding
 *      %NvMediaISPProcess() operation has finished.
 * - expiry of @a eofNvsciSyncFence for the @a eoftype
 *      \ref NVM_ISP_NVSCISYNC_STATS_EOF_FENCE indicates that the stats data
 *      are available from the preceding %NvMediaISPProcess() operation.
 *
 * \param[in] nvmisp                An NvMedia ISP %NvMediaISP handle.
 * \param[in] eofNvsciSyncObj    An EOF NvSciSyncObj associated with the
 *                                  %NvSciSyncFence which is being requested.
 * \param[in] eofType            An EOF fence type \ref NvMediaISPEOFfenceType
 *                                 to request for.
 * \param[out] eofNvsciSyncFence A pointer to EOF %NvSciSyncFence.
 *
 * \return NvMediaStatus The status of the operation.
 * Possible values are:
 * - NVMEDIA_STATUS_OK if the function is successful.
 * - NVMEDIA_STATUS_BAD_PARAMETER if @a nvmisp is not a valid %NvMediaISP handle
 *         or @a eofnvsciSyncfence is NULL, or @a eofNvsciSyncObj is not
 *         registered with @a nvmisp as type NVMEDIA_EOFSYNCOBJ or
 *         NVMEDIA_EOF_PRESYNCOBJ.
 * - NVMEDIA_STATUS_ERROR if any other error happens.
 */
NvMediaStatus
NvMediaISPGetEOFNvSciSyncFence(
    const NvMediaISP             *nvmisp,
    NvSciSyncObj           eofNvsciSyncObj,
    NvMediaISPEOFfenceType eofType,
    NvSciSyncFence         *eofNvsciSyncFence
);

/*
 * \defgroup history_nvmedia_isp_nvsciSync History
 * Provides change history for the NvMedia ISP NvSciSync API
 *
 * \section history_nvmedia_isp_nvsciSync Version History
 *
 * <b> Version 1.0 </b> March 15, 2019
 * - Initial release
 *
 */
/** @} <!-- Ends nvmedia_isp_nvscisync_api NvMediaISP NvSciSync --> */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_ISP_NVSCISYNC_H */
