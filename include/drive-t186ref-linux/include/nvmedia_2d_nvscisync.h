/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: 2D NvSciSync </b>
 *
 * @b Description: This file contains the NvMedia2D and NvSciSync related APIs.
 *
 */

#ifndef NVMEDIA_2D_NVSCISYNC_H
#define NVMEDIA_2D_NVSCISYNC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"
#include "nvscisync.h"
#include "nvmedia_2d.h"

/**
 * @defgroup nvmedia_2d_nvscisync_api Image 2D Processing Synchronization
 *
 * The NvMedia 2D NvSciSync API encompasses all NvMedia2D
 * NvSciSync handling functions.
 *
 * @ingroup nvmedia_image_top
 * @{
 */

/** \brief Major Version number */
#define NVMEDIA_2D_NVSCISYNC_VERSION_MAJOR   1
/** \brief Minor Version number */
#define NVMEDIA_2D_NVSCISYNC_VERSION_MINOR   0

/**
 * NvMedia2DInsertPreNvSciSyncFence API can be called at most
 * NVMEDIA_2D_MAX_PRENVSCISYNCFENCES times before each call to
 * NvMedia2DBlitEx().
 */
#define NVMEDIA_2D_MAX_PRENVSCISYNCFENCES  (3)

/**
 * \brief Returns the version information for the NvMedia 2D NvSciSync library.
 * \param[out] version A pointer to an \ref NvMediaVersion structure
 *                    filled by the 2D NvSciSync library.
 * \return A status code: \ref NVMEDIA_STATUS_OK if successful, or
 *  \ref NVMEDIA_STATUS_BAD_PARAMETER if the pointer is invalid.
 */
NvMediaStatus
NvMedia2DNvSciSyncGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Fills the \ref NvMedia2D specific NvSciSync attributes.
 *
 * This function assumes that @a attrlist is a valid \ref NvSciSyncAttrList.
 *
 * This function sets the public attributes:
 * - \ref NvSciSyncAttrKey_RequiredPerm
 *
 * The application must not set this attribute.
 *
 * \param[in]     i2d       An \ref NvMedia2D device handle.
 * \param[out] attrlist     A pointer to an \ref NvSciSyncAttrList structure
 *                           where NvMedia places NvSciSync attributes.
 * \param[in]    clienttype Indicates the \ref NvSciSyncAttrList requested for
 *                          an NvMedia2D signaler or waiter.
 * \retval  NVMEDIA_STATUS_OK if the function is successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER if @a attrlist is NULL, or any of the
 *           public attributes listed above are already set.
 * \retval  NVMEDIA_STATUS_OUT_OF_MEMORY if there is not enough memory for the
 *           requested operation.
 */
NvMediaStatus
NvMedia2DFillNvSciSyncAttrList(
    const NvMedia2D            *i2d,
    NvSciSyncAttrList          attrlist,
    NvMediaNvSciSyncClientType clienttype
);


/**
 * \brief Register an \ref NvSciSyncObj with \ref NvMedia2D.
 *
 * Every NvSciSyncObj (even duplicate objects) used by NvMedia2D
 * must be registered by a call to this function before it is used.
 * Only the exact same registered NvSciSyncObj can be passed to
 * NvMedia2DSetNvSciSyncObjforEOF(), NvMedia2DGetEOFNvSciSyncFence(), or
 * NvMedia2DUnregisterNvSciSyncObj().
 *
 * For a given NvMedia2D handle,
 * one NvSciSyncObj can be registered as one \ref NvMediaNvSciSyncObjType only.
 * For each NvMediaNvSciSyncObjType a maximum of 16 NvSciSyncObjs can
 * be registered.
 *
 * \param[in] i2d           An \ref NvMedia2D device handle.
 * \param[in] syncobjtype   Determines how @a nvscisync is used by @a i2d.
 * \param[in] nvscisync     The %NvSciSyncObj to be registered with @a i2d.
 * \retval  NVMEDIA_STATUS_OK if the function is successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER if @a i2d is NULL or @a syncobjtype
 *           is not a valid %NvMediaNvSciSyncObjType.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED if @a nvscisync is not a compatible
 *           %NvSciSyncObj which %NvMedia2D can support.
 * \retval  NVMEDIA_STATUS_ERROR if the maximum number of %NvSciScynObjs are
 *           already registered for the given @a syncobjtype, or if
 *           @a nvscisync is already registered with the same @a i2d handle for
 *           a different @a syncobjtype.
 */

NvMediaStatus
NvMedia2DRegisterNvSciSyncObj(
    const NvMedia2D             *i2d,
    NvMediaNvSciSyncObjType     syncobjtype,
    NvSciSyncObj                nvscisync
);

/**
 * \brief Unregisters an \ref NvSciSyncObj with \ref NvMedia2D.
 *
 * Every %NvSciSyncObj registered with %NvMedia2D by
 * NvMedia2DRegisterNvSciSyncObj() must be unregistered before you call
 * NvMedia2DDestroy().
 *
 * Before the application calls this function, it must ensure that any
 * \ref NvMedia2DBlitEx() operation that uses the %NvSciSyncObj has completed.
 * If this function is called while \ref NvSciSyncObj is still in use by any
 * NvMedia2DBlitEx() operation, behavior is undefined.
 *
 * \param[in] i2d       An \ref NvMedia2D device handle.
 * \param[in] nvscisync An %NvSciSyncObj to be unregistered with @a i2d.
 * \retval  NVMEDIA_STATUS_OK if the call is successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER if @a i2d is NULL or @a nvscisync is
 *           not registered with @a i2d.
 * \retval  NVMEDIA_STATUS_ERROR if @a i2d was destroyed before this function
 *           was called.
 */
NvMediaStatus
NvMedia2DUnregisterNvSciSyncObj(
    const NvMedia2D   *i2d,
    NvSciSyncObj      nvscisync
);

/**
 * \brief  Specifies the \ref NvSciSyncObj to be used for a EOF
 *  \ref NvSciSyncFence.
 *
 * To use %NvMedia2DGetEOFNvSciSyncFence(), the application must call
 * %NvMedia2DSetNvSciSyncObjforEOF() before it calls NvMedia2DBlitEx().
 *
 * %NvMedia2DSetNvSciSyncObjforEOF() currently may be called only once before
 * each call to %NvMedia2DBlitEx(). The application may choose to call this
 * function only once before the first call to %NvMedia2DBlitEx().
 *
 * \param[in] i2d           An \ref NvMedia2D device handle.
 * \param[in] nvscisyncEOF  A registered NvSciSyncObj object which is to be
 *                           associated with EOF \ref NvSciSyncFence.
 * \return A status code; \ref NVMEDIA_STATUS_OK if the function is successful,
 *  \ref NVMEDIA_STATUS_BAD_PARAMETER if @a i2d is NULL, or if @a nvscisyncEOF
 *  is not registered with @a i2d as either type \ref NVMEDIA_EOFSYNCOBJ or
 *  type \ref NVMEDIA_EOF_PRESYNCOBJ.
 */
NvMediaStatus
NvMedia2DSetNvSciSyncObjforEOF(
    const NvMedia2D       *i2d,
    NvSciSyncObj          nvscisyncEOF
);

/**
 * \brief Sets an \ref NvSciSyncFence as a prefence for an
 *  NvMedia2DBlitEx() operation.
 *
 * If you use %NvMedia2DInsertPreNvSciSyncFence(), you must call it
 * before calling %NvMedia2DBlitEx().
 * The %NvMedia2DBlitEx() operation is started only after
 * the expiry of the @a prenvscisyncfence.
 *
 * For example, in this sequence of code:
 * \code
 * nvmstatus = NvMedia2DInsertPreNvSciSyncFence(nvm2dhdl, prenvscisyncfence);
 * nvmstatus = NvMedia2DBlitEx(nvm2dhdl, dstsurf, dstrect, srcsurf, srcrect,
 *                      blitparams, blitparamsout);
 * \endcode
 * the %NvMedia2DBlitEx() operation is assured to start only after the expiry
 * of @a prenvscisyncfence.
 *
 * You can set a maximum of NVMEDIA_2D_MAX_PRENVSCISYNCFENCES prefences
 * by calling %NvMedia2DInsertPreNvSciSyncFence() before %NvMedia2DBlitEx().
 * After the call to %NvMedia2DBlitEx(), all NvSciSyncFences previously inserted
 * by %NvMedia2DInsertPreNvSciSyncFence() are removed, and they are not
 * reused for the subsequent %NvMedia2DBlitEx() calls.
 *
 * \param[in] i2d               An \ref NvMedia2D device handle.
 * \param[in] prenvscisyncfence A pointer to an %NvSciSyncFence.
 * \retval  NVMEDIA_STATUS_OK if the function is successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER if @a i2d is not a valid %NvMedia2D
 *           handle, @a prenvscisyncfence is NULL, or @a prenvscisyncfence
 *           was not generated with an \ref NvSciSyncObj that was registered
 *           with @a i2d as either \ref NVMEDIA_PRESYNCOBJ or
 *           \ref NVMEDIA_EOF_PRESYNCOBJ type.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED if %NvMedia2DInsertPreNvSciSyncFence()
 *           has already been called at least
 *           %NVMEDIA_2D_MAX_PRENVSCISYNCFENCES times with the same @a i2d
 *           handle before an %NvMedia2DBlitEx() call.
 */
NvMediaStatus
NvMedia2DInsertPreNvSciSyncFence(
    const NvMedia2D          *i2d,
    const NvSciSyncFence     *prenvscisyncfence
);

/**
 * \brief  Gets an EOF \ref NvSciSyncFence for an NvMedia2DBlitEx() operation.
 *
 * The EOF %NvSciSyncFence associated with an %NvMedia2DBlitEx() operation is an
 * NvSciSyncFence, and its expiry indicates that the corresponding
 * %NvMedia2DBlitEx() operation has finished.
 *
 * %NvMedia2DGetEOFNvSciSyncFence() returns the EOF %NvSciSyncFence associated
 * with the last %NvMedia2DBlitEx() call. If you use
 * %NvMedia2DGetEOFNvSciSyncFence(), you must call it after calling
 * %NvMedia2DBlitEx().
 *
 * For example, in this sequence of code:
 * \code
 * nvmstatus = NvMedia2DBlitEx(nvm2dhdl, dstsurf, dstrect, srcsurf, srcrect,
 *                      blitparams, blitparamsout);
 * nvmstatus = NvMedia2DGetEOFNvSciSyncFence(nvm2dhdl, nvscisyncEOF, eofnvscisyncfence);
 * \endcode
 * expiry of @a eofnvscisyncfence indicates that the preceding
 * %NvMedia2DBlitEx() operation has finished.
 *
 * \param[in] i2d                   An %NvMedia 2D device handle.
 * \param[in] eofnvscisyncobj       EOF %NvSciSyncObj associated with the
 *                                   NvSciSyncFence which is being requested.
 * \param[out] eofnvscisyncfence    A pointer to the EOF NvSciSyncFence.
 *
 * \retval  NVMEDIA_STATUS_OK indicates that the call was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a i2d is not a valid
 *           %NvMedia2D handle, @a eofnvscisyncfence is NULL, or
 *           @a eofnvscisyncobj is not registered with @a i2d as type
 *           \ref NVMEDIA_EOFSYNCOBJ or \ref NVMEDIA_EOF_PRESYNCOBJ.
 * \retval  NVMEDIA_STATUS_ERROR indicates that the function was called before
 *           %NvMedia2DBlitEx() was called.
 */
NvMediaStatus
NvMedia2DGetEOFNvSciSyncFence(
    const NvMedia2D   *i2d,
    NvSciSyncObj      eofnvscisyncobj,
    NvSciSyncFence    *eofnvscisyncfence
);

/*
 * \defgroup history_nvmedia_2d_nvscisync History
 * Provides change history for the NvMedia 2D NvSciSync API
 *
 * \section history_nvmedia_2d_nvscisync Version History
 *
 * <b> Version 1.0 </b> March 6, 2019
 * - Initial release
 *
 */
/** @} <!-- Ends nvmedia_2d_nvscisync_api NvMedia 2D NvSciSync --> */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_2D_NVSCISYNC_H */
