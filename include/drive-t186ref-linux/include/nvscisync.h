/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
/**
 * @file
 *
 * @brief <b> NVIDIA Software Communications Interface (SCI) : NvSciSync </b>
 *
 * The NvSciSync library allows applications to manage synchronization
 * objects which coordinate when sequences of operations begin and end.
 */
#ifndef INCLUDED_NVSCISYNC_H
#define INCLUDED_NVSCISYNC_H

/**
 * @defgroup nvsci_sync Synchronization APIs
 *
 * @ingroup nvsci_group_stream
 * @{
 *
 * The NvSciSync library allows applications to manage synchronization
 * objects which coordinate when sequences of operations begin and end.
 *
 * The following constants are defined and have type @c unsigned @c int:
 * * @c NvSciSyncMajorVersion
 * * @c NvSciSyncMinorVersion
 *
 * In C and C++ these constants are guaranteed to be defined as global
 * const variables.

 * Upon each new release of NvSciSync:
 * * If the new release changes behavior of NvSciSync that could in any way
 *   prevent an application compliant with previous versions of this
 *   specification from functioning correctly, or that could prevent
 *   interoperability with earlier versions of NvSciSync, the major version is
 *   increased and the minor version is reset to zero.
 *   @note This is typically done only after a deprecation period.
 * * Otherwise, if the new release changes NvSciSync in any application
 *   observable manner (e.g., new features, or bug fixes), the major version is
 *   kept the same and the minor version is increased.
 * * Otherwise, the major and minor version are both kept the same.
 *
 * This version of this specification corresponds to NvSciSync version 1.0
 * (major version 1, minor version 0).
 *
 * Different processes using the NvSciSync inter-process APIs may use different
 * minor versions of NvSciSync within the same major version, provided that if
 * a process uses a feature newer than another processor's NvSciSync version,
 * the latter process does not import an unreconciled NvSciSyncAttrList (directly
 * or indirectly) from the former process.
 *
 * In general, an NvSciSync API call will not return an error code if it has
 * caused any side effects other than allocating resources and subsequently
 * freeing those resources.
 *
 * In general, unless specified otherwise, if a NULL pointer is passed to an
 * NvSciSync API call, the API call will either return ::NvSciError_BadParameter
 * or (if there are other applicable error conditions as well) an error code
 * corresponding to another error.
 *
 * Each NvSciSyncAttrList is either unreconciled or reconciled.
 * It is unreconciled if it was:
 * - Created by NvSciSyncAttrListCreate(),
 * - Created by a failed NvSciSyncAttrListReconcile() call,
 * - Created by a failed NvSciSyncAttrListReconcileAndObjAlloc() call, or
 * - An import of an export of one or more unreconciled NvSciSyncAttrLists.
 *
 * It is reconciled if it was:
 * - Created by a successful NvSciSyncAttrListReconcile() call,
 * - Provided by NvSciSyncObjGetAttrList(), or
 * - An import of an export of another reconciled NvSciSyncAttrList.
 */

/**
 * \page nvscisync_page_blanket_statements NvSciSync blanket statements
 * \section nvscisync_in_out_params Input/Output parameters
 * - NvSciSyncFence passed as input parameter to an API is valid input if
 * it was first initialized to all zeroes
 * or modified by any successful NvSciSync API accepting NvSciSyncFence.
 * - NvSciSyncObj passed as an input parameter to an API is valid input if it
 * is returned from a successful call to NvSciSyncObjAlloc(), NvSciSyncObjDup(),
 * NvSciSyncObjIpcImport(), NvSciSyncIpcImportAttrListAndObj() or
 * NvSciSyncAttrListReconcileAndObjAlloc() and has not yet been
 * deallocated using NvSciSyncObjFree().
 * - NvSciSyncCpuWaitContext passed as an input parameter to an API is valid
 * input if it is returned from a successful call to
 * NvSciSyncCpuWaitContextAlloc() and has not been deallocated using
 * NvSciSyncCpuWaitContextFree().
 * - NvSciSyncModule passed as input parameter to an API is valid input if it is
 * returned from a successful call to NvSciSyncModuleOpen() and has not yet
 * been deallocated using NvSciSyncModuleClose().
 * - NvSciIpcEndpoint passed as input parameter to an API is valid if it is
 * obtained from successful call to NvSciIpcOpenEndpoint() and has not yet been
 * freed using NvSciIpcCloseEndpoint().
 * - Unreconciled NvSciSyncAttrList is valid if it is obtained from successful
 * call to NvSciSyncAttrListCreate() or if it is obtained from successful call to
 * NvSciSyncAttrListClone() where input to NvSciSyncAttrListClone() is valid
 * unreconciled NvSciSyncAttrList or if it is obtained from successful call to
 * NvSciSyncAttrListIpcImportUnreconciled() and has not been deallocated using
 * NvSciSyncAttrListFree().
 * - Reconciled NvSciSyncAttrList is valid if it is obtained from successful call
 * to NvSciSyncAttrListReconcile() or if it is obtained from successful call to
 * NvSciSyncAttrListClone() where input to NvSciSyncAttrListClone() is valid
 * reconciled NvSciSyncAttrList or if it is obtained from successful call to
 * NvSciSyncAttrListIpcImportReconciled() and has not been deallocated using
 * NvSciSyncAttrListFree() or has been obtained from a successful call to
 * NvSciSyncObjGetAttrList() and the input NvSciSyncObj to this call has not
 * yet been deallocated using NvSciSyncObjFree().
 *
 * \section nvscisync_out_params Output parameters
 * - In general, output parameters are passed by reference through pointers.
 * Also, since a null pointer cannot be used to convey an output parameter, API
 * functions typically return an error code if a null pointer is supplied for a
 * required output parameter unless otherwise stated explicitly. Output
 * parameter is valid only if error code returned by an API is
 * NvSciError_Success unless otherwise stated explicitly.
 *
 * \section nvscisync_concurrency Concurrency
 * - Every individual function can be called concurrently with itself
 * without any side-effects unless otherwise stated explicitly in
 * the interface specifications.
 * - The conditions for combinations of functions that cannot be called
 * concurrently or calling them concurrently leads to side effects are
 * explicitly stated in the interface specifications.
 *
 * \section nvscisync_fence_states Fence states
 * - A zero initialized NvSciSyncFence or one fed to NvSciSyncFenceClear()
 *   becomes cleared.
 * - NvSciSyncFence becomes not cleared if it is modified by a successful
 * NvSciSyncObjGenerateFence() or NvSciSyncFenceDup().
 * - NvSciSyncFence filled by successful NvSciSyncIpcExportFence() is cleared
 * if and only if the input fence descriptor was created from a cleared
 * NvSciSyncFence.
 *
 * \implements{14901104}
 */

#if !defined (__cplusplus)
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#endif

#include <nvscierror.h>
#include <nvsciipc.h>

#if defined (__cplusplus)
extern "C"
{
#endif

/**
 * \brief NvSciSync major version number.
 *
 * \implements{14971362}
 */
static const uint32_t NvSciSyncMajorVersion = 2U;

/**
 * \brief NvSciSync minor version number.
 *
 * \implements{14971368}
 */
static const uint32_t NvSciSyncMinorVersion = 0U;

/**
 * Maximum supported timeout value.
 * NvSciSyncFenceWait() can wait for at most this many microseconds.
 * This value also corresponds to infinite timeout.
 */
static const int64_t NvSciSyncFenceMaxTimeout = (0x7fffffffffffffff / 1000);

/**
 * \brief Represents an instance of the NvSciSyncModule. Any NvSciSyncAttrList
 * created or imported using a particular NvSciSyncModule is bound to that
 * module instance, along with any NvSciSyncObjs created or imported using those
 * NvSciSyncAttrLists and any NvSciSyncFences created or imported using those
 * NvSciSyncObjs.
 *
 * @note For any NvSciSync API call that has more than one input of type
 * NvSciSyncModule, NvSciSyncAttrList, NvSciSyncObj, and/or NvSciSyncFence, all
 * such inputs must agree on the NvSciSyncModule instance.
 */
typedef struct NvSciSyncModuleRec* NvSciSyncModule;

/**
 * \brief Represents the right to perform a CPU wait on an NvSciSyncFence. It
 * holds resources necessary to perform a CPU wait using NvSciSyncFenceWait().
 * It can be used to wait on NvSciSyncFence(s) associated with the same
 * NvSciSyncModule that this NvSciSyncCpuWaitContext is associated with.
 * An NvSciSyncCpuWaitContext can be used to wait on only one
 * NvSciSyncFence at a time. However, a single NvSciSyncCpuWaitContext
 * can be used to wait on different NvSciSyncFences at different
 * instants of time.
 */
typedef struct NvSciSyncCpuWaitContextRec* NvSciSyncCpuWaitContext;

/**
 * \brief Defines the opaque NvSciSyncFence.
 *
 * This structure describes a synchronization primitive snapshot
 * that must be reached to release the waiters.
 *
 * Unlike NvSciSyncAttrList and NvSciSyncObj objects, applications are
 * responsible for the memory management of NvSciSyncFences.
 *
 * Every NvSciSyncFence must be initialized to all zeros after its storage
 * is allocated before the first time it is passed to any NvSciSync API
 * calls. NvSciSyncFenceInitializer can be used for this, or
 * memset(), or any other mechanism for zeroing memory.
 *
 * Every NvSciSyncFence not in a cleared state holds a reference to
 * the NvSciSyncObj it is related to, preventing that NvSciSyncObj
 * from being deleted. It also contains
 * id and value of the synchronization primitive corresponding
 * to the desired snapshot.
 */
#if (NV_IS_SAFETY == 0)
/**
 * In case if the corresponding NvSciSyncObj supports timestamps,
 * this structure might also contain information about memory location
 * of the timestamp of event unblocking the NvSciSyncFence.
 */
#endif
/**
 * \implements{11272328}
 */
typedef struct {
    uint64_t payload[6];
} NvSciSyncFence;

/**
 * \brief Defines the value used to zero-initialize the NvSciSyncFence object.
 *  An NvSciSyncFence that is all zeroes is in a cleared state.
 *
 * \implements{14971376}
 */
static const NvSciSyncFence NvSciSyncFenceInitializer = {{0U}};

/**
 * \brief Defines the exported form of NvSciSyncFence intended to be shared
 * across an NvSciIpc channel.
 *
 * \implements{11272406}
 */
typedef struct {
    uint64_t payload[7];
} NvSciSyncFenceIpcExportDescriptor;

/**
 * \brief Defines the exported form of NvSciSyncObj intended to be shared
 * across an NvSciIpc channel.
 *
 * \implements{15026754}
 */
typedef struct {
    /** Exported data (blob) for NvSciSyncObj */
    uint64_t payload[128];
} NvSciSyncObjIpcExportDescriptor;

/**
 * A Synchronization Object is a container holding the reconciled
 * NvSciSyncAttrList defining constraints of the Fence and the handle of the
 * actual Primitive, with access permissions being enforced by the
 * NvSciSyncAttrKey_RequiredPerm and NvSciSyncAttrKey_NeedCpuAccess Attribute
 * Keys.
 */
#if (NV_IS_SAFETY == 0)
/**
 * If Timestamps have been requested prior to Reconciliation via the
 * NvSciSyncAttrKey_WaiterRequireTimestamps key, this will also hold the
 * Timestamp Buffer allocated by NvSciBuf.
 */
#endif

/**
 * \brief A reference to a particular Synchronization Object.
 *
 * @note Every NvSciSyncObj that has been created but not freed
 * holds a reference to the NvSciSyncModule, preventing the module
 * from being de-initialized.
 */
typedef struct NvSciSyncObjRec* NvSciSyncObj;

/**
 * \brief A reference to an immutable Synchronization Object.
 */
typedef const struct NvSciSyncObjRec* NvSciSyncObjConst;

/**
 * \brief A container constituting an NvSciSyncAttrList which contains:
 * - set of NvSciSyncAttrKey attributes defining synchronization object
 *   constraints
 * - slot count defining number of slots in an NvSciSyncAttrList
 * - flag specifying if NvSciSyncAttrList is reconciled or unreconciled.
 *
 * @note Every NvSciSyncAttrList that has been created but not freed
 * holds a reference to the NvSciSyncModule, preventing the module
 * from being de-initialized.
 */
typedef struct NvSciSyncAttrListRec* NvSciSyncAttrList;

/**
 * \brief Describes NvSciSyncObj access permissions.
 *
 * \implements{14999688}
 */
typedef enum {
    /**
     * This represents the capability to monitor an NvSciSyncObj as it
     * progresses through points on its sync timeline.
     */
    NvSciSyncAccessPerm_WaitOnly = (uint64_t)1U << 0U,
    /**
     * This represents the capability to advance an NvSciSyncObj to its
     * next point on its sync timeline.
     */
    NvSciSyncAccessPerm_SignalOnly = (uint64_t)1U << 1U,
     /**
      * This represents the capability to advance an NvSciSyncObj to its
      * next point on its sync timeline and also monitor when that next point
      * is reached.
      */
    NvSciSyncAccessPerm_WaitSignal = NvSciSyncAccessPerm_WaitOnly | NvSciSyncAccessPerm_SignalOnly,
    /**
     * Usage of Auto permissions is restricted only for export/import APIs and
     * shouldn't be used as valid value for NvSciSyncAttrKey_RequiredPerm
     * Attribute.
     */
    NvSciSyncAccessPerm_Auto = (uint64_t)1U << 63U,
} NvSciSyncAccessPerm;

/**
 * \brief Describes the NvSciSync public attribute keys holding corresponding
 * values specifying synchronization object constraints. Input attribute keys
 * specify desired synchronization object constraints and can be set/retrieved
 * from unreconciled NvSciSyncAttrList using NvSciSyncAttrListSetAttrs()/
 * NvSciSyncAttrListGetAttrs() respectively. Output attribute key specify actual
 * constraints computed by NvSciSync if reconciliation succeeds. Output
 * attribute can be retrieved from reconciled NvSciSyncAttrList using
 * NvSciSyncAttrListGetAttrs().
 *
 * \implements{14999670}
 */
typedef enum {
    /** Specifies the lower bound - for NvSciSync internal use only. */
    NvSciSyncAttrKey_LowerBound,
    /** (bool, inout) Specifies if CPU access is required. */
    NvSciSyncAttrKey_NeedCpuAccess,
    /**
     * (NvSciSyncAccessPerm, inout) Specifies the required access permissions.
     * If @ref NvSciSyncAttrKey_NeedCpuAccess is true, the CPU will be offered
     * at least these permissions.
     * Any hardware accelerators that contribute to this NvSciSyncAttrList will be
     * offered at least these permissions.
     */
    NvSciSyncAttrKey_RequiredPerm,
    /**
     * (NvSciSyncAccessPerm, out) Actual permission granted after reconciliation.
     * @note This key is read-only.
     */
    NvSciSyncAttrKey_ActualPerm,
    /**
     * (bool, inout) Importing and then exporting an
     * NvSciSyncFenceIpcExportDescriptor has no side effects and yields an
     * identical NvSciSyncFenceIpcExportDescriptor even if the NvSciIpcEndpoints
     * used for import and export are different from ones used for
     * NvSciSyncAttrList transporting.
     */
    NvSciSyncAttrKey_WaiterContextInsensitiveFenceExports,
    /** (bool, inout) Specifies if timestamps are required.
     *Only valid for waiters. */
    NvSciSyncAttrKey_WaiterRequireTimestamps,
    /** Specifies the upper bound - for NvSciSync internal use only. */
    NvSciSyncAttrKey_UpperBound,
} NvSciSyncAttrKey;

/**
 * \brief This structure defines a key/value pair used to get or set
 * the NvSciSyncAttrKey(s) and their corresponding values from or to
 * NvSciSyncAttrList.
 *
 * \implements{14999679}
 */
typedef struct {
    /** NvSciSyncAttrKey for which value needs to be set/retrieved. This member
     * is initialized to any defined value of the NvSciSyncAttrKey other than
     * NvSciSyncAttrKey_LowerBound and NvSciSyncAttrKey_UpperBound */
    NvSciSyncAttrKey attrKey;
    /** Memory which contains the value corresponding to the key */
    const void* value;
    /** Length of the value in bytes */
    size_t len;
} NvSciSyncAttrKeyValuePair;

/**
 * \brief Initializes and returns a new NvSciSyncModule with no
 * NvSciSyncAttrLists, NvSciSyncCpuWaitContexts, NvSciSyncObjs or
 * NvSciSyncFences bound to it.
 *
 * @note A process may call this function multiple times. Each successful
 * invocation will yield a new NvSciSyncModule instance.
 *
 * \param[out] newModule The new NvSciSyncModule.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a newModule is NULL.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_ResourceError if system drivers are not available or
 *    resources other then memory are unavailable.
 */
NvSciError NvSciSyncModuleOpen(
    NvSciSyncModule* newModule);

/**
 * \brief Releases an instance of the NvSciSyncModule that was
 * obtained through an earlier call to NvSciSyncModuleOpen(). Once an
 * NvSciSyncModule is closed and all NvSciSyncAttrLists, NvSciSyncObjs,
 * NvSciSyncCpuWaitContexts, NvSciSyncFences bound to that module instance are
 * freed, the NvSciSyncModule instance will be de-initialized in the calling
 * process. Until then the NvSciSyncModule will still be accessible from those
 * objects still referencing it.
 *
 * \param[in] module The NvSciSyncModule instance to close. The calling process
 * must not pass this module to another NvSciSync API call.
 *
 * \return void
 * - Panics if @a module is invalid
 */
void NvSciSyncModuleClose(
    NvSciSyncModule module);

/**
 * \brief Allocates a new NvSciSyncCpuWaitContext.
 *
 * \param[in] module NvSciSyncModule instance with which to associate
 *            the new NvSciSyncCpuWaitContext.
 * \param[out] newContext The new NvSciSyncCpuWaitContext.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a module is NULL or @a newContext is NULL.
 * - ::NvSciError_InvalidState if failed to associate @a module with @a
 *   newContext.
 * - ::NvSciError_InsufficientMemory if not enough system memory to create a
 *   new context.
 * - ::NvSciError_ResourceError if not enough system resources.
 * - Panics if @a module is not valid.
*/
NvSciError NvSciSyncCpuWaitContextAlloc(
    NvSciSyncModule module,
    NvSciSyncCpuWaitContext* newContext);

/**
 * \brief Releases the NvSciSyncCpuWaitContext.
 *
 * \param[in] context NvSciSyncCpuWaitContext to be freed.
 *
 * \return void
 * - Panics if NvSciSyncModule associated with @a context is not valid.
*/
void NvSciSyncCpuWaitContextFree(
    NvSciSyncCpuWaitContext context);

/**
 * \brief Creates a new, single-slot unreconciled NvSciSyncAttrList
 * associated with the input NvSciSyncModule with empty NvSciSyncAttrKeys.
 *
 * \param[in] module The NvSciSyncModule instance with which to associate the
 * new NvSciSyncAttrList.
 * \param[out] attrList The new NvSciSyncAttrList.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL.
 * - ::NvSciError_InsufficientMemory if there is insufficient system memory
 * - ::NvSciError_ResourceError if system lacks resource other than memory
 *   to create a NvSciSyncAttrList.
 * - ::NvSciError_InvalidState if no more references can be taken for
 *     input NvSciSyncModule to create the new NvSciSyncAttrList.
 * - Panics if @a module is not valid.
 */
NvSciError NvSciSyncAttrListCreate(
    NvSciSyncModule module,
    NvSciSyncAttrList* attrList);

/**
 * \brief Frees the NvSciSyncAttrList and removes its association with the
 * NvSciSyncModule with which it was created.
 *
 * \param[in] attrList The NvSciSyncAttrList to be freed.
 * \return void
 * - Panics if @a attrList is not valid.
 */
void NvSciSyncAttrListFree(
    NvSciSyncAttrList attrList);

/**
 * \brief Checks if the NvSciSyncAttrList is reconciled
 *
 * \param[in] attrList NvSciSyncAttrList to check.
 * \param[out] isReconciled A pointer to a boolean to store whether the
 * @a attrList is reconciled or not.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL.
 * - Panics if @a attrList is not valid.
 */
NvSciError NvSciSyncAttrListIsReconciled(
    NvSciSyncAttrList attrList,
    bool* isReconciled);

/**
 * \brief Validates a reconciled NvSciSyncAttrList against a set of input
 * unreconciled NvSciSyncAttrLists.
 *
 * \param[in] reconciledAttrList Reconciled NvSciSyncAttrList to be validated.
 * \param[in] inputUnreconciledAttrListArray Array containing the unreconciled
 * NvSciSyncAttrLists used for validation.
 * Valid value: Array of valid unreconciled NvSciSyncAttrLists
 * \param[in] inputUnreconciledAttrListCount number of elements/indices in
 * @a inputUnreconciledAttrListArray
 * Valid value: [1, SIZE_MAX]
 * \param[out] isReconciledListValid A pointer to a boolean to store whether
 * the reconciled NvSciSyncAttrList satisfies the parameters of set of
 * unreconciled NvSciSyncAttrList(s) or not.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - @a inputUnreconciledAttrListArray is NULL
 *         - @a inputUnreconciledAttrListCount is 0
 *         - @a isReconciledListValid is NULL
 *         - any of the input NvSciSyncAttrLists in
 *           inputUnreconciledAttrListArray are not unreconciled
 *         - @a reconciledAttrList is NULL or not reconciled
 *         - not all the NvSciSyncAttrLists in @a inputUnreconciledAttrListArray
 *           and the @a reconciledAttrList are bound to the same NvSciSyncModule
 *           instance.
 *         - reconciled NvSciSyncAttrList does not satisfy the unreconciled
 *           NvSciSyncAttrLists requirements.
 * - ::NvSciError_InsufficientMemory if there is insufficient system memory
 *   to create temporary data structures
 * - Panics if @a reconciledAttrList or any of the input unreconciled
 *   NvSciSyncAttrList are not valid.
 */
NvSciError NvSciSyncAttrListValidateReconciled(
    NvSciSyncAttrList reconciledAttrList,
    const NvSciSyncAttrList inputUnreconciledAttrListArray[],
    size_t inputUnreconciledAttrListCount,
    bool* isReconciledListValid);

/**
 * \brief Sets the values for NvSciSyncAttrKey(s) in slot 0 of the
 * input NvSciSyncAttrList.
 *
 * Reads values only during the call, saving copies. Only allows writing
 * attributes once, making them non writable in the NvSciSyncAttrList.
 *
 * \param[in] attrList An unreconciled NvSciSyncAttrList containing the attribute
 * key and value to set.
 * \param[in] pairArray Array of NvSciSyncAttrKeyValuePair.
 * Valid value: pairArray is valid input if it is not NULL and
 * key member of every NvSciSyncAttrKeyValuePair in the array is an input or
 * input/output attribute and it is > NvSciSyncAttrKey_LowerBound and <
 * NvSciSyncAttrKey_UpperBound and value member of every NvSciSyncAttrKeyValuePair
 * in the array is not NULL.
 * \param[in] pairCount The number of elements/entries in @a pairArray.
 * Valid value: pairCount is valid input if it is non-zero.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - @a pairArray is NULL
 *         - @a attrList is NULL
 *         - @a attrList is not unreconciled and/or not writable,
 *         - @a pairCount is 0
 *         - any of the keys in @a pairArray is not a supported public key
 *         - any of the values in @a pairArray is NULL
 *         - any of the len(s) in @a pairArray is invalid for a given attribute
 *         - any of the attributes to be written is non-writable in attrList
 * - Panics if @a attrList is not valid
 */
NvSciError NvSciSyncAttrListSetAttrs(
    NvSciSyncAttrList attrList,
    const NvSciSyncAttrKeyValuePair* pairArray,
    size_t pairCount);

/**
 * \brief Gets the value of NvSciSyncAttrKey from slot 0 of the input
 * NvSciSyncAttrList.
 *
 * The return values, stored in NvSciSyncAttrKeyValuePair, consist of
 * const void* pointers to the attribute values from NvSciSyncAttrList.
 * The application must not write to this data. If an attribute was never set,
 * the corresponding value will be set to NULL and length to 0.
 *
 * \param[in] attrList NvSciSyncAttrList to retrieve the value for given
 * NvSciSyncAttrKey(s) from
 * \param[in,out] pairArray A pointer to the array of NvSciSyncAttrKeyValuePair.
 * Valid value: pairArray is valid input if it is not NULL and key member
 * of every NvSciSyncAttrKeyValuePair in the array > NvSciSyncAttrKey_LowerBound
 * and < NvSciSyncAttrKey_UpperBound.
 * \param[in] pairCount The number of elements/entries in @a pairArray.
 * Valid value: pairCount is valid input if it is non-zero.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - any argument is NULL
 *         - @a pairCount is 0
 *         - any of the keys in @a pairArray is not a supported NvSciSyncAttrKey
 * - Panics if @a attrList is not valid
 */
NvSciError NvSciSyncAttrListGetAttrs(
    NvSciSyncAttrList attrList,
    NvSciSyncAttrKeyValuePair* pairArray,
    size_t pairCount);

/**
 * \brief Gets the slot count per NvSciSyncAttrKey in a NvSciSyncAttrList.
 *
 * \param[in] attrList NvSciSyncAttrList to get the slot count from.
 * \return Number of slots or 0 if attrList is NULL or panic if attrList
 *  is not valid
 */
size_t NvSciSyncAttrListGetSlotCount(
    NvSciSyncAttrList attrList);

/**
 * \brief Appends multiple unreconciled NvSciSyncAttrLists together, forming a
 * single new unreconciled NvSciSyncAttrList with a slot count equal to the sum
 * of all the slot counts of NvSciSyncAttrList(s) in the input array.
 *
 * \param[in] inputUnreconciledAttrListArray Array containing the unreconciled
 * NvSciSyncAttrList(s) to be appended together.
 * Valid value: Array of unreconciled NvSciSyncAttrList(s) where the array
 * size is at least 1.
 * \param[in] inputUnreconciledAttrListCount Number of unreconciled
 * NvSciSyncAttrList(s) in @a inputUnreconciledAttrListArray.
 * Valid value: inputUnreconciledAttrListCount is valid input if it
 * is non-zero.
 * \param[out] newUnreconciledAttrList Appended NvSciSyncAttrList created out of
 * the input unreconciled NvSciSyncAttrList(s). The output NvSciSyncAttrList is
 * non-writable.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - @a inputUnreconciledAttrListArray is NULL
 *         - @a inputUnreconciledAttrListCount is 0
 *         - @a newUnreconciledAttrList is NULL
 *         - any of the input NvSciSyncAttrLists in
 *           inputUnreconciledAttrListArray are not unreconciled
 *         - not all the NvSciSyncAttrLists in @a inputUnreconciledAttrListArray
 *           are bound to the same NvSciSyncModule instance.
 * - ::NvSciError_InsufficientMemory if there is insufficient system memory to
 *   create the new unreconciled NvSciSyncAttrList.
 * - ::NvSciError_Overflow if the combined slot counts of all the input
 *   NvSciSyncAttrLists exceeds UINT64_MAX
 * - ::NvSciError_InvalidState if no more references can be taken for
 *   NvSciSyncModule associated with the NvSciSyncAttrList in @a
 *   inputUnreconciledAttrListArray to create the new NvSciSyncAttrList.
 * - Panics if any of the input NvSciSyncAttrLists are not valid
 */
NvSciError NvSciSyncAttrListAppendUnreconciled(
    const NvSciSyncAttrList inputUnreconciledAttrListArray[],
    size_t inputUnreconciledAttrListCount,
    NvSciSyncAttrList* newUnreconciledAttrList);

/**
 * \brief Clones an NvSciSyncAttrList. The cloned
 * NvSciSyncAttrList will contain all the values of the original NvSciSyncAttrList.
 * If the original NvSciSyncAttrList is unreconciled, then modification will be
 * allowed on the cloned NvSciSyncAttrList using set attributes APIs even if the
 * attributes had been set in the original NvSciSyncAttrList, but the calls to
 * set attributes in either NvSciSyncAttrList will not affect the other.
 *
 * \param[in] origAttrList NvSciSyncAttrList to be cloned.
 * \param[out] newAttrList The new NvSciSyncAttrList.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a origAttrList or @a newAttrList is NULL.
 * - ::NvSciError_InsufficientMemory if there is insufficient system memory
 * - ::NvSciError_InvalidState if no more references can be taken for
 *     NvSciSyncModule associated with @a origAttrList to create the new
 *     NvSciSyncAttrList.
 * - ::NvSciError_ResourceError if system lacks resource other than memory
 *   to create a NvSciSyncAttrList.
 * - Panic if @a origAttrList is not valid
 */
NvSciError NvSciSyncAttrListClone(
    NvSciSyncAttrList origAttrList,
    NvSciSyncAttrList* newAttrList);

/**
 * \brief Gets the value(s) of NvSciSyncAttrKey(s) from an NvSciSyncAttrList at
 * given slot index in a multi-slot unreconciled NvSciSyncAttrList.
 *
 * @note When exporting an array containing multiple unreconciled
 * NvSciSyncAttrLists, the importing endpoint still imports just one unreconciled
 * NvSciSyncAttrList. This unreconciled NvSciSyncAttrList is referred to as a
 * multi-slot NvSciSyncAttrList. It logically represents an array of
 * NvSciSyncAttrLists, where each key has an array of values, one per slot.
 *
 * The returned pairArray consist of const void* pointers to the actual attribute
 * values from NvSciSyncAttrList. The application must not overwrite this data.
 * If an attribute was never set, the corresponding value will be set to NULL
 * and length to 0.
 *
 * \param[in] attrList NvSciSyncAttrList to retrieve the NvSciSyncAttrKey and value
 * from.
 * \param[in] slotIndex Index in the NvSciSyncAttrList.
 * Valid value: 0 to slot count of NvSciSyncAttrList - 1.
 * \param[in,out] pairArray Array of NvSciSyncAttrKeyValuePair. Holds the
 * NvSciSyncAttrKey(s) passed into the function and returns an array of
 * NvSciSyncAttrKeyValuePair structures.
 * Valid value: pairArray is valid input if it is not NULL and key member
 * of every NvSciSyncAttrKeyValuePair in the array > NvSciSyncAttrKey_LowerBound
 * and < NvSciSyncAttrKey_UpperBound.
 * \param[in] pairCount Indicates the number of elements/entries in @a pairArray.
 * Valid value: pairCount is valid input if it is non-zero.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - @a slotIndex is not a valid slot in @a attrList
 *         - @a attrList is NULL
 *         - @a pairArray is NULL
 *         - @a pairCount is 0
 *         - any of the keys in @a pairArray is not a NvSciSyncAttrKey
 * - Panics if @a attrList is not valid
 */
NvSciError NvSciSyncAttrListSlotGetAttrs(
    NvSciSyncAttrList attrList,
    size_t slotIndex,
    NvSciSyncAttrKeyValuePair* pairArray,
    size_t pairCount);

/**
 * \brief Reconciles the input unreconciled NvSciSyncAttrLists into a new
 * reconciled NvSciSyncAttrList.
 *
 * On success, this API call allocates memory for the reconciled NvSciSyncAttrList
 * which has to be freed by the caller using NvSciSyncAttrListFree().
 *
 */
#if (NV_IS_SAFETY == 0)
/**
 * On reconciliation failure, this API call allocates memory for the conflicting
 * NvSciSyncAttrList which has to be freed by the caller using
 * NvSciSyncAttrListFree().
 *
 */
#endif
/**
 * \param[in] inputArray Array containing unreconciled NvSciSyncAttrLists to be
 * reconciled.
 * Valid value: Array of valid NvSciSyncAttrLists where the array size is at least 1
 * \param[in] inputCount The number of unreconciled NvSciSyncAttrLists in
 * @a inputArray.
 * Valid value: inputCount is valid input if is non-zero.
 * \param[out] newReconciledList Reconciled NvSciSyncAttrList. This field is populated
 * only if the reconciliation succeeded.
 */
#if (NV_IS_SAFETY == 0)
/**
 * \param[out] newConflictList Unreconciled NvSciSyncAttrList consisting of the
 * key/value pairs which caused the reconciliation failure. This field is
 * populated only if the reconciliation failed.
 */
#else
/**
 * \param[out] newConflictList Unused.
 */
#endif
/**
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - inputCount is 0
 *         - any of the input NvSciSyncAttrLists are not unreconciled
 *         - not all the NvSciSyncAttrLists in inputArray are bound to the same
 *           NvSciSyncModule instance.
 *         - any of the attributes in any of the input NvSciSyncAttrLists has
 *           an invalid value for that attribute
 *         - inputArray is NULL
 *         - newReconciledList is NULL
 */
#if (NV_IS_SAFETY == 0)
/**        - newConflictList is NULL
 */
#endif
/**
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_ReconciliationFailed if reconciliation failed because
 *     of conflicting attributes
 * - ::NvSciError_UnsupportedConfig if there is an attribute mismatch between
 *   signaler and waiters.
 * - Panics if any of the input NvSciSyncAttrLists is not valid
 */
NvSciError NvSciSyncAttrListReconcile(
    const NvSciSyncAttrList inputArray[],
    size_t inputCount,
    NvSciSyncAttrList* newReconciledList,
    NvSciSyncAttrList* newConflictList);

#if (NV_IS_SAFETY == 0)
/**
 * \brief Dumps the NvSciSyncAttrList into a binary descriptor.
 *
 * @note This API can be used for debugging purpose.
 *
 * \param[in] attrList NvSciSyncAttrList to create the blob from.
 * \param[out] buf A pointer to binary descriptor buffer.
 * \param[out] len The length of the binary descriptor buffer created.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - any argument is NULL
 * - ::NvSciError_InsufficientMemory if memory allocation failed
 * - Panics if attrList is not valid
 */
NvSciError NvSciSyncAttrListDebugDump(
    NvSciSyncAttrList attrList,
    void** buf,
    size_t* len);
#endif

/**
 * \brief Transforms the input unreconciled NvSciSyncAttrList(s) to an exportable
 * unreconciled NvSciSyncAttrList descriptor that can be transported by the
 * application to any remote process as a serialized set of bytes over an
 * NvSciIpc channel.
 *
 * \param[in] unreconciledAttrListArray NvSciSyncAttrList(s) to be exported.
 * Valid value: Array of valid NvSciSyncAttrList(s) where the array
 * size is at least 1.
 * \param[in] unreconciledAttrListCount Number of NvSciSyncAttrList(s) in
 * @a unreconciledAttrListArray.
 * Valid value: unreconciledAttrListCount is valid input if it
 * is non-zero.
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller may
 * send the exported unreconciled NvSciSyncAttrList descriptor.
 * \param[out] descBuf A pointer to the new unreconciled NvSciSyncAttrList
 * descriptor, which the caller can deallocate later using
 * NvSciSyncAttrListFreeDesc().
 * \param[out] descLen The size of the new unreconciled NvSciSyncAttrList
 * descriptor.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a unreconciledAttrListCount is 0
 *      - @a unreconciledAttrListArray is NULL
 *      - @a ipcEndpoint is not a valid NvSciIpcEndpoint
 *      - @a descBuf is NULL
 *      - @a descLen is NULL
 *      - any of the input NvSciSyncAttrLists is not unreconciled
 *      - Not all of the NvSciSyncAttrLists in @a unreconciledAttrListArray
 *        are bound to the same NvSciSyncModule instance.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_InvalidState if no more references can be taken for the
 *     NvSciSyncModule associated with @a unreconciledAttrListArray.
 * - ::NvSciError_Overflow if the combined slot count of all the unreconciled
 *   NvSciSyncAttrList exceeds UINT64_MAX
 * - ::NvSciError_ResourceError if system lacks resource other than memory.
 * - ::NvSciError_NoSpace if no space is left in transport buffer to append the
 *        key-value pair.
 * - Panic if any of the input NvSciSyncAttrLists is not valid.
 */
NvSciError NvSciSyncAttrListIpcExportUnreconciled(
    const NvSciSyncAttrList unreconciledAttrListArray[],
    size_t unreconciledAttrListCount,
    NvSciIpcEndpoint ipcEndpoint,
    void** descBuf,
    size_t* descLen);

/**
 * \brief Transforms the reconciled NvSciSyncAttrList to an exportable reconciled
 * NvSciSyncAttrList descriptor that can be transported by the application to any
 * remote process as a serialized set of bytes over an NvSciIpc channel.
 *
 * \param[in] reconciledAttrList The NvSciSyncAttrList to be exported.
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller may
 * send the exported reconciled NvSciSyncAttrList descriptor.
 * \param[out] descBuf A pointer to the new reconciled NvSciSyncAttrList
 * descriptor, which the caller can deallocate later using
 * NvSciSyncAttrListFreeDesc().
 * \param[out] descLen The size of the new reconciled NvSciSyncAttrList
 * descriptor.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - any argument is NULL
 *         - @a ipcEndpoint is not a valid NvSciIpcEndpoint
 *         - @a reconciledAttrList does not correspond to a waiter or a signaler
 *         - @a reconciledAttrList is not a reconciled NvSciSyncAttrList
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_ResourceError if system lacks resource other than memory.
 * - ::NvSciError_InvalidState if no more references can be taken for
 *     NvSciSyncModule associated with @a reconciledAttrList to create the new
 *     NvSciSyncAttrList.
 * - Panic if @a reconciledAttrList is not valid
 */
NvSciError NvSciSyncAttrListIpcExportReconciled(
    const NvSciSyncAttrList reconciledAttrList,
    NvSciIpcEndpoint ipcEndpoint,
    void** descBuf,
    size_t* descLen);

/**
 * \brief Transforms an exported unreconciled NvSciSyncAttrList descriptor
 * (potentially received from any process) into an unreconciled
 * NvSciSyncAttrList.
 *
 * \param[in] module The NvSciSyncModule instance with which to associate the
 * imported NvSciSyncAttrList.
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller receives
 * the exported unreconciled NvSciSyncAttrList descriptor.
 * \param[in] descBuf The unreconciled NvSciSyncAttrList descriptor to be
 * translated into an unreconciled NvSciSyncAttrList. It should be the result of
 * NvSciSyncAttrListIpcExportUnreconciled
 * Valid value: descBuf is valid input if it is non-NULL.
 * \param[in] descLen The size of the unreconciled NvSciSyncAttrList descriptor.
 * Valid value: descLen is valid input if it is not 0.
 * \param[out] importedUnreconciledAttrList Imported unreconciled NvSciSyncAttrList.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - any argument is NULL
 *         - @a descLen is 0
 *         - @a ipcEndpoint is not a valid NvSciIpcEndpoint
 *         - array of bytes indicated by @a descBuf and @a descLen
 *           do not constitute a valid exported NvSciSyncAttrList descriptor
 *           for an unreconciled NvSciSyncAttrList
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_ResourceError if system lacks resource other than memory.
 * - Panics if @a module is not valid
 */
NvSciError NvSciSyncAttrListIpcImportUnreconciled(
    NvSciSyncModule module,
    NvSciIpcEndpoint ipcEndpoint,
    const void* descBuf,
    size_t descLen,
    NvSciSyncAttrList* importedUnreconciledAttrList);

/**
 * \brief Translates an exported reconciled NvSciSyncAttrList descriptor
 * (potentially received from any process) into a reconciled NvSciSyncAttrList.
 *
 * It also validates that the reconciled NvSciSyncAttrList to be imported will
 * be a reconciled NvSciSyncAttrList that is consistent with the constraints in
 * an array of input unreconciled NvSciSyncAttrList(s). This is recommended
 * while importing what is expected to be a reconciled NvSciSyncAttrList to
 * cause NvSciSync to validate the reconciled NvSciSyncAttrList against the input
 * unreconciled NvSciSyncAttrList(s), so that the importing process can be sure
 * that an NvSciSyncObj will satisfy the input constraints.
 *
 * \param[in] module The NvSciSyncModule instance with which to associate the
 * imported NvSciSyncAttrList.
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller
 *            receives the exported reconciled NvSciSyncAttrList descriptor.
 * \param[in] descBuf The reconciled NvSciSyncAttrList descriptor to be
 * transformed into a reconciled NvSciSyncAttrList.
 * Valid value: descBuf is valid if it is non-NULL.
 * \param[in] descLen The size of the reconciled NvSciSyncAttrList descriptor.
 * Valid value: descLen is valid if it is not 0.
 * \param[in] inputUnreconciledAttrListArray The array of NvSciSyncAttrLists against
 * which the new NvSciSyncAttrList is to be validated.
 * Valid value: Array of valid NvSciSyncAttrList(s)
 * \param[in] inputUnreconciledAttrListCount The number of NvSciSyncAttrLists in
 * inputUnreconciledAttrListArray. If inputUnreconciledAttrListCount is
 * non-zero, then this operation will fail with an error unless all the
 * constraints of all the NvSciSyncAttrLists in inputUnreconciledAttrListArray are
 * met by the imported NvSciSyncAttrList.
 * Valid value: [0, SIZE_MAX]
 * \param[out] importedReconciledAttrList Imported NvSciSyncAttrList.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a module is NULL
 *      - @a ipcEndpoint is not a valid NvSciIpcEndpoint
 *      - the array of bytes indicated by @a descBuf and @a descLen do not
 *        constitute a valid exported NvSciSyncAttrList descriptor for a
 *        reconciled NvSciSyncAttrList
 *      - @a inputUnreconciledAttrListArray is NULL but
 *        @a inputUnreconciledAttrListCount is not 0
 *      - any of the NvSciSyncAttrLists in inputUnreconciledAttrListArray are
 *        not unreconciled
 *      - any of the NvSciSyncAttrLists in @a inputUnreconciledAttrListArray
 *        is not bound to @a module.
 * - ::NvSciError_AttrListValidationFailed if the NvSciSyncAttrList to be
 *   imported either would not be a reconciled NvSciSyncAttrList or would not meet
 *   at least one of constraints in one of the input unreconciled
 *   NvSciSyncAttrLists.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_InvalidState if no more references can be taken onr the
 *     NvSciSyncModule.
 * - ::NvSciError_ResourceError if system lacks resource other than memory.
 * - Panics if @a module or any of the input NvSciSyncAttrLists are not valid
 */
NvSciError NvSciSyncAttrListIpcImportReconciled(
    NvSciSyncModule module,
    NvSciIpcEndpoint ipcEndpoint,
    const void* descBuf,
    size_t descLen,
    const NvSciSyncAttrList inputUnreconciledAttrListArray[],
    size_t inputUnreconciledAttrListCount,
    NvSciSyncAttrList* importedReconciledAttrList);

/**
 * \brief Frees an exported NvSciSyncAttrList descriptor previously returned by
 * any NvSciSyncAttrList exporting function.
 *
 * \param[in] descBuf The exported NvSciSyncAttrList descriptor to be freed.
 * The valid value is non-NULL.
 * \return void
 */
void NvSciSyncAttrListFreeDesc(
    void* descBuf);

/**
 * \brief Frees any resources allocated for the NvSciSyncFence.
 *
 * Upon return, the memory pointed to by the NvSciSyncFence is guaranteed to be all
 * zeros and thus the NvSciSyncFence is returned to the cleared state.
 *
 * \param[in,out] syncFence A pointer to NvSciSyncFence.
 * \return void
 * - Panics if the NvSciSyncObj associated with @a syncFence is not valid
 */
void NvSciSyncFenceClear(
    NvSciSyncFence* syncFence);

/**
 * \brief Duplicates the given NvSciSyncFence, such that any wait on duplicated
 * NvSciSyncFence will complete at the same time as a wait on given
 * NvSciSyncFence. If the given NvSciSyncFence is in a cleared state, then so
 * also will be the duplicated NvSciSyncFence. The given NvSciSyncFence will be
 * cleared before the duplication. If the given NvSciSyncFence holds any
 * reference on a NvSciSyncObj, then the duplicated NvSciSyncFence will create
 * an additional reference on it.
 *
 * \param[in] srcSyncFence NvSciSyncFence to duplicate.
 * \param[out] dstSyncFence duplicated NvSciSyncFence.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL or both parameters
     point to the same NvSciSyncFence.
 * - ::NvSciError_InvalidState if no more references can be taken
 *      for NvSciSyncObj associated with @a srcSyncFence
 * - Panics if the NvSciSyncObj associated with @a srcSyncFence
 *   or @a dstSyncFence are not valid
 */
NvSciError NvSciSyncFenceDup(
    const NvSciSyncFence* srcSyncFence,
    NvSciSyncFence* dstSyncFence);

/**
 * \brief Allocates and initializes a @ref NvSciSyncObj that meets all the
 * constraints specified in the given Reconciled NvSciSyncAttrList.
 *
 * @note This function does not take ownership of the reconciled NvSciSyncAttrList.
 * The caller remains responsible for freeing the reconciled NvSciSyncAttrList.
 * The caller may free the reconciled NvSciSyncAttrList any time after this
 * function is called.
 *
 * \param[in] reconciledList A Reconciled NvSciSyncAttrList.
 * \param[out] syncObj The allocated @ref NvSciSyncObj.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_InvalidState if not enough references remain on the
 *      Reconciled NvSciSyncAttrList
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - any argument is NULL
 *         - @a reconciledList is not a Reconciled NvSciSyncAttrList
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_InsufficientResource if system resource allocation failed.
 * - ::NvSciError_ResourceError if system lacks resource other than memory.
 * - Panics if @a reconciledList is not valid
 */
NvSciError NvSciSyncObjAlloc(
    NvSciSyncAttrList reconciledList,
    NvSciSyncObj* syncObj);

/**
 * \brief Creates a new @ref NvSciSyncObj holding a reference to the original
 * resources to which the input @ref NvSciSyncObj holds reference to.
 *
 * The duplicated NvSciSyncObj is not a completely new NvSciSyncObj. Therefore,
 * signaling and generating NvSciSyncFences from one affects the state of the
 * other, because it is the same underlying NvSciSyncObj.
 *
 * The resulting NvSciSyncObj must be freed separately by the user.
 *
 * \param[in] syncObj NvSciSyncObj to duplicate.
 * \param[out] dupObj Duplicated NvSciSyncObj.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_ResourceError if system lacks resource other than memory.
 * - Panics if @a syncObj is not valid
 */
NvSciError NvSciSyncObjDup(
    NvSciSyncObj syncObj,
    NvSciSyncObj* dupObj);

/**
 * \brief Retrieves the Reconciled NvSciSyncAttrList associated with an
 * input NvSciSyncObj.
 *
 * The retrieved Reconciled NvSciSyncAttrList is always read-only and the
 * Attribute Key values in the NvSciSyncAttrList cannot be changed using the
 * set attribute APIs. In addition, the returned NvSciSyncAttrList must not be
 * freed.
 *
 * \param[in] syncObj Handle corresponding to NvSciSyncObj from which the
 * NvSciSyncAttrList has to be retrieved.
 * \param[out] syncAttrList pointer to the retrieved NvSciSyncAttrList.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a syncObj or @a syncAttrList is NULL.
 * - Panics if @a syncObj is not valid
 */
NvSciError NvSciSyncObjGetAttrList(
    NvSciSyncObj syncObj,
    NvSciSyncAttrList* syncAttrList);

/**
 * \brief Destroys a valid @ref NvSciSyncObj and frees any resources that were
 * allocated for it.
 *
 * \param[in] syncObj NvSciSyncObj to be freed.
 *
 * \return void
 * - Panics if @a syncObj is not a valid @ref NvSciSyncObj
 */
void NvSciSyncObjFree(
    NvSciSyncObj syncObj);

/**
 * \brief Exports an NvSciSyncObj into an NvSciIpc-transferable object
 * binary descriptor.
 *
 * The binary descriptor can be transferred to a Waiter to create a matching
 * NvSciSyncObj.
 *
 * \param[in] syncObj A NvSciSyncObj to export.
 * \param[in] permissions Flag indicating the expected NvSciSyncAccessPerm.
 * Valid value: permissions is valid if it is set to NvSciSyncAccessPerm_WaitOnly
 * or NvSciSyncAccessPerm_Auto.
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller
 *            intends to transfer the exported NvSciSyncObj descriptor.
 * \param[out] desc NvSciSync fills in this caller-supplied descriptor with
 *             the exported form of NvSciSyncObj that is to be shared across
 *             an NvSciIpc channel.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a desc is NULL
 *      - @a syncObj is NULL
 *      - @a permissions is invalid
 *      - @a permissions flag contains signaling rights
 *      - @a ipcEndpoint is invalid
 * - ::NvSciError_InsufficientResource if system resource allocation failed.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - Panics if @a syncObj is not valid
 */
NvSciError NvSciSyncObjIpcExport(
    NvSciSyncObj syncObj,
    NvSciSyncAccessPerm permissions,
    NvSciIpcEndpoint ipcEndpoint,
    NvSciSyncObjIpcExportDescriptor* desc);

/**
 * \brief Creates and returns an @ref NvSciSyncObj based on the supplied binary
 * descriptor describing an exported @ref NvSciSyncObj.
 *
 * This function is called from the Waiter after it receives the binary descriptor
 * from the Signaler who has created and exported the binary descriptor. The
 * Waiter will create its corresponding NvSciSyncObj and return the allocated
 * NvSciSyncObj as output.
 *
 * @note This function does not take ownership of input NvSciSyncAttrList. The caller
 * remains responsible for freeing input NvSciSyncAttrList. The caller may free
 * input NvSciSyncAttrList any time after this function is called.
 *
 * \param[in] ipcEndpoint The @ref NvSciIpcEndpoint through which the caller
 *            received the exported NvSciSyncObj descriptor.
 * \param[in] desc The exported form of @ref NvSciSyncObj received through the
 *            NvSciIpc channel.
 * Valid value: desc is valid if it is non-NULL
 * \param[in] inputAttrList The reconciled NvSciSyncAttrList returned by
 *            @ref NvSciSyncAttrListIpcImportReconciled.
 * \param[in] permissions NvSciSyncAccessPerm indicating the expected access
 * permissions.
 * Valid value: NvSciSyncAccessPerm_WaitOnly and NvSciSyncAccessPerm_Auto
 * \param[in] timeoutUs Unused
 * \param[out] syncObj The Waiter's NvSciSyncObj.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a ipcEndpoint is invalid
 *      - @a desc is invalid
 *      - @a inputAttrList is NULL
 *      - @a permissions is invalid
 *      - @a syncObj is NULL
 *      - input unreconciled NvSciSyncAttrLists' constraints
 *        are not satisfied by attributes of imported NvSciSyncObj.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_ResourceError if system lacks resource other than memory.
 * - Panics if @a syncObj or @a inputAttrList are not valid
 */
NvSciError NvSciSyncObjIpcImport(
    NvSciIpcEndpoint ipcEndpoint,
    const NvSciSyncObjIpcExportDescriptor* desc,
    NvSciSyncAttrList inputAttrList,
    NvSciSyncAccessPerm permissions,
    int64_t timeoutUs,
    NvSciSyncObj* syncObj);

/**
 * \brief Exports the input NvSciSyncFence into a binary descriptor shareable
 * across the NvSciIpc channel.
 *
 * The resulting descriptor of a non-cleared NvSciSyncFence is associated with
 * NvSciSyncObj associated with the NvSciSyncFence. After transporting
 * the descriptor via an Ipc path, NvSciSync will be able to recognize
 * that the NvSciSyncFence is associated with this NvSciSyncObj if NvSciSyncObj
 * traversed the same Ipc path.
 *
 * \param[in] syncFence A pointer to NvSciSyncFence object to be exported.
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller may
 *            send the exported fence descriptor.
 * \param[out] desc The exported form of NvSciSyncFence shared across
 *             an NvSciIpc channel.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - any argument is NULL
 *      - ipcEndpoint is not a valid NvSciIpcEndpoint
 * - Panics if @a syncFence is associated an invalid NvSciSyncObj
 */
NvSciError NvSciSyncIpcExportFence(
    const NvSciSyncFence* syncFence,
    NvSciIpcEndpoint ipcEndpoint,
    NvSciSyncFenceIpcExportDescriptor* desc);

/**
 * \brief Fills in the NvSciSyncFence based on the input
 * binary descriptor. If the NvSciSyncFence descriptor does not describe a
 * cleared NvSciSyncFence, then NvSciSync will validate if it corresponds to the
 * NvSciSyncObj and it will associate the out NvSciSyncFence with the
 * NvSciSyncObj.
 *
 * The NvSciSyncFence will be cleared first, removing any previous
 * reference to a NvSciSyncObj.
 *
 * \param[in] syncObj The NvSciSyncObj.
 * \param[in] desc The exported form of NvSciSyncFence.
 *  Valid value: A binary descriptor produced by NvSciSyncIpcExportFence.
 * \param[out] syncFence A pointer to NvSciSyncFence object.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - any argument is NULL
 *      - fence descriptor does not describe a cleared NvSciSyncFence but
 *        it is associated with an NvSciSyncObj different from @a syncObj
 * - ::NvSciError_InvalidState if @a syncObj cannot take more references.
 * - Panics if @a syncObj is not valid
 */
NvSciError NvSciSyncIpcImportFence(
    NvSciSyncObj syncObj,
    const NvSciSyncFenceIpcExportDescriptor* desc,
    NvSciSyncFence* syncFence);

/**
 * \brief Generates next point on sync timeline of an NvSciSyncObj and fills
 * in the supplied NvSciSyncFence object.
 *
 * This function can be used when the CPU is the Signaler.
 *
 * \param[in] syncObj A valid NvSciSyncObj.
 * \param[out] syncFence NvSciSyncFence to be filled
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - any argument is NULL
 *         - @a syncObj is not a CPU signaler
 * - Panics if @a syncObj is not valid
 */
NvSciError NvSciSyncObjGenerateFence(
    NvSciSyncObj syncObj,
    NvSciSyncFence* syncFence);

/**
 * \brief Signals the @ref NvSciSyncObj using the reconciled primitive that
 * was allocated along with the NvSciSyncObj.
 *
 * This function is called when the CPU is the Signaler.
 *
 * \param[in] syncObj A valid NvSciSyncObj to signal.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - @a syncObj is NULL
 *         - @a syncObj is not a CPU Signaler
 * - Panics if @a syncObj is not valid
 */
NvSciError NvSciSyncObjSignal(
    NvSciSyncObj syncObj);

/**
 * \brief Performs a synchronous wait on the @ref NvSciSyncFence object until the
 * NvSciSyncFence has been signaled or the timeout expires. Any
 * NvSciSyncCpuWaitContext may be used for waiting on any NvSciSyncFence provided
 * they were created in the same NvSciSyncModule context.
 * One NvSciSyncCpuWaitContext can be used to wait on only one NvSciSyncFence at a time
 * but it can be used to wait on a different NvSciSyncFence at a different time.
 *
 * Waiting on a cleared NvSciSyncFence is always not blocking.
 *
 * \param[in] syncFence The NvSciSyncFence to wait on.
 * \param[in] context NvSciSyncCpuWaitContext holding resources needed
 *            to perform waiting.
 * \param[in] timeoutUs Timeout to wait for in micro seconds, -1 for infinite
 *            wait.
 *  Valid value: [-1, NvSciSyncFenceMaxTimeout]
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - @a syncFence is NULL
 *         - @a context is NULL
 *         - @a syncFence and @a context are associated with different
 *           NvSciSyncModule
 *         - @a timeoutUs is invalid
 *         - if caller doesn't have CPU wait permissions in NvSciSyncObj
 *           associated with @a syncFence
 * - ::NvSciError_ResourceError if wait operation did not complete
 *   successfully.
 * - ::NvSciError_Timeout if wait did not complete in the given timeout.
 * - ::NvSciError_Overflow if the NvSciSyncFence's id or value are not in range
 *   supported by the primitive this NvSciSyncFence corresponds to.
 * - Panics if any NvSciSyncObj associated with @a syncFence or @a context
 *   is not valid
 */
NvSciError NvSciSyncFenceWait(
    const NvSciSyncFence* syncFence,
    NvSciSyncCpuWaitContext context,
    int64_t timeoutUs);

#if (NV_IS_SAFETY == 0)
/**
 * \brief Read the timestamp associated with the NvSciSyncFence
 *
 * \param[in] syncFence object of type NvSciSyncFence
 * \param[out] timestampUS time (in microseconds) when the NvSciSyncFence expired.
 *
 * \return ::NvSciError
 * - ::NvSciError_Success if successful
 * - ::NvSciError_ClearedFence if @a syncFence is cleared
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - any argument is NULL
 *         - @a syncFence is NULL or has no non NULL NvSciSyncObj associated
 *         - timestamps are not supported by NvSciSyncObj associated with
 *         - @a syncFence
 *         - @a syncFence does not support timestamps
 * - Panics if NvSciSyncObj associated with syncFence is not valid.
 */
NvSciError NvSciSyncFenceGetTimestamp(
    const NvSciSyncFence* syncFence,
    uint64_t* timestampUS);
#endif

/*
 * NvSciSync Utility functions
 */

/**
 * \brief Gets the attribute value from the slot 0 of the passed NvSciSyncAttrList
 * with the given NvSciSyncAttrKey.
 * If an NvSciSyncAttrKey was not set, this function will set *value to NULL
 * and *len to 0.
 *
 * \param[in] attrList NvSciSyncAttrList to retrieve the NvSciSyncAttrKey and
 * value from.
 * \param[in] key NvSciSyncAttrKey for which value to retrieve.
 * Valid value: key is a valid input if it is an input or input/output attribute
 * and it is > NvSciSyncAttrKey_LowerBound and < NvSciSyncAttrKey_UpperBound
 * \param[out] value A pointer to the location where the attribute value is written.
 * \param[out] len Length of the value.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - @a attrList is NULL
 *         - @a value is NULL
 *         - @a len is NULL
 *         - @a key is not a supported public key
 * - Panics if @a attrList is not valid
 */
NvSciError NvSciSyncAttrListGetAttr(
    NvSciSyncAttrList attrList,
    NvSciSyncAttrKey key,
    const void** value,
    size_t* len);

/**
 * \brief Reconciles the input Unreconciled NvSciSyncAttrList(s) into a new
 * Reconciled NvSciSyncAttrList. If successful, a Reconciled NvSciSyncAttrList will
 * be associated with a newly-allocated @ref NvSciSyncObj that satisfies all
 * the constraints specified in the Reconciled NvSciSyncAttrList.
 *
 * Note: This function serves as a convenience function that combines calls to
 * NvSciSyncAttrListReconcile and NvSciSyncObjAlloc.
 *
 * \param[in] inputArray Array containing the Unreconciled NvSciSyncAttrList(s)
 *            to reconcile.
 * Valid value: Array of valid NvSciSyncAttrLists where the array size is at least 1
 * \param[in] inputCount Number of Unreconciled NvSciSyncAttrLists in
 *            @a inputArray.
 * Valid value: inputCount is valid input if is non-zero.
 * \param[out] syncObj The new NvSciSyncObj.
 */
#if (NV_IS_SAFETY == 0)
/**
 * \param[out] newConflictList Unreconciled NvSciSyncAttrList consisting of the
 *             key-value pairs which caused the reconciliation failure.
 * Valid value: This parameter is a valid output parameter only if the return
 *     code is ::NvSciError_ReconciliationFailed
 */
#else
/**
 * \param[out] newConflictList Unused
 */
#endif
/**
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a inputCount is 0
 *      - @a inputArray is NULL
 *      - @a syncObj is NULL
 *      - not all the NvSciSyncAttrLists in inputArray are bound to the same
 *        NvSciSyncModule instance
 *      - any of the attributes in any of the input NvSciSyncAttrLists has an
 *        invalid value for that attribute
 *      - if any of the NvSciSyncAttrList in attrList are not unreconciled
 */
#if (NV_IS_SAFETY == 0)
/**      - @a newConflictList is NULL
 */
#endif
/**
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_ReconciliationFailed if reconciliation failed.
 * - ::NvSciError_UnsupportedConfig if there is an NvSciSyncAttrList mismatch between
 *   Signaler and Waiters.
 * - ::NvSciError_InsufficientResource if system resource allocation failed.
 * - Panics if any of the input NvSciSyncAttrLists are not valid
 */
NvSciError NvSciSyncAttrListReconcileAndObjAlloc(
    const NvSciSyncAttrList inputArray[],
    size_t inputCount,
    NvSciSyncObj* syncObj,
    NvSciSyncAttrList* newConflictList);

/**
 * \brief Exports an NvSciSyncAttrList and NvSciSyncObj into an
 * NvSciIpc-transferable object binary descriptor pointed to by @a data.
 *
 * The binary descriptor can subsequently be transferred to Waiters to create
 * a matching NvSciSyncObj.
 *
 * Note: This function serves as a convenience function that combines calls to
 * NvSciSyncAttrListIpcExportReconciled and NvSciSyncObjIpcExport.
 *
 * \param[in] syncObj NvSciSyncObj to export.
 * \param[in] permissions Flag indicating the expected NvSciSyncAccessPerm.
 * Valid value: permissions is valid if it is set to NvSciSyncAccessPerm_WaitOnly
 * or NvSciSyncAccessPerm_Auto.
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller may send
 *            the exported NvSciSyncAttrList and NvSciSyncObj descriptor.
 * \param[out] attrListAndObjDesc Exported form of NvSciSyncAttrList and
 * NvSciSyncObj shareable across an NvSciIpc channel.
 * \param[out] attrListAndObjDescSize Size of the exported blob.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a attrListAndObjDesc is NULL
 *      - @a attrListAndObjDescSize is NULL
 *      - @a syncObj is NULL
 *      - @a permissions flag contains signaling rights
 *      - @a ipcEndpoint is invalid.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_ResourceError if system lacks resource other than memory.
 * - Panic if @a syncObj is not valid
 */
NvSciError NvSciSyncIpcExportAttrListAndObj(
    NvSciSyncObj syncObj,
    NvSciSyncAccessPerm permissions,
    NvSciIpcEndpoint ipcEndpoint,
    void** attrListAndObjDesc,
    size_t* attrListAndObjDescSize);

/**
 * \brief Frees an @ref NvSciSyncIpcExportAttrListAndObj descriptor
 * returned by a successful call to @ref NvSciSyncIpcExportAttrListAndObj.
 *
 * \param[in] attrListAndObjDescBuf Exported @ref NvSciSyncIpcExportAttrListAndObj
 * descriptor to be freed.
 * \return void
 */
void NvSciSyncAttrListAndObjFreeDesc(
    void* attrListAndObjDescBuf);

/**
 * \brief Creates an NvSciSyncObj based on the supplied binary descriptor
 * returned from a successful call to @ref NvSciSyncIpcExportAttrListAndObj
 * that has not yet been freed via @ref NvSciSyncAttrListAndObjFreeDesc. It
 * also validates Reconciled NvSciSyncAttrList against
 * a set of Unreconciled NvSciSyncAttrLists to ensure that the Reconciled
 * NvSciSyncAttrList satisfies the constraints of all the given Unreconciled
 * NvSciSyncAttrLists.
 *
 * This function is called from the Waiter after it receives the binary
 * descriptor from the Signaler who has created the binary descriptor.
 * Waiter will create its own NvSciSyncObj and return as output.
 *
 * Note: This function serves as a convenience function that combines calls to
 * NvSciSyncAttrListIpcImportReconciled and NvSciSyncObjIpcImport.
 *
 * \param[in] module A @ref NvSciSyncModule to associate the imported
 *            @ref NvSciSyncAttrList with.
 * \param[in] ipcEndpoint The @ref NvSciIpcEndpoint through which the caller
 *            receives the exported NvSciSyncAttrList and NvSciSyncObj descriptor.
 * \param[in] attrListAndObjDesc Exported form of NvSciSyncAttrList and
 *            NvSciSyncObj received through NvSciIpc channel.
 * Valid value: attrListAndObjDesc is valid if it is non-NULL.
 * \param[in] attrListAndObjDescSize Size of the exported blob.
 * Valid value: attrListAndObjDescSize is valid if it is not 0.
 * \param[in] attrList The array of Unreconciled NvSciSyncAttrLists
 *            against which the new NvSciSyncAttrList is to be validated.
 * Valid value: Array of valid NvSciSyncAttrList(s)
 * \param[in] attrListCount Number of Unreconciled NvSciSyncAttrLists in the
 *            @a attrList array.
 * Valid value: [0, SIZE_MAX]
 * \param[in] minPermissions Flag indicating the expected NvSciSyncAccessPerm.
 * Valid value: NvSciSyncAccessPerm_WaitOnly and NvSciSyncAccessPerm_Auto
 * \param[in] timeoutUs Unused
 * \param[out] syncObj Waiter's NvSciSyncObj.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a ipcEndpoint is invalid
 *      - @a attrListAndObjDesc is invalid
 *      - @a minPermissions is invalid
 *      - @a syncObj is NULL
 *      - input unreconciled NvSciSyncAttrLists' constraints
 *        are not satisfied by attributes of imported NvSciSyncObj.
 *      - @a attrList is NULL but @a attrListCount is not 0
 *      - if any of the NvSciSyncAttrList in attrList are not unreconciled
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_ResourceError if system lacks resource other than memory.
 * - Panics if any of the following occurs:
 *      - @a module is not valid
 *      - any of the input Unreconciled NvSciSyncAttrLists are not valid
 */
NvSciError NvSciSyncIpcImportAttrListAndObj(
    NvSciSyncModule module,
    NvSciIpcEndpoint ipcEndpoint,
    const void* attrListAndObjDesc,
    size_t attrListAndObjDescSize,
    NvSciSyncAttrList const attrList[],
    size_t attrListCount,
    NvSciSyncAccessPerm minPermissions,
    int64_t timeoutUs,
    NvSciSyncObj* syncObj);

/**
 * \brief Checks if loaded library version is compatible with the supplied
 * library build version.
 *
 * This function checks the version of all dependent libraries and sets the
 * output variable to true if all libraries are compatible, else sets output
 * to false.
 *
 * \param[in] majorVer build major version.
 * Valid value: valid if set to NvSciSyncMajorVersion
 * \param[in] minorVer build minor version.
 * Valid value: valid if set to NvSciSyncMinorVersion
 * \param[out] isCompatible pointer to the bool holding the result.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a isCompatible is NULL
 *      - @a failed to check dependent library versions.
 */
NvSciError NvSciSyncCheckVersionCompatibility(
    uint32_t majorVer,
    uint32_t minorVer,
    bool* isCompatible);

#if defined(__cplusplus)
}
#endif // __cplusplus
 /** @} */
#endif // INCLUDED_NVSCISYNC_H
