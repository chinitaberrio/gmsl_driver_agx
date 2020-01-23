/*
 * Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
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
 * the latter process does not import an unreconciled attribute list (directly
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
 * - An import of an export of one or more unreconciled attribute lists.
 *
 * It is reconciled if it was:
 * - Created by a successful NvSciSyncAttrListReconcile() call,
 * - Provided by NvSciSyncObjGetAttrList(), or
 * - An import of an export of another reconciled attribute list.
 */

#if defined(__cplusplus)
#include <cstdlib>
#else
#include <stdlib.h>
#include <stdbool.h>
#endif

#include <nvscierror.h>
#include <nvsciipc.h>

#if defined (__cplusplus)
extern "C"
{
#endif

/**
 * \brief NvSciSync version.
 */
static const uint32_t NvSciSyncMajorVersion = 1U;
static const uint32_t NvSciSyncMinorVersion = 0U;

/**
 * \brief Represents an instance of the NvSciSync module. Any NvSciSyncAttrList
 * created or imported using a particular NvSciSyncModule is bound to that
 * module instance, along with any NvSciSyncObjs created or imported using those
 * NvSciSyncAttrLists and any NvSciSyncFences created or imported using those
 * NvSciSyncObjs.
 *
 * @note For any NvSciSync API call that has more than one input of type
 * NvSciSyncModule, NvSciSyncAttrList, NvSciSyncObj, and/or NvSciSyncFence, all
 * such inputs must agree on the NvSciSync module instance.
 */
typedef struct NvSciSyncModuleRec* NvSciSyncModule;

/**
 * \brief Represents the right to perform a CPU wait on an NvSciSyncFence.
 * It holds resources necessary to perform a CPU wait.
 * It can be used to wait on NvSciSyncFences associated with the same
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
 * Unlike NvSciSyncAttrList and NvSciSyncObj objects, applications are
 * responsible for the memory management of NvSciSyncFence objects.
 *
 * Every NvSciSyncFence must be initialized to all zeros after its storage
 * is allocated before the first time it is passed to any NvSciSync API
 * calls. @ref NvSciSyncFenceInitializer can be used for this, or
 * @c memset(), or any other mechanism for zeroing memory.
 *
 * @note Every NvSciSyncFence not in a cleared state holds a reference to
 * the NvSciSyncObj it is related to, preventing that NvSciSyncObj
 * from being deleted.
 */
typedef struct {
    uint64_t payload[6];
} NvSciSyncFence;

/**
 * \brief Defines the value used to zero-initialize the NvSciSyncFence object.
 * An NvSciSyncFence that is all zeroes is in a cleared state.
 */
static const NvSciSyncFence NvSciSyncFenceInitializer = {{0U}};

/**
 * \brief Defines the exported form of NvSciSyncFence shared across an
 * NvSciIpc channel.
 */
typedef struct {
    uint64_t payload[7];
} NvSciSyncFenceIpcExportDescriptor;

/**
 * \brief Defines the exported form of NvSciSyncObj shared across an
 * NvSciIpc channel.
 */
typedef struct {
    uint64_t payload[128];
} NvSciSyncObjIpcExportDescriptor;

/**
 * \brief Holds a pointer to an opaque NvSciSync object.
 *
 * @note Every NvSciSyncObj that has been created but not freed
 * holds a reference to the NvSciSync module, preventing the module
 * from being de-initialized.
 */
typedef struct NvSciSyncObjRec* NvSciSyncObj;

/**
 * \brief Holds a pointer to an opaque NvSciSync attribute list.
 *
 * @note Every NvSciSyncAttrList that has been created but not freed
 * holds a reference to the NvSciSync module, preventing the module
 * from being de-initialized.
 */
typedef struct NvSciSyncAttrListRec* NvSciSyncAttrList;

/**
 * \brief Describes sync object access permissions.
 */
typedef enum {
    /**
     * This represents the capability to monitor an NvSciSync object as it
     * progresses through points on its sync timeline.
     */
    NvSciSyncAccessPerm_WaitOnly,
    /**
     * This represents the capability to advance an NvSciSync object to its
     * next point on its sync timeline.
     */
    NvSciSyncAccessPerm_SignalOnly,
     /**
      * This represents the capability to advance an NvSciSync object to its
      * next point on its sync timeline and also monitor when that next point
      * is reached.
      */
    NvSciSyncAccessPerm_WaitSignal,
} NvSciSyncAccessPerm;

/**
 * \brief Describes the types of NvSciSyncAttr keys.
 */
typedef enum {
    /** Specifies the lower bound - for NvSciSync internal use only. */
    NvSciSyncAttrKey_LowerBound,
    /** (bool) Specifies if CPU access is required. */
    NvSciSyncAttrKey_NeedCpuAccess,
    /**
     * (NvSciSyncAccessPerm) Specifies the required access permissions.
     * If @ref NvSciSyncAttrKey_NeedCpuAccess is true, the CPU will be offered
     * at least these permissions.
     * Any hardware accelerators that contribute to this attribute list will be
     * offered at least these permissions.
     */
    NvSciSyncAttrKey_RequiredPerm,
    /**
     * (NvSciSyncAccessPerm) Actual permission granted after reconciliation.
     * @note This key is read-only.
     */
    NvSciSyncAttrKey_ActualPerm,
    /**
     * (bool) Importing and then exporting an
     * NvSciSyncFenceIpcExportDescriptor has no side effects and yields an
     * identical NvSciSyncFenceIpcExportDescriptor even if the NvSciIpcEndpoints
     * used for import and export are distinct.
     */
    NvSciSyncAttrKey_WaiterContextInsensitiveFenceExports,
    /** Specifies the upper bound - for NvSciSync internal use only. */
    NvSciSyncAttrKey_UpperBound,
} NvSciSyncAttrKey;

/**
 * \brief Describes the NvSciSyncAttrKey-Value pair.
 */
typedef struct {
    /** Specifies the attribute key to set. */
    NvSciSyncAttrKey attrKey;
    /**
     * Specifies the pointer to the attribute value, or NULL
     * if there is no value assigned to attrKey.
     */
    const void* value;
    /** Specifies the buffer length, or 0 if there is no value assigned to @c attrKey. */
    size_t len;
} NvSciSyncAttrKeyValuePair;

/**
 * \brief Initializes an instance of the NvSciSync module within the calling
 * process and provides an @ref NvSciSyncModule representing the instance.
 *
 * @note A process may call this function multiple times. Each successful
 * invocation will yield a new NvSciSync module instance.
 *
 * \param[out] newModule The new NvSciSync module instance.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if newModule is NULL.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_ResourceError if system drivers are not available.
 */
NvSciError NvSciSyncModuleOpen(
    NvSciSyncModule* newModule);

/**
 * \brief Releases an instance of the NvSciSync module that was
 * obtained through an earlier call to NvSciSyncModuleOpen(). Once an
 * NvSciSyncModule is closed and all NvSciSyncAttrLists, NvSciSyncObjs,
 * NvSciSyncFences bound to that module instance are freed, the
 * NvSciSync module instance will be de-initialized in the calling process.
 *
 * \param[in] module The NvSciSync module instance to close. The calling process
 * must not pass this module to another NvSciSync API call.
 *
 * \return void
 */
void NvSciSyncModuleClose(
    NvSciSyncModule module);

/**
 * \brief Allocates a CPU wait context that can be used to perform a CPU
 * wait with NvSciSyncFenceWait().
 *
 * \param[in] module NvSciSync module instance with which to associate
 *            the new CPU wait context.
 * \param[out] newContext Address at which the new context is created.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a module is NULL or @a newContext is NULL.
 * - ::NvSciError_InsufficientMemory if not enough system memory to create a
 *   new context.
 * - ::NvSciError_InsufficientResource if not enough system resources.
*/
NvSciError NvSciSyncCpuWaitContextAlloc(
    NvSciSyncModule module,
    NvSciSyncCpuWaitContext* newContext);

/**
 * \brief Releases a CPU wait context.
 *
 * \param[in] context CPU wait context to be freed.
 *
 * \return void
*/
void NvSciSyncCpuWaitContextFree(
    NvSciSyncCpuWaitContext context);

/**
 * \brief Creates an attribute list holding the attributes of the NvSciSyncObj
 * to be allocated.
 *
 * \param[in] module The NvSciSync module instance with which to associate the
 * new NvSciSyncAttrList.
 * \param[out] attrList The address of the newly created attribute list.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL.
 * - ::NvSciError_InsufficientMemory if there is insufficient system memory
 * to create a list.
 */
NvSciError NvSciSyncAttrListCreate(
    NvSciSyncModule module,
    NvSciSyncAttrList* attrList);

/**
 * \brief Frees the memory of an attribute list.
 *
 * \param[in] attrList The attribute list to be freed.
 * \return void
 */
void NvSciSyncAttrListFree(
    NvSciSyncAttrList attrList);

/**
 * \brief Returns whether the attribute list is reconciled or unreconciled.
 *
 * \param[in] attrList Attribute list of type NvSciSyncAttrList.
 * \param[out] isReconciled False if attrList is unreconciled, true if
 * attrList is reconciled.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL.
 */
NvSciError NvSciSyncAttrListIsReconciled(
    NvSciSyncAttrList attrList,
    bool* isReconciled);

/**
 * \brief Validates a reconciled attribute list against a set of input
 * unreconciled attribute lists.
 *
 * \param[in] reconciledAttrList Reconciled attribute list to be validated.
 * \param[in] inputUnreconciledAttrListArray Array containing the unreconciled
 * attribute lists used for validation.
 * \param[in] inputUnreconciledAttrListCount Number of unreconciled attribute
 * lists.
 * \param[out] isReconciledListValid Flag indicating if the reconciled list
 *  satisfies the parameters of set of unreconciled lists.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - @a inputUnreconciledAttrListArray is NULL or
 *         - @a inputUnreconciledAttrListCount is 0 or
 *         - @a isReconciledListValid is NULL or
 *         - any of the input attribute lists are reconciled or
 *         - @a reconciledAttrList is not reconciled or
 *         - not all the NvSciSyncAttrLists in @a inputUnreconciledAttrListArray
 *           and the @a reconciledAttrList are bound to the same NvSciSync
 *           module instance.
 */
NvSciError NvSciSyncAttrListValidateReconciled(
    NvSciSyncAttrList reconciledAttrList,
    const NvSciSyncAttrList inputUnreconciledAttrListArray[],
    size_t inputUnreconciledAttrListCount,
    bool* isReconciledListValid);

/**
 * \brief Sets the value of an attribute in an unreconciled attribute list that
 * has not already been set.
 *
 * Reads values only during the call, saving copies.
 *
 * \param[in,out] attrList An unreconciled attribute list containing the attribute
 * key and value to set.
 * \param[in] pairArray A pointer to the array of attribute key/value pair structure.
 * \param[in] pairCount The number of elements/entries in @a pairArray.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - @a pairArray is NULL or
 *         - @a attrList is NULL or
 *         - @a attrList is a reconciled and/or imported attribute list, or
 *         - @a attrList already has a value of non-zero length for any of the keys in @a pairArray.
 * - ::NvSciError_InsufficientMemory if there is insufficient system memory to set the
 *   attribute(s).
 */
NvSciError NvSciSyncAttrListSetAttrs(
    NvSciSyncAttrList attrList,
    const NvSciSyncAttrKeyValuePair* pairArray,
    size_t pairCount);

/**
 * \brief Gets the value of attributes from an attribute list.
 *
 * The return values, stored in @ref NvSciSyncAttrKeyValuePair, consist of
 * @c const @c void* pointers to the attribute values from NvSciSyncAttrList.
 * The application must not write to this data.
 *
 * \param[in] attrList Attribute list to retrieve the attribute key and value from.
 * \param[in,out] pairArray A pointer to the array of attribute key/value pair structure.
 * \param[in] pairCount The number of elements/entries in @a pairArray.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL, or
 *         @a attrList is an imported unreconciled attribute list.
 */
NvSciError NvSciSyncAttrListGetAttrs(
    NvSciSyncAttrList attrList,
    NvSciSyncAttrKeyValuePair* pairArray,
    size_t pairCount);

/**
 * \brief Gets the total number of slots contained in a NvSciSyncAttrList.
 *
 * \param[in] attrList Attribute list to get the slot count from.
 * \return Number of slots or 0 if @a attrList is NULL or panic if attrList is invalid.
 *
 * @note When exporting an array containing multiple unreconciled attribute
 * lists, the importing endpoint still imports just one unreconciled attribute
 * list. This unreconciled attribute list is referred to as a multi-slot
 * attribute list. It logically represents an array of attribute lists, where
 * each key has an array of values, one per slot. This is mainly useful for
 * debugging purpose.
 */
size_t NvSciSyncAttrListGetSlotCount(
    NvSciSyncAttrList attrList);

/**
 * \brief Appends multiple unreconciled attribute lists together, forming a
 * single new unreconciled attribute list with a slot count equal to the sum of
 * all the slot counts of the input list.
 *
 * \param[in] inputUnreconciledAttrListArray Array containing the unreconciled
 * attribute lists to be appended together.
 * \param[in] inputUnreconciledAttrListCount Number of unreconciled attribute
 * lists to append.
 * \param[out] newUnreconciledAttrList Appended attribute list created out of
 * the input unreconciled attribute lists.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - @a inputUnreconciledAttrListArray is NULL or
 *         - @a newUnreconciledAttrList is NULL or
 *         - @a inputUnreconciledAttrListCount is zero or
 *         - any of the input attribute lists are reconciled, or
 *         - not all the NvSciSyncAttrLists in @a inputUnreconciledAttrListArray
 *           are bound to the same NvSciSync module instance.
 * - ::NvSciError_InsufficientMemory if there is insufficient system memory to create the
 *   new unreconciled attribute list.
 */
NvSciError NvSciSyncAttrListAppendUnreconciled(
    const NvSciSyncAttrList inputUnreconciledAttrListArray[],
    size_t inputUnreconciledAttrListCount,
    NvSciSyncAttrList* newUnreconciledAttrList);

/**
 * \brief Clones an unreconciled/reconciled attribute list. The cloned attribute
 * list will contain all the values of the original attribute list. If the
 * original attribute list is unreconciled, then modification will be allowed
 * on the cloned list using set attributes APIs, but these calls to set
 * attributes in either list will not affect the other.
 *
 * \param[in] origAttrList The list to be cloned.
 * \param[out] newAttrList The new clone of @a origAttrList.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if origAttrList or newAttrList is NULL.
 * - ::NvSciError_InsufficientMemory if there is insufficient system memory
 *   to create the new attribute list.
 */
NvSciError NvSciSyncAttrListClone(
    NvSciSyncAttrList origAttrList,
    NvSciSyncAttrList* newAttrList);

/**
 * \brief Gets the value of attributes from an attribute list at given index in
 * a multi-slot unreconciled attribute list.
 *
 * @note When exporting an array containing multiple unreconciled attribute
 * lists, the importing endpoint still imports just one unreconciled attribute
 * list. This unreconciled attribute list is referred to as a multi-slot
 * attribute list. It logically represents an array of attribute lists, where
 * each key has an array of values, one per slot. This is mainly useful for
 * debug purpose.
 *
 * The returned @a pairArray consist of @c const @c void* pointers to the actual attribute
 * values from @ref NvSciSyncAttrList. The application must not overwrite this data.
 *
 * \param[in] attrList Attribute list to retrieve the attribute key and value from.
 * \param[in] slotIndex Index in the attribute list array.
 * \param[in,out] pairArray A pointer to the array of attribute key/value pair structure.
 * \param[in] pairCount Indicates the number of elements/entries in @a pairArray.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - @a slotIndex is invalid or
 *         - @a attrList is invalid or
 *         - @a pairArray is NULL or
 *         - @a pairCount is invalid.
 */
NvSciError NvSciSyncAttrListSlotGetAttrs(
    NvSciSyncAttrList attrList,
    size_t slotIndex,
    NvSciSyncAttrKeyValuePair* pairArray,
    size_t pairCount);

/**
 * \brief Reconciles the input unreconciled attribute list(s) into a new
 * reconciled attribute list.
 *
 * On success, this API call allocates memory for the reconciled attribute list
 * which has to be freed by the caller using NvSciSyncAttrListFree().
 *
 * On reconciliation failure, this API call allocates memory for the conflicting
 * attribute list which has to be freed by the caller using
 * NvSciSyncAttrListFree().
 *
 * \param[in] inputArray Array containing unreconciled attribute list(s) to be
 * reconciled.
 * \param[in] inputCount The number of unreconciled attributes lists in
 * @a inputArray.
 * \param[out] newReconciledList Reconciled attribute list. This field is populated
 * only if the reconciliation succeeded.
 * \param[out] newConflictList Unreconciled attribute list consisting of the
 * key/value pairs which caused the reconciliation failure. This field is
 * populated only if the reconciliation failed.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *         - @a inputCount is zero or
 *         - @a inputArray is NULL or
 *         - @a newReconciledList is NULL or
 *         - @a newConflictList is NULL.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_ReconciliationFailed if reconciliation failed.
 * - ::NvSciError_UnsupportedConfig if there is an attribute list mismatch between
 *   signaler and waiters.
 */
NvSciError NvSciSyncAttrListReconcile(
    const NvSciSyncAttrList inputArray[],
    size_t inputCount,
    NvSciSyncAttrList* newReconciledList,
    NvSciSyncAttrList* newConflictList);

/**
 * \brief Dumps the attribute list into a binary blob.
 *
 * @note This API can be used for debugging purpose.
 *
 * \param[in] attrList Attribute list to create the blob from.
 * \param[out] buf A pointer to binary blob buffer.
 * \param[out] len The length of the binary blob buffer created.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL.
 */
NvSciError NvSciSyncAttrListDebugDump(
    NvSciSyncAttrList attrList,
    void** buf,
    size_t* len);

/**
 * \brief Translates the unreconciled attribute list(s) to an exported attribute
 * list descriptor that can potentially be transported by the application to any
 * process as a serialized array of bytes.
 *
 * \param[in] unreconciledAttrListArray Attribute list(s) to be exported.
 * \param[in] unreconciledAttrListCount Number of attribute lists in the array.
 * This value must be non-zero. For a single list, the count must be set 1.
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller may
 * send the exported attribute list descriptor.
 * \param[out] descBuf A pointer to the new attribute descriptor, which the caller
 * can deallocate later using NvSciSyncAttrListFreeDesc().
 * \param[out] descLen The size of the new attribute descriptor.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a unreconciledAttrListCount is zero or
 *      - @a unreconciledAttrListArray is NULL or
 *      - @a ipcEndpoint is NULL or
 *      - @a descBuf is NULL or
 *      - @a descLen is NULL or
 *      - Not all of the NvSciSyncAttrLists in @a attrListArray
 *        are bound to the same NvSciSync module instance.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 */
NvSciError NvSciSyncAttrListIpcExportUnreconciled(
    const NvSciSyncAttrList unreconciledAttrListArray[],
    size_t unreconciledAttrListCount,
    NvSciIpcEndpoint ipcEndpoint,
    void** descBuf,
    size_t* descLen);

/**
 * \brief Translates the reconciled attribute list to an exported attribute
 * list descriptor that can potentially be transported by the application to any
 * process as a serialized array of bytes.
 *
 * \param[in] reconciledAttrList The attribute list to be exported.
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller may
 * send the exported attribute list descriptor.
 * \param[out] descBuf A pointer to the new attribute descriptor, which the caller
 * can deallocate later using NvSciSyncAttrListFreeDesc().
 * \param[out] descLen The size of the new attribute descriptor.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 */
NvSciError NvSciSyncAttrListIpcExportReconciled(
    const NvSciSyncAttrList reconciledAttrList,
    NvSciIpcEndpoint ipcEndpoint,
    void** descBuf,
    size_t* descLen);

/**
 * \brief Translates an exported attribute list descriptor (potentially received
 * from any process) into an attribute list.
 *
 * \param[in] module The NvSciSyncModule instance with which to associate the
 * imported NvSciSyncAttrList.
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller receives
 * the exported attribute list descriptor.
 * \param[in] descBuf The attribute list descriptor to be translated into an
 * attribute list.
 * \param[in] descLen The size of the attribute list descriptor.
 * \param[out] importedUnreconciledAttrList Imported attribute list.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL or @a descLen is zero or the
 *   array of bytes indicated by @a descBuf and @a descLen do not constitute a valid
 *   exported attribute list descriptor for an unreconciled attribute list.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 */
NvSciError NvSciSyncAttrListIpcImportUnreconciled(
    NvSciSyncModule module,
    NvSciIpcEndpoint ipcEndpoint,
    const void* descBuf,
    size_t descLen,
    NvSciSyncAttrList* importedUnreconciledAttrList);

/**
 * \brief Translates an exported attribute list descriptor (potentially received
 * from any process) into an attribute list.
 * It also validates that the attribute list to be imported will
 * be a reconciled attribute list that is consistent with the constraints in an
 * array of input unreconciled attribute lists. This is recommended while
 * importing what is expected to be a reconciled attribute list to cause
 * NvSciSync to validate the reconciled attributes against the input attributes
 * so that the importing process can be sure that an NvSciSync object consistent
 * with the imported reconciled list will satisfy the input constraints.
 *
 * \param[in] module The NvSciSync module instance with which to associate the
 * imported NvSciSyncAttrList.
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller
 *            receives the exported attribute list descriptor.
 * \param[in] descBuf The attribute list descriptor to be translated into an
 * attribute list.
 * \param[in] descLen The size of the attribute list descriptor.
 * \param[in] inputUnreconciledAttrListArray The array of attribute lists against
 * which the new attribute list is to be validated.
 * \param[in] inputUnreconciledAttrListCount The number of attribute lists in
 * inputUnreconciledAttrListArray. If inputUnreconciledAttrListCount is
 * non-zero, then this operation will fail with an error unless all the
 * constraints of all the attribute lists in inputUnreconciledAttrListArray are
 * met by the imported attribute list.
 * \param[out] importedReconciledAttrList Imported attribute list.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a module is NULL or
 *      - @a ipcEndpoint is NULL or
 *      - the array of bytes indicated by @a descBuf and @a descLen do not
 *        constitute a valid exported attribute list descriptor for a
 *        reconciled attribute list or
 *      - @a importedReconciledAttrList is NULL or
 *      - both @a inputUnreconciledAttrListCount is non-zero and
 *        @a inputUnreconciledAttrListArray is NULL or
 *      - any of the NvSciSyncAttrLists in @a inputUnreconciledAttrListArray
 *        is not bound to a module.
 * - ::NvSciError_AttrListValidationFailed if the attribute list to be
 *   imported either would not be a reconciled attribute list or would not meet
 *   at least one of constraints in one of the input unreconciled attribute
 *   lists.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
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
 * \brief Frees an exported attribute list descriptor previously returned by
 * NvSciSyncAttrListExport().
 *
 * \param[in] descBuf The exported attribute list descriptor to be freed.
 * \return void
 */
void NvSciSyncAttrListFreeDesc(
    void* descBuf);

/**
 * \brief Frees any resources allocated for the specified fence.
 *
 * Upon return, the memory pointed to by @a syncFence is guaranteed to be all
 * zeros and thus the NvSciSyncFence is returned to its initialized state.
 *
 * \param[in,out] syncFence A pointer to NvSciSyncFence.
 * \return void
 */
void NvSciSyncFenceClear(
    NvSciSyncFence* syncFence);

/**
 * \brief Duplicates @a srcSyncFence into @a dstSyncFence, such that any wait on
 * destSyncFence will complete at the same time as a wait on srcSyncFence.
 * If srcSyncFence is in a cleared state, then so also will be dstSyncFence.
 *
 * \param[in] srcSyncFence The source NvSciSyncFence object.
 * \param[out] dstSyncFence The destination NvSciSyncFence object.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL.
 * - ::NvSciError_InsufficientResource if system resource allocation failed.
 */
NvSciError NvSciSyncFenceDup(
    const NvSciSyncFence* srcSyncFence,
    NvSciSyncFence* dstSyncFence);

/**
 * \brief Allocates a sync object that meets all the constraints
 * in the specified reconciled attribute list.
 *
 * @note This function does not take ownership of the reconciledList. The caller
 * remains responsible for freeing the reconciledList. The caller may free the
 * reconciledList any time after this function is called.
 *
 * \param[in] reconciledList The reconciled attribute list generated through
 * NvSciSyncAttrListReconcile().
 * \param[out] syncObj Identifier representing NvSciSync object.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL or @a reconciledList
 *   is not a reconciled attribute list.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_InsufficientResource if system resource allocation failed.
 */
NvSciError NvSciSyncObjAlloc(
    NvSciSyncAttrList reconciledList,
    NvSciSyncObj* syncObj);

/**
 * \brief Duplicates a valid NvSciSyncObj.
 *
 * \param[in] syncObj The source NvSciSyncObj.
 * \param[out] dupObj The destination NvSciSyncObj.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any argument is NULL.
 */
NvSciError NvSciSyncObjDup(
    NvSciSyncObj syncObj,
    NvSciSyncObj* dupObj);

/**
 * \brief Retrieves the attribute list from an allocated
 * NvSciSync Object.
 *
 * @note The retrieved attribute list from an object is always read-only and
 * attribute values in the list cannot be changed using set attribute API.
 * In addition, the returned attribute list must not be freed with
 * NvSciSyncAttrListFree().
 *
 * \param[in] syncObj Handle corresponding to NvSciSync object from which the
 * attribute list has to be retrieved.
 * \param[out] syncAttrList pointer to the retrieved attribute list.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a syncObj or @a syncAttrList is NULL.
 */
NvSciError NvSciSyncObjGetAttrList(
    NvSciSyncObj syncObj,
    NvSciSyncAttrList* syncAttrList);

/**
 * \brief Destroys the NvSciSync object and all memory that was allocated for it.
 *
 * \param[in] syncObj A valid NvSciSync object.
 *
 * \return void
 */
void NvSciSyncObjFree(
    NvSciSyncObj syncObj);

/**
 * \brief Exports a NvSciSync object into an NvSciIpc-transferable object
 * binary descriptor.
 *
 * The blob can be transferred to the other signalers and waiters to create a
 * matching NvSciSync object.
 *
 * \param[in] syncObj NvSciSync object to export.
 * \param[in] permissions Flag indicating the expected access permission.
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller may
 *            send the exported sync object descriptor.
 * \param[out] desc NvSciSync fills in caller-supplied descriptor with exported
 *             form of NvSciSyncObj shared across an NvSciIpc channel.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a desc is NULL or
 *      - @a syncObj is invalid or
 *      - @a permissions is invalid or
 *      - @a ipcEndpoint is invalid.
 * - ::NvSciError_InsufficientResource if system resource allocation failed.
 */
NvSciError NvSciSyncObjIpcExport(
    NvSciSyncObj syncObj,
    NvSciSyncAccessPerm permissions,
    NvSciIpcEndpoint ipcEndpoint,
    NvSciSyncObjIpcExportDescriptor* desc);

/**
 * \brief Creates an NvSciSync object based on supplied binary descriptor and
 * returns the sync object.
 *
 * It is called from the waiter after it receives the binary blob by
 * the signaler who has created the binary descriptor.
 * Waiter will create its own sync object and return as output.
 *
 * @note This function does not take ownership of @a inputAttrList. The caller
 * remains responsible for freeing @a inputAttrList. The caller may free
 * @a inputAttrList any time after this function is called.
 *
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller receives
 *            the exported sync object descriptor.
 * \param[in] desc The exported form of NvSciSyncObj received through the
 *            NvSciIpc channel.
 * \param[in] inputAttrList The attribute list returned by NvSciSyncImportAttrList().
 * \param[in] permissions Flag indicating the expected access permission.
 * \param[in] timeoutUs The import timeout in micro seconds, -1 for infinite
 *            wait.
 * \param[out] syncObj The waiter's NvSciSync Object.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a ipcEndpoint is invalid or
 *      - @a desc is invalid or
 *      - @a inputAttrList is invalid or
 *      - @a permissions is invalid or
 *      - @a timeoutUs is invalid
 *      - @a syncObj is NULL.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_AttrListValidationFailed if inputAttrList constraints
 *   are not met by attributes of imported NvSciSync object.
 * - ::NvSciError_InvalidOperation if called by waiter not already registered
 *   as part of NvSciSyncAttrList.
 * - ::NvSciError_Timeout if import not completed in given timeout.
 */
NvSciError NvSciSyncObjIpcImport(
    NvSciIpcEndpoint ipcEndpoint,
    const NvSciSyncObjIpcExportDescriptor* desc,
    NvSciSyncAttrList inputAttrList,
    NvSciSyncAccessPerm permissions,
    int64_t timeoutUs,
    NvSciSyncObj* syncObj);

/**
 * \brief Exports a NvSciSyncFence into a binary descriptor shareable across the
 * NvSciIpc channel.
 *
 * \param[in] syncFence A pointer to NvSciSyncFence object to be exported.
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller may
 *            send the exported fence descriptor.
 * \param[out] desc The exported form of NvSciSyncFence shared across an NvSciIpc
 * channel.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a syncFence is invalid or
 *      - @a ipcEndpoint is invalid or
 *      - @a desc is NULL.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 */
NvSciError NvSciSyncIpcExportFence(
    const NvSciSyncFence* syncFence,
    NvSciIpcEndpoint ipcEndpoint,
    NvSciSyncFenceIpcExportDescriptor* desc);

/**
 * \brief Fills in the NvSciSyncFence object based on supplied binary descriptor.
 *
 * \param[in] syncObj An object of type NvSciSyncObj.
 * \param[in] desc The exported form of NvSciSyncFence.
 * \param[out] syncFence A pointer to NvSciSyncFence object.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a syncFence is NULL or
 *      - @a desc is NULL or invalid or not consistent with @a syncObj or
 *      - @a syncFence is NULL.
 */
NvSciError NvSciSyncIpcImportFence(
    NvSciSyncObj syncObj,
    const NvSciSyncFenceIpcExportDescriptor* desc,
    NvSciSyncFence* syncFence);

/**
 * \brief Generates next point on sync timeline of an NvSciSync object and fills
 * in the supplied NvSciSyncFence object.
 *
 * This function is called when the CPU is the signaler.
 * \param[in] syncObj A valid NvSciSync object.
 * \param[out] syncFence A pointer to NvSciSyncFence object.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a syncObj or @a syncFence is NULL or
 *   CPU is not a signaler.
 */
NvSciError NvSciSyncObjGenerateFence(
    NvSciSyncObj syncObj,
    NvSciSyncFence* syncFence);

/**
 * \brief Signal the NvSciSync object.
 *
 * This function is called when the CPU is the signaler.
 *
 * \param[in] syncObj A valid NvSciSync object to signal.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a syncObj is invalid or CPU is not a signaler.
 * - ::NvSciError_ResourceError if signal operation cannot be completed.
 */
NvSciError NvSciSyncObjSignal(
    NvSciSyncObj syncObj);

/**
 * \brief Performs a synchronous wait on the @ref NvSciSyncFence object until the
 * fence has been signaled or the timeout expires. Any CpuWaitContext may be
 * used for waiting on any NvSciSyncFence provided they were created in the same
 * NvSciSyncModule context.
 * One CpuWaitContext can be used to wait on only one NvSciSyncFence at a time
 * but it can be used to wait on a different NvSciSyncFence at a different time.
 *
 * \param[in] syncFence The @a syncFence to wait on.
 * \param[in] context CPU wait context holding resources needed
 *            to perform waiting.
 * \param[in] timeoutUs Timeout to wait for in micro seconds, -1 for infinite
 *            wait.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a syncFence is NULL or @a context is NULL or
 *   @a syncFence and @a context are associated with different modules or
 *   @a timeoutUs is invalid or @a if caller doesn't have CPU wait permissions.
 * - ::NvSciError_ResourceError if wait operation did not complete
 *   successfully.
 * - ::NvSciError_Timeout if wait did not complete in the given timeout.
 */
NvSciError NvSciSyncFenceWait(
    const NvSciSyncFence* syncFence,
    NvSciSyncCpuWaitContext context,
    int64_t timeoutUs);

/**
 * \brief NvSciSync Utility functions
 */

/**
 * \brief Gets the attribute value from the attribute list with the given key.
 *
 * \param[in] attrList Attribute list to retrieve the attribute key and value from.
 * \param[in] key The key for the attribute value to retrieve.
 * \param[out] value A pointer to the location where the attribute value is written.
 * \param[out] len Length of the value structure.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if @a attrList is NULL or @a value is NULL or
 *      @a len is NULL.
 */
NvSciError NvSciSyncAttrListGetAttr(
    NvSciSyncAttrList attrList,
    NvSciSyncAttrKey key,
    const void** value,
    size_t* len);

/**
 * \brief Reconcile the input unreconciled attribute list(s) into a new
 * reconciled attribute list and allocate a sync object that meets all the
 * constraints in the new reconciled attribute list.
 *
 * \param[in] inputArray Array containing unreconciled attribute list(s) to reconcile.
 * \param[in] inputCount Number of unreconciled attributes lists in @a inputArray.
 * \param[out] syncObj The new NvSciSync object.
 * \param[out] newConflictList Unreconciled attribute list consisting of the
 * key/value pairs which caused the reconciliation failure. This field is
 * populated only if the reconciliation failed.
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a inputCount is zero or
 *      - @a inputArray is NULL or
 *      - @a syncObj is NULL or
 *      - @a newConflictList is NULL.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_ReconciliationFailed if reconciliation failed.
 * - ::NvSciError_InsufficientResource if system resource allocation failed.
 */
NvSciError NvSciSyncAttrListReconcileAndObjAlloc(
    const NvSciSyncAttrList inputArray[],
    size_t inputCount,
    NvSciSyncObj* syncObj,
    NvSciSyncAttrList* newConflictList);

/**
 * \brief Exports an NvSciSync attribute list and object into an
 * NvSciIpc-transferable object binary blob pointed to by the data param.
 *
 * The blob can be transferred to the other signalers and waiters to create a
 * matching NvSciSync object.
 *
 * \param[in] syncObj NvSciSync object to export.
 * \param[in] permissions Flag indicating the expected access permission.
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller may send
 *            the exported attribute list and sync object descriptor.
 * \param[out] attrListAndObjDesc Exported form of attribute list and
 * NvSciSyncObj shareable across an NvSciIpc channel.
 * \param[out] attrListAndObjDescSize Size of the exported blob.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a attrListAndObjDesc is NULL or
 *      - @a syncObj is invalid or
 *      - @a permissions is invalid or
 *      - @a ipcEndpoint is invalid.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 */
NvSciError NvSciSyncIpcExportAttrListAndObj(
    NvSciSyncObj syncObj,
    NvSciSyncAccessPerm permissions,
    NvSciIpcEndpoint ipcEndpoint,
    void** attrListAndObjDesc,
    size_t* attrListAndObjDescSize);

/**
 * \brief Frees an exported attribute-list-and-sync-object descriptor
 * returned by NvSciSyncIpcExportAttrListAndObj().
 *
 * \param[in] attrListAndObjDescBuf Exported attribute-list-and-sync-object
 * descriptor to be freed.
 * \return void
 */
void NvSciSyncAttrListAndObjFreeDesc(
    void* attrListAndObjDescBuf);

/**
 * \brief Creates an NvSciSync object based on the supplied binary descriptor and
 * returns the sync object.
 *
 * This function is called from the waiter after it receives the binary blob by
 * the signaler who has created the binary descriptor.
 * Waiter will create its own sync object and return as output.
 *
 * \param[in] module A NvSciSyncModule to associate the imported NvSciSyncAttrList with.
 * \param[in] ipcEndpoint The NvSciIpcEndpoint through which the caller receives
 *            the exported attribute list and sync object descriptor.
 * \param[in] attrListAndObjDesc Exported form of attribute list and NvSciSyncObj
 * received through NvSciIpc channel.
 * \param[in] attrListAndObjDescSize Size of the exported blob.
 * \param[in] attrList Receiver/caller side attribute list array.
 * \param[in] attrListCount Number of lists in the @a attrList array.
 * \param[in] minPermissions Flag indicating the expected access permission.
 * \param[in] timeoutUs Import timeout in micro seconds, -1 for infinite
 *            wait.
 * \param[out] syncObj Waiter's NvSciSync object.
 *
 * \return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success if successful.
 * - ::NvSciError_BadParameter if any of the following occurs:
 *      - @a ipcEndpoint is invalid or
 *      - @a attrListAndObjDesc is invalid or
 *      - @a minPermissions is invalid or
 *      - @a timeoutUs are invalid or
 *      - @a attrList[] is NULL or
 *      - @a syncObj is NULL.
 * - ::NvSciError_InsufficientMemory if memory allocation failed.
 * - ::NvSciError_AttrListValidationFailed if attrList[] constraints
 *   are not met by attributes of imported NvSciSync object.
 * - ::NvSciError_InvalidOperation if called by waiter not already registered
 *   as part of NvSciSyncAttrList.
 * - ::NvSciError_Timeout if import not completed in given timeout.
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

#if defined(__cplusplus)
}
#endif // __cplusplus
 /** @} */
#endif // INCLUDED_NVSCISYNC_H
