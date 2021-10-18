/*
 * Copyright (c) 2014-2019 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_nvrm_gpu_H
#define INCLUDED_nvrm_gpu_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "nvcommon.h"
#include "nverror.h"

#if defined(__cplusplus)
extern "C"
{
#endif

#if defined(NVRM_GPU_BUILD_VARIANT)
namespace nvrm_gpu
{
#endif

/// @brief Inserts 'nvrm_gpu' namespace prefix in nvrm_gpu's internal unit test build.
///
/// This is a no-op in all other builds.
///
/// It's used in this header to add the proper namespace specifier to the attribute structs
/// defined with NVRM_GPU_DEFINE_*_ATTR, when being part of nvrm_gpu's unit test build.
#if defined(NVRM_GPU_UNIT_TEST_BUILD) && (NVRM_GPU_UNIT_TEST_BUILD)
#define NVRM_GPU_ATTR_NAMESPACE(x) ::nvrm_gpu::x
#else
#define NVRM_GPU_ATTR_NAMESPACE(x) x
#endif

/// @file
/// @brief <b>NVIDIA GPU access API</b>
///
/// @ifnot NVRM_GPU_PRIVATE
/// @b Description: This file contains API for GPU library management, clock
/// controls, sensors, and device events.
/// @endif
///
/// @defgroup nvrm_gpu_group GPU access API
///
/// GPU management and access API.
///
/// @ifnot NVRM_GPU_PRIVATE
/// For related information, see
/// <a href="../Clock Freq and Thermal/discrete_GPU.html" target="_blank">
/// Discrete GPU</a> in the _Development Guide_.
///
/// General usage notes:
///
/// - This library uses object-oriented design. In general, objects are referred by
///   handles.
///
/// - When destroying an object, the child objects are not destroyed by default.
///   Additionally, it is an error to destroy a parent before its child.
///
/// - In object creation, the attribute pointer (where applicable) can always be
///   NULL, which indicates defaults.
///
/// - In object creation, the first parameter specifies the parent. Some objects
///   may be created for parents of different types.
///
/// - Whenever a function that returns a new handle is invoked, the handle is
///   guaranteed to be unique as well as the object it points to. There will be
///   1-to-1 mapping between handles and objects.
///
/// - It is a fatal error to provide unexpected handle type. That will produce
///   either an assert (debug builds) or undefined behavior (release builds)
///
/// - In general, the const pointers returned by functions are valid as long as
///   the related object is valid, unless otherwise stated. And in particular,
///   they shall not be freed by the caller. For example, the device list
///   returned by NvRmGpuLibListDevices() is valid until the lib handle is
///   closed.
///
/// - This library is thread-safe with the following rule: an object can be
///   closed only if there are no concurrent operations on-going on
///   it. Attempting to close an object during an on-going operation is a fatal
///   error.
/// @endif

// -------------------------------------------------
// --------------- API Groups ----------------------
// -------------------------------------------------

/// @defgroup nvrm_gpu_safety_group GPU access API (safety subset)

/// @defgroup nvrm_gpu_lib_group GPU access API: Library
/// @ingroup nvrm_gpu_group

/// @defgroup nvrm_gpu_lib_safety_group GPU access API: Library (safety subset)
/// @ingroup nvrm_gpu_lib_group
/// @ingroup nvrm_gpu_safety_group

/// @defgroup nvrm_gpu_device_group GPU access API: Device management
/// @ingroup nvrm_gpu_group

/// @defgroup nvrm_gpu_device_safety_group GPU access API: Device management (safety subset)
/// @ingroup nvrm_gpu_device_group
/// @ingroup nvrm_gpu_safety_group

// -------------------------------------------------
// --------------- Handles -------------------------
// -------------------------------------------------

/// @ingroup nvrm_gpu_lib_safety_group
/// @brief Library handle
///
/// @sa NvRmGpuLibOpen()
/// @sa NvRmGpuLibGetInfo()
/// @sa NvRmGpuLibClose()
///
typedef struct NvRmGpuLibRec NvRmGpuLib;

/// @ingroup nvrm_gpu_device_safety_group
/// @brief Device handle
///
/// @sa NvRmGpuDeviceCreate()
/// @sa NvRmGpuDeviceGetInfo()
/// @sa NvRmGpuDeviceClose()
///
typedef struct NvRmGpuDeviceRec NvRmGpuDevice;

/// @ingroup nvrm_gpu_device_group
/// @brief Device event session handle
///
/// @sa NvRmGpuDeviceEventSessionOpen()
/// @sa NvRmGpuDeviceEventSessionClose()
typedef struct NvRmGpuDeviceEventSessionRec NvRmGpuDeviceEventSession;


// -------------------------------------------------
// --------------- Library functions ---------------
// -------------------------------------------------

/// @ingroup nvrm_gpu_lib_safety_group
/// @brief Extensible attribute structure for #NvRmGpuLibOpen()
///
/// This structure specifies the attributes for opening the nvrm_gpu
/// library. Use #NVRM_GPU_DEFINE_LIB_OPEN_ATTR() to define the attribute struct
/// with defaults.
///
/// Example:
///
///     // define libOpenAttr with default values
///     NVRM_GPU_DEFINE_LIB_OPEN_ATTR(libOpenAttr);
///
///     // open the library
///     NvRmGpuLib *hLib = NvRmGpuLibOpen(&libOpenAttr);
///
/// @sa NvRmGpuLibOpen()
///
typedef struct NvRmGpuLibOpenAttrRec
{
    /// @brief Dummy field for C/C++ ABI compatibility
    uint32_t reserved;

} NvRmGpuLibOpenAttr;

/// @ingroup nvrm_gpu_lib_safety_group
/// @brief Definer macro for #NvRmGpuLibOpenAttr.
///
/// This macro defines a variable of type #NvRmGpuLibOpenAttr with
/// the default values.
///
#define NVRM_GPU_DEFINE_LIB_OPEN_ATTR(x) \
    NVRM_GPU_ATTR_NAMESPACE(NvRmGpuLibOpenAttr) x = { 0U }

/// @ingroup nvrm_gpu_lib_safety_group
/// @brief Opens a new instance of the nvrm_gpu library.
///
/// This function creates a new library handle and initializes the library if
/// necessary. After the library is no longer used, the library handle should be
/// closed with NvRmGpuLibClose() to avoid memory leaks.
///
/// @param[in]  attr  Extensible library open attributes, or @NULL for defaults.
///                   Currently unused.
///
/// @return Library handle, or @NULL if the library could not be initialized.
///
/// @remark There can be multiple concurrent instances of the library in the
///         process. Global shared data used by the library is internally
///         reference counted: the first instance will initialize the shared
///         resources; and when the last instance is closed, the shared
///         resources are freed.
///
/// @remark If the library initialization fails, an error message is printed on
///         stderr with an error code for diagnostics.
///
/// <b>Example:</b>
///
///     // open the library
///     NvRmGpuLib *hLib = NvRmGpuLibOpen(NULL);
///
///     if (hLib != NULL)
///     {
///         NvRmGpuDevice *hDevice = NULL;
///         NvError err;
///
///         err = NvRmGpuDeviceOpen(hLib, NVRM_GPU_DEVICE_INDEX_DEFAULT, NULL, &hDevice);
///         if (err == NvSuccess)
///         {
///             // use the device
///             ...
///
///             // all done, close the device
///             NvRmGpuDeviceClose(hDevice);
///         }
///         else
///         {
///             // deal with the error
///         }
///
///         /// all done, close the library
///         NvRmGpuLibClose(hLib);
///     }
///     else
///     {
///         // deal with the error
///     }
///
/// @sa NvRmGpuLibClose()
///
NvRmGpuLib *NvRmGpuLibOpen(const NvRmGpuLibOpenAttr *attr);

/// @ingroup nvrm_gpu_lib_safety_group
/// @brief Closes the library and releases all resources.
///
/// @param[in] hLib    Library handle. May be @NULL, in which case this function
///                    is a no-op.
///
/// @return The usual NvError code
/// @retval NvSuccess  The library was closed and all related resources were
///                    freed successfully
/// @retval NvError_*  Unspecified error. The error code is returned for
///                    diagnostic purposes. The library object is closed regardless
///                    but some resources may have failed to close gracefully.
///
/// @remark Every resource attached to the library must be closed before closing
/// the library to avoid leaks and dangling pointers. In debug builds, nvrm_gpu
/// will keep track of the associated resource and it will assert in case this
/// contract is violated.
///
/// @sa NvRmGpuLibOpen()
/// @sa NvRmGpuDeviceClose()
///
NvError NvRmGpuLibClose(NvRmGpuLib *hLib);

/// @ingroup nvrm_gpu_lib_safety_group
/// @brief Device attachment state.
///
/// @sa NvRmGpuLibListDevices(), NvRmGpuLibDeviceListEntry
/// @sa NvRmGpuDeviceOpen()
/// @sa NvRmGpuLibAttachDevice(), NvRmGpuLibDetachDevice()
///
typedef enum
{
    /// @brief Device is attached and may be opened with NvRmGpuDeviceOpen()
    NvRmGpuLibDeviceState_Attached,

    /// @brief Device is detached and powered off. Device must be attached
    /// before it can be opened.
    NvRmGpuLibDeviceState_Detached,

    /// @brief Device exists, but not enough privileges to access.
    NvRmGpuLibDeviceState_InsufficientPrivileges,

    /// @brief Device state is not known. Prober failed to determine device
    /// state.
    NvRmGpuLibDeviceState_Unknown,

} NvRmGpuLibDeviceState;


/// @ingroup nvrm_gpu_lib_safety_group
/// @brief Device list entry
///
/// @sa NvRmGpuLibListDevices()
///
typedef struct NvRmGpuLibDeviceListEntryRec
{
    /// @brief Internal device index. Used in NvRmGpuDeviceOpen()
    int deviceIndex;

    /// @brief Device attachment state.
    ///
    /// @sa NvRmGpuDeviceState
    NvRmGpuLibDeviceState deviceState;

    /// @brief Indicates whether the device can be attached/detached by the
    /// user.
    ///
    /// @sa NvRmGpuLibAttachDevice(), NvRmGpuLibDetachDevice()
    bool hasDeviceStateControl;

    /// @brief Informative device name
    ///
    /// This is the 'probe' name of the device. The name is backend-specific. Examples:
    /// - nouveau:/dev/dri/renderD128
    /// - nvgpu:/dev/nvhost-gpu
    /// - nvgpu:/dev/nvgpu-pci/card-0001:15:00.0
    ///
    /// @remark The path is informational only. If probe-time device
    /// identification is required, please file a feature request.
    const char *name;
} NvRmGpuLibDeviceListEntry;

/// @ingroup nvrm_gpu_lib_safety_group
/// @brief Returns the list of probed GPUs
///
/// Returns the list of probed GPUs. The list is valid until the library handle
/// is closed.
///
/// @param[in]  hLib         Library handle
/// @param[out] pNumDevices  Non-@NULL Pointer to receive the number of entries in the list
///
/// @return     Pointer to the list of probed GPUs (C array). The caller must
///             not attempt to free the pointer.
///
/// @remark The first device listed is considered the primary GPU.
///
/// @remark The device index numbers returned are non-negative, unique and in
///   ascending order. Numbering may be discontiguous, and specifically, the
///   index numbers will likely not start at 0.
///
/// @sa NvRmGpuDeviceOpen()
/// @sa NvRmGpuLibAttachDevice()
/// @sa NvRmGpuLibDetachDevice()
///
const NvRmGpuLibDeviceListEntry *NvRmGpuLibListDevices(NvRmGpuLib *hLib, size_t *pNumDevices);

/// @ingroup nvrm_gpu_lib_group
/// @brief Attaches and powers up a GPU
///
/// @param[in]   hLib         Library handle
/// @param[in]   deviceIndex  GPU device index. See NvRmGpuLibListDevices().
///
/// @return The usual error code
/// @retval NvSuccess              Device attached successfully
/// @retval NvError_BadValue       Bad \c deviceIndex
/// @retval NvError_NotSupported   This device does not support attach/detach functionality
/// @retval NvError_AccessDenied   Insufficient user privileges
/// @retval NvError_ResourceError  Device attachment failed. Likely, the device failed to boot.
/// @retval NvError_*              Unspecified error. Error code returned for diagnostics.
///
/// @remark This is privileged functionality for system management
/// purposes. Regular applications should not invoke this function.
///
/// @remark Generally, only dGPUs can be attached and detached. The typical use
/// case is zero-power idling (ZPI) for power saving when the dGPU is not
/// required.
///
/// @sa NvRmGpuLibDetachDevice()
NvError NvRmGpuLibAttachDevice(NvRmGpuLib *hLib, int deviceIndex);

/// @ingroup nvrm_gpu_lib_group
/// @brief Powers down and detaches a GPU.
///
/// @param[in]   hLib         Library handle
/// @param[in]   deviceIndex  GPU device index. See NvRmGpuLibListDevices().
///
/// @return The usual error code
/// @retval NvSuccess              Device detached successfully
/// @retval NvError_BadValue       Bad \c deviceIndex
/// @retval NvError_NotSupported   This device does not support attach/detach functionality
/// @retval NvError_AccessDenied   Insufficient use privileges
/// @retval NvError_ResourceError  Device attachment failed. Likely, the device failed to boot.
/// @retval NvError_*              Unspecified error. Error code returned for diagnostics.
///
/// @remark This is privileged functionality for system management
/// purposes. Regular applications should not invoke this function.
///
/// @remark Generally, only dGPUs can be attached and detached. The typical use
/// case is zero-power idling (ZPI) for power saving when the dGPU is not
/// required.
///
/// @sa NvRmGpuLibAttachDevice()
NvError NvRmGpuLibDetachDevice(NvRmGpuLib *hLib, int deviceIndex);


/// @ingroup nvrm_gpu_device_safety_group
/// @brief Pseudo-index for the default (primary) device.
///
/// @remark By default, this is the first GPU enumerated by
/// NvRmGpuLibListDevices(). This can be overridden with environment variable
/// <tt>NVRM_GPU_DEFAULT_DEVICE_INDEX</tt>.
///
/// @sa NvRmGpuDeviceOpen()
///
#define NVRM_GPU_DEVICE_INDEX_DEFAULT (-1)

/// @ingroup nvrm_gpu_device_safety_group
/// @brief Inter-engine synchronization type for GPU jobs
///
/// The usual GPU channels (also known as KMD kickoff channels) support
/// attaching pre-fences and post-fences with job submission. The pre-fence is a
/// synchronization condition that must be met before the job execution can
/// begin. Respectively, the post-fence is a synchronization condition that will
/// be triggered after the job execution has completed. This allows a larger
/// task to be split into parts where GPU and other engines seamlessly process
/// the data in multiple stages. For example:
/// - camera produces a frame
/// - GPU processes the frame after camera has produced it
/// - Display controller displays the frame after GPU has processed it
///
/// Depending on the operating system and HW capabilities, different
/// synchronization object types are available:
///
/// - <b>Tegra HOST 1X syncpoint</b> --- Syncpoint is a hardware register
///   provided by the SoC (generally, 32-bit integer with wrap-around safe
///   semantics). Pre-sync condition waits until the syncpoint value reaches a
///   threshold, and post-sync condition increases the syncpoint value.
///
/// - <b>Android/Linux sync fd</b> --- Synchronization fence backed up by a file
///   (sync_file). The synchronization fence has two stages: untriggered
///   (initial state) and triggered. Pre-sync condition always waits for the
///   fence to become triggered, and post-sync condition triggers the fence.
///
/// @remark This is not to be confused with GPU semaphores. GPU semaphores are
/// usually used to synchronize jobs that are executed within a single device,
/// or between multiple GPUs, or sometimes between the GPU and the CPU. GPU
/// semaphore is simply a memory location with semantics similar to Tegra HOST1X
/// syncpoints. Generally, waiters wait until the value at the memory location
/// reaches a specific threshold, and waiters are released by setting the
/// semaphore to the threshold value or above (but there are other modes, too.)
///
/// @sa https://www.kernel.org/doc/Documentation/sync_file.txt
/// @sa NvRmGpuChannelKickoffPb()
///
typedef enum
{
    /// @brief Default sync type
    ///
    /// @remark Depending on the context, this is platform default,
    /// device-default, or channel-default.
    ///
    /// @sa NvRmGpuDeviceInfo::defaultSyncType
    /// @sa NvRmGpuChannelInfo::syncType
    NvRmGpuSyncType_Default,

    /// @brief Synchronization type is Android/Linux sync fd.
    NvRmGpuSyncType_SyncFd,

    /// @brief Synchronization type is Tegra HOST1X syncpoint.
    NvRmGpuSyncType_Syncpoint,
} NvRmGpuSyncType;


/// @ingroup nvrm_gpu_device_safety_group
/// @brief Extensible attribute structure for #NvRmGpuDeviceOpen()
///
/// @remark Use NVRM_GPU_DEFINE_DEVICE_OPEN_ATTR() to define the attribute
/// variable with defaults.
///
/// @sa NvRmGpuDeviceReadTimeNs()
///
typedef struct NvRmGpuDeviceOpenAttrRec
{
    /// @brief The default sync type for this device.
    ///
    /// @deprecated This field should be left with the default value. Use
    /// NvRmGpuChannelAttr::defaultSyncType. NvRmGpuChannelKickoffPbAttr() also
    /// accepts mixing the kickoff sync types with
    /// NvRmGpuChannelKickoffPbAttr::completionSyncType .
    ///
    /// Default: #NvRmGpuSyncType_Default
    ///
    /// @remark It is a fatal error to request an unsupported sync type.
    ///
    /// @remark This field should be removed.
    NvRmGpuSyncType syncType;

    /// @brief Ignored field
    ///
    /// @deprecated This field is not in use anymore. It used to specify between
    /// sandboxable channels (in Android web browser context) and regular
    /// channels. Sandboxable channels used to require extra resources, but that
    /// is not true anymore and channels are always sandbox-friendly.
    ///
    /// Default: @false
    ///
    /// @remark This field should be removed.
    bool sandboxFriendlyChannels;
} NvRmGpuDeviceOpenAttr;

/// @ingroup nvrm_gpu_device_safety_group
/// @brief Definer macro for #NvRmGpuDeviceOpen().
///
/// This macro defines a variable of type #NvRmGpuDeviceOpenAttr with
/// the default values.
///
/// @sa NvRmGpuDeviceOpen()
///
#define NVRM_GPU_DEFINE_DEVICE_OPEN_ATTR(x) \
    NVRM_GPU_ATTR_NAMESPACE(NvRmGpuDeviceOpenAttr) x = { NVRM_GPU_ATTR_NAMESPACE(NvRmGpuSyncType_Default), false }


/// @ingroup nvrm_gpu_device_safety_group
/// @brief Opens a GPU device.
///
/// @param[in]  hLib         Library handle
/// @param[in]  deviceIndex  Device index (NvRmGpuLibDeviceListEntry::deviceIndex) or
///                          #NVRM_GPU_DEVICE_INDEX_DEFAULT for the default device.
/// @param[in]  attr         Pointer to device open attributes or @NULL for defaults.
/// @param[out] phDevice     Pointer to receive the device handle.
///
/// @return The usual NvError code
/// @retval NvSuccess                Device opened successfully.
/// @retval NvError_BadValue         Bad device index
/// @retval NvError_DeviceNotFound   Device node not found
/// @retval NvError_AccessDenied     Not enough privileges to access the device
/// @retval NvError_*                Unspecified error. Error code returned for diagnostic purposes.
///
/// @remark Only attached GPUs can be opened. See NvRmGpuLibDeviceListEntry::deviceState.
///
/// @remark See #NVRM_GPU_DEVICE_INDEX_DEFAULT for the discussion on default device.
///
/// @sa NvRmGpuLibAttachDevice(), NvRmGpuLibDetachDevice()
/// @sa NvRmGpuDeviceGetInfo()
/// @sa NvRmGpuDeviceOpenAttr, NVRM_GPU_DEFINE_DEVICE_OPEN_ATTR()
///
NvError NvRmGpuDeviceOpen(NvRmGpuLib *hLib, int deviceIndex, const NvRmGpuDeviceOpenAttr *attr,
                          NvRmGpuDevice **phDevice);

/// @ingroup nvrm_gpu_device_safety_group
/// @brief Closes the GPU device.
///
/// @param[in]  hDevice  Device handle to close. May be @NULL.
///
/// @return The usual NvError code
/// @retval NvSuccess               Device closed and all related resources released successfully,
///                                 or device handle was @NULL.
/// @retval NvError_*               Unspecified error. Device handle is closed, but some resources
///                                 may be left unreleased. Error code is returned only for diagnostic
///                                 purposes.
///
/// @remark Every resource attached to the device must be closed before closing
/// the device to avoid leaks and dangling pointers. In debug builds, nvrm_gpu
/// will keep track of the associated resource and it will assert in case this
/// contract is violated.
///
/// @sa NvRmGpuDeviceOpen()
/// @sa NvRmGpuAddressSpaceClose(), NvRmGpuChannelClose(),
/// NvRmGpuCtxSwTraceClose(), NvRmGpuTaskSchedulingGroupClose(),
/// NvRmGpuRegOpsSessionClose(), NvRmGpuDeviceEventSessionClose()
///
NvError NvRmGpuDeviceClose(NvRmGpuDevice *hDevice);


#if NVOS_IS_LINUX || NVOS_IS_QNX

/// @ingroup nvrm_gpu_device_group
/// @brief Format macro for printf for printing #NvRmGpuClockAsyncReqHandle
///
/// The use is similar to PRIu64 for printing uint64_t.
#define NVRM_GPU_CLOCK_ASYNC_REQ_HANDLE_PRIFMT "d"

/// @ingroup nvrm_gpu_device_group
/// @brief OS-specific type of asynchronous clock request handle.
///
/// Asynchronous clock request handle is a waitable handle for clock change
/// requests. This allows one to issue multiple clock change requests
/// concurrently (e.g., GPU clock and memory clock for multiple devices) and
/// then wait for all of them to complete.
///
/// @remark This is file descriptor on QNX and Linux.
typedef int NvRmGpuClockAsyncReqHandle;
#else

/// @ingroup nvrm_gpu_device_group
/// @brief Format macro for printf for printing #NvRmGpuClockAsyncReqHandle
///
/// The use is similar to PRIu64 for printing uint64_t.
#define NVRM_GPU_CLOCK_ASYNC_REQ_HANDLE_PRIFMT "p"

/// @struct NvRmGpuClockAsyncNotImplemented
/// @ingroup nvrm_gpu_device_group
/// @brief OS-specific type of asynchronous clock request handle (unimplemented).
///
/// Marker for unimplemented handle type.
///
///
/// @typedef NvRmGpuClockAsyncReqHandle
/// @ingroup nvrm_gpu_device_group
/// @brief OS-specific type of asynchronous clock request handle.
///
/// Asynchronous clock request handle is a waitable handle for clock change
/// requests. This allows one to issue multiple clock change requests
/// concurrently (e.g., GPU clock and memory clock for multiple devices) and
/// then wait for all of them to complete.
///
/// @remark This is void pointer on operating systems that do not support
/// asynchronous clock request handles. HOS does not have the support.
typedef struct NvRmGpuClockAsyncNotImplemented *NvRmGpuClockAsyncReqHandle;
#endif

/// @ingroup nvrm_gpu_device_group
/// @brief Clock domains
///
/// The GPU has different clock domains that can be queried or requested
/// separately. These include the memory clock and the graphics clock.
///
/// @sa NvRmGpuClockGetDomains(), NvRmGpuDeviceInfo::clockDomains
typedef enum
{
    /// @brief Memory clock
    NvRmGpuClockDomain_MCLK = 0,

    /// @brief Main graphics core clock
    NvRmGpuClockDomain_GPCCLK,

    /// @brief Number of clock domains
    NvRmGpuClockDomain_Count
} NvRmGpuClockDomain;

/// @ingroup nvrm_gpu_device_group
/// @brief Request type for clock get.
///
/// @sa NvRmGpuClockGet()
typedef enum NvRmGpuClockType
{
    /// @brief Target clock frequency requested by the user.
    ///
    /// This is the minimum frequency requested by the user. The programmed
    /// frequency may differ.
    NvRmGpuClockType_Target = 1,

    /// @brief Clock frequency programmed to the HW (including PLL constraints).
    ///
    /// @remark This is called the "Actual" clock frequency as this frequency is
    /// the one that is actually programmed.
    NvRmGpuClockType_Actual = 2,

    /// @brief Effective clock as measured from hardware.
    NvRmGpuClockType_Effective = 3
} NvRmGpuClockType;


/// @ingroup nvrm_gpu_device_group
/// @brief Entry for clock get request
///
/// @sa NvRmGpuClockGet()
typedef struct NvRmGpuClockGetEntryRec
{
    /// @brief \b (IN) Domain for the clock request
    ///
    /// @remark This is input parameter. NvRmGpuClockGet() will not modify this
    /// field.
    NvRmGpuClockDomain domain;

    /// @brief \b (IN) Request type
    ///
    /// @remark This is input parameter. NvRmGpuClockGet() will not modify this
    /// field.
    NvRmGpuClockType type;

    /// @brief \b (OUT) Frequency in Hz
    ///
    /// @remark This is output parameter. NvRmGpuClockGet() will modify this
    /// field on #NvSuccess. It may also modify this field on error.
    uint64_t freqHz;

} NvRmGpuClockGetEntry;

/// @ingroup nvrm_gpu_device_group
/// @brief Entry for clock set request
///
/// @sa NvRmGpuClockSet()
typedef struct NvRmGpuClockSetEntryRec
{
    /// @brief Domain for clock request
    NvRmGpuClockDomain domain;

    /// @brief Frequency for clock request
    uint64_t freqHz;
} NvRmGpuClockSetEntry;

/// @ingroup nvrm_gpu_device_group
/// @brief Frequency range for clock domain
///
/// @sa NvRmGpuClockGetDomains()
typedef struct NvRmGpuClockRangeRec
{
    uint64_t minHz;
    uint64_t maxHz;
} NvRmGpuClockRange;

/// @ingroup nvrm_gpu_device_group
/// @brief Clock voltage/frequency point
///
/// @sa NvRmGpuClockGetPoints()
typedef struct NvRmGpuClockPointRec
{
    uint64_t freqHz;
} NvRmGpuClockPoint;

/// @ingroup nvrm_gpu_device_group
/// @brief Clock domain info
///
/// @sa NvRmGpuClockGetDomains()
typedef struct NvRmGpuClockDomainInfoRec
{
    /// @brief Clock domain
    NvRmGpuClockDomain domain;

    /// @brief Frequency range of the clock domain
    NvRmGpuClockRange range;

    /// @brief Maximum number of voltage/frequency points returned by
    /// NvRmGpuClockGetPoints()
    size_t maxVfPoints;

} NvRmGpuClockDomainInfo;

/// @ingroup nvrm_gpu_device_group
/// @brief Returns available GPU clock domains for the device.
///
/// @param[in]  hDevice      Device handle.
/// @param[out] infos        Array of available clock domains. This list is valid
///                          during the life-time of the @a hDevice handle. The returned
///                          pointer must not be freed by the caller.
/// @param[out] pNumDomains  Number of domains in array.
///
/// @return The usual #NvError return code.
/// @retval NvSuccess              Successful request.
/// @retval NvError_NotSupported   Clock controls API not supported by this
///                                device. Capability
///                                NvRmGpuDeviceInfo::hasClockControls for this
///                                device is @false.
/// @retval NvError_*              Unspecified error. Error code returned for
///                                diagnostics.
///
/// @remark Requires NvRmGpuDeviceInfo::hasClockControls. This function can be
///         used to probe the capability. If \a NvError_NotSupported is
///         returned, then NvRmGpuDeviceInfo::hasClockControls is @false.
///
/// @remark There may be more actual clock domains in the GPU HW than returned
/// by this function. This function returns the domains that can be queried or
/// requested.
///
/// @sa NvRmGpuClockGetPoints()
NvError NvRmGpuClockGetDomains(NvRmGpuDevice *hDevice,
                               const NvRmGpuClockDomainInfo **infos,
                               size_t *pNumDomains);

/// @ingroup nvrm_gpu_device_group
/// @brief Retrieves voltage/frequency (VF) points for a given clock domain.
/// For information about VF points, see @ref NvRmGpuClockAsyncReqHandle.
///
/// Each clock domain has VF points, defined as frequencies for which voltage is
/// optimal. In general, the clock arbiter will try to program frequencies which
/// correspond to VF points.
///
/// @param[in]  hDevice     Device handle.
/// @param[in]  domain      Clock domain to query.
/// @param[out] pClkPoints  Pointer to receive the array of optimal VF
///                         points. The allocated array must contain space for
///                         at least NvRmGpuClockDomainInfo::maxVfPoints (as
///                         retrieved by NvRmGpuClockGetDomains()).
/// @param[out] pNumPoints  Number of VF points. May vary depending on thermal
///                         conditions, and will be at most
///                         NvRmGpuClockDomainInfo::maxVfPoints.
///
/// @return The usual NvError return code.
/// @retval NvSuccess              Successful request.
/// @retval NvError_NotSupported   Clock controls API not supported by this device.
/// @retval NvError_*              Unspecified error. Error code returned for diagnostics.
///
/// @remark Requires NvRmGpuDeviceInfo::hasClockControls
///
/// @sa NvRmGpuClockGet(), NvRmGpuClockSet()
/// @sa NvRmGpuClockGetDomains()
NvError NvRmGpuClockGetPoints(NvRmGpuDevice *hDevice,
                              NvRmGpuClockDomain domain,
                              NvRmGpuClockPoint *pClkPoints,
                              size_t *pNumPoints);

/// @ingroup nvrm_gpu_device_group
/// @brief Requests minimum clocks for one or more clock domains.
///
/// This function allows the caller to request minimum GPU clocks for one or
/// more GPU clock domains. Both synchronous and asynchronous requests are
/// supported.
///
/// In the asynchronous case, a waitable clock handle is returned. When setting
/// clocks on multiple devices, the asynchronous mode allows one to perform all
/// requests concurrently, and then wait until the clocks have been set. Clock
/// stabilization can take a while.
///
/// When another request is made for a clock domain in the same #NvRmGpuDevice
/// instance, the previous requests are overridden.
///
/// The actually programmed clock frequency may not be exactly the requested
/// frequency. Generally, the clock arbiter chooses the optimal
/// voltage-frequency point that is at least as high as the highest frequency
/// requested by any GPU user. However, depending on the thermal and other
/// conditions, the actual GPU frequency may be also lower. Use
/// NvRmGpuClockGet() to query the programmed and the effective
/// frequencies. #NvRmGpuDeviceEventSession can be also used to monitor changes
/// in the GPU frequencies.
///
///
/// @param[in]  hDevice        Device handle.
///
/// @param[in] pClkSetEntries  Array of request entries. Each entry requests
///                            target frequency for one clock domain. If a clock
///                            domain appears multiple times in one call (not
///                            recommended), then only the last entry will be
///                            taken into account.
///
/// @param[in]  numEntries     Number of entries in the @a pClkSetEntries array.
///
/// @param[out] phReq          Pointer to asynchronous request handle or @NULL
///                            for a synchronous request
///                            - If @NULL, the request is synchronous and
///                              function returns only after all clocks are
///                              programmed.
///                            - If non-@NULL, the request is asynchronous and a
///                              waitable request completion handle is returned
///                              on success. Use NvRmGpuClockWaitAsyncReq() to wait
///                              using the handle. The request handle must be
///                              closed by the caller using
///                              NvRmGpuClockCloseAsyncReq().
///
/// @return The usual NvError code. In case of error, the asynchronous request
///         handle is not returned.
///
/// @retval NvSuccess            The request was successfully made. In the
///                              synchronous case, the wait was also
///                              successful. In the asynchronous case, the
///                              request handle is returned.
///
/// @retval NvError_NotSupported Clock controls API not supported by this device.
///
/// @retval NvError_Busy         A temporary blocking condition when submitting
///                              the asynchronous request. The user should try
///                              again.
///
/// @retval NvError_*            Unspecified error. The error code is returned
///                              for diagnostic purposes.
///
/// @remark Requires NvRmGpuDeviceInfo::hasClockControls
///
/// @remark The synchronous wait case is equivalent with performing the
///         asynchronous wait request followed immediately by calls to
///         NvRmGpuClockWaitAsyncReq() and NvRmGpuClockCloseAsyncReq().
///
/// @remark A subsequent clock request to a domain supersedes the previous
///         request (per #NvRmGpuDevice instance)
///
/// @remark The lifespan of GPU clock requests is tied to the #NvRmGpuDevice
///         instance. All requests by the user are canceled when the
///         #NvRmGpuDevice handle is closed.
///
/// @remark The clock requests of all #NvRmGpuDevice instances are coalesced.
///         Generally, given that thermal and power limits are not exceeded, the
///         actual clock frequency will be at least the greatest requested. The
///         exact selection algorithm depends on the global clock management
///         driver policy.  The selection algorithm is run by the "clock
///         arbiter" within the KMD component.
///
/// @remark Actual frequency might differ depending on global policies, requests
///         from other NvRmGpuDevice instances, or thermal conditions.
///
/// @remark If specified target frequency is not a VF point, clock arbiter will
///         generally try to program the clocks with first VF point that is
///         greater than or equal to specified target frequency (assuming a
///         single application)
///
/// @sa NvRmGpuClockGet()
/// @sa NvRmGpuClockGetDomains()
/// @sa NvRmGpuClockWaitAsyncReq()
/// @sa NvRmGpuClockCloseAsyncReq()
/// @sa NvRmGpuDeviceEventSessionOpen()
/// @sa NvRmGpuClockWaitAnyEvent()
NvError NvRmGpuClockSet(NvRmGpuDevice *hDevice,
                        const NvRmGpuClockSetEntry *pClkSetEntries,
                        size_t numEntries,
                        NvRmGpuClockAsyncReqHandle *phReq);

/// @ingroup nvrm_gpu_device_group
/// @brief Waits for the completion of one or more asynchronous clock requests.
///
/// @param[in] hDevice     Device handle
/// @param[in] phReqs      Array of request handles.
/// @param[in] numEntries  Number of entries in the request array.
/// @param[in] timeoutMs   Wait timeout in milliseconds. Use as follows:
///                        - #NV_WAIT_INFINITE: No timeout, indefinite wait
///                        - `>=0`: Timeout specified. The function returns when the
///                          request completes or when the timeout expires,
///                          whichever comes first.
///                        - `0`: Peek. The function returns immediately.
///
/// @return The usual #NvError return value
/// @retval NvSuccess        All requests have completed.
/// @retval NvError_Timeout  Timeout was reached before all
///                          requests were completed.
/// @retval NvError_*        Unspecified error. The error code is returned
///                          for diagnostic purposes.
///
/// @remark Requires NvRmGpuDeviceInfo::hasClockControls
///
/// @sa NvRmGpuClockSet()
NvError NvRmGpuClockWaitAsyncReq(NvRmGpuDevice *hDevice,
                                 const NvRmGpuClockAsyncReqHandle *phReqs,
                                 size_t numEntries,
                                 uint32_t timeoutMs);


/// @ingroup nvrm_gpu_device_group
/// @brief Closes an asynchronous clock request handle.
///
/// Frees all resources related to an asynchronous request created with
/// NvRmGpuClockSet(). It is not mandatory to wait for request completion before
/// closing the handle.
///
/// @param[in] hDevice Device handle
/// @param[in] hReq    Asynchronous request handle to close. May be @NULL.
///
/// @return The usual  #NvError code
/// @retval NvSuccess  Handle closed successfully, or @NULL handle was provided.
/// @retval NvError_*  Unspecified error. The error code is returned
///                    for diagnostic purposes.
///
/// @remark Requires NvRmGpuDeviceInfo::hasClockControls
///
/// @sa NvRmGpuClockSet()
/// @sa NvRmGpuClockWaitAsyncReq()
NvError NvRmGpuClockCloseAsyncReq(NvRmGpuDevice *hDevice,
                                  NvRmGpuClockAsyncReqHandle hReq);


/// @ingroup nvrm_gpu_device_group
/// @brief This function is not implemented and it should be deleted.
NvError NvRmGpuClockWaitAnyEvent(NvRmGpuDevice *hDevice,
                                 uint32_t timeoutMs);


/// @ingroup nvrm_gpu_device_group
/// @brief Request one or more clock domain frequency state.
///
/// This function can be used to request control frequencies on clock
/// domains. This function accepts one or more requests. The request is targeted
/// for:
/// - clock domain (e.g., GPC core clock, memory clock). See
///   #NvRmGpuClockDomain.
/// - control information type: application target frequency, programmed
///   frequency, measured frequency. See #NvRmGpuClockType.
///
/// @param[in]     hDevice        Device handle
///
/// @param[in,out] pClkGetEntries Array of clock request entries. For each
///                               entry, the clock domain and clock types must
///                               be set. Upon a successful call the associated
///                               frequency will be returned on a successful
///                               call. It is allowed to mix several clocks
///                               domains and clock request types in the same
///                               request.
///
/// @param[in]     numEntries     Number of request entries.
///
/// @return The usual #NvError return code
/// @retval NvSuccess             Successful request
/// @retval NvError_NotSupported  Clock controls API not supported.
/// @retval NvError_*             Unspecified error. The error code is returned
///                               for diagnostic purposes.
///
/// @remark Requires NvRmGpuDeviceInfo::hasClockControls
///
/// @sa NvRmGpuClockSet()
/// @sa NvRmGpuClockGetDomains()
NvError NvRmGpuClockGet(NvRmGpuDevice *hDevice,
                        NvRmGpuClockGetEntry *pClkGetEntries,
                        size_t numEntries);

/// @ingroup nvrm_gpu_device_group
/// @brief Voltage sensors
typedef enum
{
    /// @brief Core GPU voltage
    NvRmGpuDeviceVoltage_Core = 1,

    /// @brief SRAM voltage
    NvRmGpuDeviceVoltage_SRAM,

    /// @brief Bus voltage
    NvRmGpuDeviceVoltage_Bus
} NvRmGpuDeviceVoltage;

/// @ingroup nvrm_gpu_device_group
/// @brief Returns the list of available voltage sensors for the device.
///
/// @param[in]  hDevice      Device handle.
/// @param[out] pSensors     Pointer to receive a pointer to the array of
///                          available sensors. The returned pointer may be
///                          @NULL if no sensors are available. The the
///                          returned pointer is valid for the life-time of \a
///                          hDevice and it must not be freed by the caller.
/// @param[out] pNumSensors  Pointer to receive the number of available sensors.
///
/// @return The usual #NvError return code
/// @retval NvSuccess             Successful request
/// @retval NvError_*             Unspecified error. The error code is returned
///                               for diagnostic purposes.
///
/// @sa NvRmGpuDeviceGetVoltage()
NvError NvRmGpuDeviceListVoltageSensors(NvRmGpuDevice *hDevice,
                                        const NvRmGpuDeviceVoltage **pSensors,
                                        size_t *pNumSensors);


/// @ingroup nvrm_gpu_device_group
/// @brief Retrieves the voltage sensor reading.
///
/// @param[in]  hDevice            Device handle
/// @param[in]  which              The voltage sensor to query
/// @param[out] pVoltageMicroVolt  non-@NULL pointer to receive the voltage in microvolts.
///
/// @return The usual #NvError return code
/// @retval NvSuccess             Successful request
/// @retval NvError_*             Unspecified error. The error code is returned
///                               for diagnostic purposes.
///
/// @remark Requires NvRmGpuDeviceInfo::hasDeviceSensorInfo
///
/// @remark See NvRmGpuDeviceListVoltageSensors() for the available voltage
/// sensors for the device.
///
/// @sa NvRmGpuDeviceListVoltageSensors()
NvError NvRmGpuDeviceGetVoltage(NvRmGpuDevice *hDevice,
                                NvRmGpuDeviceVoltage which,
                                uint64_t *pVoltageMicroVolt);

/// @ingroup nvrm_gpu_device_group
/// @brief Electric current sensors
typedef enum
{
    /// @brief Bus current
    NvRmGpuDeviceCurrent_Bus = 1,
} NvRmGpuDeviceCurrent;


/// @ingroup nvrm_gpu_device_group
/// @brief Returns the list of available electric current sensors for the
/// device.
///
/// @param[in]  hDevice      Device handle.
/// @param[out] pSensors     Pointer to receive a pointer to the array of
///                          available sensors. The returned pointer may be
///                          @NULL if no sensors are available. The the
///                          returned pointer is valid for the life-time of \a
///                          hDevice and it must not be freed by the caller.
/// @param[out] pNumSensors  Pointer to receive the number of available sensors.
///
/// @return The usual #NvError return code
/// @retval NvSuccess             Successful request
/// @retval NvError_*             Unspecified error. The error code is returned
///                               for diagnostic purposes.
///
/// @sa NvRmGpuDeviceGetCurrent()
NvError NvRmGpuDeviceListCurrentSensors(NvRmGpuDevice *hDevice,
                                        const NvRmGpuDeviceCurrent **pSensors,
                                        size_t *pNumSensors);

/// @ingroup nvrm_gpu_device_group
/// @brief Retrieves the electric current reading.
///
/// @param[in]  hDevice              Device handle.
/// @param[in]  which                The current sensor to query.
/// @param[out] pCurrentMicroAmpere  Pointer to receive the current in microamperes.
///
/// @return The usual #NvError return code
/// @retval NvSuccess             Successful request
/// @retval NvError_*             Unspecified error. The error code is returned
///                               for diagnostic purposes.
///
/// @remark Requires NvRmGpuDeviceInfo::hasDeviceSensorInfo
///
/// @remark See NvRmGpuDeviceListCurrentSensors() for the available electric
/// current sensors for the device.
///
/// @sa NvRmGpuDeviceListCurrentSensors()
NvError NvRmGpuDeviceGetCurrent(NvRmGpuDevice *hDevice,
                                NvRmGpuDeviceCurrent which,
                                uint64_t *pCurrentMicroAmpere);

/// @ingroup nvrm_gpu_device_group
/// @brief Electric power sensors
typedef enum
{
    /// @brief Power consumed at the regulator
    NvRmGpuDevicePower_Bus = 1
} NvRmGpuDevicePower;

/// @ingroup nvrm_gpu_device_group
/// @brief Returns the list of available power sensors for the device.
///
/// @param[in]  hDevice      Device handle.
/// @param[out] pSensors     Pointer to receive a pointer to the array of
///                          available sensors. The returned pointer may be
///                          @NULL if no sensors are available. The the
///                          returned pointer is valid for the life-time of \a
///                          hDevice and it must not be freed by the caller.
/// @param[out] pNumSensors  Pointer to receive the number of available sensors.
///
/// @return The usual #NvError return code
/// @retval NvSuccess             Successful request
/// @retval NvError_*             Unspecified error. The error code is returned
///                               for diagnostic purposes.
///
/// @sa NvRmGpuDeviceGetPower()
NvError NvRmGpuDeviceListPowerSensors(NvRmGpuDevice *hDevice,
                                      const NvRmGpuDevicePower **pSensors,
                                      size_t *pNumSensors);

/// @ingroup nvrm_gpu_device_group
/// @brief Retrieves the power sensor reading.
///
/// @param[in]  hDevice          Device handle.
/// @param[in]  which            The power sensor to query.
/// @param[out] pPowerMicroWatt  Power in microwatts.
///
/// @return The usual #NvError return code
/// @retval NvSuccess             Successful request
/// @retval NvError_*             Unspecified error. The error code is returned
///                               for diagnostic purposes.
///
/// @remark See NvRmGpuDeviceListPowerSensors() for the available electric
/// power sensors for the device.
///
/// @remark Requires NvRmGpuDeviceInfo::hasDeviceSensorInfo
///
/// @sa NvRmGpuDeviceListPowerSensors()
NvError NvRmGpuDeviceGetPower(NvRmGpuDevice *hDevice,
                              NvRmGpuDevicePower which,
                              uint64_t *pPowerMicroWatt);


/// @ingroup nvrm_gpu_device_group
/// @brief Temperature sensors
typedef enum
{
    /// @brief The internal GPU temperature sensor
    NvRmGpuDeviceTemperature_InternalSensor = 1
} NvRmGpuDeviceTemperature;

/// @ingroup nvrm_gpu_device_group
/// @brief Returns the list of available temperature sensors for the device.
///
/// @param[in]  hDevice      Device handle.
/// @param[out] pSensors     Pointer to receive a pointer to the array of
///                          available sensors. The returned pointer may be
///                          @NULL if no sensors are available. The the
///                          returned pointer is valid for the life-time of \a
///                          hDevice and it must not be freed by the caller.
/// @param[out] pNumSensors  Pointer to receive the number of available sensors.
///
/// @return The usual #NvError return code
/// @retval NvSuccess             Successful request
/// @retval NvError_*             Unspecified error. The error code is returned
///                               for diagnostic purposes.
///
/// @sa NvRmGpuDeviceGetTemperature()
NvError NvRmGpuDeviceListTemperatureSensors(NvRmGpuDevice *hDevice,
                                            const NvRmGpuDeviceTemperature **pSensors,
                                            size_t *pNumSensors);

/// @ingroup nvrm_gpu_device_group
/// @brief Retrieves the temperature sensor reading.
///
/// @param[in]  hDevice                   Device handle.
/// @param[in]  which                     Temperature sensor to query.
/// @param[out] pTemperatureMilliCelsius  Pointer to receive the temperature reading in millidegrees Celsius.
///
/// @return The usual #NvError return code
/// @retval NvSuccess             Successful request
/// @retval NvError_*             Unspecified error. The error code is returned
///                               for diagnostic purposes.
///
/// @remark Requires NvRmGpuDeviceInfo::hasDeviceSensorInfo
///
/// @remark See NvRmGpuDeviceListPowerSensors() for the available temperature
/// sensors for the device.
///
/// @sa NvRmGpuDeviceListTemperatureSensors()
/// @sa NvRmGpuDeviceThermalAlertSetLimit()
NvError NvRmGpuDeviceGetTemperature(NvRmGpuDevice *hDevice,
                                    NvRmGpuDeviceTemperature which,
                                    int32_t *pTemperatureMilliCelsius);

/// @ingroup nvrm_gpu_device_group
/// @brief Sets the thermal alert limit.
///
/// @param hDevice         Device handle.
/// @param temperature_mC  Thermal temperature alert threshold in millidegrees Celsius.
///
/// @return The usual #NvError return code
/// @retval NvSuccess             Successful operation
/// @retval NvError_NotSupported  Operation not supported for the device.
/// @retval NvError_*             Unspecified error. The error code is returned
///                               for diagnostic purposes.
///
/// @remark Requires NvRmGpuDeviceInfo::hasDeviceThermalAlert
///
/// @sa NvRmGpuDeviceGetTemperature()
NvError NvRmGpuDeviceThermalAlertSetLimit(NvRmGpuDevice *hDevice,
                                          int32_t temperature_mC);


/// @ingroup nvrm_gpu_device_group
/// GPU device event type
///
/// @sa NvRmGpuDeviceEventSessionOpen()
typedef enum
{
    // @brief Frequency change event
    ///
    /// Voltage/frequency update occurred for a clock domain. This can be
    /// because of:
    /// - an nvrm_gpu user issued a target frequency request; or
    /// - because of a change in thermal or power conditions.
    ///
    /// This is an informational event.
    NvRmGpuDeviceEventId_VfUpdate = 0,

    /// @brief A clock domain frequency is below target.
    ///
    /// This indicates that the GPU may be operating at lower than expected
    /// performance.
    NvRmGpuDeviceEventId_AlarmTargetVfNotPossible,

    /// @brief A clock domain frequency is below local target frequency
    /// requested by a session.
    ///
    /// This indicates that the GPU may be operating at lower than expected
    /// performance.
    NvRmGpuDeviceEventId_AlarmLocalTargetVfNotPossible,

    /// @brief The clock arbiter has failed.
    ///
    /// This is a system failure. Frequency change requests may not be honored
    /// anymore.
    NvRmGpuDeviceEventId_AlarmClockArbiterFailed,

    /// @brief VF table update failed.
    ///
    /// VF table update is typically related to operating condition
    /// change. Something went wrong and VF tables could not be updated.
    ///
    /// This is a system failure.
    NvRmGpuDeviceEventId_AlarmVfTableUpdateFailed,

    /// @brief Temperature above threshold.
    ///
    /// The GPU temperature is above threshold. Measures may have to be taken to
    /// prevent thermal throttling. For instance, target frequencies may need to
    /// be lowered.
    ///
    /// @sa NvRmGpuDeviceThermalAlertSetLimit()
    NvRmGpuDeviceEventId_AlarmThermalAboveThreshold,

    /// @brief Power above threshold.
    ///
    /// The GPU power drain is above threshold. Measures may have to be taken to
    /// remedy the condition. For instance, target frequencies may need to be
    /// lowered.
    NvRmGpuDeviceEventId_AlarmPowerAboveThreshold,

    /// @brief Device lost.
    ///
    /// The GPU device is lost. This may be due to number of reasons, such as
    /// bus failure, power failure, hardware failure, GPU hang/reboot, firmware
    /// failure, or a programming failure due to KMD.
    ///
    /// This is a system failure. The nvrm_gpu user should close all resources.
    ///
    /// @remark NvRmGpuDeviceClose()
    NvRmGpuDeviceEventId_AlarmGpuLost,

    /// @brief Number of events.
    NvRmGpuDeviceEventId_Count

} NvRmGpuDeviceEventId;

/// @ingroup nvrm_gpu_device_group
/// @brief Extensible attribute structure for
/// #NvRmGpuDeviceEventSessionOpen().
///
/// @remark Use NVRM_GPU_DEFINE_DEVICE_EVENT_SESSION_ATTR() to define the
/// attribute variable with defaults.
///
/// @sa NvRmGpuDeviceEventSessionOpen()
/// @sa NVRM_GPU_DEFINE_DEVICE_EVENT_SESSION_ATTR()
typedef struct NvRmGpuDeviceEventSessionOpenAttrRec
{
    /// @brief List of events to listen.
    ///
    /// @remark Use NvRmGpuDeviceEventSessionOpenAttrSetAllEvents() to listen to
    /// all events.
    const NvRmGpuDeviceEventId *filterList;

    /// @brief Number of entries in the event list.
    size_t filterListSize;

} NvRmGpuDeviceEventSessionOpenAttr;

/// @ingroup nvrm_gpu_device_group
/// @brief Definer macro for #NvRmGpuDeviceEventSessionOpenAttr.
///
/// This macro defines a variable of type #NvRmGpuDeviceEventSessionOpenAttr
/// with the default values.
///
/// @sa NvRmGpuDeviceEventSessionOpen()
#define NVRM_GPU_DEFINE_DEVICE_EVENT_SESSION_ATTR(x)            \
    NVRM_GPU_ATTR_NAMESPACE(NvRmGpuDeviceEventSessionOpenAttr) x = { NULL, 0 }

/// @ingroup nvrm_gpu_device_group
/// @brief Assigns device events attribute structure with a list of all events
/// to listen to.
///
/// @param[out]  attr Non-@NULL pointer to the device events attribute struct.
///
/// @sa #NvRmGpuDeviceEventSessionOpenAttr
static inline void NvRmGpuDeviceEventSessionOpenAttrSetAllEvents(NvRmGpuDeviceEventSessionOpenAttr *attr)
{
    static const NvRmGpuDeviceEventId allEvents[] =
    {
        NvRmGpuDeviceEventId_VfUpdate,
        NvRmGpuDeviceEventId_AlarmTargetVfNotPossible,
        NvRmGpuDeviceEventId_AlarmLocalTargetVfNotPossible,
        NvRmGpuDeviceEventId_AlarmClockArbiterFailed,
        NvRmGpuDeviceEventId_AlarmVfTableUpdateFailed,
        NvRmGpuDeviceEventId_AlarmThermalAboveThreshold,
        NvRmGpuDeviceEventId_AlarmPowerAboveThreshold,
        NvRmGpuDeviceEventId_AlarmGpuLost
    };
    attr->filterList = allEvents;
    attr->filterListSize = NV_ARRAY_SIZE(allEvents);
}

/// @ingroup nvrm_gpu_device_group
/// @brief Opens a session to monitor device events.
///
/// @param[in]  hDevice    Device handle.
/// @param[in]  attr       Event session attributes. The attribute structure
///                        contains the device event filter list.
/// @param[out] phSession  Pointer to receive the event session handle on success.
///
/// @return The usual #NvError return code
/// @retval NvSuccess             Device event session created successfully.
/// @retval NvError_NotSupported  This device does not support device events.
/// @retval NvError_*             Unspecified error. The error code is returned
///                               for diagnostic purposes.
///
/// @remark Requires NvRmGpuDevice::hasDeviceEvents
///
/// @sa NvRmGpuDeviceEventSessionClose()
NvError NvRmGpuDeviceEventSessionOpen(NvRmGpuDevice *hDevice,
                                      const NvRmGpuDeviceEventSessionOpenAttr *attr,
                                      NvRmGpuDeviceEventSession **phSession);

/// @ingroup nvrm_gpu_device_group
/// @brief GPU device event.
///
/// Data type for a single GPU device event. This contains the timestamp and the
/// event type.
///
/// @sa NvRmGpuDeviceEventSessionRead()
/// @sa NvRmGpuDeviceReadTimeNs()
typedef struct NvRmGpuDeviceEventInfoRec
{
    /// @brief Event type
    NvRmGpuDeviceEventId eventId;

    /// @brief GPU time (in nanoseconds)
    ///
    /// This is the unscaled GPU PTIMER timestamp at the occurrence of the
    /// event.
    ///
    /// @remark Certain integrated Tegra GPUs require GPU timestamp
    /// scaling. These GPUs are T210 and T214. See the discussion in
    /// NvRmGpuDeviceInfo::ptimerScaleNumerator for further details.
    ///
    /// @sa NvRmGpuDeviceReadTimeNs()
    uint64_t timeNs;

} NvRmGpuDeviceEventInfo;

/// @ingroup nvrm_gpu_device_group
/// @brief Read next device event
///
/// @param[in]  hSession    Event session handle
/// @param[out] pEventInfo  Pointer to receive the event on success
/// @param[in]  timeoutMs   Timeout value in milliseconds. Special values:
///                         - `0`: non-blocking peek
///                         - #NV_WAIT_INFINITE: wait indefinitely for the next event
///
/// @return NvSuccess indicates that one event occurred, and detailed
///         information has been updated in @a pEventInfo.
///         NvError_Timeout indicates that timeout was reached.
///
/// @remark When an event occurs while there is a previous pending event of the
/// same type, the events are merged. In this case, only one event is reported.
///
/// @return The usual #NvError return code
/// @retval NvSuccess             Successful request
/// @retval NvError_Timeout       Timeout occurred before an event was available
/// @retval NvError_*             Unspecified error. The error code is returned
///                               for diagnostic purposes.
///
/// @sa NvRmGpuDeviceEventSessionOpen()
/// @sa NvRmGpuDeviceEventInfo
NvError NvRmGpuDeviceEventSessionRead(NvRmGpuDeviceEventSession *hSession,
                                      NvRmGpuDeviceEventInfo *pEventInfo,
                                      uint32_t timeoutMs);

/// @ingroup nvrm_gpu_device_group
/// @brief Closes the device event session.
///
/// @param[in] hSession  Device event session handle to close. May be @NULL.
///
/// @return The usual #NvError return code
/// @retval NvSuccess             Event session closed successfully or \a hSession was @NULL.
/// @retval NvError_*             Unspecified error while closing the
///                               session. The session is closed, regardless.
///                               The error code is returned for diagnostic
///                               purposes.
///
/// @remark Regardless of possible errors in deinitialization, the object will
/// be closed.
NvError NvRmGpuDeviceEventSessionClose(NvRmGpuDeviceEventSession *hSession);

#if defined(NVRM_GPU_BUILD_VARIANT)
} // namespace nvrm_gpu
#endif

#if defined(__cplusplus)
}
#endif

#if !defined(NV_SDK_BUILD)
#include "nvrm_gpu_priv.h"
#endif

#endif
