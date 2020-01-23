/*
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef NVSCIIPC_H
#define NVSCIIPC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <nvscierror.h>
/**
 * @file
 *
 * @brief <b> NVIDIA Software Communications Interface (SCI) : NvSci Inter-Process Communication </b>
 *
 */
/**
 * @defgroup nvsci_ipc Inter-Process Communication
 *
 *
 * @ingroup nvsci_top
 * @{
 *
 * The NvSciIpc library provides interfaces for any two entities in a system to
 * communicate with each other irrespective of where they are placed. Entities
 * can be in:
 * - The same thread
 * - Different threads in the same process
 * - The same process
 * - Different processes in the same VM
 * - Different VMs on the same SoC
 * - Different SoCs
 *
 * Each of these different boundaries will be abstracted by a library providing
 * unified communication (read/write) APIs to entities. The communication
 * consists of two bi-directional send/receive queues.
 *
 */
/*******************************************************************/
/************************ OS SPECIFIC ******************************/
/*******************************************************************/
#ifdef QNX
#include <sys/neutrino.h>

/* This default code can be used by Application in NvSciIpcSetQnxPulseParam() */
/// @cond (SWDOCS_NVSCIIPC_INTERNAL)
#define NV_SCI_IPC_NOTIFY_CODE (_PULSE_CODE_MINAVAIL + 1)
/// @endcond
#endif // QNX

/*******************************************************************/
/************************ DATA TYPES *******************************/
/*******************************************************************/
/**
 * @brief Handle to the IPC endpoint.
 */
typedef uint64_t NvSciIpcEndpoint;

/* NvSciIpcEventNotifier type is for Drive OS internal use only */
/// @cond (SWDOCS_NVSCIIPC_INTERNAL)
typedef int32_t NvSciIpcEventNotifier;
/// @endcond

/**
 * @brief Defines information about the IPC endpoint.
 */
struct NvSciIpcEndpointInfo {
    /*! Holds the number of frames. */
    uint32_t nframes;
    /*! Holds the frame size. */
    uint32_t frame_size;
};

/* NvSciIPC Event type */
/** Specifies the IPC read event. */
#define	NV_SCI_IPC_EVENT_READ       1U
/** Specifies the IPC write event. */
#define	NV_SCI_IPC_EVENT_WRITE      2U
/** Specifies the IPC connection established event. */
#define	NV_SCI_IPC_EVENT_CONN_EST   4U
/** Specifies the IPC connection reset event. */
#define	NV_SCI_IPC_EVENT_CONN_RESET 8U

/********************* FUNCTION TYPES *******************************/
/*******************************************************************/

/**
 * @brief Initializes the NvSciIpc library.
 *
 * This function parses the NvSciIpc configuration file and creates
 * an internal database of NvSciIpc endpoints that exist in a system.
 *
 * @return ::NvSciError, the completion code of the operation.
 * - ::NvSciError_Success Indicates a successful operation.
 */
NvSciError NvSciIpcInit(void);

/**
 * @brief De-initializes the NvSciIpc library.
 *
 * This function cleans up the NvSciIpc endpoint internal database
 * created by NvSciIpcInit( ).
 *
 * @return @c void
 */
void NvSciIpcDeinit(void);

/**
 * Opens an endpoint with the given name.
 *
 * The function locates the NvSciIpc endpoint with the given name in the
 * NvSciIpc configuration table in the internal database, and returns a handle
 * to the endpoint if found. When the operation is successful, two endpoints can
 * utilize the allocated shared data area and the corresponding signaling
 * mechanism setup. If the operation fails, the state of the NvSciIpc endpoint
 * is undefined.
 *
 * @param endpoint The name of the NvSciIpc endpoint to open.
 * @param handle   A handle to the endpoint on success.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success            Indicates a successful operation.
 * - ::NvSciError_BadParameter       Indicates any argument was NULL or invalid.
 * - ::NvSciError_NotInitialized     Indicates NvSciIpc is uninitialized.
 * - ::NvSciError_NoSuchEntry        Indicates the @a endpoint was not found.
 * - ::NvSciError_Busy               Indicates the @a endpoint is already in use.
 * - ::NvSciError_InsufficientMemory Indicates memory allocation failed for the operation.
 */
NvSciError NvSciIpcOpenEndpoint(const char *endpoint, NvSciIpcEndpoint *handle);

/**
 * Closes an endpoint with the given handle.
 *
 * The function frees the NvSciIpc endpoint associated with the given @a handle.
 *
 * @param handle A handle to the endpoint to close.
 *
 * @return @c void
 */
void NvSciIpcCloseEndpoint(NvSciIpcEndpoint handle);

/**
 * Resets an endpoint.
 *
 * Initiates a reset on the endpoint and notifies the remote endpoint.
 * Applications must call this function and complete the reset operation before
 * using the endpoint for communication.
 *
 * @param handle A handle to the endpoint to reset.
 *
 * @return @c void
 */
void NvSciIpcResetEndpoint(NvSciIpcEndpoint handle);

/**
 * Returns the contents of the next frame from an endpoint.
 *
 * This function removes the next frame and copies its contents
 * into a buffer. If the destination buffer is smaller than the configured
 * frame size of the endpoint, the trailing bytes are discarded.
 *
 * This is a non-blocking call. The endpoint must not be empty.
 * If the endpoint was previously full, then the function notifies the
 * remote endpoint.
 *
 * This operation cannot proceed if the endpoint is being reset. However,
 * if the remote endpoint has called NvSciIpcResetEndpoint(), calls to this
 * function can still succeed until the next event notification on the local
 * endpoint.
 *
 * @param handle The handle to the endpoint to read from.
 * @param buf    A pointer to a destination buffer to receive the contents
 *               of the next frame.
 * @param size   The number of bytes to copy from the frame. If @a size
 *               is greater than the size of the destination buffer, the
 *               remaining bytes are discarded.
 * @param bytes  The number of bytes read.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success            Indicates a successful operation.
 * - ::NvSciError_BadParameter       Indicates an invalid @a handle.
 * - ::NvSciError_NotInitialized     Indicates NvSciIpc is uninitialized.
 * - ::NvSciError_InsufficientMemory Indicates memory allocation failed,
 *                                   the frames cannot be read.
 * - ::NvSciError_TooBig             Indicates @a size is larger than the
 *                                   configured frame size of the endpoint.
 * - ::NvSciError_ConnectionReset    Indicates the endpoint is being reset.
 */
NvSciError NvSciIpcRead(NvSciIpcEndpoint handle, void *buf, size_t size,
	int32_t *bytes);

/**
 * Returns a pointer to the location of the next frame from an endpoint.
 *
 * This is a non-blocking call.
 *
 * This operation cannot proceed if the endpoint is being reset. However,
 * if the remote endpoint has called NvSciIpcResetEndpoint(), calls to this
 * function can still succeed until the next event notification on the local
 * endpoint.
 *
 * @param handle The handle to the endpoint to read from.
 * @param buf    A pointer to a destination buffer to receive the contents of
 *               the next frame.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success            Indicates a successful operation.
 * - ::NvSciError_BadParameter       Indicates an invalid @a handle.
 * - ::NvSciError_NotInitialized     Indicates NvSciIpc is uninitialized.
 * - ::NvSciError_InsufficientMemory Indicates an empty endpoint with no frames
 *                                   available for reading.
 * - ::NvSciError_ConnectionReset    Indicates the endpoint is being reset.
 */
NvSciError NvSciIpcReadGetNextFrame(NvSciIpcEndpoint handle, void **buf);

/**
 * Removes the next frame from an endpoint.
 *
 * This is a non-blocking call. The endpoint must not be empty.
 * If the endpoint was previously full, then this function notifies the remote
 * endpoint.
 *
 * This operation cannot proceed if the endpoint is being reset. However,
 * if the remote endpoint has called NvSciIpcResetEndpoint(), calls to this
 * function can still succeed until the next event notification on the local
 * endpoint.
 *
 * @param handle The handle to the endpoint to read from.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success            Indicates the frame was removed successfully.
 * - ::NvSciError_BadParameter       Indicates an invalid @a handle.
 * - ::NvSciError_NotInitialized     Indicates NvSciIpc is uninitialized.
 * - ::NvSciError_InsufficientMemory Indicates an empty endpoint with no frames
 *                                   available for reading.
 * - ::NvSciError_ConnectionReset    Indicates the endpoint is being reset.
 */
NvSciError NvSciIpcReadAdvance(NvSciIpcEndpoint handle);

/**
 * Writes a new frame to the endpoint.
 *
 * If space is available in the endpoint, this functions posts a new frame,
 * copying the contents from the provided data buffer.
 * If @a size is less than the frame size, then the remaining bytes of the frame
 * are undefined.
 *
 * This is a non-blocking call.
 * If the endpoint was previously empty, then the function notifies the remote
 * endpoint.
 *
 * This operation cannot proceed if the endpoint is being reset.
 *
 * @param handle The handle to the endpoint to write to.
 * @param buf    A pointer to a destination buffer for the contents of the next frame.
 * @param size   The number of bytes to copy from the frame, not to exceed the
 *               length of the destination buffer.
 * @param bytes  The number of bytes written.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success            Indicates a successful operation.
 * - ::NvSciError_BadParameter       Indicates an invalid @a handle.
 * - ::NvSciError_NotInitialized     Indicates NvSciIpc is uninitialized.
 * - ::NvSciError_InsufficientMemory Indicates the endpoint is full and the write
 *                                   operation aborted.
 * - ::NvSciError_TooBig             Indicates @a size is larger than the
 *                                   configured frame size of the endpoint.
 * - ::NvSciError_ConnectionReset    Indicates the endpoint is being reset.
 */
NvSciError NvSciIpcWrite(NvSciIpcEndpoint handle, const void *buf, size_t size,
	int32_t *bytes);

/**
 * Returns a pointer to the location of the next frame for writing data.
 *
 * This is a non-blocking call. The endpoint must not be full.
 *
 * This operation cannot proceed if the endpoint is being reset. However,
 * if the remote endpoint has called NvSciIpcResetEndpoint(), calls to this
 * function can still succeed until the next event notification on the local
 * endpoint.
 *
 * @param handle The handle to the endpoint to write to.
 * @param buf    A pointer to a destination buffer to hold the contents of the
 *               next frame.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success            Indicates successful operation.
 * - ::NvSciError_BadParameter       Indicates an invalid @a handle.
 * - ::NvSciError_NotInitialized     Indicates NvSciIpc is uninitialized.
 * - ::NvSciError_InsufficientMemory Indicates the endpoint is full and the write
 *                                   operation aborted.
 * - ::NvSciError_ConnectionReset    Indicates the endpoint is being reset.
 */
NvSciError NvSciIpcWriteGetNextFrame(NvSciIpcEndpoint handle, void **buf);

/**
 * Writes the next frame to the endpoint.
 *
 * This is a non-blocking call.
 * If the endpoint is not full, then post the next frame.
 * If the endpoint was previously empty, then this function notifies the
 * remote endpoint.
 *
 * This operation cannot proceed if the endpoint is being reset. However,
 * if the remote endpoint has called NvSciIpcResetEndpoint(), calls to this
 * function can still succeed until the next event notification on the local
 * endpoint.
 *
 * @param handle The handle to the endpoint to write to.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success            Indicates successful operation.
 * - ::NvSciError_BadParameter       Indicates an invalid @a handle.
 * - ::NvSciError_NotInitialized     Indicates NvSciIpc is uninitialized.
 * - ::NvSciError_InsufficientMemory Indicates the endpoint is full and the write
 *                                   operation aborted.
 * - ::NvSciError_ConnectionReset    Indicates the endpoint is being reset.
 */
NvSciError NvSciIpcWriteAdvance(NvSciIpcEndpoint handle);

/**
 * Returns endpoint information.
 *
 * @param handle NvSciIpc endpoint handle.
 * @param info   A pointer to NvSciIpcEndpointInfo object that this function
 *               copies the info to.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success        Indicates a successful operation.
 * - ::NvSciError_NotInitialized Indicates NvSciIpc is uninitialized.
 * - ::NvSciError_BadParameter   Indicates an invalid or NULL argument.
 */
NvSciError NvSciIpcGetEndpointInfo(NvSciIpcEndpoint handle,
                struct NvSciIpcEndpointInfo *info);

/**
 * @if (SWDOCS_NVSCIIPC_INTERNAL)
 * Returns NvSciIpc event notifier.
 *
 * This API is for Drive OS internal use only.
 *
 * @param handle        NvSciIpc endpoint handle.
 * @param eventNotifier A pointer to NvSciIpcEventNotifier object.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success         Indicates a successful operation.
 * - ::NvSciError_NotInitialized  Indicates NvSciIpc is uninitialized.
 * - ::NvSciError_BadParameter    Indicates an invalid or NULL argument.
 * @endif
 */
/// @cond (SWDOCS_NVSCIIPC_INTERNAL)
/* For internal use only. */
NvSciError NvSciIpcGetEventNotifier(NvSciIpcEndpoint handle,
               NvSciIpcEventNotifier *eventNotifier);
/// @endcond

/**
 * Returns the NvSciIpc file descriptor for a given endpoint.
 *
 * <b> This API is specific to Linux OS. </b>
 *
 * Event handle will be used to plug OS event notification
 * (can be read, can be written, established, reset etc.)
 *
 * @param handle NvSciIpc endpoint handle
 * @param fd     A pointer to the endpoint file descriptor.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success         Indicates a successful operation.
 * - ::NvSciError_NotInitialized  Indicates NvSciIpc is uninitialized.
 * - ::NvSciError_BadParameter    Indicates an invalid or NULL argument.
 */
NvSciError NvSciIpcGetLinuxEventFd(NvSciIpcEndpoint handle, int32_t *fd);

/**
 * Returns a bitwise OR operation on new events that occurred since the
 * last call to this function.
 *
 * This function sets @a events to the result of a bitwise OR operation of zero
 * or more @c NV_SCI_IPC_EVENT_* constants corresponding to all new events that
 * have occurred on the endpoint since:
 * - the preceding call to this function on the endpoint or
 * - opening the endpoint, if this is the first call to this function on the
 *   endpoint since it was opened.
 *
 * The parameter @a events is set to zero if no new events have
 * occurred.
 *
 * There are four types of events:
 * - @c NV_SCI_IPC_EVENT_CONN_EST   : IPC connection established
 * - @c NV_SCI_IPC_EVENT_WRITE      : IPC write
 * - @c NV_SCI_IPC_EVENT_READ       : IPC read
 * - @c NV_SCI_IPC_EVENT_CONN_RESET : IPC connection reset
 *
 * These may occur in arbitrary combinations, except for the following:
 * - @c NV_SCI_IPC_EVENT_CONN_EST is always combined with @c NV_SCI_IPC_EVENT_WRITE.
 * - @c NV_SCI_IPC_EVENT_CONN_RESET cannot be combined with any other events.

 * There are seven possible event combinations:
 * - 0
 * - @c NV_SCI_IPC_EVENT_CONN_EST and @c NV_SCI_IPC_EVENT_WRITE
 * - @c NV_SCI_IPC_EVENT_CONN_EST and @c NV_SCI_IPC_EVENT_WRITE and
 *   @c NV_SCI_IPC_EVENT_READ
 * - @c NV_SCI_IPC_EVENT_READ
 * - @c NV_SCI_IPC_EVENT_WRITE
 * - @c NV_SCI_IPC_EVENT_WRITE and @c NV_SCI_IPC_EVENT_READ
 * - @c NV_SCI_IPC_EVENT_CONN_RESET
 *
 * An @c NV_SCI_IPC_EVENT_CONN_EST event occurs on an endpoint each time a
 * connection is established through the endpoint (between the endpoint and
 * the other end of the corresponding channel).
 *
 * An @c NV_SCI_IPC_EVENT_WRITE event occurs on an endpoint:
 * -# In conjunction with the delivery of each @c NV_SCI_IPC_CONN_EST event.
 * -# Each time the endpoint ceases to be full after a prior @c NvSciIpcWrite*
 * call returned @c NvSciError_InsufficientMemory. Note however that an
 * implementation is permitted to delay the delivery of this type of
 * @c NV_SCI_IPC_EVENT_WRITE event, e.g., for purposes of improving throughput.
 *
 * An @c NV_SCI_IPC_EVENT_READ event occurs on an endpoint:
 * -# In conjunction with the delivery of each @c NV_SCI_IPC_EVENT_CONN_EST event,
 * if frames can already be read as of delivery.
 * -# Each time the endpoint ceases to be empty after a prior @c NvSciRead*
 * call returned @c NvSciError_InsufficientMemory. Note however that an
 * implementation is permitted to delay the delivery of this type of
 * @c NV_SCI_IPC_EVENT_READ event, e.g., for purposes of improving throughput.
 *
 * An @c NV_SCI_IPC_EVENT_CONN_RESET event occurs on an endpoint when the user
 * calls NvSciIpcResetEndpoint.
 *
 * @param handle NvSciIpc endpoint handle.
 * @param event  A pointer to the variable into which to store the bitwise OR
 *               result of new events.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success         Indicates a successful operation.
 * - ::NvSciError_NotInitialized  Indicates NvSciIpc is uninitialized.
 * - ::NvSciError_BadParameter    Indicates an invalid or NULL argument.
 */
NvSciError NvSciIpcGetEvent(NvSciIpcEndpoint handle, uint32_t *event);

/**
 * @if (SWDOCS_NVSCIIPC_INTERNAL)
 * Sets the pulse parameters for event notifier.
 *
 * This API is for Drive OS internal use only and specific to QNX OS.
 *
 * @param handle        NvSciIpc endpoint handle.
 * @param chid          The channel ID which was created by @c ChannelCreate_r().
 * @param pulsePriority The pulse priority.
 * @param pulseCode     An 8-bit positive pulse code.
 * @param pulseValue    A 32-bit pulse value specified by the user.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success         Indicates a successful operation.
 * - ::NvSciError_NotInitialized  Indicates NvSciIpc is uninitialized.
 * - ::NvSciError_BadParameter    Indicates an invalid @a handle.
 * @endif
 */
/// @cond (SWDOCS_NVSCIIPC_INTERNAL)
/* For internal use only. */
NvSciError NvSciIpcSetEventNotifierPulseParam(NvSciIpcEndpoint handle,
    int32_t chid, int16_t pulsePriority, int16_t pulseCode,
    int32_t pulseValue);
/// @endcond

/**
 * Sets the event pulse parameters for QNX.
 *
 * <b>This API is specific to QNX OS.</b>
 *
 * When a notification from a peer endpoint is available, the NvSciIpc library
 * sends a pulse message to the application.
 * This API is to connect @a coid to the endpoint, plug OS event notification
 * and set pulse parameters (@a pulsePriority, @a pulseCode and @a pulseValue),
 * thereby enabling the application to receive peer notifications from the
 * NvSciIpc library.
 * An application can fetch notifications from a peer endpoint using
 * @c MsgReceivePulse_r() which is blocking call.
 *
 * Prior to calling this function, both @c ChannelCreate_r() and @c ConnectAttach_r()
 * must be called in the application to obtain the value for @a coid to pass to
 * this function.
 *
 * To use the priority of the calling thread, set @a pulsePriority to
 * @c SIGEV_PULSE_PRIO_INHERIT(-1). The priority must fall within the valid
 * range, which can be determined by calling @c sched_get_priority_min() and
 * @c sched_get_priority_max().
 *
 * Applications can define any value per endpoint for @a pulseCode and @a pulseValue.
 * @a pulseCode will be used by NvSciIpc to signal IPC events and should be
 * reserved for this purpose by the application. @a pulseValue can be used
 * for the application cookie data.
 *
 * @param handle        NvSciIpc endpoint handle.
 * @param coid          The connection ID created from calling @c ConnectAttach_r().
 * @param pulsePriority The value for pulse priority.
 * @param pulseCode     The 8-bit positive pulse code specified by the user. The
 *                      values must be between @c _PULSE_CODE_MINAVAIL and
 *                      @c _PULSE_CODE_MAXAVAIL
 * @param pulseValue    A pointer to the user-defined pulse value.
 *
 * @return ::NvSciError, the completion code of the operation:
 * - ::NvSciError_Success         Indicates a successful operation.
 * - ::NvSciError_NotInitialized  Indicates NvSciIpc is uninitialized.
 * - ::NvSciError_BadParameter    Indicates an invalid @a handle.
 */
NvSciError NvSciIpcSetQnxPulseParam(NvSciIpcEndpoint handle,
	int32_t coid, int16_t pulsePriority, int16_t pulseCode,
	void *pulseValue);

/**
 * @if (SWDOCS_NVSCIIPC_INTERNAL)
 * Converts an OS-specific error code to a value in NvSciError.
 *
 * @param err OS-specific error code.
 *
 * @return Error code from ::NvSciError.
 * @endif
 */
/// @cond (SWDOCS_NVSCIIPC_INTERNAL)
/* For internal use only. */
NvSciError NvSciIpcErrnoToNvSciErr(int32_t err);
/// @endcond

/**
 * @if (SWDOCS_NVSCIIPC_INTERNAL)
 * Converts an error code from NvSciError to an OS-specific error code.
 *
 * @param nvSciErr An error from NvSciError.
 *
 * @return
 * OS specific error code.
 * @endif
 */
/// @cond (SWDOCS_NVSCIIPC_INTERNAL)
/* For internal use only. */
int32_t NvSciIpcNvSciErrToErrno(NvSciError nvSciErr);
/// @endcond

/** @} */

#ifdef __cplusplus
}
#endif
#endif /* NVSCIIPC_H */
