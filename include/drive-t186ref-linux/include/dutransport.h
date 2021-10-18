/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

/*!
 * @file
 *
 * @brief Drive Update DU-Transport library interface.
 */

#ifndef DUTRANSPORT_H_
#define DUTRANSPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------ Global Includes --------------------------------- */
#include <net/if.h>

/* ------------------------ Drive Update Includes --------------------------- */
#include "ducommon.h"

/*!
 * @defgroup du_transport_group Transport API
 *
 * Drive Update DU-Transport library interface.
 *
 * @ingroup drive_update_top
 * @{
 */
/* ------------------------ Defines------------------------------------------ */

/*!
 * Defines DU-Transport error codes.
 */
#define DU_TR_ECODE(err)                   DU_ECODE(DU_UID_TRANSPORT, 0U, (err))

#define DU_TR_ERR_GENERIC                  DU_TR_ECODE(1U)
#define DU_TR_ERR_INVALID_ARGUMENT         DU_TR_ECODE(2U)
#define DU_TR_ERR_BUFFER_TOO_SMALL         DU_TR_ECODE(3U)
#define DU_TR_ERR_NOT_FOUND                DU_TR_ECODE(4U)
#define DU_TR_ERR_NOT_SUPPORTED            DU_TR_ECODE(5U)
#define DU_TR_ERR_NO_PERMISSION            DU_TR_ECODE(6U)
#define DU_TR_ERR_NOT_INITIALIZED          DU_TR_ECODE(7U)
#define DU_TR_ERR_INSUFFICIENT_MEMORY      DU_TR_ECODE(8U)
#define DU_TR_ERR_TIMEOUT                  DU_TR_ECODE(9U)
#define DU_TR_ERR_BUSY                     DU_TR_ECODE(10U)
#define DU_TR_ERR_NO_CONNECTION            DU_TR_ECODE(11U)
#define DU_TR_ERR_HANDLE_CLOSED            DU_TR_ECODE(12U)

/*! \brief Defines the max DU-Transport name string size. */
#define DUTR_NAME_MAX   (256U)

/* ------------------------ Types ------------------------------------------- */

/*! Defines the value for an invalid DU-Transport handle id. */
#define DUTR_HANDLE_ID_INVALID   (0U)
/*! Defines the DU-Transport handle id. */
typedef uint32_t DUTR_HANDLE_ID;

/*!
 * DU-Transport buffer pointer to the user allocated buffer.
 */
typedef void *PDUTR_BUF;

/*!
 * Defines DU-Transport back-end transport protocols.
 */
typedef enum DUTR_TR_TYPE
{
    /*! IVC based inter-VM communication. */
    DUTR_TR_TYPE_IVC      = 0,
    /*! Shared memory based inter-thread communication. */
    DUTR_TR_TYPE_SHM_ITC,
    /*! Shared memory based inter-process communication. */
    DUTR_TR_TYPE_SHM_IPC,
    /*! TCP based data transport. */
    DUTR_TR_TYPE_TCP,
} DUTR_TR_TYPE;

/*!
 * Defines DU-Transport security protocols.
 */
typedef enum DUTR_SEC_TYPE
{
    /*! Security type not yet defined. */
    DUTR_SEC_TYPE_NONE = 0,
    /*! TLS security type. */
    DUTR_SEC_TYPE_TLS,
} DUTR_SEC_TYPE;

/*!
 * Defines DU-Transport security parameters.
 */
typedef uint32_t DUTR_SEC_NONE_PARAM;
typedef uint32_t DUTR_SEC_TLS_PARAM;

typedef void *PDUTR_SEC_PARAM;

/*!
 * Defines DU-Transport IVC parameters.
 */
typedef struct DUTR_IVC_PARAM
{
    /// The name string to identify an IVC channel.
    char name[DUTR_NAME_MAX];
} DUTR_IVC_PARAM, *PDUTR_IVC_PARAM;

/*!
 * Defines DU-Transport shared memory based inter-thread communication parameters.
 */
typedef struct DUTR_SHMITC_PARAM
{
    /// The name string to identify a shared memory DU-Transport handle.
    char name[DUTR_NAME_MAX];
} DUTR_SHMITC_PARAM, *PDUTR_SHMITC_PARAM;

/*!
 * Defines DU-Transport shared memory based inter-process communication parameters.
 */
typedef struct DUTR_SHMIPC_PARAM
{
    /// The name string to identify a shared memory DU-Transport handle.
    char name[DUTR_NAME_MAX];
} DUTR_SHMIPC_PARAM, *PDUTR_SHMIPC_PARAM;

/*!
 * Defines the role of TCP connection.
 */
typedef enum DUTR_TCP_ROLE
{
    /// The server waiting for client connection.
    DUTR_TCP_SERVER = 0,
    /// The client to connect to remote server.
    DUTR_TCP_CLIENT,
    /// Invalid type
    DUTR_TCP_INVALID,
} DUTR_TCP_ROLE;

/*!
 * Defines the IPV4 IP address string size.
 */
#define DUTR_IPV4_ADDR_STR_SIZE (16U)
/*!
 * Defines the structure of TCP level address.
 */
typedef struct DUTR_TCP_IF_PARAM
{
    /// IPV4 IP address string.
    char      ip[DUTR_IPV4_ADDR_STR_SIZE];
    /// Port to be used.
    uint16_t  port;
} DUTR_TCP_IF_PARAM, *PDUTR_TCP_IF_PARAM;

/*!
 * Defines the interface name string size.
 */
#define DUTR_IFNAME_MAX IFNAMSIZ
/*!
 * Defines the parameter for TCP transport protocol.
 */
typedef struct DUTR_TCP_PARAM
{
    /// String of interface name.
    char               ifName[DUTR_IFNAME_MAX];
    /// The address bound to a local socket.
    DUTR_TCP_IF_PARAM  local;
    /// The remote server address to connect, used only for DUTR_TCP_CLIENT.
    DUTR_TCP_IF_PARAM  remote;
    /// Specify if it is the server or client.
    DUTR_TCP_ROLE      role;
} DUTR_TCP_PARAM, *PDUTR_TCP_PARAM;

/*!
 * Holds the pointer to parameter of DU-Transport back-end protocols.
 */
typedef void *PDUTR_TR_PARAM;

/* ------------------------ Function Declarations --------------------------- */
/*!
 * @brief Initializes a handle for DU-Transport connection.
 *
 * This function should be only called once before closing the connection.
 * It initializes a handle of DU-Transport connection with the input parameters.
 * It will try to setup the connection with the remote end point but it may fail
 * because the remote end point may not be ready yet. This function does not
 * return an error on connection setup, instead, it returns @ref DU_OK once the handle
 * is initialized successfully.
 *
 * If the connection is not setup by this function, the later calling on other
 * DU-Transport APIs will try to setup the connection and returns error code on
 * failure.
 *
 * @interface DU-Transport Interface
 *
 * @language C
 *
 * @param[in] trType
 *          Type of back-end transport protocol used.
 *
 * @param[in] seType
 *          Type of security protocol, only support @ref DUTR_SEC_TYPE_NONE for now.
 *
 * @param[in] pTrParam
 *          Pointer to the argument of selected transport protocol. For instance
 *          if trType is @c DUTR_TR_TYPE_IVC, then it shall point to the structure
 *          @ref DUTR_IVC_PARAM.
 *
 * @param[in] pSeParam
 *          Pointer to the argument of selected security protocol. For instance
 *          if seType is @c DUTR_SEC_TYPE_TLS, then it shall point to the structure
 *          @ref DUTR_SEC_TLS_PARAM.
 *
 * @param[out] pHandleId
 *          Pointer to the initialized DU-Transport handle Id on success.
 *          Return @ref DUTR_HANDLE_ID_INVALID on errors.
 *
 * @return @ref DU_OK
 *          Upon success.
 *
 * @return @ref DU_TR_ERR_INVALID_ARGUMENT
 *          The input parameters contain invalid value.
 *
 * @return @ref DU_TR_ERR_NOT_INITIALIZED
 *          DU-Transport fails to initialize itself.
 *
 * @return @ref DU_TR_ERR_NOT_SUPPORTED
 *          The input transport type or the security type is not supported.
 *
 * @return @ref DU_TR_ERR_GENERIC
 *          Failed to open and initialize a new DU-Transport handle.
 */
DU_RCODE dutrOpen
(
    DUTR_TR_TYPE            trType,
    DUTR_SEC_TYPE           seType,
    const PDUTR_TR_PARAM    pTrParam,
    const PDUTR_SEC_PARAM   pSeParam,
    DUTR_HANDLE_ID         *pHandleId
);

/*!
 * @brief Closes an active DU-Transport connection.
 *
 * @interface DU-Transport Interface
 *
 * @language C
 *
 * @param[in] handleId
 *          DU-Transport handle ID.
 *
 * @return @ref DU_OK
 *          Upon success.
 *
 * @return @ref DU_TR_ERR_INVALID_ARGUMENT
 *          The input handle ID is not a valid value return from dutrOpen,
 *          or it has been already closed.
 *
 * @return @ref DU_TR_ERR_NOT_INITIALIZED
 *          DU-Transport fails to initialize itself.
 *
 * @return @ref DU_TR_ERR_NOT_SUPPORTED
 *          The input handle ID is associated with an supported DU-Transport
 *          back-end type. It happens only if an internal error occurs.
 *
 * @return @ref DU_TR_ERR_GENERIC
 *          On unexpected internal errors.
 */
DU_RCODE dutrClose
(
    DUTR_HANDLE_ID handleId
);

/*!
 * @brief Gets the current DU-Transport version.
 *
 * @interface DU-Transport Interface
 *
 * @language C
 *
 * @param[out] pVersion
 *          Returned version number, this variable is caller allocated.
 *
 * @return @ref DU_OK
 *          Upon success.
 *
 * @return @ref DU_TR_ERR_INVALID_ARGUMENT
 *          The input parameter is a NULL pointer.s
 */
DU_RCODE dutrGetVersion
(
    uint32_t *pVersion
);

/*!
 * @brief Waits for DU-Transport connection with the remote peer.
 *
 * Wait for the DU-Transport connection with the remote peer until the
 * specified time expires. This function is a blocking call.
 *
 * @interface DU-Transport Interface
 *
 * @language C
 *
 * @param[in] handleId
 *          DU-Transport handle Id.
 *
 * @param[in] timeoutMs
 *          Maximum time in milliseconds that the function may wait for the
 *          connection setup or recovery with the other peer. Note that the
 *          timer resolution is 100ms, so the actual waiting time specified by
 *          this parameter will be (timeoutMs / 100) * 100 ms. If timeoutMs is
 *          -1, then it will wait forever until the connection is up or the
 *          connection is unrecoverable.
 *
 * @return @ref DU_OK
 *          P2P connection is setup before timeout.
 *
 * @return @ref DU_TR_ERR_INVALID_ARGUMENT
 *          The input handle ID is not a valid handle.
 *
 * @return @ref DU_TR_ERR_NOT_INITIALIZED
 *          DU-Transport fails to initialize itself.
 *
 * @return @ref DU_TR_ERR_NOT_SUPPORTED
 *          The input handle ID is associated with an supported DU-Transport
 *          backend type. It happens only if an internal error occurs.
 *
 * @return @ref DU_TR_ERR_HANDLE_CLOSED
 *          Handle is being closed by the user.
 *
 * @return @ref DU_TR_ERR_NO_CONNECTION
 *          P2P connection is not setup before timeout.
 */
DU_RCODE dutrWaitForConnection
(
    DUTR_HANDLE_ID  handleId,
    uint64_t        timeoutMs
);

/*!
 * @brief Allocates a buffer for data transmission.
 *
 * Allocate a buffer for data transmission for a DU-Transport handle.
 * This function may return error if connection is broken or not setup.
 *
 * @interface DU-Transport Interface
 *
 * @language C
 *
 * @param[in] handleId
 *          DU-Transport handle Id who allocates the buffer.
 *
 * @param[in] size
 *          Requested buffer size.
 *
 * @param[out] ppBuf
 *          On success, it returns the pointer to the allocated buffer.
 *          On failure, it returns NULL.
 *
 * @param[in] timeoutMs
 *          Maximum time in milliseconds that the function may block on the
 *          buffer allocating operation to wait for an available buffer. Note
 *          that the timer resolution is 100ms, so the actual blocking time
 *          specified by this parameter will be (timeoutMs / 100) * 100 ms.
 *          If timeoutMs is zero, then this function will return immediately
 *          no matter if there is an available buffer requested by the caller.
 *          Otherwise this function will wait for maximum timeoutMs before it
 *          returns. Particularly, if timeoutMs is -1, this function will wait
 *          forever until a required buffer is available.
 *          If the connection between two peers is not setup, or is broken or
 *          closed, then this function will return immediately with error code.
 *
 * @return @ref DU_OK
 *          Upon success.
 *
 * @return @ref DU_TR_ERR_INVALID_ARGUMENT
 *          Input parameters contain invalid values.
 *
 * @return @ref DU_TR_ERR_NOT_INITIALIZED
 *          DU-Transport fails to initialize itself.
 *
 * @return @ref DU_TR_ERR_NOT_SUPPORTED
 *          The input handle ID is associated with an supported DU-Transport
 *          back-end type. It happens only if an internal error occurs.
 *
 * @return @ref DU_TR_ERR_NO_CONNECTION
 *          P2P connection is not setup for the handle.
 *
 * @return @ref DU_TR_ERR_INSUFFICIENT_MEMORY
 *          Cannot allocate the buffer before timeout.
 *
 * @return @ref DU_TR_ERR_HANDLE_CLOSED
 *          Cannot allocate buffer due to handle closed by the user.
 *
 */
DU_RCODE dutrAllocBuf
(
    DUTR_HANDLE_ID  handleId,
    uint64_t        size,
    PDUTR_BUF      *ppBuf,
    uint64_t        timeoutMs
);

/*!
 * @brief Discards a buffer.
 *
 * This function is called by the sender to discard a buffer allocated with
 * dutrAllocBuf() on the failure of dutrTxBuf() when the sender does not need
 * to use the buffer any longer, in order to avoid memory leak of DU-Transport.
 *
 * @interface DU-Transport Interface
 *
 * @language C
 *
 * This function must not be called by the receiver.
 *
  * @param[in] pBuf
 *          Pointer to the buffer to be discarded.
 *
 * @return @ref DU_OK
 *          Upon success.
 *
 * @return @ref DU_TR_ERR_INVALID_ARGUMENT
 *          The input parameter pBuf points to an invalid buffer.
 *
 * @return @ref DU_TR_ERR_NOT_INITIALIZED
 *          DU-Transport buffer pools are not initialized.
 *
 * @return @ref DU_TR_ERR_GENERIC
 *          An unexpected internal error occurs.
 */
DU_RCODE dutrDiscardBuf
(
    PDUTR_BUF     pBuf
);

/*!
 * @brief Transmits the buffer to the other end point.
 *
 * Transmit the buffer to the other end point of the DU-Transport handle.
 * The buffer must be allocated by dutrAllocBuf() with the same handle Id, and
 * it does not require the caller to free this buffer explicitly on a successful
 * call on dutrTxBuf(). If dutrTxBuf() fails, the buffer can be reused by the
 * sender to write new data and to send the buffer again by calling dutrTxBuf(),
 * or the buffer shall be discarded with dutrDiscardBuf() if it is not needed
 * any longer.
 *
 * This function may return error if connection is broken or not setup.
 *
 * @interface DU-Transport Interface
 *
 * @language C
 *
 * @param[in] handleId
 *          DU-Transport handle Id who transmits the buffer.
 *
 * @param[in] pBuf
 *          Pointer to the start address of the buffer to be transmitted.
 *
 * @param[in] size
 *          The size in bytes to be transmitted.
 *
 * @return @ref DU_OK
 *          Upon success.
 *
 * @return @ref DU_TR_ERR_INVALID_ARGUMENT
 *          Invalid input parameters are found.
 *
 * @return @ref DU_TR_ERR_NOT_INITIALIZED
 *          DU-Transport fails to initialize itself.
 *
 * @return @ref DU_TR_ERR_NOT_SUPPORTED
 *          The input handle ID is associated with an supported DU-Transport
 *          back-end type. It happens only if an internal error occurs.
 *
 * @return @ref DU_TR_ERR_INSUFFICIENT_MEMORY
 *          The buffer size to be sent exceeds the maximum supported buffer size
 *          of the transport type of the handle.
 *
 * @return @ref DU_TR_ERR_BUSY
 *          Failed to transmit the buffer due to resource busy, can try again.
 *
 * @return @ref DU_TR_ERR_NO_CONNECTION
 *          P2P connection is not setup for the handle.
 *
 * @return @ref DU_TR_ERR_HANDLE_CLOSED
 *          Failed to transmit the buffer due to handle being closed by the user.
 *
 * @return @ref DU_TR_ERR_GENERIC
 *          Failed to transmit the buffer due to internal errors.
 */
DU_RCODE dutrTxBuf
(
    DUTR_HANDLE_ID  handleId,
    const PDUTR_BUF pBuf,
    uint64_t        size
);

/*!
 * @brief Receives data from DU-Transport handle.
 *
 * Try to receive data from the other end point of the DU-Transport handle.
 * The received data buffer is the same memory region of the transmitter in
 * the case of zero-copy. For non-zero-copy case, this function will try to
 * allocate a buffer to receive the incoming data.
 *
 * This function may return error if connection is broken or not setup.
 *
 * @interface DU-Transport Interface
 *
 * @language C
 *
 * @param[in] handleId
 *          DU-Transport handle Id who receives the buffer.
 *
 * @param[in] timeoutMs
 *          Maximum time in milliseconds that the function may block on the
 *          receiving operation. Note that the timer resolution is 100ms, so
 *          the actual blocking time specified by this parameter will be
 *          (timeoutMs / 100) * 100 ms.
 *          If timeoutMs is zero, then this function will return immediately
 *          no matter if it receives any data or not. Otherwise this function
 *          will wait for maximum timeoutMs before it returns. Particularly,
 *          if timeoutMs is -1, this function will wait forever until a buffer
 *          is received or errors on handle connections.
 *          If the connection between two peers is not setup, or is broken or
 *          closed, then this function will return immediately with error code
 *          regardless of the value of this parameter.
 *
 * @param[out] ppBuf
 *          Pointer to the buffer containing the received data.
 *          It is set as NULL on errors.
 *
 * @param[out] pSize
 *          Pointer to the integer of the size of the received buffer, in bytes.
 *
 * @return @ref DU_OK
 *          Upon success.
 *
 * @return @ref DU_TR_ERR_INVALID_ARGUMENT
 *          Invalid input parameters are found.
 *
 * @return @ref DU_TR_ERR_NOT_INITIALIZED
 *          DU-Transport fails to initialize itself.
 *
 * @return @ref DU_TR_ERR_NOT_SUPPORTED
 *          The input handle ID is associated with an supported DU-Transport
 *          back-end type. It happens only if an internal error occurs.
 *
 * @return @ref DU_TR_ERR_INSUFFICIENT_MEMORY
 *          Failed to receive buffer due to insufficient memory.
 *
 * @return @ref DU_TR_ERR_TIMEOUT
 *          No buffer to receive before timeout.
 *
 * @return @ref DU_TR_ERR_NO_CONNECTION
 *          P2P connection is not setup for the handle.
 *
 * @return @ref DU_TR_ERR_HANDLE_CLOSED
 *          Failed to receive the buffer due to handle being closed by the user.
 *
 * @return @ref DU_TR_ERR_GENERIC
 *          Failed to receive a buffer due to internal errors.
 */
DU_RCODE dutrRxBuf
(
    DUTR_HANDLE_ID  handleId,
    uint64_t        timeoutMs,
    PDUTR_BUF      *ppBuf,
    uint64_t       *pSize
);

/*!
 * @brief Frees the receiving buffer of the DU-Transport handle.
 *
 * This function shall be called by the receiver end point to free the buffer
 * of the received data when the buffer is not used any longer. Note that a
 * buffer may be freed by the user after the handle is closed, therefore it is
 * allowed to free a buffer by calling this API with an invalid handle ID, to
 * avoid memory leak in this case.
 *
 * @interface DU-Transport Interface
 *
 * @language C
 *
 * @param[in] handleId
 *          DU-Transport handle Id who receives the buffer.
 *
 * @param[in] pBuf
 *          Pointer to the buffer to be freed.
 *
 * @return @ref DU_OK
 *          Upon success.
 *
 * @return @ref DU_TR_ERR_INVALID_ARGUMENT
 *          The input parameter pBuf points to an invalid buffer.
 *
 * @return @ref DU_TR_ERR_NOT_INITIALIZED
 *          DU-Transport buffer pools are not initialized.
 */
DU_RCODE dutrFreeRxBuf
(
    DUTR_HANDLE_ID handleId,
    PDUTR_BUF      pBuf
);

/*!
 * @brief Gets the maximum supported buffer size of the DU-Transport handle.
 *
 * This function query the maximum supported buffer size of a DU-Transport
 * handle, as the maximum value of "size" for the parameter of dutrAllocBuf().
 * Note that dutrAllocBuf() may still fail due to no enough free buffer even
 * if the input "size" to dutrAllocBuf() is no more than the value returned
 * from this function.
 *
 * @interface DU-Transport Interface
 *
 * @language C
 *
 * @param[in] handleId
 *          DU-Transport handle Id who receives the buffer.
 *
 * @param[out] pSize
 *          Pointer to the integer of the returned maximum buffer size in bytes.
 *
 * @return @ref DU_OK
 *          Upon success.
 *
 * @return @ref DU_TR_ERR_INVALID_ARGUMENT
 *          Invalid input parameters are found.
 *
 * @return @ref DU_TR_ERR_NOT_INITIALIZED
 *          DU-Transport fails to initialize.
 *
 * @return @ref DU_TR_ERR_GENERIC
 *          On unexpected internal errors.
 */
DU_RCODE dutrGetMaxBufSize
(
    DUTR_HANDLE_ID  handleId,
    uint64_t       *pSize
);

/*!
 * @brief Gets the total available free buffer size of the DU-Transport handle.
 *
 * This function queries the current total available free buffer size can be
 * allocated by the specified DU-Transport handle Id. It is for diagnostic usage
 * only.
 *
 * @interface DU-Transport Interface
 *
 * @language C
 *
 * @param[in] handleId
 *          DU-Transport handle Id who receives the buffer.
 *
 * @param[out] pSize
 *          Pointer to the integer of the available buffer size in bytes.
 *
 * @return @ref DU_OK
 *          Upon success.
 *
 * @return @ref DU_TR_ERR_INVALID_ARGUMENT
 *          Invalid input parameters are found.
 *
 * @return @ref DU_TR_ERR_NOT_INITIALIZED
 *          DU-Transport fails to initialize.
 */
DU_RCODE dutrGetAvailableMem
(
    DUTR_HANDLE_ID  handleId,
    uint64_t       *pSize
);

/*!
 * @brief Forward the receiving buffer to another DU-TRANSPORT handle
 *
 * This function will forward a buffer from one DU Transport connection to
 * another. On successful forward, the original buffer will be freed.
 * If any failure occurs, buffer ownership will remain with the caller.
 *
 * @param[in] dstHandleId
 *          DU-TRANSPORT handle Id to whom the buffer is forwarded
 *
 * @param[in] srcHandleId
 *          DU-TRANSPORT handle Id who receives the buffer
 *
 * @param[in] pBuf
 *          Pointer to the buffer to be forwarded. pBuf will be freed on success
 *          but not on failure
 *
 * @return DU_OK
 *          Upon success
 *
 * @return DU_TR_INSUFFICIENT_MEMORY
 *          Could not allocate buffer for sending to dest
 *
 * @return DU_TR_ERR_INVALID_ARGUMENT
 *          Invalid dutr handle or NULL buffer passed in
 *
 * @return DU_TR_ERR_NOT_INITIALIZED
 *          DU-TRANSPORT fails to initialize itself
 *
 * @return DU_TR_ERR_NOT_SUPPORTED
 *          The input handle ID is associated with an supported DU-TRANSPORT
 *          backend type. It happens only if an internal error occurs.
 *
 * @return DU_TR_ERR_BUSY
 *          Failed to transmit the buffer due to resource busy, can try again
 *
 * @return DU_TR_ERR_NO_CONNECTION
 *          P2P connection is not setup for the handle
 *
 * @return DU_TR_ERR_GENERIC
 *          Failed to transmit the buffer due to internal errors
 */
DU_RCODE dutrFwdRxBuf
(
    DUTR_HANDLE_ID  dstHandleId,
    DUTR_HANDLE_ID  srcHandleId,
    const PDUTR_BUF pBuf
);
/** @} */

#ifdef __cplusplus
}
#endif

#endif // DUTRANSPORT_H_
