/*
 * Copyright (c) 2020 NVIDIA Corporation. All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and
 * proprietary rights in and to this software, related documentation and any
 * modifications thereto. Any use, reproduction, disclosure or distribution
 * of this software and related documentation without an express license
 * agreement from NVIDIA Corporation is strictly prohibited.
 */
/**
 * @file
 *
 * @brief <b> NVIDIA Software Communications Interface (SCI) : NvSciStream </b>
 *
 * The NvSciStream library is a layer on top of NvSciBuf and NvSciSync libraries
 * to provide utilities for streaming sequences of data packets between
 * multiple application modules to support a wide variety of use cases.
 */
#ifndef NVSCISTREAM_API_H
#define NVSCISTREAM_API_H

#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif
#include "nvscierror.h"
#include "nvscibuf.h"
#include "nvscisync.h"
#include "nvsciipc.h"
#include "nvscistream_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nvsci_stream_apis Streaming APIs
 *
 * The NvSciStream library is a layer on top of NvSciBuf and NvSciSync libraries
 * that provides utilities for streaming sequences of data packets between
 * multiple application modules to support a wide variety of use cases.
 *
 * @ingroup nvsci_group_stream
 * @{
 */

/*!
 * \brief Connects two stream blocks.
 *
 * Connects an available output of one block with an available input
 * of another block.
 *
 * Each input and output can only have one connection. A stream is fully
 * connected when all inputs and outputs of all blocks in the stream have a
 * connection.
 *
 * <b>Preconditions</b>
 *
 * - The upstream block has an available output connection.
 * - The downstream block has an available input connection.
 *
 * <b>Actions</b>
 *
 * - Establish a connection between the two blocks.
 *
 * <b>Postconditions</b>
 *
 * - When the block has a complete chain of connections to the producer,
 *   it will receive a ConnectUpstream event.
 * - When the block has a complete chain of connections to all consumers,
 *   it will receive a ConnectDownstream event.
 *
 * \param[in] upstream Handle of the upstream block.
 * \param[in] downstream Handle of the downstream block.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success The connection was made successfully.
 * - ::NvSciError_BadParameter @a upstream or @a downstream is not a valid block.
 * - ::NvSciError_InvalidState: @a upstream and @a downstream are already connected.
 * - ::NvSciError_AccessDenied: @a upstream or @a downstream does not allow explicit
 *     connection via NvSciStreamBlockConnect.
 * - ::NvSciError_StreamInternalError An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851064}
 */
NvSciError NvSciStreamBlockConnect(
    NvSciStreamBlock const upstream,
    NvSciStreamBlock const downstream
);

/*! \brief Creates a stream producer block.
 *
 *  Create a block for the producer end of a stream. All streams
 *  require one producer block. Producer blocks must have a pool
 *  attached to feed them their buffers. Producer blocks have one
 *  output connection and no input connections.

 *  Once the stream is fully connected, this block can be used to retrieve
 *  consumer requirements and finalize producer settings.
 *
 *  <b>Preconditions</b>:
 *  None.
 *
 *  <b>Actions</b>:
 *  - Creates a new instance of producer block.
 *  - Binds the producer block with the given pool block.
 *
 *  <b>Postconditions</b>:
 *  - The block is ready to be connected to other stream blocks.
 *  - The block can be monitored for events.
 *
 *  \param [in] pool The pool associated with the block.
 *  \param [out] producer Handle for new producer block.
 *
 *  \return ::NvSciError, the completion code of this operation.
 *  - ::NvSciError_Success The block was set up successfully.
 *  - ::NvSciError_BadParameter If @a pool is invalid or
 *      if the output parameter @a producer is a null pointer.
 *  - ::NvSciError_InsufficientMemory The block is not created.
 *  - ::NvSciError_StreamInternalError The block cannot be initialized properly.
 *      An internal system error occurred, such as a mutex-locking failure.
 *
 *  \implements{11851103}
 */
NvSciError NvSciStreamProducerCreate(
    NvSciStreamBlock const pool,
    NvSciStreamBlock *const producer
);

/*! \brief Creates a stream consumer block.
 *
 *  Create a block for the consumer end of a stream. All streams
 *  require at least one consumer block. Consumer blocks must have
 *  a queue attached to feed them pending frames. Consumer blocks have
 *  one input connection and no output connections.
 *
 *  Once the stream is fully connected, this block can be used to retrieve
 *  producer requirements and finalize consumer settings.
 *
 *  <b>Preconditions</b>:
 *  None.
 *
 *  <b>Actions</b>:
 *  - Creates a new instance of consumer block.
 *  - Binds the consumer block with the given queue block.
 *
 *  <b>Postconditions</b>:
 *  - The block is ready to be connected to other stream blocks.
 *  - The block can be monitored for events.
 *
 *  \param [in] queue The queue associated with the block.
 *  \param [out] consumer Handle for new consumer block.
 *
 *  \return ::NvSciError, the completion code of this operation.
 *  - ::NvSciError_Success The block was set up successfully.
 *  - ::NvSciError_BadParameter If @a queue is invalid or
 *      if the output block @a consumer is a null pointer.
 *  - ::NvSciError_InsufficientMemory The block is not created.
 *  - ::NvSciError_StreamInternalError The block cannot be initialized properly.
 *       An internal system error occurred, such as a mutex-locking failure.
 *
 *  \implements{11851160}
 */
NvSciError NvSciStreamConsumerCreate(
    NvSciStreamBlock const queue,
    NvSciStreamBlock *const consumer
);

/*!
 * \brief Creates a static stream pool block.
 *
 * Create a block for management of a stream's packet pool. Every producer
 * must own a pool block, provided at the producer's creation.
 *
 * A static pool has a fixed number of packets which must be fully defined
 * before streaming begins, and is intended for safety-certified usage.
 *
 * Once the stream is fully connected and the application (which may or may
 * not be the same as the producer application) has determined the packet
 * requirements, this block can be used to bind memory buffers to each packet.
 *
 * <b>Preconditions</b>
 *
 * None.
 *
 * <b>Actions</b>
 *
 * - Allocate data structures to describe packets.
 * - Initialize queue of available packets.
 *
 * <b>Postconditions</b>
 *
 * - The block is ready to be connected to other stream blocks.
 * - The block can be monitored for events.
 *
 * \param[in] numPackets Number of packets.
 * \param[out] pool Handle for the new pool block.
 *
 * \return ::NvSciError, the completion code of this operation.
 *  - ::NvSciError_Success The block was set up successfully.
 *  - ::NvSciError_BadParameter The output parameter @a pool is a null pointer.
 *  - ::NvSciError_InsufficientMemory The block is not created.
 *  - ::NvSciError_StreamInternalError The block cannot be initialized properly.
 *      An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851211}
 *
 */
NvSciError NvSciStreamStaticPoolCreate(
    uint32_t const numPackets,
    NvSciStreamBlock *const pool
);

/*!
 * \brief Creates a mailbox queue block.
 *
 * Create a block for tracking the next packet available to be
 * acquired. This is one available type of queue block. Every consumer
 * must own a queue block, provided at the time the consumer is created.
 *
 * A mailbox queue holds a single packet. If a new packet arrives, the old one
 * is replaced and returned to the pool for reuse without ever being acquired.
 *
 * This type of queue is intended for consumer applications
 * which don't need to process every packet and always wish to have the
 * latest input available.
 *
 * Once connected, the application does not directly interact with this block.
 * The consumer block will communicate with it to obtain new packets.
 *
 * <b>Preconditions</b>
 *
 * None.
 *
 * <b>Actions</b>
 *
 * - Initialize a queue to hold a single packet for acquire.
 *
 * <b>Postconditions</b>
 *
 * - The block is ready to be connected to other stream blocks.
 *
 * \param[out] queue Handle for new queue block.
 *
 * \return ::NvSciError, the completion code of this operation.
 *  - ::NvSciError_Success The block was set up successfully.
 *  - ::NvSciError_BadParameter The output parameter @a queue is a null pointer.
 *  - ::NvSciError_InsufficientMemory The block is not created.
 *  - ::NvSciError_StreamInternalError The block cannot be initialized properly.
 *      An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851247}
 */
NvSciError NvSciStreamMailboxQueueCreate(
    NvSciStreamBlock *const queue
);

/*!
 * \brief Creates a FIFO queue block.
 *
 * Create a block for tracking the list of packets available to be
 * acquired. This is one available type of queue block. Every consumer
 * must own a queue block, provided at the time the consumer is created.
 *
 * A FIFO queue holds a list of packets, which will be acquired in the order
 * they were presented.
 *
 * If a new packet arrives, it is added to the end of the FIFO.
 *
 * This type of queue is intended for consumer applications which must process
 * every packet that is produced.
 *
 * Once connected, the application does not directly interact with this block.
 * The consumer block will communicate with it to obtain new packets.
 *
 * <b>Preconditions</b>
 *
 * None.
 *
 * <b>Actions</b>
 *
 * - Initialize a queue to manage waiting packets.
 *
 * <b>Postconditions</b>
 *
 * - The block is ready to be connected to other stream blocks.
 *
 * \param[out] queue Handle for the new queue block.
 *
 * \return ::NvSciError, the completion code of this operation.
 *  - ::NvSciError_Success The block was set up successfully.
 *  - ::NvSciError_BadParameter The output parameter @a queue is a null pointer.
 *  - ::NvSciError_InsufficientMemory The block is not created.
 *  - ::NvSciError_StreamInternalError The block cannot be initialized properly.
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851298}
 */
NvSciError NvSciStreamFifoQueueCreate(
    NvSciStreamBlock *const queue
);

/*!
 * \brief Creates a multicast block.
 *
 * Create a block allowing for one input and one or more outputs.
 *
 * Multicast block broadcasts messages sent from upstream to all of the
 * downstream blocks.
 *
 * Mulitcast block aggregates messages of the same type from downstreams into
 * one before sending it upstream.
 *
 * <b>Preconditions</b>
 *
 * None.
 *
 * <b>Actions</b>
 *
 * - Initialize a block that takes one input and one or more outputs.
 *
 * <b>Postconditions</b>
 *
 * - The block is ready to be connected to other stream blocks.
 *
 * \param[in] outputCount number of output blocks that will be connected.
 * \param[out] multicast Handle for the new multicast block.
 *
 * \return ::NvSciError, the completion code of this operation.
 *  - ::NvSciError_Success The block was set up successfully.
 *  - ::NvSciError_BadParameter The output parameter @a multicast is a null
 *     pointer, or @b outputCount is larger than the number allowed.
 *  - ::NvSciError_InsufficientMemory The block is not created.
 *  - ::NvSciError_StreamInternalError The block cannot be initialized properly.
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851361}
 */
NvSciError NvSciStreamMulticastCreate(
    uint32_t const outputCount,
    NvSciStreamBlock *const multicast
);

/*!
 * \brief Creates an IPC source block.
 *
 * Creates the upstream half of an IPC block pair which allows
 * packets to be transmitted between processes.
 *
 * IPC source blocks have one input connection and no output connection.
 *
 * A IPC source block connects to downstream through the @a ipcEndpoint used to
 * create the block.
 *
 * <b>Preconditions</b>
 *
 * None.
 *
 * <b>Actions</b>
 *
 * - Ready IPC endpoint.
 * - Allocate buffers for data transmission.
 *
 * <b>Postconditions</b>
 *
 * - The block is ready to be connected to other stream blocks.
 *
 * \param[in] ipcEndpoint IPC endpoint handle.
 * \param[in] syncModule NvSciSyncModule that is used to import a
 *        NvSciSyncAttrList across an IPC / IVC boundary.
 *
 *        This must be same module that was used to create NvSciSyncAttrList
 *        when specifying the associated NvSciStreamSyncAttr object.
 * \param[in] bufModule NvSciBufModule that is used to import a
 *        NvSciBufAttrList across an IPC / IVC boundary.
 *
 *        This must be same module that was used to create NvSciBufAttrList
 *        when specifying the associated NvSciStreamElementAttr object.
 * \param[out] ipc Handle for the new NvSciStream IPC source block.
 *
 * \return ::NvSciError, the completion code of this operation.
 *  - ::NvSciError_Success The block was set up successfully.
 *  - ::NvSciError_BadParameter The output parameter @a ipc is a null pointer.
 *  - ::NvSciError_InsufficientMemory The block is not created.
 *  - ::NvSciError_StreamInternalError The block cannot be initialized properly.
 *      An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851391}
 */
NvSciError NvSciStreamIpcSrcCreate(
    NvSciIpcEndpoint const ipcEndpoint,
    NvSciSyncModule const syncModule,
    NvSciBufModule const bufModule,
    NvSciStreamBlock *const ipc
);

/*!
 * \brief Creates an IPC destination block.
 *
 * Creates the downstream half of an IPC block pair which allows
 * packets to be transmitted between processes.
 *
 * IPC destination blocks have one output connection and no input connection.
 *
 * An IPC destination block connects to upstream through the ipcEndpoint used
 * to create the block.
 *
 * <b>Preconditions</b>
 *
 * None.
 *
 * <b>Actions</b>
 *
 * - Ready IPC endpoint.
 * - Allocate buffers for data transmission.
 *
 * <b>Postconditions</b>
 *
 * - The block is ready to be connected to other stream blocks.
 *
 * \param[in] ipcEndpoint IPC endpoint handle.
 * \param[in] syncModule NvSciSyncModule that is used to import a
 *        NvSciSyncAttrList across an IPC / IVC boundary.
 *
 *        This must be same module that was used to create NvSciSyncAttrList
 *        when specifying the associated NvSciStreamSyncAttr object.
 * \param[in] bufModule NvSciBufModule that is used to import a
 *        NvSciBufAttrList across an IPC / IVC boundary.
 *
 *        This must be same module that was used to create NvSciBufAttrList
 *        when specifying the associated NvSciStreamElementAttr object.
 * \param[out] ipc Handle for new NvSciStream IPC destination block.
 *
 * \return ::NvSciError, the completion code of this operation.
 *  - ::NvSciError_Success The block was set up successfully.
 *  - ::NvSciError_BadParameter @a ipc is a null pointer.
 *  - ::NvSciError_InsufficientMemory The block is not created.
 *  - ::NvSciError_StreamInternalError The block can't be initialized properly.
 *      An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851412}
 */
NvSciError NvSciStreamIpcDstCreate(
    NvSciIpcEndpoint const ipcEndpoint,
    NvSciSyncModule const syncModule,
    NvSciBufModule const bufModule,
    NvSciStreamBlock *const ipc
);

/*!
 * \brief Queries for the next event from block's event queue,
 * optionally waiting, and returns it to the caller.
 *
 * The appropriate handling of each type of event is described in the section
 * for the corresponding event structure.
 *
 * <b>Preconditions</b>
 *
 * - The block to query has been created.
 *
 * <b>Actions</b>
 *
 * - Wait until an event is pending on the block or the specified timeout
 *   period is reached, whichever comes first.
 * - Remove the next event (if any) from the block's event queue, fill in
 *   the event data structure.
 *
 * <b>Postconditions</b>
 *
 * - As defined for each event type, dequeuing an event may allow other
 *   operations on the block to proceed.
 *
 * \param[in] block Block handle.
 * \param[in] timeoutUsec Timeout in microseconds (-1 to wait forever).
 * \param[out] event Event and the corresponding event data.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success @a event is filled with the queried event data.
 * - ::NvSciError_Timeout The @a timeoutUsec period was reached before an event
 *     became available.
 * - ::NvSciError_BadParameter @a event is null or @a block is invalid.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 *     On safety-critical systems, this function never generates this error.
 *     A system-wide mechanism shall be notified and intervene when such error
 *     occurs.
 *
 * \implements{11851433}
 */
NvSciError NvSciStreamBlockEventQuery(
    NvSciStreamBlock const block,
    int64_t const timeoutUsec,
    NvSciStreamEvent *const event
);

/*!
 * \brief Sets block sync object requirements.
 *
 * Used with producer and consumer to establish their requirements for
 * sync objects provided by the other endpoint(s).
 *
 * <b>Preconditions</b>
 *
 * - Block must have received appropriate Connect event from the other
 *   endpoint(s).
 *
 * <b>Actions</b>
 *
 * - A syncAttr event is sent to the other endpoint(s).
 *
 * <b>Postconditions</b>
 *
 * None.
 *
 * \param[in] block Block handle.
 * \param[in] synchronousOnly If true, endpoint cannot support sync objects
 * \param[in] waitSyncAttrList Requirements for endpoint to wait for sync objects
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Sync requirements registered successfully.
 * - ::NvSciError_BadParameter @a block refers to an invalid instance.
 * - ::NvSciError_NotImplemented @a block is not a producer or consumer block.
 * - ::NvSciError_StreamNotConnected Blocks are not fully connected.
 * - ::NvSciError_NotPermitted Blocks are in safety mode. Operation is not allowed.
 * - ::NvSciError_InsufficientMemory Memory allocation failed for IPC-related
 *     operations.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851484}
 */
NvSciError NvSciStreamBlockSyncRequirements(
    NvSciStreamBlock const block,
    bool const synchronousOnly,
    NvSciSyncAttrList const waitSyncAttrList
);

/*!
 * \brief Provides block's number of sync objects.
 *
 * Used with producer and consumer to pass the number of sync objects they will
 * write into to the other endpoint(s).
 *
 * The sync object count is implicitly set to one.
 *
 * If the block provides no sync object, the application must make this call,
 * with count equal to zero, to advance the state of the stream and inform the
 * other endpoint(s).
 *
 * <b>Preconditions</b>
 *
 * - Block must have received SyncAttr event from the
 *   other endpoint(s).
 * - This call could be skipped if the block provides only one sync object.
 *
 * <b>Actions</b>
 *
 * - Send sync count to block.
 *   If count is zero, a SyncCount event will be sent to the
 *   other endpoint(s) immediately.
 *
 * <b>Postconditions</b>
 *
 * None.
 *
 * \param[in] block Block handle.
 * \param[in] count Number of sync objects to be sent.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Sync count sent to the block successfully.
 * - ::NvSciError_BadParameter @a block refers to an invalid instance or
 *     the @a count exceeds the maximum allowed.
 * - ::NvSciError_NotImplemented @a block is not a producer or consumer block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - ::NvSciError_NotPermitted @a block is in safety mode. Operation is not allowed.
 * - ::NvSciError_InsufficientMemory Memory allocation failed for IPC-related
 *     operations.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851517}
 */
NvSciError NvSciStreamBlockSyncObjCount(
    NvSciStreamBlock const block,
    uint32_t const count
);

/*!
 * \brief Provides block's sync object
 *
 * Used with producer and consumer to pass the sync objects they will
 * write into to the other endpoint(s).
 *
 * <b>Preconditions</b>
 *
 * - Block must have received SyncAttr event from the
 *   other endpoint(s).
 * - If the sync count is already set to zero by NvSciStreamBlockSyncObjCount,
 *   this call will fail.
 *
 * <b>Actions</b>
 *
 * - Send sync object to block.
 * - One sync object results in one SyncDesc event. When the last sync
 *   object is provided, these SyncDesc events and a SyncCount event
 *   will be sent to the other endpoint(s).
 *
 * <b>Postconditions</b>
 *
 * None.
 *
 * \param[in] block Block handle.
 * \param[in] index Index in list of sync objects
 * \param[in] syncObj Handle of sync object
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Sync objects sent to the other endpoint successfully.
 * - ::NvSciError_BadParameter @a block refers to an invalid instance or @a index
 *     exceeds the sync count or sync object for the same @a index was already set.
 * - ::NvSciError_NotImplemented @a block is not a producer or consumer block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - ::NvSciError_NotPermitted @a block is in safety mode. Operation not allowed.
 * - ::NvSciError_InvalidOperation @a block has not received SyncAttr yet,
 *     or @a block has not sent its sync requirement to the other endpoint(s),
 *     or @a block has set sync count to zero.
 * - ::NvSciError_InsufficientMemory Memory allocation failed for IPC-related
 *     operations.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851559}
 */
NvSciError NvSciStreamBlockSyncObject(
    NvSciStreamBlock const block,
    uint32_t const index,
    NvSciSyncObj const syncObj
);

/*!
 * \brief Block sets the number of elements in a packet.
 *
 * Used with the consumer to indicate the number of packet elements it desires,
 * the producer to indicate the number of packet elements it can provide, and
 * the pool to determine the combined number of packet elements.
 *
 * The element count is implicitly set to one.
 *
 * If the block needs or provides no element, the application must make this
 * call with count equal to zero.
 *
 * The number of packet elements is immutable during streaming.
 *
 * <b>Preconditions</b>
 *
 * - For producer and consumer, must have received appropriate Connect
 *   event from the other endpoint(s).
 * - For pool, must have received all PacketAttrProducer and
 *   PacketAttrConsumer events.
 * - This call could be skipped if block needs (or provides) only one element
 *   per packet.
 *
 * <b>Actions</b>
 *
 * - Send number of packet elements to block.
 *
 * <b>Postconditions</b>
 *
 * - If the count is zero, a PacketElementCount* event will be sent to the
 *   other endpoint(s) immediately.
 * - If the count is zero, for the pool, packets may now be created.
 *
 * \param[in] block The block handle.
 * \param[in] count Number of elements per packet.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Element count is sent to the block successfully.
 * - ::NvSciError_BadParameter @a block refers to an invalid instance or @a count
 *     exceeds the maximum allowed.
 * - ::NvSciError_NotImplemented @a block is not a producer, consumer, or pool
 *     block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - ::NvSciError_NotPermitted @a block is in safety mode. Operation is not allowed.
 * - ::NvSciError_InsufficientMemory Memory allocation failed for
 *     IPC-related operations.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851598}
 */
NvSciError NvSciStreamBlockPacketElementCount(
    NvSciStreamBlock const block,
    uint32_t const count
);

/*!
 * \brief Sets block packet requirements/capabilities.
 *
 * Used with the consumer to indicate what packet elements it desires,
 * the producer to indicate what packet elements it can provide, and
 * the pool to establish the combined packet element list.
 *
 * For safety-critical platforms, this function is called only once per packet
 * element at setup time. The packet attributes are immutable during
 * streaming.
 *
 * For non-safety platforms, the producer may call it again during
 * streaming to modify portions of the memory attributes, such as
 * changing the dimensions or image format of some elements.
 *
 * It may not modify the types of elements. The pool application is expected
 * to respond by updating the complete packet attribute set, deleting
 * the old packets, and creating new ones.
 *
 * <b>Preconditions</b>
 *
 * - For producer and consumer, must have received appropriate Connect
 *   event from the other endpoint(s).
 * - For pool, must have received PacketAttrProducer and
 *   PacketAttrConsumer events.
 * - If element count is already set to zero by
 *   NvSciStreamBlockPacketElementCount, this call will fail.
 *
 * <b>Actions</b>
 *
 * - Send packet attributes to block.
 * - For producer when the attributes for the last element are sent,
 *   PacketAttrProducer events (one per element) and a
 *   PacketElementCountProducer event will be sent to pool.
 * - For consumer when the attributes for the last element are sent,
 *   PacketAttrConsumer events (one per element) and a
 *   PacketElementCountConsumer event will be sent to pool.
 * - For pool, when the attributes for the last element are sent, PacketAttr
 *   events (one per element), and a PacketElementCount event will be sent
 *   to consumer and producer.
 *
 * <b>Postconditions</b>
 *
 * - For the pool, packets may now be created.
 *
 * \param[in] block Block handle.
 * \param[in] index Index of element within list
 * \param[in] type User-defined type to identify element
 * \param[in] syncMode Prefered access mode for element data
 * \param[in] bufAttrList Attributes for element buffer
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Packet attributes sent to the block successfully.
 * - ::NvSciError_BadParameter @a block refers to an invalid instance or @a index
 *     exceeds the maximum allowed or buffer attributes for the same @a index were
 *     already set.
 * - ::NvSciError_NotImplemented @a block is not a producer, consumer, or pool
 *     block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - ::NvSciError_NotPermitted @a block is in safety mode. Operation is not allowed.
 * - ::NvSciError_InsufficientMemory Memory allocation failed for
 *     IPC-related operations.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851640}
 */
NvSciError NvSciStreamBlockPacketAttr(
    NvSciStreamBlock const block,
    uint32_t const index,
    uint32_t const type,
    NvSciStreamElementMode const syncMode,
    NvSciBufAttrList const bufAttrList
);

/*!
 * \brief Adds a new packet to the pool.
 *
 * Creates a new packet for the pool to make available to the producer.
 * The description should contain the pool's cookie for the new packet.
 *
 * <b>Preconditions</b>
 *
 * - Final packet attributes must have been specified for the pool.
 * - For static pool, the number of packets already created has not reached the
 *   limit.
 *
 * <b>Actions</b>
 *
 * - A new packet handle is assigned to the packet and returned to the
 *   caller.
 * - If the element count is set to zero, a PacketCreate event will be sent to
 *   the producer and the consumer.
 *
 * <b>Postconditions</b>
 *
 * - The application can register buffers to the created packet.
 *
 * \param[in] pool Pool block handle.
 * \param[in] cookie Pool's cookie for the packet.
 * \param[out] handle New handle assigned to the packet.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Packet successfully created.
 * - ::NvSciError_BadParameter @a pool refers to an invalid
 *    instance, or @a cookie is either 0 or already used for other packet.
 * - ::NvSciError_NotImplemented @a pool is not a pool block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - ::NvSciError_NotPermitted Block is in safety mode. The operation is not allowed.
 * - ::NvSciError_InsufficientMemory Memory allocation failed.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851667}
 */
NvSciError NvSciStreamPoolPacketCreate(
    NvSciStreamBlock const pool,
    NvSciStreamCookie const cookie,
    NvSciStreamPacket *const handle
);

/*!
 * \brief
 *    Registers a buffer to the packet.
 *
 * <b>Preconditions</b>
 *
 * - The number of buffers already registered to the packet hasn't reached
 *   the number set by NvSciStreamBlockPacketElementCount.
 *
 * <b>Actions</b>
 *
 * - The buffer is added to the packet.
 *
 * <b>Postconditions</b>
 *
 * - When the last buffer is added to the packet, a PacketCreate
 *   and PacketElement event (one per packet element) will be sent to
 *   producer and consumer.
 *
 * \param[in] pool Pool block handle.
 * \param[in] handle Packet handle.
 * \param[in] index Index of element within packet.
 * \param[in] bufObj Handle of buffer for element.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Buffer successfully registered.
 * - ::NvSciError_BadParameter @a pool refers to an invalid
 *     instance, or @a index exceeds the maximum allowed.
 * - ::NvSciError_NotImplemented @a pool is not a pool block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - ::NvSciError_NotPermitted Block is in safety mode. The operation is not allowed.
 * - ::NvSciError_InvalidOperation @a pool has not sent reconciled packet attributes
 *     to the producer and consumer, or element count has been set to zero.
 * - ::NvSciError_InsufficientMemory Memory allocation failed for IPC-related operations.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851688}
 */
NvSciError NvSciStreamPoolPacketInsertBuffer(
    NvSciStreamBlock const pool,
    NvSciStreamPacket const handle,
    uint32_t const index,
    NvSciBufObj const bufObj
);

/*!
 * \brief Removes a packet from the pool.
 *
 * Schedules an existing packet to be removed from the pool. If the
 * packet is currently in the pool, it is removed right away. Otherwise
 * this is deferred until the packet returns to the pool.
 *
 * Note: This will not be supported in the safety-certified driver. It
 * is only to support non-safety cases where buffers can be added and
 * removed at any time.
 *
 * <b>Preconditions</b>
 *
 * - Specified packet must exist.
 *
 * <b>Actions</b>
 *
 * - When the specified packet is returned to the pool, the pool releases
 *   resources associated with it and sends a PacketDelete event to the
 *   producer and consumer.
 *
 * <b>Postconditions</b>
 *
 * - The packet may no longer be used for pool operations.
 *
 * \param[in] pool Pool block handle.
 * \param[in] handle Packet handle.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Packet successfully deleted.
 * - ::NvSciError_BadParameter @a pool refers to an invalid instance, or @a handle
 *     does not match any packet in the pool.
 * - ::NvSciError_NotImplemented @a pool is not a pool block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - ::NvSciError_NotPermitted Block is in safety mode. The operation is not allowed.
 * - ::NvSciError_InvalidOperation Producer or consumer fails to enqueue a
 *     packetDelete event because no packetCreate event was received.
 * - ::NvSciError_InsufficientMemory Memory allocation failed for IPC-related operations.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851706}
 */
NvSciError NvSciStreamPoolPacketDelete(
    NvSciStreamBlock const pool,
    NvSciStreamPacket const handle
);

/*!
 * \brief Accepts a packet provided by the pool.
 *
 * Upon receiving a PacketCreate event, the producer and consumer should
 * set up their own internal data structures for the packet and assign
 * a cookie which they will use to look up the data structure. Afterwards,
 * they should then call this function.
 *
 * If the client setup is successful, the error value should be set to
 * NvSciError_Success, and the cookie assigned to the packet should be provided.
 * NvSciStream will track the cookie with the packet and use it for all
 * subsequent events related to the packet.
 *
 * If the client setup is not successful, an error value should be provided to
 * indicate what went wrong. The cookie is ignored.
 *
 * <b>Preconditions</b>
 *
 * - The packet handle has been received in a previous
 *   PacketCreate event, and has not yet been accepted.
 *
 * <b>Actions</b>
 *
 * - Sends a PacketStatus event to the pool.
 *
 * <b>Postconditions</b>
 *
 * - If successful, the packet elements may now be received.
 *
 * \param[in] block Producer or consumer block handle.
 * \param[in] handle Packet handle.
 * \param[in] cookie Block's cookie for the packet handle.
 * \param[in] err Status of packet structure setup.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Packet cookie/status successfully reported.
 * - ::NvSciError_BadParameter @a block refers to an invalid instance, or @a handle
 *     does not match any packet in the block, or @a cookie is either 0 or already
 *     used for other packet.
 * - ::NvSciError_NotImplemented @a block is not a producer or consumer block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - NvSciError_NotPermitted: @a block is in safety mode. The operation is not allowed.
 * - ::NvSciError_InvalidState The packet is not in a state to be accepted (already
 *     accepted).
 * - NvSciError_InsufficientMemory: Memory allocation failed for IPC-related operations.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851733}
 */
NvSciError NvSciStreamBlockPacketAccept(
    NvSciStreamBlock const block,
    NvSciStreamPacket const handle,
    NvSciStreamCookie const cookie,
    NvSciError const err
);

/*!
 * \brief Accepts a packet element provided by the pool.
 *
 * Upon receiving a PacketElement event, the producer and consumer should
 * map the buffers into their space and report the status by calling this
 * function.
 *
 * If successfully mapped, the error value should be NvSciError_Success.
 * Otherwise it should be an error code indicating what failed.
 *
 * <b>Preconditions</b>
 *
 * - The packet handle has been received in a previous
 *   PacketCreate event, and the indexed element has been
 *   received in a previous PacketElement event.
 *
 * <b>Actions</b>
 *
 * - Sends a ElementStatus event to the pool.
 *
 * <b>Postconditions</b>
 *
 * - If successful, the element buffers may now be used to send/receive
 *   data.
 *
 * \param[in] block Producer or consumer block handle.
 * \param[in] handle Packet handle.
 * \param[in] index Index of the element within the packet.
 * \param[in] err Status of mapping operation.
 *
 * \return status code
 * - NvSciError_Success: Element successfully mapped.
 * - NvSciError_BadParameter: @a block refers to an invalid instance, or @a handle
 *     does not match any packet in the block, or @a index is not valid.
 * - NvSciError_NotImplemented: @a block is not a producer or consumer block.
 * - NvSciError_StreamNotConnected: Stream is not fully connected.
 * - NvSciError_NotPermitted: @a block is in safety mode. Operation not allowed.
 * - NvSciError_InvalidState: Element is not in a state to be accepted (already
 *     accepted).
 * - NvSciError_InsufficientMemory: Memory not enough for allocation or for
 *     ipc-related operations.
 * - NvSciError_StreamInternalError:
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 * \implements{11851760}
 */
NvSciError NvSciStreamBlockElementAccept(
    NvSciStreamBlock const block,
    NvSciStreamPacket const handle,
    uint32_t const index,
    NvSciError const err
);

/*!
 * \brief Instructs the producer to get a packet from the pool.
 *
 * If a packet is available for writing, this function will retrieve
 * it from the pool and assign it to the producer.
 *
 * The producer may hold multiple packets for rendering,
 * and is not required to present them in the order they were obtained.
 *
 * <b>Preconditions</b>
 *
 * - The PacketReady event for the next packet in the pool has
 *   been handled.
 * - On safety-certified platforms, all packets have been accepted by the
 *   producer and the consumer.
 *
 * <b>Actions</b>
 *
 * - Retrieves an available packet and returns it to the caller.
 *
 * <b>Postconditions</b>
 *
 * - The packet now resides in the list owned by the producer.
 *
 * \param[in] producer Producer block handle.
 * \param[out] cookie Cookie identifying packet.
 * \param[out] prefences Array of fences to wait for before using the packet.
 *             Must be at least large enough to hold one fence for each sync
 *             object created by the consumer. If the sync object count is
 *             zero, may be NULL.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Packet successfully retrieved.
 * - ::NvSciError_BadParameter @a producer or @a cookie refers to an invalid instance.
 * - ::NvSciError_NotImplemented @a producer is not a producer block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - ::NvSciError_NoStreamPacket No packet available.
 * - ::NvSciError_InvalidState Pool returns a packet that has not been accepted
 *     by the producer or consumer.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 *     On safety-critical systems, this function never generates this error.
 *     A system-wide mechanism shall be notified and intervene when such error
 *     occurs.
 *
 * \implements{11851790}
 */
NvSciError NvSciStreamProducerPacketGet(
    NvSciStreamBlock const producer,
    NvSciStreamCookie *const cookie,
    NvSciSyncFence *const prefences
);

/*!
 * \brief Inserts a data packet into the stream.
 *
 * Instructs the producer to insert a completed data packet into the
 * stream, where it will be sent to the consumer(s) for acquisition.
 *
 * <b>Preconditions</b>
 *
 * - The referenced packet is owned by the producer.
 *
 * <b>Actions</b>
 *
 * - The packet is sent to the queue(s) where it will be held until
 * acquired by the consumer or returned without acquisition.
 * - If one is not already pending, a PacketReady event will be
 * sent to each consumer.
 *
 * <b>Postconditions</b>
 *
 * - The packet now resides in the list owned by the queue(s).
 *
 * \param[in] producer Producer block handle.
 * \param[in] handle Packet handle.
 * \param[in] postfences A pointer to array of postfences for the data.
 *    The postfences in the array should be in the order according to the
 *    indices specified in NvSciStreamSyncDesc. That is, if the sync
 *    object is set with index == 1, its fence should be the 2nd element
 *    in the postfences array.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Packet successfully presented.
 * - ::NvSciError_BadParameter @a producer refers to an invalid instance,
       or @a handle is not valid, or @a postfences is not valid.
 * - ::NvSciError_NotImplemented @a producer is not a producer block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - ::NvSciError_InvalidState The packet has not been accepted by producer or
 *     consumer.
 * - ::NvSciError_InvalidOperation The queue cannot enqueue the packet,
       or the consumer cannot enqueue a PacketReady event.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 *     On safety-critical systems, this function never generates this error.
 *     A system-wide mechanism shall be notified and intervene when such error
 *     occurs.
 *
 * \implements{11851832}
 */
NvSciError NvSciStreamProducerPacketPresent(
    NvSciStreamBlock const producer,
    NvSciStreamPacket const handle,
    NvSciSyncFence const *const postfences
);

/*!
 * \brief Instructs the consumer to get a packet from the queue.
 *
 * If a packet is available for reading, this function will retrieve
 * it from the queue and assign it to the consumer.
 *
 * The consumer may hold multiple packets for reading, and is not required to
 * return them in the order they were obtained.
 *
 * <b>Preconditions</b>
 *
 * - The PacketReady event for the next packet in the queue has
 * been handled.
 * - On safety-certified platforms, all packets have been accepted by the
 *   producer and the consumer.
 *
 * <b>Actions</b>
 *
 * - Retrieves an available packet and returns it to the caller.
 *
 * <b>Postconditions</b>
 *
 * - The packet now resides in the list owned by the consumer.
 *
 * \param[in] consumer Consumer block handle.
 * \param[out] cookie Cookie identifying packet.
 * \param[out] prefences Array of fences to wait for before using the packet.
 *             Must be at least large enough to hold one fence for each sync
 *             object created by the producer. If the sync object count is
 *             zero, may be NULL.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Packet was successfully retrieved.
 * - ::NvSciError_BadParameter @a consumer or @a payload refers to an invalid instance.
 * - ::NvSciError_NotImplemented @a consumer is not a consumer block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - ::NvSciError_NoStreamPacket No packet is available.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 *     On safety-critical systems, this function never generates this error.
 *     A system-wide mechanism shall be notified and intervene when such error
 *     occurs.
 *
 * \implements{11851868}
 */
NvSciError NvSciStreamConsumerPacketAcquire(
    NvSciStreamBlock const consumer,
    NvSciStreamCookie *const cookie,
    NvSciSyncFence *const prefences
);

/*!
 * \brief Returns a data packet to the stream.
 *
 * Instructs the consumer to return a used data packet to the
 * stream, where it will be sent to the producer for reuse.
 *
 * <b>Preconditions</b>
 *
 * - The referenced packet is owned by the consumer.
 *
 * <b>Actions</b>
 *
 * - The packet is sent back upstream. Once released by all consumers:
 *     - It will be put in the pool where it can be obtained by the
 *       producer.
 *     - A PacketReady event will be sent to the producer.
 *
 * <b>Postconditions</b>
 *
 * - The packet will eventually reside in the list owned by the pool.
 *
 * \param[in] consumer Consumer block handle.
 * \param[in] handle Packet handle.
 * \param[in] postfences A pointer to list of postfences for the data.
 *    The postfences in the array should be in the order according to the
 *    indices specified in NvSciStreamSyncDesc.
 *
 *    That is, if the sync object is set with index == 1, its fence should be
 *    the 2nd element in the postfences array.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Packet was successfully released.
 * - ::NvSciError_BadParameter @a consumer refers to an invalid instance, or @a handle
 *     is not valid, or @a postfences is not valid.
 * - ::NvSciError_NotImplemented @a consumer is not a consumer block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - ::NvSciError_InvalidOperation Producer block cannot enqueue a PacketReady
 *     event.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 *     On safety-critical systems, this function never generates this error.
 *     A system-wide mechanism shall be notified and intervene when such error
 *     occurs.
 *
 * \implements{11851928}
 */
NvSciError NvSciStreamConsumerPacketRelease(
    NvSciStreamBlock const consumer,
    NvSciStreamPacket const handle,
    NvSciSyncFence const *const postfences
);

/*!
 * \brief Destroys a stream block.
 *
 * Schedules a stream block for destruction, disconnecting the stream if this
 * hasn't already occurred.
 *
 * The block's handle may no longer be used for any function calls,
 * and may be reassigned to a new block if more are created.
 *
 * Resources associated with the block may not be freed immediately.
 *
 * Any pending data packets downstream of the destroyed block will
 * still be available for the consumer to acquire.
 *
 * No new packets upstream of the destroyed block can be presented. Once packets
 * are released, they will be freed.
 *
 * <b>Preconditions</b>
 *
 * None.
 *
 * <b>Actions</b>
 *
 * - The block is scheduled for destruction.
 * - A DisconnectUpstream event is sent to all upstream blocks, if
 *   they haven't received one already.
 * - A DisconnectDownstream event is sent to all downstream blocks,
 *   if they haven't received one already.
 *
 * <b>Postconditions</b>
 *
 * - The block handle is no longer valid.
 *
 * \param[in] block Block handle.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Block successfully destroyed.
 * - ::NvSciError_BadParameter @a block refers to an invalid instance.
 *
 * \implements{11851961}
 */
NvSciError NvSciStreamBlockDelete(
    NvSciStreamBlock const block
);

/*!
 * \brief Queries NvSciStream attributes.
 *
 * Queries the value of one of the queryable NvSciStream attributes.
 *
 * <b>Preconditions</b>
 *
 * None.
 *
 * <b>Actions</b>
 *
 * - NvSciStream looks up the value of the attribute.
 *
 * <b>Postconditions</b>
 *
 * None changed.
 *
 * \param[in] attr The attribute to query.
 * \param[out] value The value queried.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Query is successful.
 * - ::NvSciError_BadParameter @a attr is invalid or @a value is null.
 *
 * \implements{11851979}
 */
NvSciError NvSciStreamAttributeQuery(
    NvSciStreamQueryableAttrib const attr,
    int32_t *const value
);

#ifdef __cplusplus
}
#endif
/** @} */
#endif /* NVSCISTREAM_API_H */
