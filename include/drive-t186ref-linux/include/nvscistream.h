/*
 * Copyright (c) 2018-2019 NVIDIA Corporation. All rights reserved.
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
#ifndef NVSCISTREAM_H
#define NVSCISTREAM_H

#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif

#include "nvscierror.h"
#include "nvscibuf.h"
#include "nvscisync.h"
#include "nvsciipc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nvsci_stream Streaming APIs
 *
 * The NvSciStream library is a layer on top of NvSciBuf and NvSciSync libraries
 * that provides utilities for streaming sequences of data packets between
 * multiple application modules to support a wide variety of use cases.
 *
 * @ingroup nvsci_group_stream
 * @{
 */

/*! \brief Handle to a stream component. */
typedef uintptr_t NvSciStreamBlock;

/*! \brief NvSciStream-assigned handle for a packet. */
typedef uintptr_t NvSciStreamPacket;

/*! \brief Component-assigned cookie for a packet. */
typedef uintptr_t NvSciStreamCookie;

/*! \brief Defines NvSciStream attributes that are queryable. */
typedef enum {
    /*! \brief Maximum number of elements allowed per packet. */
    NvSciStreamQueryableAttrib_MaxElements,
    /*! \brief Maximum number of NvSciSync objects allowed. */
    NvSciStreamQueryableAttrib_MaxSyncObj
} NvSciStreamQueryableAttrib;

/*! \brief Defines packet element access modes. */
typedef enum {
    /*! \brief
     *   Written asynchronously, typically by hardware engine.
     *   Requires waiting for associated syncpoints before reading.
     */
    NvSciStreamElementMode_Asynchronous,
    /*! \brief
     *   Written synchronously, typically by CPU.
     *   Available for reading immediately.
     */
    NvSciStreamElementMode_Immediate,
} NvSciStreamElementMode;

/*! \brief Defines attributes of a packet element. */
typedef struct {
    /*! \brief
     *   Zero-based index of this packet element in a packet.
     */
    uint32_t index;
    /*! \brief
     *   Application-defined type to identify the required data.
     *   Application suites that must coordinate over streams are
     *   expected to provide a universal set of type enums.
     */
    uint32_t type;
    /*! \brief Holds the mode with which data will be written/accessed. */
    NvSciStreamElementMode mode;
    /*! \brief Holds memory buffer allocation requirements for this element.
     *   This attribute list corresponds to the packet element with the
     *   same index.
     */
    NvSciBufAttrList bufAttr;
} NvSciStreamElementAttr;

/*! \brief
 * Structure representing a packet element.
 */
typedef struct {
    /*! \brief Zero-based index of this buffer in the packet. */
    uint32_t index;
    /*! \brief
     *  Handle for the packet.
     */
    NvSciStreamPacket handle;
    /*! \brief Refers to the NvSciBuf object belonging to this element. */
    NvSciBufObj buffer;
} NvSciStreamElementDesc;

/*! \brief
 *   Defines a "frame" of data sent to the consumer or returned to the producer.
 */
typedef struct {
    /*! \brief
     *   Holds the recipient's cookie for the packet. This value cannot be @c 0U.
     */
    NvSciStreamCookie cookie;
    /*! \brief
     *   Pointer array in which prefences will be written.
     *   The recipient of the packet is expected to provide an array
     *   large enough to hold the maximum packet count defined in the
     *   sync description.
     *
     *   The recipient must wait for all fences to be reached before
     *   safely accessing the ASYNCHRONOUS buffers. The pointer
     *   is guaranteed to remain valid until the component inserts
     *   the packet back into the stream.
     *
     *   The fences in the array should be in the order according to the
     *   indices specified in NvSciStreamSyncDesc. That is, if the sync
     *   object is set with index == 1, its fence should be the 2nd element
     *   in the prefences array.
     */
    NvSciSyncFence *prefences;
} NvSciStreamPayload;

/*! \brief
 *   Defines the requirements for an endpoint to be able to read sync objects provided
 *   by the other endpoint.
 */
typedef struct {
    /*! \brief
     *   Indicates endpoint cannot process sync objects of any kind and
     *   data must be published synchronously. (This case is not common.)
     */
    bool synchronousOnly;
    /*! \brief
     *   Holds requirements for UMDs, as an NvSciSyncAttrList object.
     */
    NvSciSyncAttrList waiterSyncAttr;
} NvSciStreamSyncAttr;

/*! \brief
 *   Describes a single sync object that will be sent to or received from
 *   the other endpoint.
 */
typedef struct {
    /*! \brief
     *   Zero-based index of the sync object.
     *   The postfences sent in producer present or consumer acquire shall
     *   match the order of the sync objects.
     *
     *   That is, the i-th postfence belongs to the i-th sync object.
     */
    uint32_t index;
    /*! \brief Handle to sync object. */
    NvSciSyncObj sync;
} NvSciStreamSyncDesc;

/*! \brief Defines component event types. */
typedef enum {
    /*! \brief Indicates complete downstream connection to consumer. */
    NvSciStreamEventType_ConnectDownstream,

    /*! \brief Indicates complete upstream connection to producer. */
    NvSciStreamEventType_ConnectUpstream,

    /*! \brief Specifies sync object requirements. */
    NvSciStreamEventType_SyncAttr,

    /*! \brief Specifies the number of sync objects sent from the opposite endpoint. */
    NvSciStreamEventType_SyncCount,

    /*! \brief Specifies a sync object sent from the opposite endpoint. */
    NvSciStreamEventType_SyncDesc,

    /*! \brief Specifies number of packets elements request from producer. */
    NvSciStreamEventType_PacketElementCountProducer,

    /*! \brief Specifies number of packets elements requirement from consumer. */
    NvSciStreamEventType_PacketElementCountConsumer,

    /*! \brief Specifies the number of packet elements determined by pool. */
    NvSciStreamEventType_PacketElementCount,

    /*! \brief Specifies the packet capabilities from producer. */
    NvSciStreamEventType_PacketAttrProducer,

    /*! \brief Specifies the packet requests from consumer. */
    NvSciStreamEventType_PacketAttrConsumer,

    /*! \brief Specifies the packet final settings from pool. */
    NvSciStreamEventType_PacketAttr,

    /*! \brief Specifies the new packet object. */
    NvSciStreamEventType_PacketCreate,

    /*! \brief Specifies the new packet element. */
    NvSciStreamEventType_PacketElement,

    /*! \brief Specifies the discontinued packet object. */
    NvSciStreamEventType_PacketDelete,

    /*! \brief Specifies the status of packet. */
    NvSciStreamEventType_PacketStatusProducer,

    /*! \brief Specifies the status of packet. */
    NvSciStreamEventType_PacketStatusConsumer,

    /*! \brief Specifies the status of element. */
    NvSciStreamEventType_ElementStatusProducer,

    /*! \brief Specifies the status of element. */
    NvSciStreamEventType_ElementStatusConsumer,

    /*! \brief Specifies the packet available for reuse or acquire. */
    NvSciStreamEventType_PacketReady,

    /*! \brief Indicates a downstream component has been destroyed. */
    NvSciStreamEventType_DisconnectDownstream,

    /*! \brief Indicates an upstream component has been destroyed. */
    NvSciStreamEventType_DisconnectUpstream,
} NvSciStreamEventType;

/*! \brief Describes a component event. */
typedef struct {
    /*! \brief Holds the type of event. */
    NvSciStreamEventType type;

    /*! \brief
     *   Used with SyncAttr events.
     *
     *   Received by producer and consumer, this contains the
     *   requirements for sync objects to be usable by the other
     *   endpoint(s).
     *
     *   Components in the stream may modify the attributes specified by each
     *   endpoint before they are received by the other endpoint. In particular,
     *   multi-cast components combine the requirements of all consumers before
     *   sending to the producer, and IPC components may replace the
     *   sync requirements if they must copy the data.
     *
     *   The recipient owns the memory attribute handles in the event and is
     *   responsible for freeing them when no longer needed.
     */
    NvSciStreamSyncAttr syncAttr;

    /*! \brief
     *   Used with SyncDesc events.
     *
     *   Received by producer and consumer, this contains the description
     *   of sync objects which will be used to send fences by the other
     *   endpoint(s).
     *
     *   As with sync attributes, components in the stream may modify the
     *   description received by the other endpoint.
     *
     *   The recipient owns the memory buffer handles in the event and is
     *   responsible for freeing them when no longer needed.
     */
    NvSciStreamSyncDesc syncDesc;

    /*! \brief
     *   Used with PacketAttrConsumer, PacketAttrProducer, and PacketAttr.
     *
     *   Received by pool when producer and consumer specify their
     *   packet capabilities and requirements, respectively, and by
     *   the producer and consumer when the pool specifies the final
     *   packet setup.
     *
     *   Components in the stream may modify the attributes specified by each
     *   endpoint before they are received by the other endpoint.
     *
     *   In particular, multi-cast components will combine the packet requests of
     *   all consumer(s) before sending them to the producer, and IPC components
     *   which perform a copy may replace requirements on the type of memory
     *   used with their own.
     *
     *   The recipient owns the memory attribute handles in the event and
     *   is responsible for freeing them when no longer needed.
     */
    NvSciStreamElementAttr packetAttr;

    /*! \brief
     *   Used with PacketCreate events.
     *
     *   Received by producer and consumer when a packet is added to the
     *   pool.
     *
     *   The handle provided will be valid, and should be used
     *   whenever the component references the packet in the future.
     *
     *   The recipient owns the memory buffer handles in the event and is
     *   responsible for freeing them when no longer needed.
     */
    NvSciStreamPacket packetCreate;

    /*! \brief
     *   Used with PacketElement events.
     *   Received by producer and consumer.
     *
     *   After receiving a PacketCreate event, producer and consumer will
     *   receive [number of packet elements] PacketElement events.
     *
     *   The buffer belongs to the handle specified.
     */
    NvSciStreamElementDesc packetElement;

    /*! \brief
     *   Used with PacketDelete, PacketStatus, and PacketElement events.
     *
     *   Identifies the packet to which the event refers.
     */
    NvSciStreamCookie packetCookie;

    /*! \brief
     *   Used with PacketStatus and ElementStatus events.
     *
     *   Provides the status code.
     */
    NvSciError error;

    /*! \brief
     *   Used with events that require an index.
     *
     *   For ElementStatus, indicates the element for which the status is
     *   provided.
     */
    uint32_t index;

    /*! \brief
     *   Used with events that require a count.
     *   For SyncCount, indicates the number of available sync desc to
     *   be queried, as SyncDesc events, by consumer or producer.
     *
     *   For PacketElementCountProducer, indicates the number of elements
     *   per packet producer will provide.
     *
     *   For PacketElementCountConsumer, indicates the number of elements
     *   per packet consumer could accept.
     *
     *   For PacketElementCount, indicates the number of packet elements
     *   determined by the pool.
     */
    uint32_t count;
} NvSciStreamEvent;

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
 * Establish a connection between the two blocks.
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
 * - ::NvSciError_InvalidState: @ upstream and @a downstream are already connected.
 * - ::NvSciError_AccessDenied: @a upstream or @a downstream does not allow explicit
 *     connection via NvSciStreamBlockConnect.
 * - ::NvSciError_StreamInternalError An internal system error occurred, such as a mutex-locking failure.
 */
NvSciError NvSciStreamBlockConnect(
    NvSciStreamBlock upstream,
    NvSciStreamBlock downstream
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
 *  <b>Postconditions</b>:
 *  * The block is ready to be connected to other stream blocks.
 *  * The block can be monitored for events.
 *
 *  \param [in] pool The pool associated with the block.
 *  \param [out] producer Handle for new producer block.
 *
 *  \return ::NvSciError, the completion code of this operation.
 *  - ::NvSciError_Success The block was set up successfully.
 *  - ::NvSciError_BadParameter the output parameter @a producer is a null pointer.
 *  - ::NvSciError_InsufficientMemory The block is not created.
 *  - ::NvSciError_StreamInternalError The block cannot be initialized properly.
 *      An internal system error occurred, such as a mutex-locking failure.
 */
NvSciError NvSciStreamProducerCreate(
    NvSciStreamBlock pool,
    NvSciStreamBlock *producer
);

/*! \brief Creates a stream consumer block.
 *
 *  Create a block for the consumer end of a stream. All streams
 *  require at least one consumer block. Consumer blocks must have
 *  a queue to feed them pending frames. Consumer blocks have one
 *  input connection and no output connections.
 *
 *  Once the stream is fully connected, this block can be used to retrieve
 *  producer requirements and finalize consumer settings.
 *
 *  <b>Postconditions</b>:
 *  * The block is ready to be connected to other stream blocks.
 *  * The block can be monitored for events.
 *
 *  \param [in] queue The queue associated with the block.
 *  \param [out] consumer Handle for new consumer block.
 *
 *  \return ::NvSciError, the completion code of this operation.
 *  - ::NvSciError_Success The block was set up successfully.
 *  - ::NvSciError_BadParameter The output block @a consumer is a null pointer.
 *  - ::NvSciError_InsufficientMemory The block is not created.
 *  - ::NvSciError_StreamInternalError The block cannot be initialized properly.
 *       An internal system error occurred, such as a mutex-locking failure.
 */
NvSciError NvSciStreamConsumerCreate(
    NvSciStreamBlock queue,
    NvSciStreamBlock *consumer
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
 * Once the stream is fully connected and the producer has determined the packet
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
 */
NvSciError NvSciStreamStaticPoolCreate(
    uint32_t numPackets,
    NvSciStreamBlock *pool
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
 */
NvSciError NvSciStreamMailboxQueueCreate(
    NvSciStreamBlock *queue
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
 */
NvSciError NvSciStreamFifoQueueCreate(
    NvSciStreamBlock *queue
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
 * \param[in] syncModule NvSciSync Module that is used to import a
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
 */
NvSciError NvSciStreamIpcSrcCreate(
    NvSciIpcEndpoint ipcEndpoint,
    NvSciSyncModule syncModule,
    NvSciBufModule bufModule,
    NvSciStreamBlock *ipc
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
 * \param[in] syncModule NvSciSync Module that is used to import a
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
 */
NvSciError NvSciStreamIpcDstCreate(
    NvSciIpcEndpoint ipcEndpoint,
    NvSciSyncModule syncModule,
    NvSciBufModule bufModule,
    NvSciStreamBlock *ipc
);

/*!
 * \brief Queries for the next event from block's event queue.
 *
 * Gets the next event off of a block's event queue, optionally waiting,
 * and returns it to the caller.
 *
 * The appropriate handling of each type of event is described in the section
 * for the corresponding event structure.
 *
 * <b>Preconditions</b>
 *
 * The block to query has been created.
 *
 * <b>Actions</b>
 *
 * - If no event pending on the block, wait for the timeout period.
 * - Remove the next event (if any) from the block's event queue, fill in
 *   the event data structure, and update the event file descriptor if
 *   there are no more events pending.
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
 * - ::NvSciError_Timeout Timeout before an event is available.
 * - ::NvSciError_BadParameter @a event is null or @a block is invalid.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 *
 *     On safety-critical systems, this function never generates this error.
 *     A system-wide mechanism shall be notified and intervene when such error
 *     occurs.
 */
NvSciError NvSciStreamBlockEventQuery(
    NvSciStreamBlock block,
    int64_t timeoutUsec,
    NvSciStreamEvent *event
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
 * \param[in] attr Sync attributes.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Sync requirements sent to the other endpoint
 *     successfully.
 * - ::NvSciError_BadParameter @a block refers to an invalid instance.
 * - ::NvSciError_NotImplemented @a block is not a producer or consumer block.
 * - ::NvSciError_StreamNotConnected Blocks are not fully connected.
 * - ::NvSciError_NotPermitted Blocks are in safety mode. Operation is not allowed.
 * - ::NvSciError_InsufficientMemory Memory allocation failed for IPC-related
 *     operations.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 */
NvSciError NvSciStreamBlockSyncRequirements(
    NvSciStreamBlock block,
    const NvSciStreamSyncAttr *attr
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
 */
NvSciError NvSciStreamBlockSyncObjCount(
    NvSciStreamBlock block,
    uint32_t count
);

/*!
 * \brief Provides block's sync objects
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
 * \param[in] sync Sync description.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Sync objects sent to the other endpoint successfully.
 * - ::NvSciError_BadParameter @a block refers to an invalid instance.
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
 */
NvSciError NvSciStreamBlockSyncObject(
    NvSciStreamBlock block,
    const NvSciStreamSyncDesc *sync
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
 */
NvSciError NvSciStreamBlockPacketElementCount(
    NvSciStreamBlock block,
    uint32_t count
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
 * - For producer when the attributes for the last element is sent,
 *   PacketAttrProducer events (one per element) and a
 *   PacketElementCountProducer event will be sent to pool.
 * - For consumer when the attributes for the last element is sent,
 *   PacketAttrConsumer events (one per element) and a
 *   PacketElementCountConsumer event will be sent to pool.
 * - For pool, when the attributes for the last element is sent, PacketAttr
 *   events (one per element), and a PacketElementCount event will be sent
 *   to consumer and producer.
 *
 * <b>Postconditions</b>
 *
 * - For the pool, packets may now be created.
 *
 * \param[in] block Block handle.
 * \param[in] attr Attributes for a packet element.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Packet attributes sent to the block successfully.
 * - ::NvSciError_BadParameter @a block refers to an invalid instance or element
 *     @a count exceeds the maximum allowed.
 * - ::NvSciError_NotImplemented @a block is not a producer, consumer, or pool
 *     block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - ::NvSciError_NotPermitted @a block is in safety mode. Operation is not allowed.
 * - ::NvSciError_InsufficientMemory Memory allocation failed for
 *     IPC-related operations.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 */
NvSciError NvSciStreamBlockPacketAttr(
    NvSciStreamBlock block,
    const NvSciStreamElementAttr *attr
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
 *   the caller.
 * - If the element count is set to zero, a PacketCreate event will be sent to
 *   the producer and the consumer.
 *
 * <b>Postconditions</b>
 *
 *  The application can register buffers to the created packet.
 *
 * \param[in] pool Pool block handle.
 * \param[in] cookie Pool's cookie for the packet.
 * \param[out] handle New handle assigned to the packet.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Packet successfully created.
 * - ::NvSciError_BadParameter @a pool refers to an invalid
 *    instance, or @a cookie value can't be used.
 * - ::NvSciError_NotImplemented @a pool is not a pool block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - ::NvSciError_NotPermitted Block is in safety mode. The operation is not allowed.
 * - ::NvSciError_InsufficientMemory Memory allocation failed.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 */
NvSciError NvSciStreamPoolPacketCreate(
    NvSciStreamBlock pool,
    NvSciStreamCookie cookie,
    NvSciStreamPacket *handle
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
 *  When the last buffer is added to the packet, a PacketCreate
 *  and PacketElement event (one per packet element) will be sent to
 *  producer and consumer.
 *
 * \param[in] pool Pool block handle.
 * \param[in] desc Element description in which index, packet handle, and
 *  buffer handle are filled by application.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Buffer successfully registered.
 * - ::NvSciError_BadParameter @a pool refers to an invalid
 *    instance, or @a desc contains invalid values.
 * - ::NvSciError_NotImplemented @a pool is not a pool block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - ::NvSciError_NotPermitted Block is in safety mode. The operation is not allowed.
 * - ::NvSciError_InvalidOperation @a pool has not sent reconciled packet attributes
 *     to the producer and consumer, or element count has been set to zero and
 *     packets contain no elements.
 * - ::NvSciError_InsufficientMemory Memory allocation failed for IPC-related operations.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 */
NvSciError NvSciStreamPoolPacketInsertBuffer(
    NvSciStreamBlock pool,
    const NvSciStreamElementDesc *desc
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
 */
NvSciError NvSciStreamPoolPacketDelete(
    NvSciStreamBlock pool,
    NvSciStreamPacket handle
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
 * data.
 *
 * \param[in] block Producer or consumer block handle.
 * \param[in] handle Packet handle.
 * \param[in] cookie Block's cookie for the handle.
 * \param[in] err Status of packet structure setup.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Packet cookie/status successfully reported.
 * - ::NvSciError_BadParameter @a block refers to an invalid instance, or @a handle
 *     does not match any packet in the block, or @a cookie value is invalid.
 * - ::NvSciError_NotImplemented @a block is not a producer or consumer block.
 * - ::NvSciError_StreamNotConnected Stream is not fully connected.
 * - NvSciError_NotPermitted: @a block is in safety mode. The operation is not allowed.
 * - ::NvSciError_InvalidState The packet is not in a state to be accepted (already
 *     accepted).
 * - NvSciError_InsufficientMemory: Memory allocation failed for IPC-related operations.
 * - ::NvSciError_StreamInternalError
 *     An internal system error occurred, such as a mutex-locking failure.
 */
NvSciError NvSciStreamBlockPacketAccept(
    NvSciStreamBlock block,
    NvSciStreamPacket handle,
    NvSciStreamCookie cookie,
    NvSciError err
);

/*!
 * \brief Accepts a packet element provided by the pool.
 *
 * Upon receiving a PacketElement event, the producer and consumer should
 * map the buffers into their space. Upon success or failure, they should
 * then call this function.
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
 * - NvSciError_InvalidState: Packet is not in a state to be accepted (already
 *     accepted).
 * - NvSciError_InsufficientMemory: Memory not enough for allocation or for
 *     ipc-related operations.
 * - NvSciError_StreamInternalError:
 *     An internal system error occurred, such as a mutex-locking failure.
 */
NvSciError NvSciStreamBlockElementAccept(
    NvSciStreamBlock block,
    NvSciStreamPacket handle,
    uint32_t index,
    NvSciError err
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
 * \param[out] payload Packet cookie and prefences.
 *
 * \return ::NvSciError, the completion code of this operation.
 * - ::NvSciError_Success Packet successfully retrieved.
 * - ::NvSciError_BadParameter @a producer or @a payload refers to an invalid instance.
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
 */
NvSciError NvSciStreamProducerPacketGet(
    NvSciStreamBlock producer,
    NvSciStreamPayload *payload
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
 */
NvSciError NvSciStreamProducerPacketPresent(
    NvSciStreamBlock producer,
    NvSciStreamPacket handle,
    NvSciSyncFence *postfences
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
 * \param[out] payload Application-provided struct, packet cookie and prefences
 *    will be filled by the function.
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
 */
NvSciError NvSciStreamConsumerPacketAcquire(
    NvSciStreamBlock consumer,
    NvSciStreamPayload  *payload
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
 * - ::NvSciError_Success Packet was successfully retrieved.
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
 */
NvSciError NvSciStreamConsumerPacketRelease(
    NvSciStreamBlock consumer,
    NvSciStreamPacket handle,
    NvSciSyncFence *postfences
);

/*!
 * \brief Destroys a stream block.
 *
 * Schedules a stream block for destruction, disconnecting the stream if this
 * hadn't already occurred.
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
 */
NvSciError NvSciStreamBlockDelete(
    NvSciStreamBlock block
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
 * NvSciStream looks up the value of the attribute.
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
 */
NvSciError NvSciStreamAttributeQuery(
    NvSciStreamQueryableAttrib attr,
    int32_t *value
);

#ifdef __cplusplus
}
#endif
/** @} */
#endif /* NVSCISTREAM_H */
