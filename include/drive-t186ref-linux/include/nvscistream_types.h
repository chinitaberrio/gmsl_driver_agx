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
#ifndef NVSCISTREAM_TYPES_H
#define NVSCISTREAM_TYPES_H

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
 * @defgroup nvsci_stream_data_types Streaming data types
 *
 * The NvSciStream library is a layer on top of NvSciBuf and NvSciSync libraries
 * that provides utilities for streaming sequences of data packets between
 * multiple application modules to support a wide variety of use cases.
 *
 * @ingroup nvsci_group_stream
 * @{
 */

/*! \brief Handle to a stream component.
 *
 *  \implements{11859461}
 */
typedef uintptr_t NvSciStreamBlock;

/*! \brief NvSciStream-assigned handle for a packet.
 *
 *  \implements{11859464}
 */
typedef uintptr_t NvSciStreamPacket;

/*! \brief Component-assigned cookie for a packet.
 *
 *  \implements{11859467}
 */
typedef uintptr_t NvSciStreamCookie;

/*! \brief Constant variable denoting invalid packet
 *
 *  \implements{11859515}
 */
static const NvSciStreamPacket NvSciStreamPacket_Invalid = 0U;

/*! \brief Constant variable denoting invalid cookie
 *
 *  \implements{11859530}
 */
static const NvSciStreamCookie NvSciStreamCookie_Invalid = 0U;

/*! \brief Defines NvSciStream attributes that are queryable.
 *
 *  \implements{11859476}
 */
typedef enum {
    /*! \brief Maximum number of elements allowed per packet. */
    NvSciStreamQueryableAttrib_MaxElements          = 0x000000,
    /*! \brief Maximum number of NvSciSync objects allowed. */
    NvSciStreamQueryableAttrib_MaxSyncObj           = 0x000001,
    /*! \brief Maximum number of multicast outputs allowed. */
    NvSciStreamQueryableAttrib_MaxMulticastOutputs  = 0x000002
} NvSciStreamQueryableAttrib;

/*! \brief Defines packet element access modes.
 *
 *  \implements{11859479}
 */
typedef enum {
    /*! \brief
     *   Written asynchronously, typically by hardware engine.
     *   Requires waiting for associated syncpoints before reading.
     */
    NvSciStreamElementMode_Asynchronous     = 0x000000,
    /*! \brief
     *   Written synchronously, typically by CPU.
     *   Available for reading immediately.
     */
    NvSciStreamElementMode_Immediate        = 0x000001
} NvSciStreamElementMode;

/*! \brief Defines component event types.
 *
 *  \implements{11859482}
 */
typedef enum {

    /*! \brief
     *  Indicates block has complete connection to producer and consumer
     *    endpoints. The user may now proceed to perform other operations
     *    on the block.
     *
     *  Received by all blocks.
     *
     *  No event data fields are used.
     */
    NvSciStreamEventType_Connected                  = 0x004004,

    /*! \brief
     *  Indicates portions of the stream have disconnected such that no
     *    more useful work can be done with the block. Note that this
     *    event is not always triggered immediately when any disconnect
     *    occurs. For instance:
     *    - If a consumer still has payloads waiting in its queue when a
     *      producer is destroyed, it will not be informed of the
     *      disconnection until all payloads are acquired.
     *    - If one consumer in a multicast stream is destroyed, the
     *      producer will not be informed of the disconnection as long
     *      as other consumers are still able to receive payloads.
     *
     *  Received by all blocks.
     *
     *  No event data fields are used.
     */
    NvSciStreamEventType_Disconnected               = 0x004005,

    /*! \brief
     *  Specifies sync object requirements.
     *
     *  Received by producer and consumer blocks.
     *
     *  The following event fields will be set:
     *    syncAttrList
     *      Provides an attribute list which the recipient can combine
     *      with its own requirements to create sync objects that will
     *      be used to signal the other endpoint.
     *    synchronousOnly
     *      If set, sync objects cannot be used by the other side. The
     *      recipient should not create sync objects, and should instead
     *      perform CPU waits before posting or returning payloads.
     *
     *  The values in the fields may not exactly match those sent from the
     *    other endpoint. The stream may transform them as they pass through.
     *    In particular, multi-cast components combine the requirements of
     *    all consumers before passing them to the producer, and IPC components
     *    may replace the requirements if they need to wait for the data to
     *    be ready before performing a copy step.
     */
    NvSciStreamEventType_SyncAttr                   = 0x004010,

    /*! \brief
     *  Specifies the number of sync objects sent from the opposite endpoint.
     *
     *  Received by producer and consumer blocks.
     *
     *  The following event fields will be set:
     *    count
     *      Indicates the number of sync objects that will be provided by
     *      the other side. The recipient can expect this many SyncDesc
     *      events to follow.
     */
    NvSciStreamEventType_SyncCount                  = 0x004011,

    /*! \brief Specifies a sync object sent from the opposite endpoint.
     *
     *  Received by producer and consumer blocks.
     *
     *  The following event fields will be set:
     *    index
     *      Specifies an index within the array of sync objects.
     *    syncObj
     *      Provides a handle to a sync object which the recipient should
     *      map into the libraries which will operate on stream data.
     */
    NvSciStreamEventType_SyncDesc                   = 0x004012,

    /*! \brief Specifies number of packets elements request from producer.
     *
     *  Received by pool block.
     *
     *  The following event fields will be set:
     *   count
     *      Indicates the number of packet element types that the producer
     *      is capable of generating. The recipient can expect this many
     *      PacketAttrProducer events.
     */
    NvSciStreamEventType_PacketElementCountProducer = 0x004020,

    /*! \brief Specifies number of packets elements request from consumer.
     *
     *  Received by pool block.
     *
     *  The following event fields will be set:
     *   count
     *      Indicates the number of packet element types that the consumer
     *      wishes to receive. The recipient can expect this many
     *      PacketAttrConsumer events.
     */
    NvSciStreamEventType_PacketElementCountConsumer = 0x004021,

    /*! \brief Specifies the number of packet elements determined by pool.
     *
     *  Received by producer and consumer blocks.
     *
     *  The following event fields will be set:
     *   count
     *      Indicates the number of packet element buffers that the pool
     *      will provide for each packet. The recipient can expect this
     *      many PacketAttr events and this many PacketElement events for
     *      each packet.
     */
    NvSciStreamEventType_PacketElementCount         = 0x004022,

    /*! \brief Specifies the packet capabilities from producer.
     *
     *  Received by pool block.
     *
     *  The following event fields will be set:
     *   index
     *      Index within the list of elements provided by the producer.
     *   userData
     *      A user-defined type which applications use to identify the
     *      element and allow elements provided by the producer to be
     *      matched with those desired by the consumer. At most one
     *      element of any given type can be specified.
     *   bufAttrList
     *      Provides an attribute list which the recipient can combine
     *      with its own requirements and those of the consumer to allocate
     *      buffers which all parties can use.
     *   syncMode
     *      Indicates whether the buffer data will be available immediately
     *      when the producer provides a payload or if the user must wait
     *      for the producer's sync objects first.
     *
     *  The values in the fields may not exactly match those sent from the
     *    producer. The stream may transform them as they pass through.
     */
    NvSciStreamEventType_PacketAttrProducer         = 0x004023,

    /*! \brief Specifies the packet requests from consumer.
     *
     *  Received by pool block.
     *
     *  The following event fields will be set:
     *   index
     *      Index within the list of elements requested by the consumer.
     *   userData
     *      A user-defined type which applications use to identify the
     *      element and allow elements provided by the producer to be
     *      matched with those desired by the consumer. At most one
     *      element of any given type can be specified.
     *   bufAttrList
     *      Provides an attribute list which the recipient can combine
     *      with its own requirements and those of the producer to allocate
     *      buffers which all parties can use.
     *   syncMode
     *      Indicates whether the consumer desires the buffer data to be
     *      available immediately when the payload is acquired or if it can
     *      wait until the producer's sync objects have triggered.
     *
     *  The values in the fields may not exactly match those sent from the
     *    consumer. The stream may transform them as they pass through.
     *    In particular, multi-cast components combine the requirements of
     *    all consumers before passing them to the pool.
     */
    NvSciStreamEventType_PacketAttrConsumer         = 0x004024,

    /*! \brief Specifies the packet final settings from pool.
     *
     *  Received by producer and consumer blocks.
     *
     *  The following event fields will be set:
     *   index
     *      Index within the list of elements allocated by the pool.
     *   userData
     *      A user-defined type which applications use to identify the
     *      element and allow elements provided by the producer to be
     *      matched with those desired by the consumer. At most one
     *      element of any given type can be specified.
     *   bufAttrList
     *      Provides the combined attribute list used by the pool to
     *      allocate the element.
     *   syncMode
     *      Indicates the synchronization mode that the producer should use
     *      and the consumer should expect.
     *
     *  The values in the fields may not exactly match those sent from the
     *    pool. The stream may transform them as they pass through.
     */
    NvSciStreamEventType_PacketAttr                 = 0x004025,

    /*! \brief
     *  Indicates new packet object has been introduced by pool.
     *
     *  Received by producer and consumer blocks.
     *
     *  The following event fields will be set:
     *    packetHandle
     *      Contains the handle for the new packet.
     *      This should be used whenever the component
     *      references the packet in the future.
     */
    NvSciStreamEventType_PacketCreate               = 0x004030,

    /*! \brief
     *  Specifies new packet element.
     *
     *  Received by producer and consumer blocks.
     *
     *  The following event fields will be set:
     *    packetCookie
     *      Contains the cookie which the recipient provided to identify
     *      its data structure for the packet upon accepting it.
     *    index
     *      Index within the list of the packet's elements.
     *    bufObj
     *      Provides a handle to a buffer object which the recipient should
     *      map into the libraries which will operate on stream data.
     */
    NvSciStreamEventType_PacketElement              = 0x004031,

    /*! \brief
     *  Specifies the discontinued packet object.
     *
     *  Received by producer and consumer blocks.
     *
     *  The following event fields will be set:
     *    packetCookie
     *      Contains the cookie which the recipient provided to identify
     *      its data structure for the packet upon accepting it.
     */
    NvSciStreamEventType_PacketDelete               = 0x004032,

    /*! \brief
     *  Specifies the producer-side status of packet.
     *
     *  Received by pool block.
     *
     *  The following event fields will be set:
     *    packetCookie
     *      Contains the cookie which the pool provided to identify
     *      its data structure for the packet upon creating it.
     *    error
     *      An error code indicating whether the producer was able
     *      to add the new packet.
     */
    NvSciStreamEventType_PacketStatusProducer       = 0x004033,

    /*! \brief
     *  Specifies the consumer-side status of packet.
     *
     *  Received by pool block.
     *
     *  The following event fields will be set:
     *    packetCookie
     *      Contains the cookie which the pool provided to identify
     *      its data structure for the packet upon creating it.
     *    error
     *      An error code indicating whether the consumer was able
     *      to add the new packet.
     */
    NvSciStreamEventType_PacketStatusConsumer       = 0x004034,

    /*! \brief
     *  Specifies the producer-side status of packet element.
     *
     *  Received by pool block.
     *
     *  The following event fields will be set:
     *    packetCookie
     *      Contains the cookie which the pool provided to identify
     *      its data structure for the packet upon creating it.
     *    index
     *      Index of packet element for which status is provided.
     *    error
     *      An error code indicating whether the producer was able
     *      to map in the element buffer.
     */
    NvSciStreamEventType_ElementStatusProducer      = 0x004035,

    /*! \brief
     *  Specifies the consumer-side status of packet element.
     *
     *  Received by pool block.
     *
     *  The following event fields will be set:
     *    packetCookie
     *      Contains the cookie which the pool provided to identify
     *      its data structure for the packet upon creating it.
     *    index
     *      Index of packet element for which status is provided.
     *    error
     *      An error code indicating whether the consumer was able
     *      to map in the element buffer.
     */
    NvSciStreamEventType_ElementStatusConsumer      = 0x004036,

    /*! \brief
     *  Specifies a packet is available for reuse or acquire.
     *
     *  Received by producer and consumer block.
     *
     *  The following event fields will be set:
     *    None
     */
    NvSciStreamEventType_PacketReady                = 0x004040,

    /*! \brief
     *  Indicates a failure not directly triggered by user action.
     *
     *  Received by any block.
     *
     *  The following event fields will be set:
     *    error
     *      An error code providing information about what went wrong.
     */
    NvSciStreamEventType_Error                      = 0x0040FF

} NvSciStreamEventType;

/*! \brief Describes a component event.
 *
 *  \implements{11859491}
 */
typedef struct {
    /*! \brief Holds the type of event. */
    NvSciStreamEventType type;

    /*! \brief
     *   Used with events that specify sync object attributes:
     *     SyncAttr
     *
     *   For other events, or if not needed, will be set to NULL.
     *
     *   If set, provides an attribute list handle of which the recipient
     *   becomes the owner. It should free the handle when it is no longer
     *   required.
     */
    NvSciSyncAttrList syncAttrList;

    /*! \brief
     *   Used with events that specify buffer object attributes:
     *     PacketAttr
     *     PacketAttrProducer
     *     PacketAttrConsumer
     *
     *   For other events, will be set to NULL.
     *
     *   If set, provides an attribute list handle of which the recipient
     *   becomes the owner. It should free the handle when it is no longer
     *   required.
     */
    NvSciBufAttrList bufAttrList;

    /*! \brief
     *   Used with events that provide a sync object:
     *     SyncDesc
     *
     *   For other events, will be set to NULL.
     *
     *   If set, provides a sync object handle of which the recipient
     *   becomes the owner. It should free the handle when it is no
     *   longer required.
     */
    NvSciSyncObj syncObj;

    /*! \brief
     *   Used with events that provide a buffer object:
     *     PacketElement
     *
     *   For other events, will be set to NULL.
     *
     *   If set, provides a buffer object handle of which the recipient
     *   becomes the owner. It should free the handle when it is no
     *   longer required.
     */
    NvSciBufObj bufObj;

    /*! \brief
     *   Used with events that return a packet handle:
     *     PacketCreate
     *
     *   For other events, will be set to NvSciStreamPacket_Invalid.
     */
    NvSciStreamPacket packetHandle;

    /*! \brief
     *   Used with events that indicate a packet operation:
     *     PacketDelete
     *     PacketElement
     *     PacketStatusProducer
     *     PacketStatusConsumer
     *     ElementStatusProducer
     *     ElementStatusConsumer
     *
     *   For other events, will be set to NvSciStreamCookie_Invalid.
     *
     *   Provides the component's cookie identifying the packet.
     */
    NvSciStreamCookie packetCookie;

    /*! \brief
     *   Used with events that require a count:
     *     SyncCount
     *     PacketElementCount
     *     PacketElementCountProducer
     *     PacketElementCountConsumer
     *
     *   For other events, will be set to 0.
     *
     *   Indicates a number of items for this or subsequent events.
     */
    uint32_t count;

    /*! \brief
     *   Used with events that require an index:
     *     SyncDesc
     *     PacketAttr
     *     PacketElement
     *     ElementStatusProducer
     *     ElementStatusConsumer
     *
     *   For other events, will be set to 0.
     *
     *   Indicates position of item to which the event applies within a list.
     */
    uint32_t index;

    /*! \brief
     *   Used with events that return an error:
     *     PacketStatus
     *     ElementStatus
     *
     *   For other events, will be set to NvSciError_Success.
     *
     *   Indicates the status of an operation.
     */
    NvSciError error;

    /*! \brief
     *   Used with events that require a user-defined data field:
     *     PacketAttr
     *     PacketAttrProducer
     *     PacketAttrConsumer
     *
     *   For other events, will be set to 0.
     *
     *   A value provided by application and passed through the stream
     *     without interpretation.
     */
    uint32_t userData;

    /*! \brief
     *   Used with events that specify sync object attributes:
     *     SyncAttr
     *
     *   For other events, will be set to false.
     *
     *   Indicates endpoint cannot process sync objects of any kind and
     *   data must be published synchronously. (This case is not common.)
     */
    bool synchronousOnly;

    /*! \brief
      *  Used with events that specify a synchronization mode:
      *    PacketAttr
      *    PacketAttrProducer
      *    PacketAttrConsumer
      *
      *  For other events, defaults to NvSciStreamElementMode_Asynchronous,
      *  and can be ignored.
      *
      *  Indicates whether data requires synchronization before using.
      */
    NvSciStreamElementMode syncMode;

} NvSciStreamEvent;


/*!
 * The following data structures are no longer used by any interfaces
 *   and are deprecated. They will be removed if no applications depend
 *   on them.
 */

typedef struct {
    uint32_t index;
    uint32_t type;
    NvSciStreamElementMode mode;
    NvSciBufAttrList bufAttr;
} NvSciStreamElementAttr;

typedef struct {
    uint32_t index;
    NvSciStreamPacket handle;
    NvSciBufObj buffer;
} NvSciStreamElementDesc;

typedef struct {
    bool synchronousOnly;
    NvSciSyncAttrList waiterSyncAttr;
} NvSciStreamSyncAttr;

typedef struct {
    uint32_t index;
    NvSciSyncObj sync;
} NvSciStreamSyncDesc;

typedef struct {
    NvSciStreamCookie cookie;
    NvSciSyncFence *prefences;
} NvSciStreamPayload;

#ifdef __cplusplus
}
#endif
/** @} */
#endif /* NVSCISTREAM_TYPES_H */
