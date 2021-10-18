/*
 * Copyright (c) 2014-2020, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file nvmedia_icp.h
 * \brief <b> NVIDIA Media Interface: Image Capture Processing </b>
 *
 * This file contains the \ref image_capture_api "Image Capture Processing API".
 */

#ifndef NVMEDIA_ICP_H
#define NVMEDIA_ICP_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "nvmedia_core.h"
#include "nvmedia_surface.h"
#include "nvmedia_image.h"

/**
 * \defgroup image_capture_api Image Capture
 *
 * Captures uncompressed image data, writing
 * the results to an \ref NvMediaImage.
 *
 * A specific NvMedia implementation may support capturing multiple
 * types of uncompressed image data. However, NvMediaICP objects
 * are able to capture only a specific type of uncompressed image data.
 * This type must be specified during creation.
 *
 * @ingroup nvmedia_image_top
 * @{
 */

/** \brief Major version number. */
#define NVMEDIA_ICP_VERSION_MAJOR   4
/** \brief Minor version number. */
#define NVMEDIA_ICP_VERSION_MINOR   22

/**
 * \hideinitializer
 * \brief  Defines an infinite timeout for NvMediaICPGetImageGroup().
 */
#define NVMEDIA_IMAGE_CAPTURE_TIMEOUT_INFINITE  0xFFFFFFFFu

/** Defines the maximum number of virtual groups. */
#define NVMEDIA_ICP_MAX_VIRTUAL_GROUPS     4u

/** Defines the maximum number of virtual channels per virtual group. */
#define NVMEDIA_ICP_MAX_VIRTUAL_CHANNELS   3u

/** A macro that gets an NvMediaICP handler from an \ref NvMediaICPEx handler.
 * \param[in] icpEx     An %NvMediaICPEx handler returned by
 *                       NvMediaICPCreateEx().
 * \param[in] groupIdx  The index of the capture group instance.
 */
#define NVMEDIA_ICP_HANDLER(icpEx, groupIdx) (icpEx->icp[groupIdx].hIcp)

/** A macro that gets the \ref NvMediaICPSettings handler from an
 *  \ref NvMediaICPSettingsEx object.
 * \param[in] icpSettingsEx An \ref NvMediaICPSettingsEx object passed to
 *                           NvMediaICPCreateEx().
 * \param[in] groupIdx      The index of the capture group instance.
 * \param[in] vcIdx         The index of the capture virtual channel instance.
 */
#define NVMEDIA_ICP_SETTINGS_HANDLER(icpSettingsEx, groupIdx, vcIdx) \
        (&icpSettingsEx.virtualGroups[groupIdx].virtualChannels[vcIdx].icpSettings)

/** A macro that creates an enumerator in enum \ref NvMediaICPErrorStatus.
 * @param \_type_   The hardware module that originated the error: CSI or VI.
 * @param \_error_  A symbolic code that represents the error; used to construct
 *                   the enumerator's symbol.
 * @param \_value_  A numeric value that represents nth bit field.
 * @param \_desc_   A string that describes the error.
 */
#define NVMEDIA_ICP_ERROR(_type_, _error_, _value_, _desc_)   \
        NVMEDIA_ICP_##_type_##_ERROR_##_error_ = _value_##u,

/**
 * \hideinitializer
 * \brief  Maximum number of capture buffers.
 */
#define NVMEDIA_MAX_ICP_FRAME_BUFFERS   32u

/** Defines the minimum supported image width. */
#define NVMEDIA_ICP_MIN_IMAGE_WIDTH       640u

/** Defines the maximum supported image width. */
#define NVMEDIA_ICP_MAX_IMAGE_WIDTH       3848u

/** Defines the minimum supported image height. */
#define NVMEDIA_ICP_MIN_IMAGE_HEIGHT      480u

/** Defines the maximum supported image height. */
#define NVMEDIA_ICP_MAX_IMAGE_HEIGHT      2168u

/** Defines the minimum supported frame rate. */
#define NVMEDIA_ICP_MIN_FRAME_RATE        10U

/** Defines the maximum supported frame rate. */
#define NVMEDIA_ICP_MAX_FRAME_RATE        60U

#if (NV_IS_SAFETY == 1)
/** Defines the maximum number of embedded lines supported. */
#define NVMEDIA_ICP_MAX_EMBEDDED_LINES    28u

/** Defines the supported value of pixelFrequency */
#define NVMEDIA_ICP_PIXEL_FREQUENCY        0U

/** Defines the supported value of thsSettle */
#define NVMEDIA_ICP_THS_SETTLE_TIME        0U
#else
/** Defines the maximum number of embedded lines supported. */
#define NVMEDIA_ICP_MAX_EMBEDDED_LINES    256u

#endif /* NV_IS_SAFETY */

/**
 * \brief Specifies the image capture interface type for the CSI interface.
 */
typedef enum {
    /*! Specifies CSI port A. */
    NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_A,
    /*! Specifies CSI port B. */
    NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_B,
    /*! Specifies CSI port AB. */
    NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_AB,
    /*! Specifies CSI port C. */
    NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_C,
    /*! Specifies CSI port D. */
    NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_D,
    /*! Specifies CSI port CD. */
    NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_CD,
    /*! Specifies CSI port E. */
    NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_E,
    /*! Specifies CSI port F. */
    NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_F,
    /*! Specifies CSI port EF. */
    NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_EF,
    /*! Specifies CSI port G. */
    NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_G,
    /*! Specifies CSI port H. */
    NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_H,
    /*! Specifies CSI port GH. */
    NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_GH,
} NvMediaICPInterfaceType;

/**
 * \brief Specifies the image capture input format type.
 */
typedef enum {
    /*! Specifies YUV 4:2:2 8 bits. */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_YUV422,
    /*! Specifies YUV 4:2:2 10 bits. */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_YUV422_10,
    /*! Specifies YUV 4:4:4.
     * This input format is not supported */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_YUV444,
    /*! Specifies RGB. */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_RGB888,
    /*! Specifies RAW 6. */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_RAW6,
    /*! Specifies RAW 7. */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_RAW7,
    /*! Specifies RAW 8. */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_RAW8,
    /*! Specifies RAW 10. */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_RAW10,
    /*! Specifies RAW 12. */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_RAW12,
    /*! Specifies RAW 14. */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_RAW14,
    /*! Specifies RAW 16. */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_RAW16,
    /*! Specifies RAW 20. */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_RAW20,
    /*! Specifies User defined 1 (0x30). */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_USER_DEFINED_1,
    /*! Specifies User defined 2 (0x31). */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_USER_DEFINED_2,
    /*! Specifies User defined 3 (0x32). */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_USER_DEFINED_3,
    /*! Specifies User defined 4 (0x33). */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_USER_DEFINED_4,
    /*! Specifies User defined 5 (0x34). */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_USER_DEFINED_5,
    /*! Specifies User defined 6 (0x35). */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_USER_DEFINED_6,
    /*! Specifies User defined 7 (0x36). */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_USER_DEFINED_7,
    /*! Specifies User defined 8 (0x37). */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_USER_DEFINED_8
} NvMediaICPInputFormatType;

/**
 * \brief  Specifies bits per pixel.
 */
typedef enum {
    /*! Specifies 8 bits per pixel. */
    NVMEDIA_BITS_PER_PIXEL_8 = 0,
    /*! Specifies 10 bits per pixel. */
    NVMEDIA_BITS_PER_PIXEL_10,
    /*! Specifies 12 bits per pixel. */
    NVMEDIA_BITS_PER_PIXEL_12,
    /*! Specifies 14 bits per pixel. */
    NVMEDIA_BITS_PER_PIXEL_14,
    /*! Specifies 16 bits per pixel. */
    NVMEDIA_BITS_PER_PIXEL_16,
    /*! Specifies 20 bits per pixel. This value is not supported */
    NVMEDIA_BITS_PER_PIXEL_20
} NvMediaBitsPerPixel;

/**
 * \brief Holds the capture input format.
 */
typedef struct {
    /*! Holds capture input format type. */
    NvMediaICPInputFormatType inputFormatType;
    /*! Holds number of bits per pixel for NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_USER_DEFINED_x
     input format types.*/
    NvMediaBitsPerPixel bitsPerPixel;
} NvMediaICPInputFormat;

#if (NV_IS_SAFETY == 1)
/**
 * \brief Specifies the CSI mipi data rate.
 */
typedef enum {
    NVMEDIA_ICP_CSI_MIPISPEED_297000 = 297000,
    NVMEDIA_ICP_CSI_MIPISPEED_633000 = 633000,
    NVMEDIA_ICP_CSI_MIPISPEED_800000 = 800000,
    NVMEDIA_ICP_CSI_MIPISPEED_1700000 = 1700000
} NvMediaICPCsiMipiSpeed;
#endif /* NV_IS_SAFETY */

/**
 * \brief Specifies the CSI phy mode.
 */
typedef enum {
    /*! Specifies that CSI is in DPHY mode. */
    NVMEDIA_ICP_CSI_DPHY_MODE = 0,
    /*! Specifies that CSI is in CPHY mode. */
    NVMEDIA_ICP_CSI_CPHY_MODE
} NvMediaICPCsiPhyMode;

/**
 * Holds image capture settings for the CSI format.
 */
typedef struct {
    /*! Holds the interface type.
     *  This parameter will be ignored.
     *  Please set interfaceType in \ref NvMediaICPSettingsEx */
    NvMediaICPInterfaceType interfaceType;
    /*! Holds the input format. */
    NvMediaICPInputFormat inputFormat;
    /*! Holds the capture surface type */
    NvMediaSurfaceType surfaceType;
    /*! Holds the capture width in pixels. */
    uint16_t width;
    /*! Holds the capture height in pixels. */
    uint16_t height;
    /*! Holds the horizontal start position.
     *  This feature is not supported, this parameter will be ignored. */
    uint16_t startX;
    /*! Holds the vertical start position.
     *  This feature is not supported, this parameter will be ignored. */
    uint16_t startY;
    /*! Holds the embedded data lines. */
    uint16_t embeddedDataLines;
    /*! Holds the number of CSI interface lanes active.
     *  This parameter will be ignored.
     *  Please set interfaceLanes in \ref NvMediaICPSettingsEx */
    uint32_t interfaceLanes;
    /*! Holds the pixel clock frequency. If pixelFrequency is lower then
     *  mipiSpeed, \ref mipiSpeed is required instead of \ref pixelFrequency.
     *  It can be calculated from the expression `HTS * VTS * framerate`,
     *  with `framerate` in Hertz. */
    uint32_t pixelFrequency;
    /*! For DPHY mode, holds the mipi data rate in kbps.
     *  For CPHY mode, holds the mipi data rate in ksps.
     *  This is used to program CSI WDT(watch dog timer) for a long packet(a line),
     *  it helps to detect incomplete line packet early as much as a line cycle.
     *  if it is zero, CSI WDT won't be enabled */
    uint32_t mipiSpeed;
    /*! Holds the MIPI THS-SETTLE time. The meaning of the value is
     SOC-specific. */
    uint16_t thsSettle;
    /*! Holds an indicator of embedded data type. @c true means
     embedded lines come with embedded data type in CSI packets. */
    NvMediaBool embeddedDataType;
    /*! Holds a Boolean; indicates whether TPG is enabled. */
    uint8_t tpgEnable;
    /*! Holds the CSI phy mode.
     *  This parameter will be ignored.
     *  Please use phyMode in \ref NvMediaICPSettingsEx */
    NvMediaICPCsiPhyMode phyMode;
    /*! Holds the capture frame rate in frames per second. If the actual
     frame rate is slower than this value, a frame completion timeout error
     occurs. Set the frame rate to zero to disable the frameout timer. */
    uint32_t frameRate;
} NvMediaICPSettings;

/**
* Holds the image capture settings for the CSI format per virtual group.
*/
typedef struct {
    /*! Holds the interface type. */
    NvMediaICPInterfaceType interfaceType;
    /*! Holds the number of active CSI interface lanes.
     In case of DPHY, each interfaceLane corresponds to two data lanes.
     In case of CPHY, each interfaceLane corresponds to three data lanes. */
    uint32_t interfaceLanes;
    struct {
        struct {
            /* Holds the virtual Channel Index from CSI TX. */
            uint16_t virtualChannelIndex;
            /* Holds the image capture settings for CSI format. */
            NvMediaICPSettings icpSettings;
            /**
             * Holds the clipping rectangle.
             * This feature is not supported, this parameter will be ignored.
             */
            NvMediaRect clipRect;
            /**
             * Holds a flag for disabling PFSD (Permanent Fault Software
             * Diagnostic) functionality in the capture engine. This function
             * is enabled by default, meaning that the capture engine overwrites
             * the 64 pixels (2 pixels/bit) or 128 pixels (1 pixel/bit) from
             * the left side of the bottom line. Disable this flag if the
             * application must use the whole frame. */
            NvMediaBool disablePFSD;
        } virtualChannels[NVMEDIA_ICP_MAX_VIRTUAL_CHANNELS];
        /*! Holds the number of virtual channels per group. */
        uint16_t numVirtualChannels;
    } virtualGroups[NVMEDIA_ICP_MAX_VIRTUAL_GROUPS];
    /*! Holds the number of virtual groups. */
    uint16_t numVirtualGroups;
    /*! Holds the CSI phy mode. */
    NvMediaICPCsiPhyMode phyMode;
} NvMediaICPSettingsEx;

/**
 * \brief Holds an image capture object per virtual group.
 */
typedef struct NvMediaICP NvMediaICP;

/**
 * \brief Holds an image capture object created by NvMediaICPCreateEx().
 */
typedef struct {
    /*! Holds the number of virtual groups used. */
    uint16_t numVirtualGroups;
    /*! Holds an array of \ref NvMediaICP structs, one for each virtual group. */
    struct {
        /*! Holds the virtual group index. */
        uint16_t virtualGroupId;
        /*! Holds a pointer to the image capture object. */
        NvMediaICP *hIcp;
    } icp[NVMEDIA_ICP_MAX_VIRTUAL_GROUPS];
} NvMediaICPEx;

/**
 * \brief Defines error status codes.
 *
 * Enumerators are defined by the @ref NVMEDIA_ICP_ERROR macro. In the
 * following table, @a Module, @a Type, and @a Code represent macro parameters.
 * For example:
 * @code
 *     NVMEDIA_ICP_ERROR(CSI, FS_FAULT, 6, "A frame start occurred before previous frame end")
 * @endcode
 * <br>
 *
 * | Module | Type                | Code  | Manifest Constant                          | Description |
 * |--------|---------------------|-------|--------------------------------------------|-------------|
 * | n/a    | NONE                |     0 | `NVMEDIA_ICP_CAPTURE_ERROR_NONE`           | No error occurred, or error could not be translated. |
 * | CSI mux<br>frame errors     |||||
 * | CSI    | FE_CSI_FAULT        |     6 | `NVMEDIA_ICP_CSI_ERROR_FE_CSI_FAULT`       | NVCSI fault received at frame end. |
 * | CSI    | FS_FAULT            |     7 | `NVMEDIA_ICP_CSI_ERROR_FS_FAULT`           | Frame start occurred before previous frame end. |
 * | CSI    | FE_FRAMEID_FAULT    |     8 | `NVMEDIA_ICP_CSI_ERROR_FE_FRAMEID_FAULT`   | Frame ID for FE packet does not match that of FS packet. |
 * | CSI    | FE_FORCED_FAULT     |     9 | `NVMEDIA_ICP_CSI_ERROR_FE_FORCED_FAULT`    | Frame end was injected by the CSI hardware. |
 * | CSI    | PXL_ENABLE_FAULT    |    10 | `NVMEDIA_ICP_CSI_ERROR_PXL_ENABLE_FAULT`   | Illegal pixel encoding detected. |
 * | CSI    | PP_FSM_TIMEOUT      |    20 | `NVMEDIA_ICP_CSI_ERROR_PP_FSM_TIMEOUT`     | Pixel Parser FSM timeout. |
 * | CSI    | PH_ECC_DPHY         |    21 | `NVMEDIA_ICP_CSI_ERROR_PH_ECC_DPHY`        | Single bit error correction code in DPHY packet header. |
 * | CSI    | PAYLOAD_CRC         |    22 | `NVMEDIA_ICP_CSI_ERROR_PAYLOAD_CRC`        | Packet payload CRC check failure. |
 * | CSI    | LINE_SHORT          |    23 | `NVMEDIA_ICP_CSI_ERROR_LINE_SHORT`         | Packet payload is less than word count in packet header. |
 * | CSI    | PH_CRC_CPHY         |    24 | `NVMEDIA_ICP_CSI_ERROR_PH_CRC_CPHY`        | One of the CPHY packet headers contained a CRC error. |
 * | CSI    | EMB_CRC             |    25 | `NVMEDIA_ICP_CSI_ERROR_EMB_CRC`            | Embedded line CRC error. |
 * | CSI mux<br>stream errors    |||||
 * | CSI    | SPURIOUS_DATA       |     0 | `NVMEDIA_ICP_CSI_ERROR_SPURIOUS_DATA`      | FIFO data was found in the gap between frame start and frame end. |
 * | CSI    | FIFO_OVERFLOW       |     1 | `NVMEDIA_ICP_CSI_ERROR_FIFO_OVERFLOW`      | An overflow occurred in a stream FIFO. |
 * | CSI    | FIFO_LOF            |     2 | `NVMEDIA_ICP_CSI_ERROR_FIFO_LOF`           | A loss of frame overflow occurred. |
 * | CSI    | FIFO_BADPKT         |     3 | `NVMEDIA_ICP_CSI_ERROR_FIFO_BADPKT`        | An unknown packet type has been received on a stream. |
 * | VI channel<br>selection faults      |||||
 * | VI     | FS_TIMEOUT          |     0 | `NVMEDIA_ICP_VI_ERROR_FS_TIMEOUT`          | Frame start timeout error. |
 * | VI     | FE_TIMEOUT          |     1 | `NVMEDIA_ICP_VI_ERROR_FE_TIMEOUT`          | Frame end timeout error. |
 * | VI     | PXL_MISSING_LE      |     5 | `NVMEDIA_ICP_VI_ERROR_PXL_MISSING_LE`      | Two LS packets detected without an LE packet between them. |
 * | VI     | PXL_RUNAWAY         |     6 | `NVMEDIA_ICP_VI_ERROR_PXL_RUNAWAY`         | More lines received than expected. |
 * | VI     | PXL_SPURIOUS        |     7 | `NVMEDIA_ICP_VI_ERROR_PXL_SPURIOUS`        | Pixel data packet was received without an LS packet first. |
 * | VI     | PXL_LONG_LINE       |     8 | `NVMEDIA_ICP_VI_ERROR_PXL_LONG_LINE`       | More pixels received than expected in a line. |
 * | VI     | PXL_SHORT_LINE      |     9 | `NVMEDIA_ICP_VI_ERROR_PXL_SHORT_LINE`      | Fewer pixels received than expected in a line. |
 * | VI     | EMB_MISSING_LE      |    10 | `NVMEDIA_ICP_VI_ERROR_EMB_MISSING_LE`      | Two LS packets detected without an LE packet between them in emb data. |
 * | VI     | EMB_RUNAWAY         |    11 | `NVMEDIA_ICP_VI_ERROR_EMB_RUNAWAY`         | More lines received than expected in emb data. |
 * | VI     | EMB_SPURIOUS        |    12 | `NVMEDIA_ICP_VI_ERROR_EMB_SPURIOUS`        | Pixel data packet received without an LS packet first in emb data. |
 * | VI     | EMB_LONG_LINE       |    13 | `NVMEDIA_ICP_VI_ERROR_EMB_LONG_LINE`       | More pixels received than expected in a line in emb data. |
 * | VI     | EMB_INFRINGE        |    14 | `NVMEDIA_ICP_VI_ERROR_EMB_INFRINGE`        | Embedded data received on a frame where no embedded data was expected. |
 * | VI     | DTYPE_MISMATCH      |    15 | `NVMEDIA_ICP_VI_ERROR_DTYPE_MISMATCH`      | Pixel datatype changed in the middle of a line. |
 * | VI     | FORCED_FE           |    16 | `NVMEDIA_ICP_VI_ERROR_FORCED_FE`           | FE didn't arrive within expected frame duration, capture engine injected FE. |
 * | VI     | PXL_INCOMPLETE      |    17 | `NVMEDIA_ICP_VI_ERROR_PXL_INCOMPLETE,`     | Fewer lines of pixels in a frame were received than expected. |
 * | VI     | EMB_INCOMPLETE      |    18 | `NVMEDIA_ICP_VI_ERROR_EMB_INCOMPLETE,`     | Fewer lines of emb data in a frame were received than expected. |
 * | VI     | PFSD_FAULT          |    19 | `NVMEDIA_ICP_VI_ERROR_PFSD_FAULT,`         | Permanent Fault Software Diagnostic was detected from engine status. |
 * | VI channel<br>frame faults  |||||
 * | VI     | SHORT_FRAME         |     0 | `NVMEDIA_ICP_VI_ERROR_SHORT_FRAME`         | FE packet arrived before the normal number of pixels appeared. |
 * | VI     | SHORT_EMB           |     1 | `NVMEDIA_ICP_VI_ERROR_SHORT_EMB`           | FE of EMB Data packet arrived before the normal number of pixels appeared. |
 * | VI     | PFSD_FAULT_NOTIFIED |     2 | `NVMEDIA_ICP_VI_ERROR_PFSD_FAULT_NOTIFIED` | Permanent Fault Software Diagnostic was detected between frames. |
 * | VI     | FAULT_FE            |     3 | `NVMEDIA_ICP_VI_ERROR_FAULT_FE`            | FE was inserted due to a stream reset or early abort. |
 * | VI     | NOMATCH             |     4 | `NVMEDIA_ICP_VI_ERROR_NOMATCH`             | CSI packets were discarded due to no matched capture channels. |
 * | VI     | COLLISION           |     5 | `NVMEDIA_ICP_VI_ERROR_COLLISION`           | An FS packet matched a channel which was not finished processing a previous matching frame. |
 * | VI     | LOAD_FRAMED         |     6 | `NVMEDIA_ICP_VI_ERROR_LOAD_FRAMED`         | A LOAD command was received for a channel that was currently in a frame. |
 * | VI memory write<br>faults   |||||
 * | VI     | PACKET_OVERFLOW     |     0 | `NVMEDIA_ICP_VI_ERROR_PACKET_OVERFLOW`     | The FIFO for a surface packer overflowed. |
 * | VI     | FRAME_TRUNCATED     |     1 | `NVMEDIA_ICP_VI_ERROR_FRAME_TRUNCATED`     | A packer dropped a partial frame. |
 * | VI     | FRAME_TOSSED        |     2 | `NVMEDIA_ICP_VI_ERROR_FRAME_TOSSED`        | A packer dropped an entire frame. |
 */

/**
 * \brief Specifies top-level image capture error types.
 *
 * The error bits represented by the NvMediaICPCsiInput... enums provide
 * additional detail about individual errors. For example, the
 * \ref NvMediaICPCsiInputStreamError error bits provide additional detail
 * about the \ref NVMEDIA_ICP_ERROR_STATUS_CSI_INPUTFRAME error.
 */
typedef enum {
    NVMEDIA_ICP_ERROR_STATUS_NONE,
    NVMEDIA_ICP_ERROR_STATUS_CSI_INPUTFRAME,
    NVMEDIA_ICP_ERROR_STATUS_CSI_INPUTSTREAM,
    NVMEDIA_ICP_ERROR_STATUS_PIXEL_FAULT,
    NVMEDIA_ICP_ERROR_STATUS_FRAME_FAULT,
    NVMEDIA_ICP_ERROR_STATUS_MEMORYWRITE
} NvMediaICPErrorStatus;

typedef enum {
    /* Multiple Error Bits may be set accompanying a CsiInputFrameError */
    NVMEDIA_ICP_ERROR(CSI, FE_CSI_FAULT,          6,  "A NVCSI fault received at frame end")
    NVMEDIA_ICP_ERROR(CSI, FS_FAULT,              7,  "A frame start occurred before previous frame end")
    NVMEDIA_ICP_ERROR(CSI, FE_FORCED_FAULT,       8,  "A frame end was injected by the CSI hardware")
    NVMEDIA_ICP_ERROR(CSI, FE_FRAMEID_FAULT,      9,  "Frame ID for FE packet does not match that of FS packet")
    NVMEDIA_ICP_ERROR(CSI, PXL_ENABLE_FAULT ,     10, "An illegal pixel encoding has been detected")
    NVMEDIA_ICP_ERROR(CSI, PP_FSM_TIMEOUT,        20, "Pixel Parser FSM timeout")
    NVMEDIA_ICP_ERROR(CSI, PH_ECC_DPHY,           21, "Single bit error correction code in DPHY packet header")
    NVMEDIA_ICP_ERROR(CSI, PAYLOAD_CRC,           22, "Packet Payload CRC check fail error")
    NVMEDIA_ICP_ERROR(CSI, LINE_SHORT,            23, "Packet Payload is less than word count in packet header")
    NVMEDIA_ICP_ERROR(CSI, PH_CRC_CPHY,           24, "One of the CPHY packet headers CRC error")
    NVMEDIA_ICP_ERROR(CSI, EMB_CRC,               25, "Embedded line CRC error")
} NvMediaICPCsiInputFrameError;

typedef enum {
    /* CSI Stream Errors */
    NVMEDIA_ICP_ERROR(CSI, SPURIOUS_DATA,         0,  "FIFO data was found in the gap between frame start and frame end")
    /**
     * FIFO_OVERFLOW & FIFO_LOF Events will trigger capture engine to stop the capture channel,
     * the client has to call NvMediaICPDestoryEx, when those errors are reported */
    NVMEDIA_ICP_ERROR(CSI, FIFO_OVERFLOW,         1,  "An overflow occurred in a stream FIFO")
    NVMEDIA_ICP_ERROR(CSI, FIFO_LOF,              2,  "A loss of frame overflow occurred")
    NVMEDIA_ICP_ERROR(CSI, FIFO_BADPKT,           3,  "An unknown packet type has been received on a stream")
} NvMediaICPCsiInputStreamError;

typedef enum {
    /* VI Channel Sel faults from engine status */
    NVMEDIA_ICP_ERROR(VI, FS_TIMEOUT,             0,  "Frame Start Timeout Error")
    NVMEDIA_ICP_ERROR(VI, FE_TIMEOUT,             1,  "Frame End Timeout Error")
    NVMEDIA_ICP_ERROR(VI, PXL_MISSING_LE,         5,  "Two LS packets were detected without a LE packet between them")
    NVMEDIA_ICP_ERROR(VI, PXL_RUNAWAY,            6,  "More lines were received than expected")
    NVMEDIA_ICP_ERROR(VI, PXL_SPURIOUS,           7,  "A pixel data packet was received without a LS packet first")
    NVMEDIA_ICP_ERROR(VI, PXL_LONG_LINE,          8,  "More pixels were received than expected in a line")
    NVMEDIA_ICP_ERROR(VI, PXL_SHORT_LINE,         9,  "Fewer pixels were received than expected in a line")
    NVMEDIA_ICP_ERROR(VI, EMB_MISSING_LE,         10, "Two LS packets were detected without a LE packet between them in emb data")
    NVMEDIA_ICP_ERROR(VI, EMB_RUNAWAY,            11, "More lines were received than expected in emb data")
    NVMEDIA_ICP_ERROR(VI, EMB_SPURIOUS,           12, "A pixel data packet was received without a LS packet first in emb data")
    NVMEDIA_ICP_ERROR(VI, EMB_LONG_LINE,          13, "More pixels were received than expected in a line in emb data")
    NVMEDIA_ICP_ERROR(VI, EMB_INFRINGE,           14, "Embedded data was received on a frame where no embedded data was expected")
    NVMEDIA_ICP_ERROR(VI, DTYPE_MISMATCH,         15, "The pixel datatype changed in the middle of the line")
    NVMEDIA_ICP_ERROR(VI, FORCED_FE,              16, "FE didn't arrive within expected frame duration, capture engine injected FE")
    NVMEDIA_ICP_ERROR(VI, PXL_INCOMPLETE,         17, "Fewer lines of pixels in a frame were received than expected")
    NVMEDIA_ICP_ERROR(VI, EMB_INCOMPLETE,         18, "Fewer lines of emb data in a frame were received than expected")
    NVMEDIA_ICP_ERROR(VI, PFSD_FAULT,             19, "Permanent Fault Software Diagnostic is detected from engine status")
} NvMediaICPPixelFaultError;

typedef enum {
    /* VI Channel Frame faults */
    NVMEDIA_ICP_ERROR(VI, SHORT_FRAME,            0, "FE packet arrived before the normal number of pixels has appeared ")
    NVMEDIA_ICP_ERROR(VI, SHORT_EMB,              1, "FE of EMB Data packet arrived before the normal number of pixels has appeared ")
    NVMEDIA_ICP_ERROR(VI, PFSD_FAULT_NOTIFED,     2, "Permanent Fault Software Diagnostic is detected between frames")
    NVMEDIA_ICP_ERROR(VI, FAULT_FE,               3, "FE is inserted due to a stream reset or early abort")
    NVMEDIA_ICP_ERROR(VI, NOMATCH,                4, "CSI packets are discarded due to no matched capture channels")
    NVMEDIA_ICP_ERROR(VI, COLLISION,              5, "An FS packet matches a channel which is not done processing a previously matching frame")
    NVMEDIA_ICP_ERROR(VI, LOAD_FRAMED,            6, "A LOAD command is received for a channel while that channel is currently in a frame")
} NvMediaICPFrameFaultError;

typedef enum {
    /* Errors while outputing data to memory */
    NVMEDIA_ICP_ERROR(VI, PACKET_OVERFLOW,        0, "The FIFO for a surface packer has overflowed")
    NVMEDIA_ICP_ERROR(VI, FRAME_TRUNCATED,        1, "A packer has dropped a partial frame")
    NVMEDIA_ICP_ERROR(VI, FRAME_TOSSED,           2, "A packer has dropped an entire frame")
} NvMediaICPMemoryWriteError;

/**
 * \brief Holds the capture error information populated by
 *  NvMediaICPGetErrorInfo().
 */
typedef struct {
    /*! Holds the CSI stream index. */
    uint32_t csiStreamId;
    /*! Holds the CSI frame index generated by CSI Tx. */
    uint32_t csiFrameId;
    /*! Holds the virtual channel index. */
    uint32_t virtualChannelId;
    /*! Holds the error status from capture HW engine. */
    NvMediaICPErrorStatus errorStatus;
    /*! Holds a detailed error code corresponding to \ref NvMediaICPErrorStatus,
     above. */
    uint32_t errorData;
    /*! Holds notified errors.
     The array holds all notified errors to ICP that occurred between the current
     frame and the previously captured frame. @c notifiedErrors must be parsed
     for each \ref NvMediaICPErrorStatus, and may contain one or more types of
     errors per @c NvMediaICPErrorStatus. */
    uint32_t notifiedErrors[NVMEDIA_ICP_ERROR_STATUS_MEMORYWRITE + 1u];
} NvMediaICPErrorInfo;

/**
 * \brief Gets the version of the NvMedia ICP library.
 *
 * The client should compare the version of the NvMedia ICP library with
 * \ref NVMEDIA_ICP_VERSION_MAJOR and \ref NVMEDIA_ICP_VERSION_MINOR to
 * ensure compatibility.
 *
 * \param[out] version A pointer to a version information structure that
 *             the function will write the library version to.
 * \return \ref NvMediaStatus The completion status of the operation. Possible values are:
 * - \ref NVMEDIA_STATUS_OK indicates that the library version has been
 *        successfully written to @a version.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER indicates that @a version was NULL.
 */
NvMediaStatus
NvMediaICPGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Creates an image capture object used to capture various formats
 *        of input into an \ref NvMediaImage.
 *
 * \param[in] settings A pointer to the settings for the capture.
 * \return  The new image capture's handle if successful, or NULL otherwise.
 */
NvMediaICPEx *
NvMediaICPCreateEx(
    NvMediaICPSettingsEx *settings
);

/**
 * \brief Destroys an image capture object created by NvMediaICPCreateEx().
 * \param[in] icpEx A pointer to the image capture object to be destroyed.
 */
void
NvMediaICPDestroyEx(
    NvMediaICPEx *icpEx
);

/**
 * \brief Maps an image group to the capture engine.
 * \param[in] icp       A pointer to the image capture object to be used.
 * \param[in] imageGrp  A pointer to the NvMedia image group to be mapped to
 *                       the capture engine.
 * \return \ref NvMediaStatus The completion status of the operation. Possible values are:
 * - \ref NVMEDIA_STATUS_OK indicates the operation was successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER indicates that @a icp was NULL or pointing to
 *   non-icp object, @a imageGrp was NULL or @a imageGrp contained an incorrect number of
 *   virtual channels or @a imageGrp is currently registered.
 * - \ref NVMEDIA_STATUS_ERROR if any other error occurred.
 */
NvMediaStatus
NvMediaICPRegisterImageGroup(
    const NvMediaICP            *icp,
    const NvMediaImageGroup     *imageGrp
);

/**
 * \brief  Adds an image group to the image capture pool. The pool size is
 *  determined by \ref NVMEDIA_MAX_ICP_FRAME_BUFFERS.
 *
 * This function will do the following for each image in the image group:
 * 1. Block until all \ref NvSciSyncFence prefences for the image's virtual channel have posted.
 * 2. Submit the image to the capture engine.
 * 3. Remove all prefences for the image's virtual channel.
 *
 * \param[in] icp       The image capture object to be used.
 * \param[in] imageGrp  The NvMedia image group to be added to the pool.
 * \param[in] millisecondTimeout
 *    A timeout in milliseconds. Represents time elapsed from the VI hardware
 *    start of processing the capture request to the time when a frame start
 *    occurs. It helps catch frame errors early, before the capture completion
 *    timeout. If this value is 0, timeout is infinite.
 *    \n\n
 *    Take care when programming the frame start timeout for the first frame,
 *    for which the time required to set up the sensor to start streaming must
 *    be included.
 *    The @a imageGrp might still be used by the capture engine even if NVMEDIA_STATUS_OK is not
 *    returned. In this case, @a imageGrp must not be destroyed till \ref NvMediaICPDestroyEx
 *    is called.
 *
 * \return \ref NvMediaStatus The completion status of the operation. Possible values are:
 * - \ref NVMEDIA_STATUS_OK indicates that the operation was successful.
 * - \ref NVMEDIA_STATUS_TIMED_OUT indicates that the operation timed out.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER indicates that @ icp was NULL or pointing to non-icp object,
 *   @a imageGrp was NULL, or @a imageGrp contained an incorrect number of virtual channels.
 * - \ref NVMEDIA_STATUS_NOT_INITIALIZED indicates that @a imageGrp was not previously registered.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED indicates that the requested operation was not supported.
 * - \ref NVMEDIA_STATUS_INSUFFICIENT_BUFFERING indicates that the image capture pool is full.
 * - \ref NVMEDIA_STATUS_ERROR indicates any other error.
 */
NvMediaStatus
NvMediaICPFeedImageGroup(
    const NvMediaICP     *icp,
    NvMediaImageGroup    *imageGrp,
    uint32_t             millisecondTimeout
);

/**
 * \brief Gets a captured image group with frame status.
 *
 * This function blocks until an image group is available, or until a timeout occurs.
 *
 * \param[in] icp A pointer to the image capture object to be used.
 * \param[in] millisecondTimeout Timeout in milliseconds. To block
 * without a timeout, specify \ref NVMEDIA_IMAGE_CAPTURE_TIMEOUT_INFINITE.
 * \param[in,out] imageGrp An indirect pointer to the NvMedia image group
 *                          that is ready for use.
 * \return \ref NvMediaStatus The completion status of the operation. Possible values are:
 * - \ref NVMEDIA_STATUS_OK indicates that @a imageGrp was valid.
 * - \ref NVMEDIA_STATUS_NONE_PENDING indicates that the internal pool was empty.
 * - \ref NVMEDIA_STATUS_TIMED_OUT indicates that @a imageGrp was NULL because the
 *   frame was not ready for use.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER indicates that @ icp was NULL or pointing to non-icp object,
 *   @a imageGrp was NULL, or @a imageGrp contained an incorrect number of virtual channels.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED indicates that the requested operation was not supported.
 * - \ref NVMEDIA_STATUS_PFSD_ERROR indicates that the Permanent Fault Software Diagnostic (PFSD)
 *        pattern on the image was not correct.
 * - \ref NVMEDIA_STATUS_ERROR indicates that @a imageGrp was not valid because the
 *   capture engine has reported an error.
 */
NvMediaStatus
NvMediaICPGetImageGroup(
    const NvMediaICP    *icp,
    uint32_t            millisecondTimeout,
    NvMediaImageGroup   **imageGrp
);

/**
 * \brief Unmaps an image group from the capture engine.
 * \param[in] icp A pointer to the image capture object to be used.
 * \param[in] imageGrp A pointer to the NvMedia image group to be unmapped.
 * \return \ref NvMediaStatus The completion status of the operation. Possible values are:
 * - \ref NVMEDIA_STATUS_OK indicates the operation was successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER indicates that @ icp was NULL or pointing to non-icp object,
 *   @a imageGrp was NULL or contained an incorrect number of virtual channels or @a imageGrp is
 *   currently not registered.
 * - \ref NVMEDIA_STATUS_ERROR if any other error occurred.
 */
NvMediaStatus
NvMediaICPUnregisterImageGroup(
    const NvMediaICP          *icp,
    const NvMediaImageGroup   *imageGrp
);

/**
 * \brief Gets capture error information.
 *
 * This function provides capture error details such as CSI stream ID,
 * frame ID, and error status.
 * You can use this information to determine a suitable response to
 * a CSI capture error.
 * This function should be called after NvMediaICPGetFrameEx() to catch all
 * error notifications.
 *
 * \param[in]  icp          A pointer to the image capture object to be used.
 * \param[out] icpErrorInfo A pointer to the structure where information is
 *                           to be put.
 * \return \ref NvMediaStatus The completion status of the operation. Possible values are:
 * - \ref NVMEDIA_STATUS_OK indicates that the operation was successful.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER indicates that @a icpErrorInfo was NULL or @ icp was NULL or
 *   pointing to non-icp object
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED indicates that the requested operation was not
 *   supported.
 * - \ref NVMEDIA_STATUS_ERROR indicates that the error information could not be retrieved.
 *   @a icpErrorInfo was NULL, and thus no error information was returned.
 */
NvMediaStatus
NvMediaICPGetErrorInfo(
    const NvMediaICP        *icp,
    NvMediaICPErrorInfo     *icpErrorInfo
);

#if (NV_IS_SAFETY == 0)
/**
 * \brief Gets an image group from the internal pool that the client
 * previously supplied with NvMediaICPFeedImageGroup(). [deprecated]
 *
 * @note This function is deprecated. Please refer to the NvMedia Porting Guide on how to use
 * NvMediaICPGetImageGroup() to achieve release functionality.
 * If this function is called when there is only one image group in the internal pool,
 * then it will remove the image group from the pool, stop the capture channel, and
 * return \ref NVMEDIA_STATUS_NONE_PENDING.
 *
 * When the client needs to stop the capture channel, call this function
 * repeatedly until it returns \ref NVMEDIA_STATUS_NONE_PENDING
 *
 * \param[in] icp           A pointer to the image capture object to be used.
 * \param[in,out] imageGrp  The image group that was fed from user but hardware
 *                           gives up to capture.
 * \return \ref NvMediaStatus The completion status of the operation. Possible values are:
 * - \ref NVMEDIA_STATUS_OK indicates that the function has returned the
 *   image group that the user fed.
 * - \ref NVMEDIA_STATUS_BAD_PARAMETER indicates that @a icp was NULL.
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED indicates that the requested operation was not
 *   supported.
  * - \ref NVMEDIA_STATUS_ERROR indicates that an error occurred while
 *   returning the image group.
 * - \ref NVMEDIA_STATUS_NONE_PENDING indicates that there was nothing
 *   to return.
 */
NvMediaStatus
NvMediaICPReleaseImageGroup(
    const NvMediaICP    *icp,
    NvMediaImageGroup   **imageGrp
)NVM_DEPRECATED_MSG("Use NvMediaICPGetImageGroup");;
#endif /* NV_IS_SAFETY */

/*
 * \defgroup history_nvmedia_icp History
 * Provides change history for the NvMedia ICP API.
 *
 * \section history_nvmedia_icp Version History
 *
 * <b> Version 2.0 </b> April 4, 2016
 * - Initial release
 *
 * <b> Version 2.1 </b> May 11, 2016
 * - Added \ref NvMediaICPCheckVersion API
 *
 * <b> Version 2.2 </b> August 17, 2016
 * - Added \ref NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_UYVY image capture input
 * format type
 *
 * <b> Version 2.3 </b> August 31, 2016
 * - Added option for enabling embeddedDataType in \ref NvMediaICPSettings
 *
 * <b> Version 2.4 </b> Jan 23, 2017
 * - Added \ref NvMediaICPErrorInfo and \ref NvMediaICPGetErrorInfo
 *
 * <b> Version 2.5 </b> March 16, 2017
 * - Added \ref NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_USER ,
 * \ref NvMediaICPUserFormatType and option for enabling userFormatType in
 * \ref NvMediaICPInputFormat
 *
 * <b> Version 3.0 </b> April 19, 2017
 * Updated to use stdint.h instead of "unsigned int",
 *   "int", "unsigned short" and "char".
 * NvMediaICPCreate and NvMediaICPDestroy are now deprecated.
 * NvMediaICPGetFrame is now deprecated.
 * NvMediaICPInterfaceFormat is now deprecated, it will be always CSI.
 *
 * <b> Version 3.1 </b> May 12, 2017
 * - Added \ref NvMediaICPGetVersion API to get the version of NvMedia ICP library
 * - NvMediaICPCheckVersion is deprecated. Use NvMediaICPGetVersion() instead
 *
 * <b> Version 3.2 </b> May 19, 2017
 * Removed \ref NvMediaICPSurfaceFormatType
 * NvMediaICPSettings now takes NvMediaSurfaceType to indicate capture surface format
 * Removed NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_TPG0
 * Combined NvMediaICPUserFormatType with NvMediaICPInputFormatType
 * Removed NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_YUV420
 *   from \ref NvMediaICPInputFormatType
 * Added RAW6/RAW7/RAW8/RAW10/RAW12/RAW14 in NvMediaICPInputFormatType
 * Removed pixelOrder from \ref NvMediaICPInputFormat
 * Added tpgEnagle in \ref NvMediaICPSettings
 *
 * <b> Version 4.2 </b> June 12, 2017
 * Added virtualGroupIndex inside \ref NvMediaICPSettingsEx
 * Renamed virtualChannel to virtualGroup inside \ref NvMediaICPEx
 * Added \ref NvMediaICPFeedImageGroup and \ref NvMediaICPGetImageGroup APIs
 *
 * <b> Version 4.3 </b> October 5, 2017
 * - Added \ref NvMediaICPResume API
 *
 * <b> Version 4.4 </b> October 11, 2017
 * Changed the prototypes for \ref NvMediaICPGetImageGroup and
 * \ref NvMediaICPReleaseImageGroup to accept double pointer to imageGrp
 *
 * <b> Version 4.5 </b> November 10, 2017
 * Added YUV422_10, RAW16, RAW20 support in \ref NvMediaICPInputFormatType
 * \ref NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_YUV422_10,
 * \ref NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_RAW16,
 * \ref NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_RAW20
 *
 * <b> Version 4.6 </b> November 18, 2017
 * Added additional CSI interfaces, CSI phy mode parameter to \ref NvMediaICPSettingsEX
 *
 * <b> Version 4.7 </b> March 16, 2018
 * Added mipiSpeed in \ref NvMediaICPSettings
 *
 * <b> Version 4.8 </b> September 12, 2018
 * Added NvMediaICPWaitForSoF in \ref NvMediaICPWaitForSoF
 *
 * <b> Version 4.9 </b> March 7, 2019
 * Added detailed capture error errorData in \ref NvMediaICPErrorInfo
 * Refactor \ref NvMediaICPErrorStatus to group capture errors
 *
 * <b> Version 4.10 </b> March 10, 2019
 * Added NvMediaICPRegisterImageGroup in \ref NvMediaICPRegisterImageGroup
 * Added NvMediaICPUnregisterImageGroup in \ref NvMediaICPUnregisterImageGroup
 *
 * <b> Version 4.11 </b> March 18, 2019
 * Deprecated API's- NvMediaICPFeedFrame, NvMediaICPGetFrameEx and NvMediaICPReleaseFrame
 * Instead NvMediaICPFeedImageGroup,NvMediaICPGetImageGroup and NvMediaICPReleaseImageGroup
 * should be used
 *
 * <b> Version 4.12 </b> March 25, 2019
 * Added \ref NvMediaICPCSIErrorType Enum
 * Added csiErrorMask, csiErrorType to \ref NvMediaICPSettings
 *
 * <b> Version 4.13 </b> March 27, 2019
 * Added notifiedErrors in \ref NvMediaICPErrorInfo and disablePFSD \ref NvMediaICPSettingsEx.
 *
 * <b> Version 4.14 </b> April 2, 2019
 * Changed \ref NvMediaICP type from void to struct.
 *
 * <b> Version 4.15 </b> April 4, 2019
 * Add \ref NvMediaBitsPerPixel, NVMEDIA_MAX_ICP_FRAME_BUFFERS
 *
 * <b> Version 4.16 </b> April 10, 2019
 * Added deprecated warning message for \ref NvMediaICPWaitForSoF.
 *
 * <b> Version 4.17 </b> May 22, 2019
 * Remove deprecated NvMediaICPWaitForSoF
 *
 * <b> Version 4.18 </b> July 12, 2019
 * removed NvMediaICPCSIErrorType Enum
 * removed csiErrorMask, csiErrorType from \ref NvMediaICPSettings
 * removed NVMEDIA_ICP_ERROR_STATUS_FATAL from \ref NvMediaICPErrorStatus
 *
 * <b> Version 4.19 </b> July 18, 2019
 * Remove NvMediaICPStop and NvMediaICPResume
 *
 * <b> Version 4.20 </b> October 21, 2019
 * Added the const keyword to appropriate pointer parameters for the following:
 * - \ref NvMediaICPRegisterImageGroup
 * - \ref NvMediaICPFeedImageGroup
 * - \ref NvMediaICPGetImageGroup
 * - \ref NvMediaICPUnregisterImageGroup
 * - \ref NvMediaICPReleaseImageGroup
 * - \ref NvMediaICPGetErrorInfo
 * Added comments for the following values of \ref NvMediaStatus:
 * - \ref NVMEDIA_STATUS_NOT_SUPPORTED
 * - \ref NVMEDIA_STATUS_NOT_INITIALIZED
 *
 * <b> Version 4.21 </b> November 20, 2019
 * NvMediaICPReleaseImageGroup() is removed from safety build and deprecated in non-safety build.
 * Use NvMediaICPGetImageGroup() instead
 *
 * <b> Version 4.22 </b> January 22, 2020
 * Add NVMEDIA_ICP_MIN_FRAME_RATE and NVMEDIA_ICP_MAX_FRAME_RATE in safety build.
 * Remove the return value NVMEDIA_STATUS_NOT_INITIALIZED for some functions
 *
 */
/** @} */

#ifdef __cplusplus
};     /* extern "C" */
#endif /* __cplusplus */

#endif /* NVMEDIA_ICP_H */
