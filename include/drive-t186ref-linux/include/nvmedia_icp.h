/*
 * Copyright (c) 2014-2019, NVIDIA CORPORATION.  All rights reserved.  All
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
#endif

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
#define NVMEDIA_ICP_VERSION_MINOR   16

/**
 * \hideinitializer
 * \brief  Defines an infinite timeout for NvMediaICPGetImageGroup().
 */
#define NVMEDIA_IMAGE_CAPTURE_TIMEOUT_INFINITE  (0xFFFFFFFFu)

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
    /*! Specifies YUV 4:4:4. */
    NVMEDIA_IMAGE_CAPTURE_INPUT_FORMAT_TYPE_YUV444,
    /*! Specifies RGBA. */
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
    /*! Specifies 20 bits per pixel. */
    NVMEDIA_BITS_PER_PIXEL_20
} NvMediaBitsPerPixel;

/**
 * \brief Holds the capture input format.
 */
typedef struct {
    /*! Holds capture input format type. */
    NvMediaICPInputFormatType inputFormatType;
    /*! Holds number of bits per pixel. Used when the bits per pixel doesn't
     match the input format type. */
    NvMediaBitsPerPixel bitsPerPixel;
} NvMediaICPInputFormat;

/**
 * \brief  Specifies the CSI phy mode.
 */
typedef enum {
    /* Specifies that CSI is in DPHY mode. */
    NVMEDIA_ICP_CSI_DPHY_MODE = 0,
    /* Specifies that CSI is in CPHY mode. */
    NVMEDIA_ICP_CSI_CPHY_MODE
} NvMediaICPCsiPhyMode;

/**
 * Holds image capture settings for the CSI format.
 */
typedef struct {
    /*! Holds the interface type. */
    NvMediaICPInterfaceType interfaceType;
    /*! Holds the input format. */
    NvMediaICPInputFormat inputFormat;
    /*! Holds the capture surface type */
    NvMediaSurfaceType surfaceType;
    /*! Holds the capture width. */
    uint16_t width;
    /*! Holds the capture height. */
    uint16_t height;
    /*! Holds the horizontal start position. */
    uint16_t startX;
    /*! Holds the vertical start position. */
    uint16_t startY;
    /*! Holds the embedded data lines. */
    uint16_t embeddedDataLines;
    /*! Holds the number of CSI interface lanes active. */
    uint32_t interfaceLanes;
    /*! Holds the pixel clock frequency. This parameter is required.
     *  It can be calculated from the expression `HTS * VTS * framerate`,
     *  with `framerate` in Hertz. */
    uint32_t pixelFrequency;
    /*! Holds the mipi speed in kilohertz. */
    uint32_t mipiSpeed;
    /*! Holds the MIPI THS-SETTLE time. The meaning of the value is
     SOC-specific. */
    uint16_t thsSettle;
    /*! Holds an indicator of embedded data type. @c true means
     embedded lines come with embedded data type in CSI packets. */
    NvMediaBool embeddedDataType;
    /*! Holds a Boolean; indicates whether TPG is enabled. */
    uint8_t tpgEnable;
    /*! Holds the CSI phy mode. */
    NvMediaICPCsiPhyMode phyMode;
    /*! Holds the capture frame rate in frames per second. If the actual
     frame rate is slower than this value, a frame completion timeout error
     occurs. Set the frame rate to zero to disable the frameout timer. */
    uint32_t frameRate;
    /*! Holds the CSI error mask type. Each error bit is set to 1
     to mask the corresponding error. */
    uint32_t csiErrorMask;
    /*! Holds the CSI error interrupt type. Each error bit is set to 1
     to indicate that the corresponding error type is uncorrectable. */
    uint32_t csiErrorType;
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
            /* Holds the clipping rectangle. */
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
 * \brief Defines error status codes that specify the meanings of the "masked"
 *  bits in \ref NvMediaICPSettings::csiErrorMask, and the "non-correctable"
 *  bits in \ref NvMediaICPSettings::csiErrorType.
 */
typedef enum {
    /* Specifies an LP sequence error detected on clock lane[A/B]. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DPHY_CLK_LANE_CTRL_ERR = (1 << 0),
    /* Specifies a one-bit error detected on the lane[A/B]0 sync word. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DATA_LANE_SOT_SB_ERR0 = (1 << 1),
    /* Specifies a more than one-bit error detected on the lane[A/B]0 sync
     word. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DATA_LANE_SOT_MB_ERR0 = (1 << 2),
    /* Specifies an LP sequence error detected on lane [A/B]0. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DATA_LANE_CTRL_ERR0 = (1 << 3),
    /* Specifies a lane [A/B]0 FIFO overflow. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DATA_LANE_RXFIFO_FULL_ERR0 = (1 << 4),
    /* Specifies a one-bit error detected on the lane[A/B]1 sync word. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DATA_LANE_SOT_SB_ERR1 = (1 << 5),
    /* Specifies a more than one-bit error detected on the lane[A/B]1 sync
     word. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DATA_LANE_SOT_MB_ERR1 = (1 << 6),
    /* Specifies an LP sequence error detected on lane [A/B]1. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DATA_LANE_CTRL_ERR1 = (1 << 7),
    /* Specifies a Lane [A/B]1 FIFO overflow. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DATA_LANE_RXFIFO_FULL_ERR1 = (1 << 8),
    /* Specifies a DPHY de-skew calibration not complete when sweeping the
     trimmer of data lane [A/B]0. Happens when the calibration sequence length
     is not long enough. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DPHY_DESKEW_CALIB_ERR_LANE0 = (1 << 9),
    /* Specifies a DPHY de-skew calibration not complete when sweeping the
     trimmer of data lane [A/B]1. Happens when the calibration sequence length
     is not long enough. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DPHY_DESKEW_CALIB_ERR_LANE1 = (1 << 10),
    /* Specifies a DPHY de-skew calibration not complete when sweeping the
     trimmer of clock lane [A/B]. Happens when the calibration sequence length
     is not long enough. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DPHY_DESKEW_CALIB_ERR_CTRL = (1 << 11),
    /* Specifies that an error is detected in the PHY partition [A/B]: some
     lanes detected a sync word while other lanes did not. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DPHY_LANE_ALIGN_ERR = (1 << 12),
    /* Specifies that an error is detected in the escape mode sync of lane
     [A/B]0. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DATA_LANE_ESC_MODE_SYNC_ERR0 = (1 << 13),
    /* Specifies that an error is detected in the escape mode sync of lane
     [A/B]0. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DATA_LANE_ESC_MODE_SYNC_ERR1 = (1 << 14),
    /* Specifies that an error is detected in the 2 LSBs of the lane [A/B]0
     sync word. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DATA_LANE_SOT_2LSB_ERR0 = (1 << 15),
    /* Specifies that an error is detected in the 2 LSBs of the lane [A/B]1
     sync word. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_DATA_LANE_SOT_2LSB_ERR1 = (1 << 16),
    /* For DPHY mode, CSI packet header uses ECC check. If multi-bit errors
     are detected in the packet header the error bits are not able to correct
     them, the packet is not able to do the unpack, the packet is dropped, and
     this error status is set. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_NOVC_PH_ECC_MULTI_BIT_ERR = (1 << 17),
    /* Specifies that for CPHY mode, the CSI packet has two packet headers,
     each with CRC check. If both packet headers fail the CRC check, the packet
     cannot be unpacked and is dropped, and this error status is set. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_NOVC_PH_BOTH_CRC_ERR = (1 << 18),
    /* Specifies that there is a watchdog timer inside NVCSI to detect packets
     that are incomplete after a sufficient amount of time. If the timer
     expires, this error status is set. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_VC_PPFSM_TIMEOUT = (1 << 19),
    /* Specifies that for DPHY mode, the CSI packet header includes an ECC
     check. If a one-bit error in the packet header is detected the error is
     corrected, the packet can still be unpacked, and this error status is
     set. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_VC_PH_ECC_SINGLE_BIT_ERR = (1 << 20),
    /* Specifies that the CSI packet include a payload CRC check. If the CRC
     check fails, this error status is set. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_VC_PD_CRC_ERR = (1 << 21),
    /* Specifies that the CSI packet header includes the length of the packet.
     If the packet is shorter than the expected length, this error status is
     set. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_VC_PD_WC_SHORT_ERR = (1 << 22),
    /* Specifies that for CPHY mode, the CSI packet has two packet headers,
     each with CRC check. If one packet header passes CRC check and the other
     fails, the packet can still be unpacked and this error status is set. */
    NVMEDIA_ICP_CSI_ERROR_TYPE_VC_PH_SINGLE_CRC_ERR = (1 << 23),
} NvMediaICPCSIErrorType;

/**
 * \brief Holds an image capture object per virtual group.
 */
typedef struct NvMediaICP NvMediaICP;

/**
 * \brief Holds an image capture object created by NvMediaICPCreateEx().
 */
typedef struct {
    /* Holds the number of virtual groups used. */
    uint16_t numVirtualGroups;
    struct {
        /* Holds the virtual group index. */
        uint16_t virtualGroupId;
        /* Holds a pointer to the image capture object. */
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
 *     %NVMEDIA_ICP_ERROR(CSI, FS_FAULT, 6, "A frame start occurred before previous frame end")
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
 * | CSI    | PP_ECC_DPHY         |    21 | `NVMEDIA_ICP_CSI_ERROR_PP_ECC_DPHY`        | Single bit error correction code in DPHY packet header. |
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
    NVMEDIA_ICP_ERROR_STATUS_MEMORYWRITE,
    NVMEDIA_ICP_ERROR_STATUS_FATAL
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
    /* FIFO_OVERFLOW & FIFO_LOF Events are NOT Recoverable, application is expected to stop the capture on these errors */
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
    /* Holds the CSI stream index. */
    uint32_t csiStreamId;
    /* Holds the CSI frame index generated by CSI Tx. */
    uint32_t csiFrameId;
    /* Holds the virtual channel index. */
    uint32_t virtualChannelId;
    /* Holds the error status from capture HW engine. */
    NvMediaICPErrorStatus errorStatus;
    /* Holds a detailed error code corresponding to \ref NvMediaICPErrorStatus,
     above. */
    uint32_t errorData;
    /* Holds notified errors. Notified errors are applicable only when the
     @c errorStatus member is @c NVMEDIA_ICP_ERROR_STATUS_NONE. The array holds
     all notified errors to ICP that occurred between the current successful
     frame and the previously captured frame. @c notifiedErrors must be parsed
     for each \ref NvMediaICPErrorStatus, and may contain one or more types of
     errors per @c NvMediaICPErrorStatus. */
    uint32_t notifiedErrors[NVMEDIA_ICP_ERROR_STATUS_FATAL];
} NvMediaICPErrorInfo;

/**
 * \brief Checks version compatibility for the NvMedia ICP library.
 *
 * \param[in] version A pointer to a version information structure that
 *  describes ICP versions compatible with the client.
 * \retval  NVMEDIA_STATUS_OK indicates that the ICP library is compatible
 *  with the client.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a version is invalid.
 * \retval  NVMEDIA_STATUS_INCOMPATIBLE_VERSION indicates that the ICP library
 *  is not compatible with the client.
 */
NvMediaStatus
NvMediaICPGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Creates an image capture object used to capture various formats
 *        of input into an \ref NvMediaImage.
 *
 * The supported surfaces
 * must be obtained by NvMediaSurfaceFormatGetType() with:
 * - \ref NVM_SURF_FMT_SET_ATTR_RAW(attr, RGGB/BGGR/GRBG/GBRG, INT,
 *      8/10/12/16/20, PL)
 * - \ref NVM_SURF_FMT_SET_ATTR_RAW(attr, RCCB/BCCR/CRBC/CBRC, INT, 12, PL)
 * - \ref NVM_SURF_FMT_SET_ATTR_RAW(attr, RCCC/CCCR/CRCC/CCRC, INT, 12, PL)
 * - \ref NVM_SURF_FMT_SET_ATTR_YUV(attr, YUV, 422, SEMI_PLANAR, UINT,
 *      8, PL)
 * - \ref NVM_SURF_FMT_SET_ATTR_YUV(attr, YUYV, 422, PACKED, UINT, 8, PL)
 * - \ref NVM_SURF_FMT_SET_ATTR_YUV(attr, YVYU, 422, PACKED, UINT, 8, PL)
 * - \ref NVM_SURF_FMT_SET_ATTR_YUV(attr, VYUY, 422, PACKED, UINT, 8, PL)
 * - \ref NVM_SURF_FMT_SET_ATTR_YUV(attr, UYVY, 422, PACKED, UINT, 8, PL)
 * - \ref NVM_SURF_FMT_SET_ATTR_RGBA(attr, RGBA, UINT, 8, PL)
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
 * \brief Maps a frame to the capture engine.
 * \param[in] icp       A pointer to the image capture object to be used.
 * \param[in] imageGrp  A pointer to the NvMedia image group to be mapped to
 *                       the capture engine.
 * \return  A status code; \ref NVMEDIA_STATUS_OK if the operation was
 *  successful, or \ref NVMEDIA_STATUS_ERROR otherwise.
 */
NvMediaStatus
NvMediaICPRegisterImageGroup(
    NvMediaICP   *icp,
    NvMediaImageGroup *imageGrp
);


/**
 * \brief  Adds an image group to the image capture pool. The pool size is
 *  determined by \ref NVMEDIA_MAX_ICP_FRAME_BUFFERS.
 * \param[in] icp       The image capture object to be used.
 * \param[in] imageGrp  The NvMedia image group to be added to the pool.
 * \param[in] millisecondTimeout
 *    A timeout in milliseconds. Represents time elapsed from the VI hardware
 *    start of processing the capture request to the time when a frame start
 *    occurs. It helps catch frame errors early, before the capture completion
 *    timeout.
 *    \n\n
 *    Take care when programming the frame start timeout for the first frame,
 *    for which the time required to set up the sensor to start streaming must
 *    be included.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_TIMED_OUT indicates that the operation timed out.
 * \retval  NVMEDIA_STATUS_ERROR indicates any other error.
 */
NvMediaStatus
NvMediaICPFeedImageGroup(
    NvMediaICP   *icp,
    NvMediaImageGroup *imageGrp,
    unsigned int millisecondTimeout
);

/**
 * \brief Stops the hardware engine from capturing the image.
 *
 * To restart image capture:
 * 1. Call NvMediaICPReleaseFrame().
 * 2. Call NvMediaICPResume().
 *
 * \param[in] icp A pointer to the image capture object to be used.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a icp is NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that the image capture pool is not
 *  empty yet.
 */
NvMediaStatus
NvMediaICPStop(
    NvMediaICP *icp
);

/**
 * \brief Resumes a stopped image capture operation.
 *
 * \param[in] icp  A pointer to the image capture object to be used.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a icp was NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that the capture engine was not
 *  stopped.
 */
NvMediaStatus
NvMediaICPResume(
    NvMediaICP *icp);

/**
 * \brief Gets a captured image group with frame status.
 *
 * This function blocks until a frame is available, or until a timeout occurs.
 *
 * \param[in] icp A pointer to the image capture object to be used.
 * \param[in] millisecondTimeout Timeout in milliseconds. To block
 * without a timeout, specify \ref NVMEDIA_IMAGE_CAPTURE_TIMEOUT_INFINITE.
 * \param[in,out] imageGrp An indirect pointer to the NvMedia image group
 *                          that is ready for use.
 * \retval  NVMEDIA_STATUS_OK indicates that the image is valid.
 * \retval  NVMEDIA_STATUS_NONE_PENDING indicates that @a image is NULL, there
 *  is no fed frame, or the internal pool is empty.
 * \retval  NVMEDIA_STATUS_TIMED_OUT indicates that @a image is NULL because the
 * frame is not ready for use.
 * \retval  NVMEDIA_STATUS_ERROR indicates that @a image is NULL because the
 *  hardware engine has stopped trying to capture images.
 */
NvMediaStatus
NvMediaICPGetImageGroup(
    NvMediaICP *icp,
    uint32_t millisecondTimeout,
    NvMediaImageGroup **imageGrp
);

/**
 * \brief Unmaps a frame from the capture engine.
 * \param[in] icp A pointer to the image capture object to be used.
 * \param[in] imageGrp A pointer to the NvMedia image group to be unmapped.
 * \return A status code; \ref NVMEDIA_STATUS_OK if the operation was
 *  successful, or \ref NVMEDIA_STATUS_ERROR otherwise.
 */
NvMediaStatus
NvMediaICPUnregisterImageGroup(
    NvMediaICP   *icp,
    NvMediaImageGroup *imageGrp
);

/**
 * \brief Gets an image group from the internal pool that the client
 * previously supplied with NvMediaICPFeedImageGroup().
 *
 * After the hardware engine abandons a capture, call this function
 * repeatedly until it returns \ref NVMEDIA_STATUS_ERROR.
 *
 * \param[in] icp           A pointer to the image capture object to be used.
 * \param[in,out] imageGrp  The image group that was fed from user but hardware
 *                           gives up to capture.
 * \retval  NVMEDIA_STATUS_OK indicates that the function has returned the
 *  image group that the user fed.
 * \retval  NVMEDIA_STATUS_ERROR indicates that an error occurred while
 *  returning the image group.
 * \retval  NVMEDIA_STATUS_NONE_PENDING indicates that there is nothing
 *  to return.
 */
NvMediaStatus
NvMediaICPReleaseImageGroup(
    NvMediaICP *icp,
    NvMediaImageGroup **imageGrp
);

/**
 * \brief Gets capture error information.
 *
 * This function provides capture error details such as CSI stream ID,
 * frame ID, and error status.
 * You can use this information to determine a suitable response to
 * a CSI capture error.
 *
 * \param[in]  icp          A pointer to the image capture object to be used.
 * \param[out] icpErrorInfo A pointer to the structure where information is
 *                           to be put.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a icp or
 *  @a icpErrorInfo was NULL, and thus no error information is returned.
 */
NvMediaStatus
NvMediaICPGetErrorInfo(
    NvMediaICP *icp,
    NvMediaICPErrorInfo *icpErrorInfo
);

/**
 * \brief Waits for next SoF (Start of Frame).
 *
 * The function blocks until it receives the next SoF event.
 *
 * \deprecated  Use NvMediaICPGetSOFNvSciSyncFence instead.
 * \param[in] icp  A pointer to the image capture object to be used.
 * \retval  NVMEDIA_STATUS_OK indicates that a SoF has been received.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a icp was NULL.
 * \retval  NVMEDIA_STATUS_NONE_PENDING indicates that there was no fed frame,
 *  and so the internal capture pool was empty.
 * \retval  NVMEDIA_STATUS_ERROR indicates that another error occurred while
 *  the function was waiting for SoF.
 */
NvMediaStatus
NvMediaICPWaitForSoF(
    NvMediaICP *icp
)NVM_DEPRECATED_MSG("Use NvMediaICPGetSOFNvSciSyncFence");

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
 * <b> version 4.16 </b> April 10, 2019
 * Added deprecated warning message for \ref NvMediaICPWaitForSoF.
 */
/** @} */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* _NVMEDIA_ICP_H */
