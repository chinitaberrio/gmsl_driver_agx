/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved. All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

#ifndef _NVEEPROM_SPEC_H_
#define _NVEEPROM_SPEC_H_

#ifdef __cplusplus
extern "C" {
#endif

#define NVEEPROM_MAJOR_VERSION                                   1
#define NVEEPROM_MINOR_VERSION                                   0

/* Version History of the NvEEPROM specification.
 *
 * Version 1.0: November 3rd, 2017
 * - Initial release
 */

/******************************************************************************
 *              NvEEPROM Specification Version 1.0                            *
 ******************************************************************************
 *
 * 1. Overview of Data Format
 * ============================================================================
 * The layout of data in the EEPROM is as follows:
 *
 *                   SECTION NAME          SECTION SIZE
 * Address: 0x0000   ==================================
 *                   |                                |
 *                   | Factory Blob         (variable)|
 *                   |                                |
 *                   ==================================
 *                   |                                |
 *                   | Writable Section     (variable)|
 *                   |                                |
 *                   ==================================
 *                   |                                |
 *                   | Empty                (variable)|
 *                   |                                |
 * (End of EEPROM)   ==================================
 *
 * Factory Blob:     The factory blob is a data structure containing read only
 *                   parameters provided by the Camera Module Manufacturer. The
 *                   layout of the factory blob is provided in more detail in a
 *                   document given to the Camera Module Manufacturers.
 *
 * Writable Section: The Writable section begins immediately after the factory
 *                   blob.  This contains parameters that are added after the
 *                   module is received from the manufacturer.
 *
 * Empty:            Unused EEPROM memory after the writable section.
 *
 *
 * 2. Description of the Factory Blob
 * ============================================================================
 * All data stored in the factory blob is in big endian format.  The factory
 * blob is populated by the Camera Module Manufacturer and should not be overwritten.
 * The layout of the data in the factory blob is as follows:
 *
 *                   SECTION NAME          SECTION SIZE
 *                   ==================================
 *                   | Magic Number         (4 bytes) |
 *                   ==================================
 *                   | Major Version number (1 byte)  |
 *                   ==================================
 *                   | Minor Version number (1 byte)  |
 *                   ==================================
 *                   | Patch Number         (1 byte)  |
 *                   ==================================
 *                   | Padding              (1 byte)  |
 *                   ==================================
 *                   | Header Entry 1       (4 bytes) |---+
 *                   ==================================   |
 *                   | Header Entry 2       (4 bytes) |---|--+
 *                   ==================================   |  |
 *                                 ...                    |  |
 *                   ==================================   |  |
 *                   | Header Entry N       (4 bytes) |---|--|--+
 *                   ==================================   |  |  |
 *                   | MD5 Header Entry     (4 bytes) |   |  |  |
 *                   ==================================---|--|--|--+
 *                   | Empty                (variable)|   |  |  |  |
 *                   ================================== <-+  |  |  |
 *                   |                                |      |  |  |
 *                   | Group 1              (variable)|      |  |  |
 *                   |                                |      |  |  |
 *                   ==================================      |  |  |
 *                   | Empty                (variable)|      |  |  |
 *                   ================================== <----+  |  |
 *                   |                                |         |  |
 *                   | Group 2              (variable)|         |  |
 *                   |                                |         |  |
 *                   ==================================         |  |
 *                   | Empty                (variable)|         |  |
 *                   ==================================         |  |
 *                                  ...                         |  |
 *                   ================================== <-------+  |
 *                   |                                |            |
 *                   | Group N              (variable)|            |
 *                   |                                |            |
 *                   ==================================            |
 *                   | Empty                (variable)|            |
 *                   ================================== <----------+
 *                   | MD5                  (16 bytes)|
 *                   ==================================
 *
 * Magic Number:     The byte sequence 0xca1ba7ed.  This sequence indicates that
 *                   the factory blob is present on the EEPROM device.
 *
 * Version Number:   A major version number, minor version number, and patch
 *                   number indicating the version of the factory blob format
 *                   used.
 *
 * Padding:          One byte containing 0x00 after the version number.
 *
 * Header Entry n:   Each group of parameters stored in the factory blob has
 *                   a corresponding header entry that gives its address in
 *                   the blob.  The first two bytes of a header entry are a
 *                   16 bit unsigned integer factory ID, and the second two
 *                   bytes are the address of the group in the EEPROM device
 *                   stored as a 16 bit unsigned integer.
 *
 * MD5 Header Entry: A header entry with the factory ID 0xb10b that contains
 *                   the address of the MD5 checksum at the end of the blob.
 *
 * Group N:          A group is a set of contiguously stored parameters.  Each
 *                   group is associated with a "group ID" as listed in this file.
 *                   A group with a given group ID must always contain the same
 *                   set of parameters in the same order.
 *
 * Empty:            Unused space is permitted but not required between groups.
 *
 * MD5:              The 16 byte MD5 checksum calculated on the entire factory
 *                   blob up until the start of the MD5 checksum.
 *
 *
 * 3. Description of the Writable Section
 * ============================================================================
 * All data in the writable section is in big endian format.  The writable section
 * might not be present when the camera module is first received from the Camera
 * Module Manufacturer.  The layout of data in the writable section is as follows:
 *
 *                   SECTION NAME          SECTION SIZE
 *                   ================================== -----
 *                   | Magic Number         (4 bytes) |    |
 *                   ==================================    |
 *                   | Major Version number (1 byte)  |    |
 *                   ==================================    |
 *                   | Minor Version number (1 byte)  |    | --- Header
 *                   ==================================    |
 *               +---| Head Pointer         (2 bytes) |    |
 *               |   ==================================    |
 *            +--|---| End Pointer          (2 bytes) |    |
 *            |  |   ================================== -----
 *            |  |   | Empty                (variable)|
 *            |  +-> ==================================
 *            |      |                                |
 *            |      | Node 1               (variable)|---+
 *            |      |                                |   |
 *            |      ==================================   |
 *            |      | Empty                (variable)|   |
 *            |      ================================== <-+
 *            |      |                                |
 *            |      | Node 2               (variable)|---+
 *            |      |                                |   |
 *            |      ==================================   |
 *            |      | Empty                (variable)|   |
 *            |      ==================================   |
 *            |                     ...                   |
 *            |      ================================== <-+
 *            |      |                                |
 *            |      | Node N               (variable)|
 *            |      |                                |
 *            +----> ==================================
 *
 * Magic Number: 0xca1ba7ef.  This is to let the software know if the writable
 *          section is present or not.  If the magic number does not match,
 *          the software may assume that the writable section isn't present
 *          and initialize the writable section header, potentially overwriting
 *          data that may be stored after the factory blob.
 *
 * Version Number: A major and minor version number indicating the version of
 *          the writable section format.
 *
 * Head Pointer: The EEPROM address of the beginning of the first node. If there
 *          are no nodes, this is 0x0000.
 *
 * End Pointer: The EEPROM address of the first byte after the writable section.
 *
 * Node n:  A linked list node, the internal structure of which is described below.
 *          Each list node contains exactly one writable group, and the list nodes
 *          may appear in any order.  There does not need to be a list node for
 *          every writable group, but a group cannot appear more than once.
 *
 * Empty:   Empty space is allowed between nodes and between the header and
 *          the first node, but it is not required.  Nodes may be packed
 *          with no space between them.
 *
 * 4. Internal Structure of a List Node
 * ============================================================================
 * There are several parameters stored within the EEPROM, so to reduce the
 * overhead needed to store the metadata for each parameter, relevant parameters
 * are put together into a group.  Groups are used in both the factory blob, and
 * in linked list nodes.
 *
 *                   SECTION NAME          SECTION SIZE
 *                   ==================================
 *                   | Group ID             (2 bytes) |
 *                   ==================================
 *                   | Next                 (2 bytes) |
 *                   ==================================
 *                   | Valid Vector         (variable)|
 *                   ================================== -----
 *                   | Parameter 1          (variable)|    |
 *                   ==================================    |
 *                   | Parameter 2          (variable)|    |
 *                   ==================================    |-- Group
 *                                  ...                    |
 *                   ==================================    |
 *                   | Parameter N          (variable)|    |
 *                   ================================== -----
 *                   | CRC                  (2 bytes) |
 *                   ==================================
 *
 * Group ID:    A unique 2 byte identifier associated with the group.
 *
 * Next:        The address of the beginning of the next group.  In this way,
 *              the groups form a linked list.  This cannot point to a group
 *              before the current group.  This is done to prevent the possibility
 *              of a circular linked list.  The last group should have its
 *              "Next" value set to 0x0000.
 *
 * Valid Vector: This is a bit vector that keeps track of which parameters in the
 *              group contain valid data.  The number of bytes is determined by
 *              CEILING(numParamsInGroup / 8).  Which parameter maps to which
 *              bit can be determined as follows:
 *
 *                byteNumber = (parameterNumber - 1) / 8
 *                bitNumber = (parameterNumber - 1) % 8
 *
 *              parameterNumber is the ordinal number of the parameter (1 for 1st,
 *              2 for 2nd, ...).
 *              byteNumber is the byte of the valid vector to which the bit belongs,
 *              with 0 being the first byte.
 *              bitNumber is the bit that represents the parameter, with 0 being the
 *              least significant bit and 7 being the most significant bit.
 *
 *              If a parameter contains valid data, its bit will be set, otherwise,
 *              its bit will be cleared.
 *
 * Parameter n: A parameter stored within the group.  There may not be any empty
 *              space between parameters.  In addition, the order of the
 *              parameters cannot be changed.  The parameters are defined below
 *              in order after the definition of the group they belong to.
 *
 * CRC:         The CRC16 checksum calculated on the valid vector and all of the
 *              parameters concatenated together.  The initial value of the CRC
 *              calculation is 0xFFFF and the polynomial is 0xA2EB.
 *
 *****************************************************************************/

/* All data is stored in big endian format.  If a parameter is larger than
   the size of its type, it is assumed to be an array.*/
typedef enum {
    /* Unsigned 8 bit integer */
    NVEEPROM_TYPE_UINT8,

    /* Unsigned 16 bit integer */
    NVEEPROM_TYPE_UINT16,

    /* Unsigned 32 bit integer */
    NVEEPROM_TYPE_UINT32,

    /* Unsigned 64 bit integer */
    NVEEPROM_TYPE_UINT64,

    /* Fixed point number. See LENS_SHADING_DATA for a description of how this
       is stored */
    NVEEPROM_TYPE_FIXED,

    /* A null terminated string.  The string does not need to be null terminated
       if it fills the entire size of its parameter */
    NVEEPROM_TYPE_STRING,

    /* Floating point number encoded according to IEEE 754 */
    NVEEPROM_TYPE_FLOAT,

    /* Binary data or data that does not fit into any of the other categories */
    NVEEPROM_TYPE_BINARY

} NvEEPROMParameterType;

#define NVEEPROM_NUM_GROUPS                                  10
#define NVEEPROM_NUM_PARAMS                                  40
#define NVEEPROM_NUM_READONLY_GROUPS                         5

#define NVEEPROM_GROUP_ID_OFFSET                             16
#define NVEEPROM_PARAM_ORDER_MASK                            0xFFFF

/* The two most significant bytes of a parameter ID are its group ID
   so that it is easy to determine what group a parameter belongs to */
#define NVEEPROM_PARAM_ID(group_name, ordinalNumber) \
    (((NVEEPROM_GROUP_## group_name ## _ID) << NVEEPROM_GROUP_ID_OFFSET) + \
        (ordinalNumber & NVEEPROM_PARAM_ORDER_MASK))

/******************************************************************************
 * GROUP: CAMERA_MODULE_SPECIFICATION
 *****************************************************************************/
#define NVEEPROM_GROUP_CAMERA_MODULE_SPECIFICATION_ID        0x00000001
#define NVEEPROM_GROUP_CAMERA_MODULE_SPECIFICATION_READONLY  true
#define NVEEPROM_GROUP_CAMERA_MODULE_SPECIFICATION_MANDATORY true

/* Sensor name */
#define NVEEPROM_PARAM_SENSOR_NAME_ID                        NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 1)
#define NVEEPROM_PARAM_SENSOR_NAME_TYPE                      NVEEPROM_TYPE_STRING
#define NVEEPROM_PARAM_SENSOR_NAME_OFFSET                    0
#define NVEEPROM_PARAM_SENSOR_NAME_SIZE                      0x00000010

/* Sensor revision */
#define NVEEPROM_PARAM_SENSOR_REVISION_ID                    NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 2)
#define NVEEPROM_PARAM_SENSOR_REVISION_TYPE                  NVEEPROM_TYPE_UINT16
#define NVEEPROM_PARAM_SENSOR_REVISION_OFFSET                (NVEEPROM_PARAM_SENSOR_NAME_OFFSET + NVEEPROM_PARAM_SENSOR_NAME_SIZE)
#define NVEEPROM_PARAM_SENSOR_REVISION_SIZE                  0x00000002

/* Chip id of the serializer */
#define NVEEPROM_PARAM_SERIALIZER_NAME_ID                    NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 3)
#define NVEEPROM_PARAM_SERIALIZER_NAME_TYPE                  NVEEPROM_TYPE_STRING
#define NVEEPROM_PARAM_SERIALIZER_NAME_OFFSET                (NVEEPROM_PARAM_SENSOR_REVISION_OFFSET + NVEEPROM_PARAM_SENSOR_REVISION_SIZE)
#define NVEEPROM_PARAM_SERIALIZER_NAME_SIZE                  0x00000010

/* Serializer revision */
#define NVEEPROM_PARAM_SERIALIZER_REVISION_ID                NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 4)
#define NVEEPROM_PARAM_SERIALIZER_REVISION_TYPE              NVEEPROM_TYPE_UINT16
#define NVEEPROM_PARAM_SERIALIZER_REVISION_OFFSET            (NVEEPROM_PARAM_SERIALIZER_NAME_OFFSET + NVEEPROM_PARAM_SERIALIZER_NAME_SIZE)
#define NVEEPROM_PARAM_SERIALIZER_REVISION_SIZE              0x00000002

/* Bayer order of the sensor, and type of Bayer stored as ASCII text (RGGB, RCCB, RCCC) */
#define NVEEPROM_PARAM_BAYER_TYPE_ID                         NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 5)
#define NVEEPROM_PARAM_BAYER_TYPE_TYPE                       NVEEPROM_TYPE_STRING
#define NVEEPROM_PARAM_BAYER_TYPE_OFFSET                     (NVEEPROM_PARAM_SERIALIZER_REVISION_OFFSET + NVEEPROM_PARAM_SERIALIZER_REVISION_SIZE)
#define NVEEPROM_PARAM_BAYER_TYPE_SIZE                       0x00000004

/* Scan line order 0-none, 1 side, 2 up-down, 3 both */
#define NVEEPROM_PARAM_SCANLINE_ORDER_ID                     NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 6)
#define NVEEPROM_PARAM_SCANLINE_ORDER_TYPE                   NVEEPROM_TYPE_UINT8
#define NVEEPROM_PARAM_SCANLINE_ORDER_OFFSET                 (NVEEPROM_PARAM_BAYER_TYPE_OFFSET + NVEEPROM_PARAM_BAYER_TYPE_SIZE)
#define NVEEPROM_PARAM_SCANLINE_ORDER_SIZE                   0x00000001

/* Name of the lens manufacturer */
#define NVEEPROM_PARAM_LENS_MANUFACTURER_ID                  NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 7)
#define NVEEPROM_PARAM_LENS_MANUFACTURER_TYPE                NVEEPROM_TYPE_STRING
#define NVEEPROM_PARAM_LENS_MANUFACTURER_OFFSET              (NVEEPROM_PARAM_SCANLINE_ORDER_OFFSET + NVEEPROM_PARAM_SCANLINE_ORDER_SIZE)
#define NVEEPROM_PARAM_LENS_MANUFACTURER_SIZE                0x00000010

/* Name of the lens */
#define NVEEPROM_PARAM_LENS_NAME_ID                          NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 8)
#define NVEEPROM_PARAM_LENS_NAME_TYPE                        NVEEPROM_TYPE_STRING
#define NVEEPROM_PARAM_LENS_NAME_OFFSET                      (NVEEPROM_PARAM_LENS_MANUFACTURER_OFFSET + NVEEPROM_PARAM_LENS_MANUFACTURER_SIZE)
#define NVEEPROM_PARAM_LENS_NAME_SIZE                        0x00000010

/* Field of view of the lens */
#define NVEEPROM_PARAM_LENS_FOV_ID                           NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 9)
#define NVEEPROM_PARAM_LENS_FOV_TYPE                         NVEEPROM_TYPE_UINT16
#define NVEEPROM_PARAM_LENS_FOV_OFFSET                       (NVEEPROM_PARAM_LENS_NAME_OFFSET + NVEEPROM_PARAM_LENS_NAME_SIZE)
#define NVEEPROM_PARAM_LENS_FOV_SIZE                         0x00000002

/* Effective focal length (spec value) */
#define NVEEPROM_PARAM_EFL_ID                                NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 10)
#define NVEEPROM_PARAM_EFL_TYPE                              NVEEPROM_TYPE_FLOAT
#define NVEEPROM_PARAM_EFL_OFFSET                            (NVEEPROM_PARAM_LENS_FOV_OFFSET + NVEEPROM_PARAM_LENS_FOV_SIZE)
#define NVEEPROM_PARAM_EFL_SIZE                              0x00000004

/* Image width in pixel sensor spec */
#define NVEEPROM_PARAM_IMAGE_WIDTH_ID                        NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 11)
#define NVEEPROM_PARAM_IMAGE_WIDTH_TYPE                      NVEEPROM_TYPE_UINT16
#define NVEEPROM_PARAM_IMAGE_WIDTH_OFFSET                    (NVEEPROM_PARAM_EFL_OFFSET + NVEEPROM_PARAM_EFL_SIZE)
#define NVEEPROM_PARAM_IMAGE_WIDTH_SIZE                      0x00000002

/* Image height in pixel sensor spec */
#define NVEEPROM_PARAM_IMAGE_HEIGHT_ID                       NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 12)
#define NVEEPROM_PARAM_IMAGE_HEIGHT_TYPE                     NVEEPROM_TYPE_UINT16
#define NVEEPROM_PARAM_IMAGE_HEIGHT_OFFSET                   (NVEEPROM_PARAM_IMAGE_WIDTH_OFFSET + NVEEPROM_PARAM_IMAGE_WIDTH_SIZE)
#define NVEEPROM_PARAM_IMAGE_HEIGHT_SIZE                     0x00000002

#define NVEEPROM_PARAM_IR_NAME_ID                            NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 13)
#define NVEEPROM_PARAM_IR_NAME_TYPE                          NVEEPROM_TYPE_STRING
#define NVEEPROM_PARAM_IR_NAME_OFFSET                        (NVEEPROM_PARAM_IMAGE_HEIGHT_OFFSET + NVEEPROM_PARAM_IMAGE_HEIGHT_SIZE)
#define NVEEPROM_PARAM_IR_NAME_SIZE                          0x00000010

/* Frequency in nanometers */
#define NVEEPROM_PARAM_IR_CUT_FREQUENCY_ID                   NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 14)
#define NVEEPROM_PARAM_IR_CUT_FREQUENCY_TYPE                 NVEEPROM_TYPE_UINT16
#define NVEEPROM_PARAM_IR_CUT_FREQUENCY_OFFSET               (NVEEPROM_PARAM_IR_NAME_OFFSET + NVEEPROM_PARAM_IR_NAME_SIZE)
#define NVEEPROM_PARAM_IR_CUT_FREQUENCY_SIZE                 0x00000002

/*  name of the module maker */
#define NVEEPROM_PARAM_MODULE_MAKER_ID                       NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 15)
#define NVEEPROM_PARAM_MODULE_MAKER_TYPE                     NVEEPROM_TYPE_STRING
#define NVEEPROM_PARAM_MODULE_MAKER_OFFSET                   (NVEEPROM_PARAM_IR_CUT_FREQUENCY_OFFSET + NVEEPROM_PARAM_IR_CUT_FREQUENCY_SIZE)
#define NVEEPROM_PARAM_MODULE_MAKER_SIZE                     0x00000010

/* Used to select camera config file by NvMedia */
#define NVEEPROM_PARAM_MODULE_NAME_ID                        NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 16)
#define NVEEPROM_PARAM_MODULE_NAME_TYPE                      NVEEPROM_TYPE_STRING
#define NVEEPROM_PARAM_MODULE_NAME_OFFSET                    (NVEEPROM_PARAM_MODULE_MAKER_OFFSET + NVEEPROM_PARAM_MODULE_MAKER_SIZE)
#define NVEEPROM_PARAM_MODULE_NAME_SIZE                      0x00000010

/* PCB or module serial number. This is stored as a zero extended unsigned big endian number. */
#define NVEEPROM_PARAM_MODULE_SERIAL_NUMBER_ID               NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 17)
#define NVEEPROM_PARAM_MODULE_SERIAL_NUMBER_TYPE             NVEEPROM_TYPE_BINARY
#define NVEEPROM_PARAM_MODULE_SERIAL_NUMBER_OFFSET           (NVEEPROM_PARAM_MODULE_NAME_OFFSET + NVEEPROM_PARAM_MODULE_NAME_SIZE)
#define NVEEPROM_PARAM_MODULE_SERIAL_NUMBER_SIZE             0x00000014

/* Time of assembly in POSIX epoch */
#define NVEEPROM_PARAM_ASSEMBLY_TIME_ID                      NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 18)
#define NVEEPROM_PARAM_ASSEMBLY_TIME_TYPE                    NVEEPROM_TYPE_UINT64
#define NVEEPROM_PARAM_ASSEMBLY_TIME_OFFSET                  (NVEEPROM_PARAM_MODULE_SERIAL_NUMBER_OFFSET + NVEEPROM_PARAM_MODULE_SERIAL_NUMBER_SIZE)
#define NVEEPROM_PARAM_ASSEMBLY_TIME_SIZE                    0x00000008

#define NVEEPROM_PARAM_ASSEMBLY_LINE_NUMBER_ID               NVEEPROM_PARAM_ID(CAMERA_MODULE_SPECIFICATION, 19)
#define NVEEPROM_PARAM_ASSEMBLY_LINE_NUMBER_TYPE             NVEEPROM_TYPE_UINT32
#define NVEEPROM_PARAM_ASSEMBLY_LINE_NUMBER_OFFSET           (NVEEPROM_PARAM_ASSEMBLY_TIME_OFFSET + NVEEPROM_PARAM_ASSEMBLY_TIME_SIZE)
#define NVEEPROM_PARAM_ASSEMBLY_LINE_NUMBER_SIZE             0x00000004

/******************************************************************************
 * GROUP: MODULE_MAKER_INTRINSIC
 *****************************************************************************/
#define NVEEPROM_GROUP_MODULE_MAKER_INTRINSIC_ID             0x00000002
#define NVEEPROM_GROUP_MODULE_MAKER_INTRINSIC_READONLY       true
#define NVEEPROM_GROUP_MODULE_MAKER_INTRINSIC_MANDATORY      false

/* Type of calibration and tell what is the purpose of each of 16 following numbers */
#define NVEEPROM_PARAM_MM_INTRINSIC_MODEL_ID                 NVEEPROM_PARAM_ID(MODULE_MAKER_INTRINSIC, 1)
#define NVEEPROM_PARAM_MM_INTRINSIC_MODEL_TYPE               NVEEPROM_TYPE_UINT8
#define NVEEPROM_PARAM_MM_INTRINSIC_MODEL_OFFSET             0
#define NVEEPROM_PARAM_MM_INTRINSIC_MODEL_SIZE               0x00000001

/* 16 coefficients */
#define NVEEPROM_PARAM_MM_INTRINSIC_COEFFICIENTS_ID          NVEEPROM_PARAM_ID(MODULE_MAKER_INTRINSIC, 2)
#define NVEEPROM_PARAM_MM_INTRINSIC_COEFFICIENTS_TYPE        NVEEPROM_TYPE_FLOAT
#define NVEEPROM_PARAM_MM_INTRINSIC_COEFFICIENTS_OFFSET      (NVEEPROM_PARAM_MM_INTRINSIC_MODEL_OFFSET + NVEEPROM_PARAM_MM_INTRINSIC_MODEL_SIZE)
#define NVEEPROM_PARAM_MM_INTRINSIC_COEFFICIENTS_SIZE        0x00000040

/* 1 for measured, 0 for spec */
#define NVEEPROM_PARAM_MM_INTRINSIC_MEASURED_FLAG_ID         NVEEPROM_PARAM_ID(MODULE_MAKER_INTRINSIC, 3)
#define NVEEPROM_PARAM_MM_INTRINSIC_MEASURED_FLAG_TYPE       NVEEPROM_TYPE_UINT8
#define NVEEPROM_PARAM_MM_INTRINSIC_MEASURED_FLAG_OFFSET     (NVEEPROM_PARAM_MM_INTRINSIC_COEFFICIENTS_OFFSET + NVEEPROM_PARAM_MM_INTRINSIC_COEFFICIENTS_SIZE)
#define NVEEPROM_PARAM_MM_INTRINSIC_MEASURED_FLAG_SIZE       0x00000001

/******************************************************************************
 * GROUP: CHANNEL_COLOR_RESPONSES
 *****************************************************************************/
#define NVEEPROM_GROUP_CHANNEL_COLOR_RESPONSES_ID            0x00000003
#define NVEEPROM_GROUP_CHANNEL_COLOR_RESPONSES_READONLY      true
#define NVEEPROM_GROUP_CHANNEL_COLOR_RESPONSES_MANDATORY     false

/* Light used for measurement */
#define NVEEPROM_PARAM_CHANNEL_LIGHT_ID_ID                   NVEEPROM_PARAM_ID(CHANNEL_COLOR_RESPONSES, 1)
#define NVEEPROM_PARAM_CHANNEL_LIGHT_ID_TYPE                 NVEEPROM_TYPE_UINT16
#define NVEEPROM_PARAM_CHANNEL_LIGHT_ID_OFFSET               0
#define NVEEPROM_PARAM_CHANNEL_LIGHT_ID_SIZE                 0x00000002

/* Light level measurement of channels in order of R, GR, GB, B */
#define NVEEPROM_PARAM_CHANNEL_MEASUREMENT_ID                NVEEPROM_PARAM_ID(CHANNEL_COLOR_RESPONSES, 2)
#define NVEEPROM_PARAM_CHANNEL_MEASUREMENT_TYPE              NVEEPROM_TYPE_UINT16
#define NVEEPROM_PARAM_CHANNEL_MEASUREMENT_OFFSET            (NVEEPROM_PARAM_CHANNEL_LIGHT_ID_OFFSET + NVEEPROM_PARAM_CHANNEL_LIGHT_ID_SIZE)
#define NVEEPROM_PARAM_CHANNEL_MEASUREMENT_SIZE              0x00000008

/******************************************************************************
 * GROUP: LENS_SHADING
 *****************************************************************************/
#define NVEEPROM_GROUP_LENS_SHADING_DATA_ID                  0x00000004
#define NVEEPROM_GROUP_LENS_SHADING_DATA_READONLY            true
#define NVEEPROM_GROUP_LENS_SHADING_DATA_MANDATORY           false

/* Light used for measurement */
#define NVEEPROM_PARAM_SHADING_LIGHT_ID_ID                   NVEEPROM_PARAM_ID(LENS_SHADING_DATA, 1)
#define NVEEPROM_PARAM_SHADING_LIGHT_ID_TYPE                 NVEEPROM_TYPE_UINT8
#define NVEEPROM_PARAM_SHADING_LIGHT_ID_OFFSET               0
#define NVEEPROM_PARAM_SHADING_LIGHT_ID_SIZE                 0x00000001

/* MSB is 1 for signed, 0 for unsigned.  The next 7 bits represent the number of
   integer bits.  The last 8 bits represent the number of fractional bits. */
#define NVEEPROM_PARAM_SHADING_DATA_TYPE_ID                  NVEEPROM_PARAM_ID(LENS_SHADING_DATA, 2)
#define NVEEPROM_PARAM_SHADING_DATA_TYPE_TYPE                NVEEPROM_TYPE_BINARY
#define NVEEPROM_PARAM_SHADING_DATA_TYPE_OFFSET              (NVEEPROM_PARAM_SHADING_LIGHT_ID_OFFSET + NVEEPROM_PARAM_SHADING_LIGHT_ID_SIZE)
#define NVEEPROM_PARAM_SHADING_DATA_TYPE_SIZE                0x00000002

/* 4x10x10 mesh stored in R, GR, GB, B order in row major order.  Each element
   of the mesh is a fixed point number, whose properties (signed/unsigned, integer bits,
   fractional bits) are described by the NVEEPROM_PARAM_SHADING_DATA_TYPE
   parameter.

   The MSB of a fixed point number is the sign bit (if it is signed)
   followed by the integer bits, followed by the fractional bits.  A fixed point
   number cannot exceed more than 16 bits of size.

   The fixed point numbers are packed as tightly as possible, and are not
   aligned to byte boundaries.  For example, if the fixed point numbers are
   signed, have one integer bit and 3 fractional bits, each fixed point number
   will take up 5 bits, and the entire array of 400 elements will only take 250
   bytes.*/
#define NVEEPROM_PARAM_LENS_SHADING_DATA_ID                  NVEEPROM_PARAM_ID(LENS_SHADING_DATA, 3)
#define NVEEPROM_PARAM_LENS_SHADING_DATA_TYPE                NVEEPROM_TYPE_FIXED
#define NVEEPROM_PARAM_LENS_SHADING_DATA_OFFSET              (NVEEPROM_PARAM_SHADING_DATA_TYPE_OFFSET + NVEEPROM_PARAM_SHADING_DATA_TYPE_SIZE + 1)
#define NVEEPROM_PARAM_LENS_SHADING_DATA_SIZE                0x00000320

/******************************************************************************
 * GROUP: FUSE_ID
 *****************************************************************************/
#define NVEEPROM_GROUP_FUSE_ID_ID                            0x00000005
#define NVEEPROM_GROUP_FUSE_ID_READONLY                      true
#define NVEEPROM_GROUP_FUSE_ID_MANDATORY                     true

/* The size of the fuse id in bytes, plus 2 */
#define NVEEPROM_PARAM_FUSE_ID_SIZE_ID                       NVEEPROM_PARAM_ID(FUSE_ID, 1)
#define NVEEPROM_PARAM_FUSE_ID_SIZE_TYPE                     NVEEPROM_TYPE_UINT16
#define NVEEPROM_PARAM_FUSE_ID_SIZE_OFFSET                   0
#define NVEEPROM_PARAM_FUSE_ID_SIZE_SIZE                     0x00000002

/* The fuse id of the image sensor. Only the number of bytes specified in FUSE_ID_SIZE - 2 are valid */
#define NVEEPROM_PARAM_FUSE_ID_DATA_ID                       NVEEPROM_PARAM_ID(FUSE_ID, 2)
#define NVEEPROM_PARAM_FUSE_ID_DATA_TYPE                     NVEEPROM_TYPE_BINARY
#define NVEEPROM_PARAM_FUSE_ID_DATA_OFFSET                   (NVEEPROM_PARAM_FUSE_ID_SIZE_OFFSET + NVEEPROM_PARAM_FUSE_ID_SIZE_SIZE)
#define NVEEPROM_PARAM_FUSE_ID_DATA_SIZE                     0x00000014

/******************************************************************************
 * GROUP: INTRINSIC_PARAMETERS
 *****************************************************************************/
#define NVEEPROM_GROUP_INTRINSIC_PARAMETERS_ID               0x00000006
#define NVEEPROM_GROUP_INTRINSIC_PARAMETERS_READONLY         false
#define NVEEPROM_GROUP_INTRINSIC_PARAMETERS_MANDATORY        false

/* Type of calibration and tell what it the purpose of each of 16 following numbers */
#define NVEEPROM_PARAM_INTRINSIC_MODEL_ID                    NVEEPROM_PARAM_ID(INTRINSIC_PARAMETERS, 1)
#define NVEEPROM_PARAM_INTRINSIC_MODEL_TYPE                  NVEEPROM_TYPE_UINT8
#define NVEEPROM_PARAM_INTRINSIC_MODEL_OFFSET                0
#define NVEEPROM_PARAM_INTRINSIC_MODEL_SIZE                  0x00000001

/* 16 coefficients */
#define NVEEPROM_PARAM_INTRINSIC_COEFFICIENTS_ID             NVEEPROM_PARAM_ID(INTRINSIC_PARAMETERS, 2)
#define NVEEPROM_PARAM_INTRINSIC_COEFFICIENTS_TYPE           NVEEPROM_TYPE_FLOAT
#define NVEEPROM_PARAM_INTRINSIC_COEFFICIENTS_OFFSET         (NVEEPROM_PARAM_INTRINSIC_MODEL_OFFSET + NVEEPROM_PARAM_INTRINSIC_MODEL_SIZE)
#define NVEEPROM_PARAM_INTRINSIC_COEFFICIENTS_SIZE           0x00000040

/* 1 for measured, 0 for spec */
#define NVEEPROM_PARAM_INTRINSIC_MEASURED_FLAG_ID            NVEEPROM_PARAM_ID(INTRINSIC_PARAMETERS, 3)
#define NVEEPROM_PARAM_INTRINSIC_MEASURED_FLAG_TYPE          NVEEPROM_TYPE_UINT8
#define NVEEPROM_PARAM_INTRINSIC_MEASURED_FLAG_OFFSET        (NVEEPROM_PARAM_INTRINSIC_COEFFICIENTS_OFFSET + NVEEPROM_PARAM_INTRINSIC_COEFFICIENTS_SIZE)
#define NVEEPROM_PARAM_INTRINSIC_MEASURED_FLAG_SIZE          0x00000001

/******************************************************************************
 * GROUP: EXTRINSIC_PARAMETERS
 *****************************************************************************/
#define NVEEPROM_GROUP_EXTRINSIC_PARAMETERS_ID               0x00000007
#define NVEEPROM_GROUP_EXTRINSIC_PARAMETERS_READONLY         false
#define NVEEPROM_GROUP_EXTRINSIC_PARAMETERS_MANDATORY        false

/* 6 coefficients */
#define NVEEPROM_PARAM_EXTRINSIC_PARAMETERS_ID               NVEEPROM_PARAM_ID(EXTRINSIC_PARAMETERS, 1)
#define NVEEPROM_PARAM_EXTRINSIC_PARAMETERS_TYPE             NVEEPROM_TYPE_FLOAT
#define NVEEPROM_PARAM_EXTRINSIC_PARAMETERS_OFFSET           0
#define NVEEPROM_PARAM_EXTRINSIC_PARAMETERS_SIZE             0x00000018

/******************************************************************************
 * GROUP: SECOND_INTRINSIC_PARAMETERS
 *****************************************************************************/
#define NVEEPROM_GROUP_SECOND_INTRINSIC_PARAMETERS_ID        0x00000008
#define NVEEPROM_GROUP_SECOND_INTRINSIC_PARAMETERS_READONLY  false
#define NVEEPROM_GROUP_SECOND_INTRINSIC_PARAMETERS_MANDATORY false

/* Type of calibration and tell what it the purpose of each of 16 following numbers */
#define NVEEPROM_PARAM_SECOND_INTRINSIC_MODEL_ID             NVEEPROM_PARAM_ID(SECOND_INTRINSIC_PARAMETERS, 1)
#define NVEEPROM_PARAM_SECOND_INTRINSIC_MODEL_TYPE           NVEEPROM_TYPE_UINT8
#define NVEEPROM_PARAM_SECOND_INTRINSIC_MODEL_OFFSET         0
#define NVEEPROM_PARAM_SECOND_INTRINSIC_MODEL_SIZE           0x00000001

/* 16 coefficients */
#define NVEEPROM_PARAM_SECOND_INTRINSIC_COEFFICIENTS_ID      NVEEPROM_PARAM_ID(SECOND_INTRINSIC_PARAMETERS, 2)
#define NVEEPROM_PARAM_SECOND_INTRINSIC_COEFFICIENTS_TYPE    NVEEPROM_TYPE_FLOAT
#define NVEEPROM_PARAM_SECOND_INTRINSIC_COEFFICIENTS_OFFSET  (NVEEPROM_PARAM_SECOND_INTRINSIC_MODEL_OFFSET + NVEEPROM_PARAM_SECOND_INTRINSIC_MODEL_SIZE)
#define NVEEPROM_PARAM_SECOND_INTRINSIC_COEFFICIENTS_SIZE    0x00000040

/* 1 for measured, 0 for spec */
#define NVEEPROM_PARAM_SECOND_INTRINSIC_MEASURED_FLAG_ID     NVEEPROM_PARAM_ID(SECOND_INTRINSIC_PARAMETERS, 3)
#define NVEEPROM_PARAM_SECOND_INTRINSIC_MEASURED_FLAG_TYPE   NVEEPROM_TYPE_UINT8
#define NVEEPROM_PARAM_SECOND_INTRINSIC_MEASURED_FLAG_OFFSET (NVEEPROM_PARAM_SECOND_INTRINSIC_COEFFICIENTS_OFFSET + NVEEPROM_PARAM_SECOND_INTRINSIC_COEFFICIENTS_SIZE)
#define NVEEPROM_PARAM_SECOND_INTRINSIC_MEASURED_FLAG_SIZE   0x00000001

/******************************************************************************
 * GROUP: THIRD_INTRINSIC_PARAMETERS
 *****************************************************************************/
#define NVEEPROM_GROUP_THIRD_INTRINSIC_PARAMETERS_ID         0x00000009
#define NVEEPROM_GROUP_THIRD_INTRINSIC_PARAMETERS_READONLY   false
#define NVEEPROM_GROUP_THIRD_INTRINSIC_PARAMETERS_MANDATORY  false

/* Type of calibration and tell what it the purpose of each of 16 following numbers */
#define NVEEPROM_PARAM_THIRD_INTRINSIC_MODEL_ID              NVEEPROM_PARAM_ID(THIRD_INTRINSIC_PARAMETERS, 1)
#define NVEEPROM_PARAM_THIRD_INTRINSIC_MODEL_TYPE            NVEEPROM_TYPE_UINT8
#define NVEEPROM_PARAM_THIRD_INTRINSIC_MODEL_OFFSET          0
#define NVEEPROM_PARAM_THIRD_INTRINSIC_MODEL_SIZE            0x00000001

/* 16 coefficients */
#define NVEEPROM_PARAM_THIRD_INTRINSIC_COEFFICIENTS_ID       NVEEPROM_PARAM_ID(THIRD_INTRINSIC_PARAMETERS, 2)
#define NVEEPROM_PARAM_THIRD_INTRINSIC_COEFFICIENTS_TYPE     NVEEPROM_TYPE_FLOAT
#define NVEEPROM_PARAM_THIRD_INTRINSIC_COEFFICIENTS_OFFSET   (NVEEPROM_PARAM_THIRD_INTRINSIC_MODEL_OFFSET + NVEEPROM_PARAM_THIRD_INTRINSIC_MODEL_SIZE)
#define NVEEPROM_PARAM_THIRD_INTRINSIC_COEFFICIENTS_SIZE     0x00000040

/* 1 for measured, 0 for spec */
#define NVEEPROM_PARAM_THIRD_INTRINSIC_MEASURED_FLAG_ID      NVEEPROM_PARAM_ID(THIRD_INTRINSIC_PARAMETERS, 3)
#define NVEEPROM_PARAM_THIRD_INTRINSIC_MEASURED_FLAG_TYPE    NVEEPROM_TYPE_UINT8
#define NVEEPROM_PARAM_THIRD_INTRINSIC_MEASURED_FLAG_OFFSET  (NVEEPROM_PARAM_THIRD_INTRINSIC_COEFFICIENTS_OFFSET + NVEEPROM_PARAM_THIRD_INTRINSIC_COEFFICIENTS_SIZE)
#define NVEEPROM_PARAM_THIRD_INTRINSIC_MEASURED_FLAG_SIZE    0x00000001

/******************************************************************************
 * GROUP: SCRATCH_PAD
 *****************************************************************************/
#define NVEEPROM_GROUP_SCRATCH_PAD_ID                        0x0000000a
#define NVEEPROM_GROUP_SCRATCH_PAD_READONLY                  false
#define NVEEPROM_GROUP_SCRATCH_PAD_MANDATORY                 false

/* Free space for the application to store any data */
#define NVEEPROM_PARAM_SCRATCH_PAD_ID                        NVEEPROM_PARAM_ID(SCRATCH_PAD, 1)
#define NVEEPROM_PARAM_SCRATCH_PAD_TYPE                      NVEEPROM_TYPE_BINARY
#define NVEEPROM_PARAM_SCRATCH_PAD_OFFSET                    0
#define NVEEPROM_PARAM_SCRATCH_PAD_SIZE                      0x00000400

#ifdef __cplusplus
}
#endif

#endif /* _NVEEPROM_SPEC_H_ */
