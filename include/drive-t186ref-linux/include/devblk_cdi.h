/*
 * Copyright (c) 2015-2020, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Device Block Interface: Camera Device Interface (CDI)</b>
 *
 * This file contains the Camera Device Interface API.
 */

#ifndef _DEVBLK_CDI_H
#define _DEVBLK_CDI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"
#include "nvmedia_icp.h"

/**
 * \defgroup devblk_cdi_api Camera Device Interface (CDI)
 *
 * The Camera Device Interface API encompasses all DevBlk I2C control related
 * functions, including programming of all I2C controlled components
 * such as deserializers, serializers, EEPROMs, and image sensors.
 *
 * DevBlkCDI needs a device driver for each attached device. This provides the
 * flexibility of adding new devices easily.
 *
 * @ingroup nvmedia_image_top
 * @{
 */

/**
 * \defgroup devblk_cdi_types Basic CDI Types
 * The Camera Device Interface API provides common CDI processing functions.
 * @ingroup basic_api_top
 *
 * @{
 */

/** \brief Device address to use for an CDI simulator device. Select the
 * \ref DEVBLK_CDI_I2C_SIMULATOR port to use simulator mode for all devices.
 */
#define DEVBLK_CDI_SIMULATOR_ADDRESS   0xFF1u
/** Bits reserved for the  I2C bus number in \ref CDI_RDEV_CFG(csi, i2c). */
#define RDEV_CFG_I2C_BITS           8
/** Bits reserved for the CSI port in \ref CDI_SLV_RDEV_CFG(csi, i2c). */
#define RDEV_CFG_CSI_BITS           (RDEV_CFG_I2C_BITS + 8)
/** Bit reserved for the slave mode flag in \ref CDI_SLV_RDEV_CFG(csi, i2c). */
#define RDEV_CFG_SLV_BIT            (RDEV_CFG_CSI_BITS + 1)

/** \brief Macro to create root device configuration with the connected CSI port
 * and I2C bus.
 * \param[in] csi The CSI port number defined in \ref DevBlkICPInterfaceType.
 * \param[in] i2c The I2C bus number defined in \ref DevBlkCDI_I2CPort.
 */
#define CDI_RDEV_CFG(csi, i2c)   (((uint32_t)(csi) << RDEV_CFG_I2C_BITS) | (i2c))

/**
 * \brief  Extended macro to create root device configuration with the connected
 *  CSI port, I2C bus, and  an option to disable power control from root device.
 *
 * \param[in] csi        The CSI port number defined in
 *                        \ref DevBlkICPInterfaceType.
 * \param[in] i2c        The I2C bus number defined in \ref DevBlkCDI_I2CPort.
 * \param[in] disPwrCtrl A flag to disable power control. Value may be 0
 *                        (root device turns on power for devices in
 *                        DevBlkCDIRootDeviceCreate() and turns off power
 *                        for devices in DevBlkCDIRootDeviceDestroy())
 *                        or 1 (root device does not control power for devices
 *                        in %DevBlkCDIRootDeviceCreate() and
 *                        %DevBlkCDIRootDeviceDestroy()).
 */
#define CDI_RDEV_CFG_EX(csi, i2c, disPwrCtrl) \
         ((i2c & 0xffu) | \
          ((uint32_t)(csi & 0xffu) << RDEV_CFG_I2C_BITS) | \
          ((uint32_t)(disPwrCtrl & 1u) << RDEV_CFG_SLV_BIT))

/**
 * \brief  Macro to create a slave root device configuration with the
 *  connected CSI port and I2C bus when the application is run on a slave SoC.
 *
 * \param[in] csi   The CSI port number defined in \ref NvMediaICPInterfaceType.
 * \param[in] i2c   The I2C bus number defined in \ref DevBlk_I2CPort.
 */
#define CDI_SLV_RDEV_CFG(csi, i2c) \
        ((i2c) | ((uint32_t)(csi) << RDEV_CFG_I2C_BITS) | ((uint32_t)(1u) << RDEV_CFG_CSI_BITS))

/** \brief  Macro to initialize an \ref DevBlkCDIAdvancedConfig object.
 *
 * Clears the parameter and updates with the
 * \ref DevBlkCDIAdvancedConfig::clientContext pointer.
 * \param[in] cfg The configuration to initialize.
 * \param[in] ctx The \a clientContext pointer to be updated.
 */
#define ADV_CONFIG_INIT(cfg, ctx) \
        do { \
            memset(&cfg, 0, sizeof(cfg)); \
            cfg.clientContext = (void *)(ctx); \
        } while (0)

/** \brief Maximum number of exposures. */
#define DEVBLK_CDI_MAX_EXPOSURES           (8u)

/** \brief Maximum number of color components. */
#define DEVBLK_CDI_MAX_COLOR_COMPONENT         (4u)

/** \brief Maximum number of sensor contexts. */
#define DEVBLK_CDI_MAX_SENSOR_CONTEXTS     (4u)

/**
 * \brief Maximum number of sensor companding
 *  piecewise linear (PWL) curve knee points.
 */
#define DEVBLK_CDI_MAX_PWL_KNEEPOINTS      (64u)

/** \brief Maximum number of frame report bytes.  */
#define DEVBLK_CDI_MAX_FRAME_REPORT_BYTES  (4u)

/** \brief Maximum number of sensor temperature values. */
#define DEVBLK_CDI_MAX_NUM_TEMPERATURES    (4u)

/** \brief Maximum possible length of sensor name. */
#define DEVBLK_CDI_MAX_SENSOR_NAME_LENGTH  (32u)

/** \brief Maximum possible length of sensor fuse id. */
#define DEVBLK_CDI_MAX_FUSE_ID_LENGTH      (32u)

struct DevBlkCDISensorControl;

struct DevBlkCDIEmbeddedDataChunk;

struct DevBlkCDIEmbeddedDataInfo;

struct DevBlkCDISensorAttributes;

#if !NV_IS_SAFETY
struct DevBlkCDIModuleConfig;
#endif

/** @} */ /* Ends Basic CDI Types group */

/**
 * \brief  Holds the handle for an DevBlkCDIDevice object.
 */
typedef struct {
    void *deviceDriverHandle;
} DevBlkCDIDevice;

/**
 * \defgroup cdi_root_device_api CDI Root Device
 *
 * Manage \ref DevBlkCDIRootDevice objects, which represent the root of the
 * Nvmedia CDI object system.
 *
 * The DevBlkCDIRootDevice object manages an I2C port on the host hardware
 * device.
 * @{
 */

/**
 * \hideinitializer
 * \brief Defines the I2C buses on the host hardware device.
 */
typedef enum {
    DEVBLK_CDI_I2C_BUS_0 = 0, /**< Specifies i2c-0. */
    DEVBLK_CDI_I2C_BUS_1 = 1, /**< Specifies i2c-1. */
    DEVBLK_CDI_I2C_BUS_2 = 2, /**< Specifies i2c-2. */
    DEVBLK_CDI_I2C_BUS_3 = 3, /**< Specifies i2c-3. */
    DEVBLK_CDI_I2C_BUS_4 = 4, /**< Specifies i2c-4. */
    DEVBLK_CDI_I2C_BUS_5 = 5, /**< Specifies i2c-5. */
    DEVBLK_CDI_I2C_BUS_6 = 6, /**< Specifies i2c-6. */
    DEVBLK_CDI_I2C_BUS_7 = 7, /**< Specifies i2c-7. */
    DEVBLK_CDI_I2C_BUS_8 = 8, /**< Specifies i2c-8. */
    DEVBLK_CDI_I2C_BUS_9 = 9, /**< Specifies i2c-9. */
    DEVBLK_CDI_I2C_BUS_10 = 10, /**< Specifies i2c-10. */
    DEVBLK_CDI_I2C_BUS_11 = 11, /**< Specifies i2c-11. */
    DEVBLK_CDI_I2C_SIMULATOR = 255, /**< Port SIMPULATOR (20) */
} DevBlkCDI_I2CPort;

/**
 * \brief  An opaque handle for an DevBlkCDIRootDevice object.
 */
typedef void DevBlkCDIRootDevice;

/**
 * \brief Creates an \ref DevBlkCDIRootDevice object.
 *
 * \param[in] portCfg
 *          The CSI/I2C ports that this root device use. It is strongly
 *          recommended that you use the macro \ref CDI_RDEV_CFG(csi, i2c) or
 *          \ref CDI_RDEV_CFG_EX(csi, i2c, disPwrCtrl) to
 *          generate this value. If the application runs on a slave SoC while
 *          a master SoC controls CDI devices, use CDI_SLV_RDEV_CFG(csi, i2c).
 *          Supported I2C are:
 *          - \ref DEVBLK_CDI_I2C_BUS_0
 *          - \ref DEVBLK_CDI_I2C_BUS_1
 *          - \ref DEVBLK_CDI_I2C_BUS_2
 *          - \ref DEVBLK_CDI_I2C_BUS_3
 *          - \ref DEVBLK_CDI_I2C_BUS_4
 *          - \ref DEVBLK_CDI_I2C_BUS_5
 *          - \ref DEVBLK_CDI_I2C_BUS_6
 *          - \ref DEVBLK_CDI_I2C_BUS_7
 *          - \ref DEVBLK_CDI_I2C_BUS_8
 *          - \ref DEVBLK_CDI_I2C_BUS_9
 *          - \ref DEVBLK_CDI_I2C_BUS_10
 *          - \ref DEVBLK_CDI_I2C_BUS_11
 *          - \ref DEVBLK_CDI_I2C_SIMULATOR
 * Supported CSI ports are:
 *          - \ref DEVBLK_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_A
 *          - \ref DEVBLK_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_B
 *          - \ref DEVBLK_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_AB
 *          - \ref DEVBLK_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_C
 *          - \ref DEVBLK_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_D
 *          - \ref DEVBLK_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_CD
 *          - \ref DEVBLK_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_E
 *          - \ref DEVBLK_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_F
 *          - \ref DEVBLK_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_EF
 *          - \ref DEVBLK_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_G
 *          - \ref DEVBLK_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_H
 *          - \ref DEVBLK_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_GH
 * \return  The new device's handle if successful, or NULL otherwise.
 */
DevBlkCDIRootDevice *
DevBlkCDIRootDeviceCreate(
    uint32_t portCfg
);

/**
 * \brief Destroys an \ref DevBlkCDIRootDevice object.
 * \param[in] device Handle of the device to be destroyed.
 */
void
DevBlkCDIRootDeviceDestroy(
    DevBlkCDIRootDevice *device
);

/**
 * \brief Waits until an error condition is reported or
 *  DevBlkCDIRootDeviceAbortWaitForError() is called.
 *
 *  For safety use cases, application software shall verify the successful reception of camera
 *  error GPIO interrupt as a first step of programming the deserializer. This can be implemented
 *  by calling DevBlkCDIRootDeviceWaitForError() followed by programming the deserializer to
 *  toggle the camera error GPIO pin which would cause DebBlkCDIRootDeviceWaitForError() to return.
 *
 * \param[in] device The root device to use.
 * \retval  DEVBLK_STATUS_OK indicates that the call was successful.
 * \retval  DEVBLK_STATUS_BAD_PARAMETER indicates that @a device was NULL.
 * \retval  DEVBLK_STATUS_ERROR indicate that some other error occurred.
 */
NvMediaStatus
DevBlkCDIRootDeviceWaitForError(
    DevBlkCDIRootDevice *device
);

/**
 * \brief Aborts a call to DevBlkCDIRootDeviceWaitForError().
 * \param[in] device The root device to use.
 * \retval  NVMEDIA_STATUS_OK to indicate that the operatrion was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER to indicate that @a device was NULL.
 * \retval  NVMEDIA_STATUS_ERROR to indicate that some other error occurred.
 */
NvMediaStatus
DevBlkCDIRootDeviceAbortWaitForError(
    DevBlkCDIRootDevice *device
);

/**@} <!-- Ends cdi_root_device_api CDI Root Device --> */

/**
 * \defgroup cdi_device_driver_api CDI Device Driver
 *
 * Program elements related to \ref DevBlkCDIDeviceDriver, which defines a
 * device driver.
 * The core DevBlkCDI calls the driver when the client calls the related
 * public DevBlkCDI function.
 *
 * Before the client can create an DevBlkCDIDevice object (a device), it must
 * provide a device driver.
 * The DevBlkCDIDeviceDriver object contains the following data fields and
 * function pointers.
 *
 * @par Data Fields
 * \li deviceName The name of the device. This is a null-terminated string.
 * \li regLength  Target device offset length in bytes.
 * \li dataLength Target device data length in bytes.
 *
 * @par Function Pointers
 *  - DriverCreate (mandatory). Invoked when the client calls
 *    DevBlkCDIDeviceCreate().
 *  - DriverDestroy (mandatory). Invoked when the client calls
 *    DevBlkCDIDeviceDestroy().
 *  - GetModuleConfig (optional). Invoked when the client calls
 *    DevBlkCDIGetModuleConfig().
 *  - ParseEmbedDataInfo (optional). Invoked when the client calls
 *    DevBlkCDIParseEmbedDataInfo().
 *  - SetSensorControls (optional). Invoked when the client calls
 *    DevBlkCDISetSensorControls().
 *  - GetSensorAttributes (optional). Invoked when the client calls
 *    DevBlkCDIGetSensorAttributes().
 *  - SetSensorCharMode (optional). Invoked when the client calls
 *    DevBlkCDISetSensorCharMode().
 *  - ReadRegister (optional).
 *  - WriteRegister (optional).
 *
 * Here is a sample device driver implementation. The source file defines the
 * driver by creating an DevBlkCDIDeviceDriver struct and setting
 * its function pointers. The header file provides a function that retrieves
 * a pointer to the driver struct.
 *
 * @par Header File
 *
 * \code
 *
 * #include <devblk_cdi.h>
 *
 * DevBlkCDIDeviceDriver *GetSAMPLEDEVICEDriver(void);
 *
 * \endcode
 *
 * <b>Source File</b>
 * \code
 *
 * #include "cdi_sample_device.h"
 *
 * static NvMediaStatus
 * DriverCreate(
 *     DevBlkCDIDevice *handle,
 *     void *clientContext)
 * {
 *     if(!handle)
 *         return NVMEDIA_STATUS_BAD_PARAMETER;
 *
 *     // Can be used to maintain local device context
 *     // or can be set to NULL.
 *     handle->deviceDriverHandle = NULL;
 *
 *     return NVMEDIA_STATUS_OK;
 * }
 *
 * static DevBlkCDIDeviceDriver deviceDriver = {
 *     .deviceName = "Sample Sensor Device",
 *     .DriverCreate = DriverCreate
 * };
 *
 * DevBlkCDIDeviceDriver *
 * GetSAMPLEDEVICEDriver(void)
 * {
 *     // Return device driver descriptor structure
 *     return &deviceDriver;
 * }
 *
 * \endcode
 *
 * @{
 */

/**
 * \brief  Holds device driver data.
 */
typedef struct {
    /** Holds the device name. */
    char *deviceName;
    /** Holds the target device offset length in bytes. */
    int32_t regLength;
    /** Holds the target device data length in bytes. */
    int32_t dataLength;

   /** Holds the function that creates device driver
    * \n This function is invoked by \ref DevBlkCDIDeviceCreate function call.
    * \n
    * @par Sample Pseudo Code:
    *
    * \code
    * //Pseudo code for cdi device driver function DriverCreate invoked by DevBlkCDIDriverCreate function call.
    *  NvMediaStatus
    *  DriverCreate(
    *      DevBlkCDIDevice *handle,
    *      void *clientContext)
    *  {
    *      NvMediaStatus status = NVMEDIA_STATUS_OK;
    *      _DriverHandle *driverHandle = NULL;
    *
    *      // check input parameters
    *      if (!handle || !clientContext)
    *      {
    *          return NVMEDIA_STATUS_BAD_PARAMETER;
    *      }
    *
    *      // allocate memory to driver handle
    *      driverHandle = calloc(...);
    *      if (!driverHandle)
    *      {
    *          return NVMEDIA_STATUS_OUT_OF_MEMORY;
    *      }
    *
    *      // initialize other members
    *      driverHandle->... = ...;
    *
    *      // initialize the context
    *      driverHandle->modulecfg... = clientContext->modulecfg...;
    *
    *      // set the driver handle
    *      handle->deviceDriverHandle = (void *)drvHandle;
    *
    *      return NVMEDIA_STATUS_OK;
    *  }
    * \endcode
    *
    */
    NvMediaStatus (* DriverCreate)(
        DevBlkCDIDevice *handle,
        void *clientContext);

   /** Holds the function that destroy the device driver
    * \n This function is invoked by \ref DevBlkCDIDeviceDestroy function call.
    * \n
    * @par Sample Pseudo Code:
    *
    * \code
    *  //Pseudo code for cdi device driver function DriverDestroy invoked by DevBlkCDIDeviceDestroy function call.
    *
    *  NvMediaStatus
    *  DriverDestroy(
    *     DevBlkCDIDevice *handle)
    *  {
    *
    *     // check input parameters
    *     if (!handle)
    *     {
    *        return NVMEDIA_STATUS_BAD_PARAMETER;
    *     }
    *
    *     // free driver handle memory
    *     if (handle->deviceDriverHandle != NULL) {
    *         free(handle->deviceDriverHandle);
    *         handle->deviceDriverHandle = NULL;
    *     }
    *
    *     return NVMEDIA_STATUS_OK;
    *  }
    *
    * \endcode
    *
    */
    NvMediaStatus (* DriverDestroy)(
        DevBlkCDIDevice *handle);

    /** Holds the function that sets sensor controls
     * \n This function is invoked by \ref DevBlkCDISetSensorControls function call.
     * \n
     * @par Sample Usage:
     *
     * \code
     *  //Pseudo code for cdi device driver function SetSensorControls invoked by DevBlkCDISetSensorControls function call.
     *
     *  NvMediaStatus
     *  SetSensorControls(
     *     DevBlkCDIDevice *handle,
     *     const DevBlkCDISensorControl *sensorControl,
     *     const size_t sensrCtrlStructSize)
     *  {
     *
     *     NvMediaStatus status = NVMEDIA_STATUS_OK;
     *     uint8_t regVal[2];
     *
     *     // check input parameters
     *     if (!handle || !sensorControl || !sensrCtrlStructSize)
     *     {
     *        return NVMEDIA_STATUS_BAD_PARAMETER;
     *     }
     *
     *     // check api version
     *     if(sensrCtrlStructSize != sizeof(DevBlkCDISensorControl))
     *     {
     *        return NVMEDIA_STATUS_NOT_SUPPORTED;
     *     }
     *
     *     // check num sensor context
     *     if(sensorControl->numSensorContexts IS NOT SUPPORTED) {
     *        return NVMEDIA_STATUS_NOT_SUPPORTED;
     *     }
     *
     *     //  set sensor group hold acquire register, if supported
     *     regVal[0] = ...
     *     status = WriteRegister(handle,...,REG_GROUP_HOLD, regVal);
     *     if (status is NOT NVMEDIA_STATUS_OK) {
     *         // Error Handling
     *     }
     *
     *     For each i in array [0, sensorControl->numSensorContexts)
     *     {
     *         // apply sensor exposure control settings
     *         if(sensorControl->exposureControl[i].expTimeValid IS NVMEDIA_TRUE ||
     *            sensorControl->exposureControl[i].gainValid IS NVMEDIA_TRUE)
     *         {
     *             status = SetExposure(&sensorControl->exposureControl[i], ...);
     *             if (status is NOT NVMEDIA_STATUS_OK) {
     *                 // Error Handling
     *             }
     *         }
     *
     *         // apply sensor white balance control settings
     *         if(sensorControl->wbControl[i].wbValid == NVMEDIA_TRUE)
     *         {
     *             status = SetSensorWbGain(&sensorControl->wbControl[i], ...);
     *             if (status is NOT NVMEDIA_STATUS_OK) {
     *                 // Error Handling
     *             }
     *         }
     *     }
     *
     *     // apply sensor frame report control settings
     *     if(sensorControl->frameReportControl.frameReportValid == NVMEDIA_TRUE)
     *     {
     *         status = SetSensorFrameReport(&sensorControl->frameReportControl, ...);
     *         if (status is NOT NVMEDIA_STATUS_OK) {
     *             // Error Handling
     *         }
     *     }
     *
     *     // set group hold release register
     *     regVal[0] = ...
     *     status = WriteRegister(handle,.., REG_GROUP_HOLD, regVal);
     *     if (status is NOT NVMEDIA_STATUS_OK) {
     *         // Error Handling //
     *     }
     *
     *     return status;
     *  }
     *
     * \endcode
     *
     */
    NvMediaStatus (* SetSensorControls)(
        DevBlkCDIDevice *handle,
        const struct DevBlkCDISensorControl *sensorControl,
        const size_t sensrCtrlStructSize);

    /**
     * \brief  Holds a pointer to the function that parses embedded data
     *  returned as part of a captured buffer.
     *
     * DevBlkCDIParseEmbedDataInfo() invokes this function.
     *
     * @par Sample Usage:
     *
     * \code
     *  //Pseudo code for cdi device driver function ParseEmbedDataInfo invoked by \ref DevBlkCDIParseEmbedDataInfo function call.
     *
     *  NvMediaStatus
     *  ParseEmbedDataInfo(
     *     DevBlkCDIDevice *handle,
     *     const DevBlkCDIEmbeddedDataChunk *embeddedTopDataChunk,
     *     const DevBlkCDIEmbeddedDataChunk *embeddedBotDataChunk,
     *     const size_t embeddedDataChunkStructSize,
     *     DevBlkCDIEmbeddedDataInfo *embeddedDataInfo,
     *     const size_t dataInfoStructSize);
     *  {
     *
     *     NvMediaStatus status = NVMEDIA_STATUS_OK;
     *
     *     // check input parameters
     *     if(!handle || !embeddedTopDataChunk || !embeddedBotDataChunk ||
     *        !embeddedDataChunkStructSize || !dataInfoStructSize)
     *     {
     *        return NVMEDIA_STATUS_BAD_PARAMETER;
     *     }
     *
     *     // check api version
     *     if ((embeddedDataChunkStructSize != sizeof(DevBlkCDIEmbeddedDataChunk)) ||
     *         (dataInfoStructSize != sizeof(DevBlkCDIEmbeddedDataInfo))) {
     *         // Handle version mismatch
     *     }
     *
     *     memset(parsedInfo, 0, sizeof(DevBlkCDIEmbeddedDataInfo));
     *
     *     // decode top embedded lines
     *     status = DepackTopEmbeddedLine (handle, embeddedTopDataChunk, … );
     *     if (status is NOT NVMEDIA_STATUS_OK) {
     *         // Error Handling
     *     }
     *
     *     // decode bottom embedded lines, if supported by sensor driver
     *     status = DepackBottomEmbeddedLine (handle, embeddedBotDataChunk, … );
     *     if (status is NOT NVMEDIA_STATUS_OK) {
     *         // Error Handling
     *     }
     *
     *     // populate number of active exposures for the captured frame
     *     status = ParseNumExposures(&parsedInfo->numActiveExposures);
     *     if (status is NOT NVMEDIA_STATUS_OK) {
     *         // Error Handling
     *     }
     *
     *     // populate frame exposure info block
     *     status = ParseExposure(&parsedInfo->sensorExpInfo, ...);
     *     if (status is NOT NVMEDIA_STATUS_OK) {
     *         // Error Handling
     *     }
     *
     *     // populate frame white balance info block
     *     status = ParseWBGain(&parsedInfo->sensorWBInfo, ...);
     *     if (status is NOT NVMEDIA_STATUS_OK) {
     *         // Error Handling
     *     }
     *
     *     // populate frame temperature info block
     *     status = ParseTemperatureInfo(&parsedInfo->sensorTempInfo, ...);
     *     if (status is NOT NVMEDIA_STATUS_OK) {
     *         // Error Handling
     *     }
     *
     *     // Set valid flag to FALSE for unsupported info blocks
     *     parsedInfo->sensorReportInfo.frameReportValid = NVMEDIA_FALSE;
     *     parsedInfo->sensorTempInfo.tempValid = NVMEDIA_FALSE;
     *
     *     return status;
     *  }
     *
     * \endcode
     */
    NvMediaStatus (* ParseEmbedDataInfo)(
        DevBlkCDIDevice *handle,
        const struct DevBlkCDIEmbeddedDataChunk *embeddedTopDataChunk,
        const struct DevBlkCDIEmbeddedDataChunk *embeddedBotDataChunk,
        const size_t dataChunkStructSize,
        struct DevBlkCDIEmbeddedDataInfo *embeddedDataInfo,
        const size_t dataInfoStructSize);

    /** Holds the function that gets sensor attributes.
     *
     * @par Sample Usage:
     *
     * \code
     *  //Pseudo code for cdi device driver function GetSensorAttributes invoked by DevBlkCDIGetSensorAttributes function call.
     *
     *  NvMediaStatus
     *  GetSensorAttributes(
     *     DevBlkCDIDevice *handle,
     *     DevBlkCDISensorAttributes *sensorAttr,
     *     const size_t sensorAttrStructSize)
     *  {
     *     NvMediaStatus status = NVMEDIA_STATUS_OK;
     *
     *     // check input parameters
     *     if (!handle || !sensorAttr || !sensorAttrStructSize)
     *     {
     *        return NVMEDIA_STATUS_BAD_PARAMETER;
     *     }
     *
     *     // check api version
     *     if(sensorAttrStructSize != sizeof(DevBlkCDISensorAttributes))
     *     {
     *        return NVMEDIA_STATUS_NOT_SUPPORTED;
     *     }
     *
     *     // populate sensorAttr(DevBlkCDISensorAttributes) structure
     *     memset(sensorAttr, 0, sizeof(DevBlkCDISensorAttributes));
     *
     *
     *     //populate attribute sensor name
     *     status = GetDeviceDriverName(handle,&sensorAttr->sensorName, ...);
     *     if (status is NOT NVMEDIA_STATUS_OK) {
     *         // Error Handling
     *     }
     *
     *     //populate attribute sensor fuse id
     *     status = GetSensorFuseId(handle, &sensorAttr->fuseId, ...);
     *     if (status is NOT NVMEDIA_STATUS_OK) {
     *         // Error Handling
     *     }
     *
     *     //populate attribute number of active exposures
     *     status = GetNumActiveExposures(handle, &sensorAttr->numActiveExposures, ...);
     *     if (status is NOT NVMEDIA_STATUS_OK) {
     *         // Error Handling
     *     }
     *
     *     // populate attributes sensor exposure range, gain range for active exposures
     *     For each i in array [0, sensorAttr->numActiveExposures)
     *     {
     *         sensorAttr->sensorExpRange[i].min = ...
     *         sensorAttr->sensorExpRange[i].max = ...
     *         .
     *         .
     *         .
     *         sensorAttr->sensorGainRange[i].min = ...
     *         sensorAttr->sensorGainRange[i].max = ...
     *         sensorAttr->sensorGainFactor[i] = 1.0;
     *     }
     *
     *     // populate sensor frame report bytes, if supported by sensor driver
     *     sensorAttr->numFrameReportBytes = ...
     *
     *     return status;
     *  }
     *
     * \endcode
     *
     */
    NvMediaStatus (* GetSensorAttributes)(
        DevBlkCDIDevice *handle,
        struct DevBlkCDISensorAttributes *sensorAttr,
        const size_t sensorAttrStructSize);

#if !NV_IS_SAFETY
    /** Holds the function that gets module configuration. */
    NvMediaStatus (* GetModuleConfig)(
        DevBlkCDIDevice *handle,
        struct DevBlkCDIModuleConfig *moduleConfig);

    /** Holds the function that sets sensor in characterization mode.
     *
     * \n This function is invoked by \ref DevBlkCDISetSensorCharMode function call.
     * \n
     * @par Sample Usage:
     *
     * \code
     *  //Pseudo code for cdi device driver function SetSensorCharMode invoked by DevBlkCDISetSensorCharMode function call.
     *
     * NvMediaStatus
     * SetSensorCharMode(
     *     DevBlkCDIDevice *handle,
     *     uint8_t expNo);
     *  {
     *     NvMediaStatus status = NVMEDIA_STATUS_OK;
     *
     *     // check input parameters
     *     if (!handle || !expNo)
     *     {
     *        return NVMEDIA_STATUS_BAD_PARAMETER;
     *     }
     *
     *     // set sensor in characterization mode for expNo
     *     status = ExpBypass(handle, expNo, ...)
     *     if (status is NOT NVMEDIA_STATUS_OK) {
     *         // Error Handling
     *     }
     *
     *     // update driver internal state and sensor attributes
     *     drvrHandle->numActiveExposures = 1;
     *     drvrHandle->charModeEnabled = NVMEDIA_TRUE;
     *     drvrHandle->charModeExpNo = expNo;
     *
     *     return status;
     *  }
     *
     * \endcode
     *
     */
    NvMediaStatus (* SetSensorCharMode)(
        DevBlkCDIDevice *handle,
        uint8_t expNo);
#endif

    /**
     * Holds the function that reads a block from an I2C device
     */
    NvMediaStatus (* ReadRegister)(
        DevBlkCDIDevice *handle,
        uint32_t deviceIndex,
        uint32_t registerNum,
        uint32_t dataLength,
        uint8_t *dataBuff);

    /**
     * Holds the function that writes a block to an I2C device
     */
    NvMediaStatus (* WriteRegister)(
        DevBlkCDIDevice *handle,
        uint32_t deviceIndex,
        uint32_t registerNum,
        uint32_t dataLength,
        uint8_t *dataBuff);

} DevBlkCDIDeviceDriver;

/**@} <!-- Ends cdi_device_driver_api CDI Device driver --> */

/**
 * \defgroup cdi_device_api CDI Device
 *
 * An CDI device represents a device that is attached or linked to the root I2C
 * port.
 *
 * @{
 */

/**
 * Holds the description of the target I2C device.
 */
typedef struct {
    /** Holds the client context. */
    void *clientContext;
} DevBlkCDIAdvancedConfig;

/**
 * \brief Creates an \ref DevBlkCDIDevice object.
 * \param[in] rootDevice A pointer to the root device that you created
 *            with DevBlkCDIRootDeviceCreate().
 * \param[in] deviceAddressList The list of I2C device addresses corresponding to
                                this \ref DevBlkCDIDevice object
 * \param[in] numDevices The number of I2C addresses in the above list
 * \param[in] deviceDriver The driver structure that defines the behavior of the
 *  device.
 * \param[in] advancedConfig Advanced configuration.
 * \return device The new device's handle or NULL if unsuccessful.
 */
DevBlkCDIDevice *
DevBlkCDIDeviceCreate(
    DevBlkCDIRootDevice *rootDevice,
    uint32_t *deviceAddressList,
    uint32_t numDevices,
    DevBlkCDIDeviceDriver *deviceDriver,
    DevBlkCDIAdvancedConfig *advancedConfig
);

/**
 * \brief Destroys the object that describes an CDI device.
 * \param[in] device Handle to the device to destroy.
 */
void
DevBlkCDIDeviceDestroy(
    DevBlkCDIDevice *device
);

/**
 * \brief Performs a read operation over I2C.
 *
 * For safety use cases, application software shall call %DevBlkCDIDeviceRead() 2 times to read
 * the contents of the same register location. If a register value is not expected to change,
 * return values of the two reads should match.
 *
 * \param[in] device A pointer to the device to use.
 * \param[in] deviceIndex Index of the subdevice to use
 * \param[in] regLength Length of the register address, in bytes.
 * \param[in] regData A pointer to the register address.
 * \param[in] dataLength Length of data to be read, in bytes.
 * \param[out] data A pointer to the location for storing the read data.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more
 *           pointer parameters was NULL.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the device driver
 *           does not support this functionality.
 * \retval  NVMEDIA_STATUS_ERROR indicates that any other error occurred.
 */
NvMediaStatus
DevBlkCDIDeviceRead(
    DevBlkCDIDevice *device,
    uint32_t deviceIndex,
    uint32_t regLength,
    uint8_t *regData,
    uint32_t dataLength,
    uint8_t *data
);

/**
 * \brief Performs a write operation over I2C.
 *
 * For safety use cases, application software shall call DevBlkCDIDeviceRead() after calling
 * %DevBlkCDIDeviceWrite() to verify whether the write operation was successful. If a register
 * value is not expected to change, read value should match the value written.
 *
 * \param[in] device A pointer to the device to use.
 * \param[in] deviceIndex Index of the subdevice to use.
 * \param[in] dataLength Length of data to be written, in bytes.
 * \param[in] data       A pointer to data to be written to device via I2C.
 * \return \ref NvMediaStatus The completion status of the operation.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more
 *           pointer parameters was NULL.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the device driver
 *           does not support this functionality.
 * \retval  NVMEDIA_STATUS_ERROR indicates that any other error occurred.
 */
NvMediaStatus
DevBlkCDIDeviceWrite(
    DevBlkCDIDevice *device,
    uint32_t deviceIndex,
    uint32_t dataLength,
    const uint8_t *data
);

/**
 * \brief  Holds the range of a sensor attribute.
 */
typedef struct DevBlkCDIAttrRange {
    /**
     * Holds the sensor attribute's minimum value.
     */
    float_t  min;

    /**
     * Holds the sensor attribute's maximum value.
     */
    float_t  max;

} DevBlkCDIAttrRange;

/**
 * \brief  Holds the sensor attributes.
 */
typedef struct DevBlkCDISensorAttributes {
    /**
     * Holds the name attribute. If not supported, set to NULL.
     */
    char  sensorName[DEVBLK_CDI_MAX_SENSOR_NAME_LENGTH];

    /**
     * Holds the CFA attribute. If not supported, set to zero.
     */
    uint32_t  sensorCFA;

    /**
     * Holds the fuse ID attribute. If not supported, set to NULL.
     */
    char sensorFuseId[DEVBLK_CDI_MAX_FUSE_ID_LENGTH];

    /**
     * Holds the number of active exposures attribute. Range is
     * [1, \ref DEVBLK_CDI_MAX_EXPOSURES]
     */
    uint8_t  numActiveExposures;

    /**
     * Holds the sensor exposure ranges for active exposures.
     */
    DevBlkCDIAttrRange sensorExpRange[DEVBLK_CDI_MAX_EXPOSURES];

    /**
     * Holds the sensor gain ranges for active exposures.
     */
    DevBlkCDIAttrRange sensorGainRange[DEVBLK_CDI_MAX_EXPOSURES];

    /**
     * Holds the sensor white balance ranges for active exposures.
     */
    DevBlkCDIAttrRange sensorWhiteBalanceRange[DEVBLK_CDI_MAX_EXPOSURES];

    /**
     * Holds the additional sensor gain factor between active exposures.
     *
     * Element @a holds the sensitivity ratio between capture 0 and capture
     * @a a. It is usually set to 1.
     *
     * The HDR ratio between exposure @a a and exposure @a b can be computed as:
     *
     *  \f$
     *  hdrRatio_{ab} = ( Et[a] * Gain[a] * sensorGainFactor[a] ) / (Et[b] * Gain[b] * sensorGainFactor[b])
     *  \f$
     *
     * Where:
     * - Et[a] = exposure time value for HDR exposure @a a.
     * - Gain[a] = sensor gain value for HDR exposure @a a.
     * - sensorGainFactor[a] = sensor gain factor for HDR exposure @a a.
     * - Et[b] = exposure time value for HDR exposure @a b.
     * - Gain[b] = sensor gain value for HDR exposure @a b.
     * - sensorGainFactor[b] = sensor gain factor for HDR exposure @a b.
     */
    float_t  sensorGainFactor[DEVBLK_CDI_MAX_EXPOSURES];

    /**
     * Holds the number of frame report bytes supported by the sensor. If not
     * supported, set to zero.
     * Supported values : [1 , DEVBLK_CDI_MAX_FRAME_REPORT_BYTES]
     */
    uint32_t  numFrameReportBytes;

} DevBlkCDISensorAttributes;

/**
 * \brief Gets the sensor attributes.
 *
 * Sensor attributes are static properties like sensor name,
 * exposure-gain ranges supported, and number of active exposures.
 *
 * \param[in]  device               A pointer to the device to use.
 * \param[out] sensorAttr           A pointer a sensor attributes structure.
 * \param[in]  sensorAttrStructSize Size of the @a sensorAttr structure.
 *
 * \retval  DEVBLK_STATUS_OK indicates that the operation was successful.
 * \retval  DEVBLK_STATUS_BAD_PARAMETER indicates that one or more
 *           pointer parameters was NULL.
 * \retval  DEVBLK_STATUS_NOT_SUPPORTED indicates that the device driver
 *           does not support this functionality.
 * \retval  DEVBLK_STATUS_ERROR indicates that any other error occurred.
 */
NvMediaStatus
DevBlkCDIGetSensorAttributes(
    DevBlkCDIDevice *device,
    DevBlkCDISensorAttributes *sensorAttr,
    const size_t sensorAttrStructSize);

/**
 * \brief  Holds sensor exposure information.
 */
typedef struct DevBlkCDIExposure {
    /**
     * Holds a flag which enables or disables the exposure block.
     */
    NvMediaBool expTimeValid;

    /**
     * Holds exposure time for each active exposure, in seconds. The array has
     * \ref DevBlkCDISensorAttributes.numActiveExposures elements.
     */
    float_t exposureTime[DEVBLK_CDI_MAX_EXPOSURES];

    /**
     * Holds a flag which enables or disables the sensor gain block.
     */
    NvMediaBool gainValid;

    /**
     * Holds sensor a gain value for each active gain. The array has
     * \ref DevBlkCDISensorAttributes.numActiveExposures elements.
     */
    float_t sensorGain[DEVBLK_CDI_MAX_EXPOSURES];

} DevBlkCDIExposure;


/**
 * \brief  Holds the sensor white balance gain structure.
 */
typedef struct DevBlkCDIWhiteBalance {
    /**
     * Holds a flag which enables or disables the white balance gain block.
     */
    NvMediaBool wbValid;

    /**
     * Holds sensor white balance gain values for active
     * exposures, in R Gr Gb B order.
     *
     * @a wbGain has \ref DevBlkCDISensorAttributes.numActiveExposures
     * elements. Each element is a
     * @a value array, which has \ref DEVBLK_CDI_MAX_COLOR_COMPONENT elements.
     */
    struct {
        float_t value[DEVBLK_CDI_MAX_COLOR_COMPONENT];
    } wbGain[DEVBLK_CDI_MAX_EXPOSURES];

} DevBlkCDIWhiteBalance;

/**
 * \brief  Holds the sensor report frame report structure.
 */
typedef struct DevBlkCDIFrameReport {
    /**
     * Holds a flag which enables or disables frame report block.
     */
    NvMediaBool frameReportValid;

    /**
     * Holds the number of active frame report bytes. Range is
     * [1, DEVBLK_CDI_MAX_FRAME_REPORT_BYTES].
     */
    uint8_t numBytes;

    /**
     * Holds the values of active frame report bytes. Array indexes must be
     * in the range [0, (@a numBytes-1)].
     */
    uint8_t sensorframeReport[DEVBLK_CDI_MAX_FRAME_REPORT_BYTES];

} DevBlkCDIFrameReport;

/**
 * \brief  Holds the sensor companding piecewise linear (PWL) structure.
 */
typedef struct DevBlkCDIPWL {
    /**
     * Holds a flag which enables or disables the sensor PWL block.
     */
    NvMediaBool  pwlValid;

    /**
     * Holds the number of active PWL knee points. Must be in the range
     * [1, \ref DEVBLK_CDI_MAX_PWL_KNEEPOINTS].
     */
    uint8_t numKneePoints;

    /**
     * Holds the values of active PWL knee points. Array indexes must be in the
     * range [0, (@a numKneePoints-1)].
     */
    NvMediaPointDouble kneePoints[DEVBLK_CDI_MAX_PWL_KNEEPOINTS];

} DevBlkCDIPWL;

/**
 * \brief  Holds the sensor temperature structure.
 */
typedef struct DevBlkCDITemperature {
    /**
     * Holds a flag which enables or disables the sensor temperature block.
     */
    NvMediaBool  tempValid;

    /**
     * Holds the number of active temperatures. Must be in the range
     * [1, \ref DEVBLK_CDI_MAX_NUM_TEMPERATURES].
     */
    uint8_t numTemperatures;

    /**
     * Holds the values of active sensor temperatures in degrees Celsius. Array
     * indexes must be in the range [0, (@a numTemperatures-1)].
     */
    float_t sensorTempCelsius[DEVBLK_CDI_MAX_NUM_TEMPERATURES];

} DevBlkCDITemperature;

/**
 * \brief  Holds the sensor CRC structure.
 */
typedef struct DevBlkCDICRC {
    /**
     * Holds a flag which enables or disables the CRC block.
     */
    NvMediaBool  crcValid;

    /**
     * Holds the frame CRC value computed from embedded data.
     */
    uint32_t computedCRC;

    /**
     * Holds the frame CRC value parsed from embedded data.
     */
    uint32_t embeddedCRC;

} DevBlkCDICRC;

/**
 * \brief  Holds the sensor frame sequence number structure.
 */
typedef struct DevBlkCDIFrameSeqNum {
    /**
     * Holds a flag which enables OR DISABLES the frame sequence number block.
     */
    NvMediaBool  frameSeqNumValid;

    /**
     * Holds the sensor frame sequence number value.
     */
    uint64_t frameSequenceNumber;

} DevBlkCDIFrameSeqNum;

/**
 * \brief Holds the sensor control structure.
 * \note
 *   \parblock
 *      To activate a sensor control block, set the corresponding @a valid flag
 *      to TRUE and populate the control settings to be set.
 *
 *      To disable a sensor control block, set the corresponding @a valid flag
 *      to FALSE.
 *
 *      For example, to activate the white balance control block, set the
 *      @a wbValid flag in the @a wbControl structure to TRUE and populate the
 *      white balance settings to be programmed. To disable white balance
 *      control block, set the @a wbValid flag to FALSE.
 *   \endparblock
 */
typedef struct DevBlkCDISensorControl {
    /**
     * Holds the number of sensor contexts to activate.
     * A sensor context is a mode of operation, supported by some sensors,
     * in which multiple set of settings (contexts) are programmed and
     * the sensor toggles between them at run time.
     *
     * Must be in the range [1, DEVBLK_CDI_MAX_SENSOR_CONTEXTS].
     * For sensors that do support sensor context, set to 1.
     */
    uint8_t                 numSensorContexts;

    /**
     * Holds the sensor exposure settings to set for each context.
     */
    DevBlkCDIExposure       exposureControl[DEVBLK_CDI_MAX_SENSOR_CONTEXTS];

    /**
     * Holds the sensor white balance settings to set for each context.
     */
    DevBlkCDIWhiteBalance   wbControl[DEVBLK_CDI_MAX_SENSOR_CONTEXTS];

    /**
     * Holds the sensor frame report value to be programmed.
     * Sensor frame report control enables you to program
     * custom user information per frame into sensor scratch space (registers)
     * provided by the sensor for private use.
     */
    DevBlkCDIFrameReport    frameReportControl;

} DevBlkCDISensorControl;

/**
 * \brief Sets sensor control parameters.
 *
 * This function enables you to control sensor
 * image settings like exposure time, sensor gain, and white balance gain.
 * All parameters provided to this function are applied together at a frame boundary
 * through CDI's "group hold" functionality, if supported by the sensor.
 *
 * \note This function invokes the device driver function specified by the
 *  call to SetSensorControls().
 *
 * \param[in] device  A pointer to the sensor control device in use.
 * \param[in] sensorControl  A pointer to a sensor control structure for @a device.
 * \param[in] sensrCtrlStructSize Size of the @a sensorControl structure.
 * \retval  DEVBLK_STATUS_OK indicates that the operation was successful.
 * \retval  DEVBLK_STATUS_BAD_PARAMETER indicates that one or more
 *           pointer parameters was NULL.
 * \retval  DEVBLK_STATUS_NOT_SUPPORTED indicates that the device driver
 *           does not support this functionality.
 * \retval  DEVBLK_STATUS_ERROR indicates that any other error occurred.
 */
NvMediaStatus
DevBlkCDISetSensorControls(
    DevBlkCDIDevice *device,
    const DevBlkCDISensorControl *sensorControl,
    const size_t sensrCtrlStructSize
);

/**
 * \brief  Holds the sensor embedded data parsed info structure.
 *
 * The sensor driver can selectively activate or deactivate any of the parsed
 * info blocks, depending on whether the sensor supports it.
 *
 * To activate a
 * sensor info block, the sensor driver must set the info block's @a valid flag to
 * TRUE and populate the parsed information corresponding to the block.
 * To disable a sensor info block, it must set the valid flag to FALSE.
 *
 * For example, if a sensor supports only exposure, white balance and CRC info,
 * the sensor driver activates only the @a sensorExpInfo, @a sensorWBInfo, and
 * @a sensorCRCInfo blocks (it sets their @a valid flags to TRUE) and disables
 * all of the others (it sets their @a valid flags to FALSE).
 */
typedef struct DevBlkCDIEmbeddedDataInfo {
    /**
     * Holds the parsed embedded data frame number of exposures for the
     * captured frame.
     */
    uint32_t                    numExposures;

    /**
     * Holds the parsed embedded data sensor exposure info for the
     * captured frame.
     */
    DevBlkCDIExposure           sensorExpInfo;

    /**
     * Holds the parsed embedded data sensor white balance info for the
     * captured frame.
     */
    DevBlkCDIWhiteBalance       sensorWBInfo;

    /**
     * Holds the parsed embedded data sensor PWL info for the captured frame.
     */
    DevBlkCDIPWL                sensorPWLInfo;

    /**
     * Holds the parsed embedded data sensor CRC info for the captured frame.
     */
    DevBlkCDICRC                sensorCRCInfo;

    /**
     * Holds the parsed embedded data frame report info for the captured frame.
     */
    DevBlkCDIFrameReport        sensorReportInfo;

    /**
     * Holds the parsed embedded data sensor temperature info for the
     * captured frame.
     */
    DevBlkCDITemperature        sensorTempInfo;

    /**
     * Holds parsed embedded data frame sequence number info for the
     * captured frame.
     */
    DevBlkCDIFrameSeqNum        frameSeqNumInfo;

} DevBlkCDIEmbeddedDataInfo;

/*
 * \brief  Holds the sensor embedded data chunk structure.
 */
typedef struct DevBlkCDIEmbeddedDataChunk {
    /**
     * Holds the line length of an embedded chunk, in bytes.
     */
    uint32_t lineLength;

    /**
     * Holds a pointer to the data chunk.
     */
    const uint8_t *lineData;
} DevBlkCDIEmbeddedDataChunk;

/**
 * \brief  Parses sensor embedded data info and
 * provides sensor image settings information for the captured frame.
 *
 * This function can be used to retrieve sensor image settings like exposure,
 * gain, and white balance information applied to the frame.
 *
 * \param[in]  device               A pointer to the device to use.
 * \param[in]  embeddedTopDataChunk A pointer to the top sensor embedded data
 *                                   chunk structure.
 * \param[in]  embeddedBotDataChunk A pointer to the bottom sensor embedded
 *                                   data chunk structure.
 * \param[in]  embeddedDataChunkStructSize
 *                                  Size of the @a embeddedTopDataChunk and
 *                                   @a embeddedBottomDataChunk structures, in
 *                                   bytes.
 * \param[out] embeddedDataInfo     A pointer to the embedded data parsed info
 *                                   structure.
 * \param[in]  dataInfoStructSize   Size of the @a embeddedDataInfo structure,
 *                                   in bytes.
 * \retval  DEVBLK_STATUS_OK indicates that the operation was successful.
 * \retval  DEVBLK_STATUS_BAD_PARAMETER indicates that one or more
 *           pointer parameters was NULL or invalid.
 * \retval  DEVBLK_STATUS_NOT_SUPPORTED indicates that the device driver
 *           does not support this functionality.
 * \retval  DEVBLK_STATUS_ERROR indicates that any other error occurred.
 */
NvMediaStatus
DevBlkCDIParseEmbedDataInfo(
    DevBlkCDIDevice *device,
    const DevBlkCDIEmbeddedDataChunk *embeddedTopDataChunk,
    const DevBlkCDIEmbeddedDataChunk *embeddedBotDataChunk,
    const size_t embeddedDataChunkStructSize,
    DevBlkCDIEmbeddedDataInfo *embeddedDataInfo,
    const size_t dataInfoStructSize);

/**@} <!-- Ends cdi_device_api CDI Device --> */

#if !NV_IS_SAFETY
/**
 * \hideinitializer
 * \brief CDI Power control items
 */
typedef enum {
    /** Aggregator Power */
    DEVBLK_CDI_PWR_AGGREGATOR,
    /** LINK 0 Power */
    DEVBLK_CDI_PWR_LINK_0,
    /** LINK 1 PWR */
    DEVBLK_CDI_PWR_LINK_1,
    /** LINK 2 PWR */
    DEVBLK_CDI_PWR_LINK_2,
    /** LINK 3 PWR */
    DEVBLK_CDI_PWR_LINK_3,
} DevBlkCDIPowerItems;

/**
 * \hideinitializer
 * \brief Holds the CDI Module ISP configuration.
 */
typedef struct DevBlkCDIModuleConfig {
    /** Holds the camera module name. */
    char cameraModuleCfgName[128];
    /** Holds the camera-specific configuration string. */
    const char *cameraModuleConfigPass1;
    const char *cameraModuleConfigPass2;
} DevBlkCDIModuleConfig;

/**
 * \brief Defines Exposure mode.
 * \deprecated  Use the DevBlkCDISetSensorCharMode() parameter @a expNo
 *  instead.
 */
typedef enum {
    /**  Specifies long exposure mode. */
    DEVBLK_CDI_EXPOSURE_MODE_LONG,
    /**  Specifies short exposure mode. */
    DEVBLK_CDI_EXPOSURE_MODE_SHORT,
    /**  Specifies very short exposure mode. */
    DEVBLK_CDI_EXPOSURE_MODE_VERY_SHORT,
    /**  Specifies the exposure mode count. */
    DEVBLK_CDI_EXPOSURE_MODE_MAX
} DevBlkCDIExposureMode;

/**
 * \brief  Holds exposure control information.
 * \deprecated  Use \ref DevBlkCDISensorControl instead.
 */
typedef struct DevBlkCDIExposureControl {
    /** Holds exposure time for each exposure mode. */
    struct {
        float_t value;
        NvMediaBool valid;
    } exposureTime[DEVBLK_CDI_EXPOSURE_MODE_MAX];
    /** Holds sensor gain for each exposure mode. */
    struct {
        float_t value;
        NvMediaBool valid;
    } sensorGain[DEVBLK_CDI_EXPOSURE_MODE_MAX];
    uint8_t sensorFrameId;
} DevBlkCDIExposureControl;

/**
 * \brief  Holds the white balance control structure.
 * \deprecated  Use \ref DevBlkCDISensorControl instead.
 */
typedef struct DevBlkCDIWBGainControl {
    /** Holds white balance for each exposure mode
        in R Gr Gb B order. */
    struct {
        float_t value[4];
        NvMediaBool valid;
    } wbGain[DEVBLK_CDI_EXPOSURE_MODE_MAX];
} DevBlkCDIWBGainControl;

/**
 * \brief  Holds the embedded data buffer structure.
 * \deprecated  Use the new structure \ref DevBlkCDIEmbeddedDataChunk instead.
 */
typedef struct {
    /** Holds the address of the first register. */
    uint32_t baseRegAddress;
    /** Holds the data size. */
    uint32_t size;
    /** Holds the buffer size. */
    uint32_t bufferSize;
    /** Holds the actual data. */
    uint8_t *data;
} DevBlkCDIEmbeddedDataBuffer;

/**
 * \brief  Holds the embedded data structure.
 * \deprecated  Use the new structure \ref DevBlkCDIEmbeddedDataInfo instead.
 */
typedef struct DevBlkCDIEmbeddedData {
    DevBlkCDIEmbeddedDataBuffer top;
    DevBlkCDIEmbeddedDataBuffer bottom;
    /** Holds parsed exposure control. */
    DevBlkCDIExposureControl exposureControl;
    /** Holds parsed WB gains in R Gr Gb B order. */
    DevBlkCDIWBGainControl wbControl;
    /** Holds parsed frame counter. */
    uint32_t frameSequenceNumber;
} DevBlkCDIEmbeddedData;

/**
 * \hideinitializer
 * \brief CDI sensor attributes
 * \note This enum will not be supported in future release.
 * It is recommended to use the new API \ref DevBlkCDIGetSensorAttributes
 */
typedef enum {
    /** Specifies a unique ID per instance of camera module.
      *
      * Data type: char[32] */
    DEVBLK_CDI_SENSOR_ATTR_FUSE_ID,
    /** Specifies minimum possible gain values.
      *
      * Data type: float_t[DEVBLK_CDI_EXPOSURE_MODE_MAX] */
    DEVBLK_CDI_SENSOR_ATTR_GAIN_MIN,
    /** Specifies maximum possible gain values.
      *
      * Data type: float_t[DEVBLK_CDI_EXPOSURE_MODE_MAX] */
    DEVBLK_CDI_SENSOR_ATTR_GAIN_MAX,
    /** Specifies minimum possible exposure time values in seconds.
      *
      * Data type: float_t[DEVBLK_CDI_EXPOSURE_MODE_MAX] */
    DEVBLK_CDI_SENSOR_ATTR_ET_MIN,
    /** Specifies maximum possible exposure time values in seconds.
      *
      * Data type: float_t[DEVBLK_CDI_EXPOSURE_MODE_MAX] */
    DEVBLK_CDI_SENSOR_ATTR_ET_MAX,
    /** Specifies fine integration time values in seconds.
      *
      * Data type: float_t[DEVBLK_CDI_EXPOSURE_MODE_MAX] */
    DEVBLK_CDI_SENSOR_ATTR_ET_FINE,
    /** Specifies exposure time step values in seconds.
      *
      * Data type: double[DEVBLK_CDI_EXPOSURE_MODE_MAX] */
    DEVBLK_CDI_SENSOR_ATTR_ET_STEP,
    /** Specifies maximum possible HDR ratio value.
      *
      * Data type: uint32_t */
    DEVBLK_CDI_SENSOR_ATTR_HDR_MAX,
    /** Specifies gain factor between the exposures.
      * If non zero, possible factors are 1, x, 1/x.
      *
      * Data type: float_t */
    DEVBLK_CDI_SENSOR_ATTR_GAIN_FACTOR,
    /** Specifies sensitivity ratios for different exposures.
      * The ratios are normalized to the maximum sensitivity,
      * so that all ratios are in the range [0.0, 1.0].
      *
      * Data type: float_t[DEVBLK_CDI_EXPOSURE_MODE_MAX] */
    DEVBLK_CDI_SENSOR_ATTR_QE_PIXEL_RATIO,
    /** Specifies frames per second.
      *
      * Data type: float_t */
    DEVBLK_CDI_SENSOR_ATTR_FRAME_RATE,
    /** Specifies active number of exposures.
      *
      * Data type: uint32_t */
    DEVBLK_CDI_SENSOR_ATTR_NUM_EXPOSURES,
} DevBlkCDISensorAttrType;

/** Set sensor in characterization mode.
 *
 * @par Description
 * DevBlkCDISetSensorCharMode API provides ability for the user to configure
 * the sensor for characterization. Sensor characterization provides optimal
 * parameters, corresponding to sensor physical and functional
 * characteristics, for image processing.
 * \n Sensor characterization for High Dynamic Range (HDR) sensors with
 * multiple exposures (T1, T2, … , Tn ) involves characterizing induvial
 * exposures separately, if required by the sensor. This API provides the
 * ability to configure sensor to capture each exposure separately,
 * if required by sensor characterization.
 * This function re-configures the sensor i.e. changes the sensor static
 * attributes like numActiveExposures, sensorExpRange, sensorGainRange
 * and hence, should be called during sensor initialization time.
 * In order to characterize the sensor exposure number ‘n’,
 * where n = {1,2,3, … , N} for N-exposure HDR sensor, the input parameter
 * ‘expNo’ should be set to ‘n’.
 * \n For a non-HDR sensor, the input parameter ‘expNo’ should always be set to ‘1’.
 *
 * \param[in]  device A pointer to the sensor control device in use.
 * \param[in]  expNo  Sensor exposure number to be used for characterization.
 * Valid range for expNo : [0, (DEVBLK_CDI_MAX_EXPOSURES-1)]
 * For Non-HDR sensor, this should be set to '1'
 *
 * \return \ref NvMediaStatus  The completion status of the operation.
 * Possible values are:
 * \li \ref NVMEDIA_STATUS_OK if successful.
 * \li \ref NVMEDIA_STATUS_BAD_PARAMETER if an input parameter is NULL or invalid.
 * \li \ref NVMEDIA_STATUS_NOT_SUPPORTED if the functionality is not supported
 * by the device driver.
 * \li \ref NVMEDIA_STATUS_ERROR if some other error occurred.
 */
NvMediaStatus
DevBlkCDISetSensorCharMode(
    DevBlkCDIDevice *device,
    uint8_t expNo);

/**
 * \brief Gets the Module ISP configuration.
 * \param[in] device A pointer to the device to use.
 * \param[out] moduleConfig A pointer to the module ISP configuration.
 * \return \ref NvMediaStatus The completion status of the operation.
 * Possible values are:
 * \li \ref NVMEDIA_STATUS_OK
 * \li \ref NVMEDIA_STATUS_BAD_PARAMETER if any of the input parameter is NULL.
 * \li \ref NVMEDIA_STATUS_NOT_SUPPORTED if the functionality is not supported
 * by the device driver.
 * \li \ref NVMEDIA_STATUS_ERROR if any other error occurred.
 */
NvMediaStatus
DevBlkCDIGetModuleConfig(
    DevBlkCDIDevice *device,
    DevBlkCDIModuleConfig *moduleConfig
);
#endif /* #if !NV_IS_SAFETY */

/**@} <!-- Ends devblk_cdi_api Camera Device Interface --> */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* _DEVBLK_CDI_H */
