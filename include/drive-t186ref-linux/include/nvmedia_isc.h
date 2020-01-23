/*
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.  All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA Media Interface: Image Sensor Control (ISC)</b>
 *
 * This file contains the Image Sensor Control API.
 */

#ifndef _NVMEDIA_ISC_H
#define _NVMEDIA_ISC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nvmedia_core.h"
#include "nvmedia_icp.h"

/**
 * \defgroup nvmedia_isc_api Image Sensor Control (ISC)
 *
 * The Image Sensor Control API encompasses all NvMedia I2C control related
 * functions, including programming of all I2C controlled components
 * such as deserializers, serializers, EEPROMs, and image sensors.
 *
 * NvMediaISC needs a device driver for each attached device. This provides the
 * flexibility of adding new devices easily.
 *
 * @ingroup nvmedia_image_top
 * @{
 */

/**
 * \defgroup nvmedia_isc_types Basic ISC Types
 * The Image Sensor Control API provides common ISC processing functions.
 * @ingroup basic_api_top
 *
 * @{
 */

/** \brief Major version number. */
#define NVMEDIA_ISC_VERSION_MAJOR   3
/** \brief Minor version number. */
#define NVMEDIA_ISC_VERSION_MINOR   0

/** \brief Device address to use for an ISC simulator device. Select the
 * \ref NVMEDIA_ISC_I2C_SIMULATOR port to use simulator mode for all devices.
 */
#define NVMEDIA_ISC_SIMULATOR_ADDRESS   0xFF1u
/** Bits reserved for the  I2C bus number in \ref ISC_RDEV_CFG(csi, i2c). */
#define RDEV_CFG_I2C_BITS           8
/** Bits reserved for the CSI port in \ref ISC_SLV_RDEV_CFG(csi, i2c). */
#define RDEV_CFG_CSI_BITS           (RDEV_CFG_I2C_BITS + 8)
/** Bit reserved for the slave mode flag in \ref ISC_SLV_RDEV_CFG(csi, i2c). */
#define RDEV_CFG_SLV_BIT            (RDEV_CFG_CSI_BITS + 1)

/** \brief Macro to create root device configuration with the connected CSI port
 * and I2C bus.
 * \param[in] csi The CSI port number defined in \ref NvMediaICPInterfaceType.
 * \param[in] i2c The I2C bus number defined in \ref NvMediaISC_I2CPort.
 */
#define ISC_RDEV_CFG(csi, i2c)   (((uint32_t)(csi) << RDEV_CFG_I2C_BITS) | (i2c))

/**
 * \brief  Extended macro to create root device configuration with the connected
 *  CSI port, I2C bus, and  an option to disable power control from root device.
 *
 * \param[in] csi        The CSI port number defined in
 *                        \ref NvMediaICPInterfaceType.
 * \param[in] i2c        The I2C bus number defined in \ref NvMediaISC_I2CPort.
 * \param[in] disPwrCtrl A flag to disable power control. Value may be 0
 *                        (root device turns on power for devices in
 *                        NvMediaISCRootDeviceCreate() and turns off power
 *                        for devices in NvMediaISCRootDeviceDestroy())
 *                        or 1 (root device does not control power for devices
 *                        in %NvMediaISCRootDeviceCreate() and
 *                        %NvMediaISCRootDeviceDestroy()).
 */
#define ISC_RDEV_CFG_EX(csi, i2c, disPwrCtrl) \
         ((i2c & 0xffu) | \
          ((uint32_t)(csi & 0xffu) << RDEV_CFG_I2C_BITS) | \
          ((uint32_t)(disPwrCtrl & 1u) << RDEV_CFG_SLV_BIT))

/**
 * \brief  Macro to create a slave root device configuration with the
 *  connected CSI port and I2C bus when the application is run on a slave SoC.
 *
 * \param[in] csi   The CSI port number defined in \ref NvMediaICPInterfaceType.
 * \param[in] i2c   The I2C bus number defined in \ref NvMediaISC_I2CPort.
 */
#define ISC_SLV_RDEV_CFG(csi, i2c) \
        ((i2c) | ((uint32_t)(csi) << RDEV_CFG_I2C_BITS) | ((uint32_t)(1u) << RDEV_CFG_CSI_BITS))

/** \brief  Macro to initialize an \ref NvMediaISCAdvancedConfig object.
 *
 * Clears the parameter and updates with the
 * \ref NvMediaISCAdvancedConfig::clientContext pointer.
 * \param[in] cfg The configuration to initialize.
 * \param[in] ctx The \a clientContext pointer to be updated.
 */
#define ADV_CONFIG_INIT(cfg, ctx) \
        do { \
            memset(&cfg, 0, sizeof(cfg)); \
            cfg.clientContext = (void *)(ctx); \
        } while (0)

/** \brief Maximum number of exposures. */
#define NVMEDIA_ISC_MAX_EXPOSURES           (8u)

/** \brief Maximum number of color components. */
#define NVM_ISC_MAX_COLOR_COMPONENT         (4u)

/** \brief Maximum number of sensor contexts. */
#define NVMEDIA_ISC_MAX_SENSOR_CONTEXTS     (4u)

/**
 * \brief Maximum number of sensor companding
 *  piecewise linear (PWL) curve knee points.
 */
#define NVMEDIA_ISC_MAX_PWL_KNEEPOINTS      (64u)

/** \brief Maximum number of frame report bytes.  */
#define NVMEDIA_ISC_MAX_FRAME_REPORT_BYTES  (4u)

/** \brief Maximum number of sensor temperature values. */
#define NVMEDIA_ISC_MAX_NUM_TEMPERATURES    (4u)

/** \brief Maximum possible length of sensor name. */
#define NVMEDIA_ISC_MAX_SENSOR_NAME_LENGTH  (32u)

/** \brief Maximum possible length of sensor fuse id. */
#define NVMEDIA_ISC_MAX_FUSE_ID_LENGTH      (32u)

struct NvMediaISCSensorControl;

struct NvMediaISCEmbeddedDataChunk;

struct NvMediaISCEmbeddedDataInfo;

struct NvMediaISCSensorAttributes;

struct NvMediaISCModuleConfig;

/** @} */ /* Ends Basic ISC Types group */

/**
 * \brief  Holds the handle for an NvMediaISCDevice object.
 */
typedef struct {
    void *deviceDriverHandle;
} NvMediaISCDevice;

/**
 * \defgroup version_info_isc_api ISC Version Information
 *
 * Provides version information for the NvMedia ISC library.
 * @{
 */

/**
 * \brief Holds version information for the NvMedia ISC library.
 */
typedef struct {
    /*! Holds library version information. */
    NvMediaVersion libVersion;
} NvMediaISCVersionInfo;

/**
 * \brief Returns the version information for the NvMedia ISC library.
 * \param[in,out] versionInfo   A pointer to an \ref NvMediaISCVersionInfo
 *                               structure that NvMediaISCGetVersionInfo()
 *                               fills with data.
 * \return  A status code; \ref NVMEDIA_STATUS_OK if the operation was
 *  successful, or \ref NVMEDIA_STATUS_BAD_PARAMETER if the @a versionInfo was
 *  invalid.
 */
NvMediaStatus
NvMediaISCGetVersionInfo(
    NvMediaISCVersionInfo *versionInfo
);

/**
 * \brief Gets version compatibility information for the NvMedia ISC library.
 * \param[in] version A pointer to an \ref NvMediaVersion structure
 *                    for the client.
 * \return A status code; \ref NVMEDIA_STATUS_OK if the operation was
 *  successful, or NVMEDIA_STATUS_BAD_PARAMETER if @a version was invalid.
 */
NvMediaStatus
NvMediaISCGetVersion(
    NvMediaVersion *version
);

/**@}*/ /* Ends version_info_isc_api ISC Version Information sub-group. */

/**
 * \defgroup isc_root_device_api ISC Root Device
 *
 * Manage \ref NvMediaISCRootDevice objects, which represent the root of the
 * Nvmedia ISC object system.
 *
 * The NvMediaISCRootDevice object manages an I2C port on the host hardware
 * device.
 * @{
 */

/**
 * \hideinitializer
 * \brief Defines the I2C buses on the host hardware device.
 */
typedef enum {
    NVMEDIA_ISC_I2C_BUS_0 = 0, /**< Specifies i2c-0. */
    NVMEDIA_ISC_I2C_BUS_1 = 1, /**< Specifies i2c-1. */
    NVMEDIA_ISC_I2C_BUS_2 = 2, /**< Specifies i2c-2. */
    NVMEDIA_ISC_I2C_BUS_3 = 3, /**< Specifies i2c-3. */
    NVMEDIA_ISC_I2C_BUS_4 = 4, /**< Specifies i2c-4. */
    NVMEDIA_ISC_I2C_BUS_5 = 5, /**< Specifies i2c-5. */
    NVMEDIA_ISC_I2C_BUS_6 = 6, /**< Specifies i2c-6. */
    NVMEDIA_ISC_I2C_BUS_7 = 7, /**< Specifies i2c-7. */
    NVMEDIA_ISC_I2C_BUS_8 = 8, /**< Specifies i2c-8. */
    NVMEDIA_ISC_I2C_BUS_9 = 9, /**< Specifies i2c-9. */
    NVMEDIA_ISC_I2C_BUS_10 = 10, /**< Specifies i2c-10. */
    NVMEDIA_ISC_I2C_BUS_11 = 11, /**< Specifies i2c-11. */
    NVMEDIA_ISC_I2C_SIMULATOR = 255, /**< Port SIMPULATOR (20) */
} NvMediaISC_I2CPort;

/**
 * \brief  An opaque handle for an NvMediaISCRootDevice object.
 */
typedef void NvMediaISCRootDevice;

/**
 * \brief Creates an \ref NvMediaISCRootDevice object.
 *
 * \param[in] portCfg
 *          The CSI/I2C ports that this root device use. It is strongly
 *          recommended that you use the macro \ref ISC_RDEV_CFG(csi, i2c) or
 *          \ref ISC_RDEV_CFG_EX(csi, i2c, disPwrCtrl) to
 *          generate this value. If the application runs on a slave SoC while
 *          a master SoC controls ISC devices, use ISC_SLV_RDEV_CFG(csi, i2c).
 *          Supported I2C are:
 *          - \ref NVMEDIA_ISC_I2C_BUS_0
 *          - \ref NVMEDIA_ISC_I2C_BUS_1
 *          - \ref NVMEDIA_ISC_I2C_BUS_2
 *          - \ref NVMEDIA_ISC_I2C_BUS_3
 *          - \ref NVMEDIA_ISC_I2C_BUS_4
 *          - \ref NVMEDIA_ISC_I2C_BUS_5
 *          - \ref NVMEDIA_ISC_I2C_BUS_6
 *          - \ref NVMEDIA_ISC_I2C_BUS_7
 *          - \ref NVMEDIA_ISC_I2C_BUS_8
 *          - \ref NVMEDIA_ISC_I2C_BUS_9
 *          - \ref NVMEDIA_ISC_I2C_BUS_10
 *          - \ref NVMEDIA_ISC_I2C_BUS_11
 *          - \ref NVMEDIA_ISC_I2C_SIMULATOR
 * Supported CSI ports are:
 *          - \ref NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_A
 *          - \ref NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_B
 *          - \ref NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_AB
 *          - \ref NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_C
 *          - \ref NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_D
 *          - \ref NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_CD
 *          - \ref NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_E
 *          - \ref NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_F
 *          - \ref NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_EF
 *          - \ref NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_G
 *          - \ref NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_H
 *          - \ref NVMEDIA_IMAGE_CAPTURE_CSI_INTERFACE_TYPE_CSI_GH
 * \return  The new device's handle if successful, or NULL otherwise.
 */
NvMediaISCRootDevice *
NvMediaISCRootDeviceCreate(
    uint32_t portCfg
);

/**
 * \brief Destroys an \ref NvMediaISCRootDevice object.
 * \param[in] device Handle of the device to be destroyed.
 */
void
NvMediaISCRootDeviceDestroy(
    NvMediaISCRootDevice *device
);

/**
 * \brief Waits until an error condition is reported or
 *  NvMediaISCRootDeviceAbortWaitForError() is called.
 * \param[in] device The root device to use.
 * \retval  NVMEDIA_STATUS_OK indicates that the call was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a device was NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicate that some other error occurred.
 */
NvMediaStatus
NvMediaISCRootDeviceWaitForError(
    NvMediaISCRootDevice *device
);

/**
 * \brief Aborts a call to NvMediaISCRootDeviceWaitForError().
 * \param[in] device The root device to use.
 * \retval  NVMEDIA_STATUS_OK to indicate that the operatrion was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER to indicate that @a device was NULL.
 * \retval  NVMEDIA_STATUS_ERROR to indicate that some other error occurred.
 */
NvMediaStatus
NvMediaISCRootDeviceAbortWaitForError(
    NvMediaISCRootDevice *device
);

/**
 * \hideinitializer
 * \brief Defines ISC power items, objects whose power can be turned on or off
 *  and queried with NvMediaISCRootDevicePowerControl() and
 *  NvMediaISCRootDeviceGetPowerStatus().
 */
typedef enum {
    /** Specifies aggregator power. */
    NVMEDIA_ISC_PWR_AGGREGATOR,
    /** Specifies LINK 0 power. */
    NVMEDIA_ISC_PWR_LINK_0,
    /** Specifies LINK 1 power.*/
    NVMEDIA_ISC_PWR_LINK_1,
    /** Specifies LINK 2 power. */
    NVMEDIA_ISC_PWR_LINK_2,
    /** Specifies LINK 3 power. */
    NVMEDIA_ISC_PWR_LINK_3,
} NvMediaISCPowerItems;

/**
 * \brief Sets a power item's power status (on or off).
 * \param[in] device    The root device to use.
 * \param[in] powerItem The power item of @a device whose power status is to be set.
 * \param[in] powerOn   The power status to set.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a device was NULL.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the device driver
 *            does not support this functionality.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISCRootDevicePowerControl(
    NvMediaISCRootDevice *device,
    NvMediaISCPowerItems  powerItem,
    NvMediaBool powerOn
);

/**
 * \brief Gets a power item's power status (on or off).
 * \param[in]  device       The root device to use.
 * \param[in]  powerItem    The power item of @a device for which to get
 *                           the power status.
 * \param[out] powerStatus  A pointer to the power status.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more
 *           pointer parameters were NULL.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the device driver
 *           does not support this functionality.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISCRootDeviceGetPowerStatus(
    NvMediaISCRootDevice *device,
    NvMediaISCPowerItems  powerItem,
    NvMediaBool *powerStatus
);

/**
 * \brief Enables or disables pulse width modulation (PWM) for external
 *  synchronization.
 *
 * \param[in] device    The root device to use.
 * \param[in] enable    The PWM state. Use NVMEDIA_TRUE to enable PWM and
 *                       NVMEDIA_FALSE to disable it.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a device was NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISCRootDeviceEnableSync(
    NvMediaISCRootDevice *device,
    NvMediaBool enable
);

/**
 * \brief Sets the pulse width modulation (PWM) frequency and duty cycle.
 *
 * \param[in] device The root device to use.
 * \param[in] freq PWM frequency, in hertz.
 * \param[in] dutyRatio PWM duty cycle (fraction of time spent high).
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a device was NULL.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISCRootDeviceSetSyncConfig(
    NvMediaISCRootDevice *device,
    float_t freq,
    float_t dutyRatio
);

/**
 * \brief Gets the pulse width modulation (PWM) frequency and duty cycle.
 * \param[in]  device       The root device to use.
 * \param[out] freq         A pointer to PWM frequency.
 * \param[out] dutyRatio    A pointer to PWM duty cycle (fraction of time spent
 *                           high).
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more
 *           pointer parameters were NULL.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the device does not
 *           support this functionality.
 * \retval  NVMEDIA_STATUS_ERROR indicates that some other error occurred.
 */
NvMediaStatus
NvMediaISCRootDeviceGetSyncConfig(
    NvMediaISCRootDevice *device,
    float_t *freq,
    float_t *dutyRatio
);

/*@} <!-- Ends isc_root_device_api ISC Root Device --> */

/**
 * \defgroup isc_device_driver_api ISC Device Driver
 *
 * Program elements related to \ref NvMediaISCDeviceDriver, which defines a
 * device driver.
 * The core NvMediaISC calls the driver when the client calls the related
 * public NvMediaISC function.
 *
 * Before the client can create an NvMediaISCDevice object (a device), it must
 * provide a device driver.
 * The NvMediaISCDeviceDriver object contains the following data fields and
 * function pointers.
 *
 * @par Data Fields
 * \li deviceName The name of the device. This is a null-terminated string.
 * \li regLength  Target device offset length in bytes.
 * \li dataLength Target device data length in bytes.
 *
 * @par Function Pointers
 *  - DriverCreate (mandatory). Invoked when the client calls
 *    NvMediaISCDeviceCreate().
 *  - DriverDestroy (mandatory). Invoked when the client calls
 *    NvMediaISCDeviceDestroy().
 *  - GetModuleConfig (optional). Invoked when the client calls
 *    NvMediaISCGetModuleConfig().
 *  - ParseEmbedDataInfo (optional). Invoked when the client calls
 *    NvMediaISCParseEmbedDataInfo().
 *  - SetSensorControls (optional). Invoked when the client calls
 *    NvMediaISCSetSensorControls().
 *  - GetSensorAttributes (optional). Invoked when the client calls
 *    NvMediaISCGetSensorAttributes().
 *  - SetSensorCharMode (optional). Invoked when the client calls
 *    NvMediaISCSetSensorCharMode().
 *
 * Here is a sample device driver implementation. The source file defines the
 * driver by creating an NvMediaISCDeviceDriver struct and setting
 * its function pointers. The header file provides a function that retrieves
 * a pointer to the driver struct.
 *
 * @par Header File
 *
 * \code
 *
 * #include <nvmedia_isc.h>
 *
 * NvMediaISCDeviceDriver *GetSAMPLEDEVICEDriver(void);
 *
 * \endcode
 *
 * <b>Source File</b>
 * \code
 *
 * #include "isc_sample_device.h"
 *
 * static NvMediaStatus
 * DriverCreate(
 *     NvMediaISCDevice *handle,
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
 * static NvMediaISCDeviceDriver deviceDriver = {
 *     .deviceName = "Sample Sensor Device",
 *     .DriverCreate = DriverCreate
 * };
 *
 * NvMediaISCDeviceDriver *
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

    /**
    * \brief  Holds a pointer to the function that creates a device driver.
    *
    * NvMediaISCDeviceCreate() invokes this function.
    *
    * @par Sample Pseudo Code:
    *
    * \code
    * //Pseudo code for isc device driver function DriverCreate invoked by NvMediaISCDriverCreate function call.
    *  NvMediaStatus
    *  DriverCreate(
    *      NvMediaISCDevice *handle,
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
        NvMediaISCDevice *handle,
        void *clientContext);

    /**
    * \brief  Holds a pointer to the function that destroys the device driver.
    *
    * NvMediaISCDeviceDestroy() invokes this function.
    *
    * @par Sample Pseudo Code:
    *
    * \code
    *  //Pseudo code for isc device driver function DriverDestroy invoked by NvMediaISCDeviceDestroy function call.
    *
    *  NvMediaStatus
    *  DriverDestroy(
    *     NvMediaISCDevice *handle)
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
        NvMediaISCDevice *handle);
    /** Holds a pointer to the function that gets module configuration. */
    NvMediaStatus (* GetModuleConfig)(
        NvMediaISCDevice *handle,
        struct NvMediaISCModuleConfig *moduleConfig);
    /**
     * \brief Holds the function that sets sensor controls.
     *
     * NvMediaISCSetSensorControls() invokes this function.
     *
     * @par Sample Usage:
     *
     * \code
     *  //Pseudo code for isc device driver function SetSensorControls invoked by NvMediaISCSetSensorControls function call.
     *
     *  NvMediaStatus
     *  SetSensorControls(
     *     NvMediaISCDevice *handle,
     *     const NvMediaISCSensorControl *sensorControl,
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
     *     if(sensrCtrlStructSize != sizeof(NvMediaISCSensorControl))
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
        NvMediaISCDevice *handle,
        const struct NvMediaISCSensorControl *sensorControl,
        const size_t sensrCtrlStructSize);

    /**
     * \brief  Holds a pointer to the function that parses embedded data
     *  returned as part of a captured buffer.
     *
     * NvMediaISCParseEmbedDataInfo() invokes this function.
     *
     * @par Sample Usage:
     *
     * \code
     *  //Pseudo code for isc device driver function ParseEmbedDataInfo invoked by \ref NvMediaISCParseEmbedDataInfo function call.
     *
     *  NvMediaStatus
     *  ParseEmbedDataInfo(
     *     NvMediaISCDevice *handle,
     *     const NvMediaISCEmbeddedDataChunk *embeddedTopDataChunk,
     *     const NvMediaISCEmbeddedDataChunk *embeddedBotDataChunk,
     *     const size_t embeddedDataChunkStructSize,
     *     NvMediaISCEmbeddedDataInfo *embeddedDataInfo,
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
     *     if ((embeddedDataChunkStructSize != sizeof(NvMediaISCEmbeddedDataChunk)) ||
     *         (dataInfoStructSize != sizeof(NvMediaISCEmbeddedDataInfo))) {
     *         // Handle version mismatch
     *     }
     *
     *     memset(parsedInfo, 0, sizeof(NvMediaISCEmbeddedDataInfo));
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
        NvMediaISCDevice *handle,
        const struct NvMediaISCEmbeddedDataChunk *embeddedTopDataChunk,
        const struct NvMediaISCEmbeddedDataChunk *embeddedBotDataChunk,
        const size_t dataChunkStructSize,
        struct NvMediaISCEmbeddedDataInfo *embeddedDataInfo,
        const size_t dataInfoStructSize);

    /**
     * \brief  Holds a pointer to the function that puts a sensor in
     *  characterization mode.
     *
     * NvMediaISCSetSensorCharMode() invokes this function.
     *
     * @par Sample Usage:
     *
     * \code
     *  //Pseudo code for isc device driver function SetSensorCharMode invoked by NvMediaISCSetSensorCharMode function call.
     *
     * NvMediaStatus
     * SetSensorCharMode(
     *     NvMediaISCDevice *handle,
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
        NvMediaISCDevice *handle,
        uint8_t expNo);

    /**
     * Holds a pointer to the function that gets sensor attributes.
     *
     * NvMediaISCGetSensorAttributes() invokes this function.
     *
     * @par Sample Usage:
     *
     * \code
     *  //Pseudo code for isc device driver function GetSensorAttributes invoked by NvMediaISCGetSensorAttributes function call.
     *
     *  NvMediaStatus
     *  GetSensorAttributes(
     *     NvMediaISCDevice *handle,
     *     NvMediaISCSensorAttributes *sensorAttr,
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
     *     if(sensorAttrStructSize != sizeof(NvMediaISCSensorAttributes))
     *     {
     *        return NVMEDIA_STATUS_NOT_SUPPORTED;
     *     }
     *
     *     // populate sensorAttr(NvMediaISCSensorAttributes) structure
     *     memset(sensorAttr, 0, sizeof(NvMediaISCSensorAttributes));
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
        NvMediaISCDevice *handle,
        struct NvMediaISCSensorAttributes *sensorAttr,
        const size_t sensorAttrStructSize);

} NvMediaISCDeviceDriver;

/*@} <!-- Ends isc_device_driver_api ISC Device driver --> */

/**
 * \defgroup isc_device_api ISC Device
 *
 * An ISC device represents a device that is attached or linked to the root I2C
 * port.
 *
 * @{
 */

/**
 * Holds a pointer to a description of the target I2C device.
 */
typedef struct {
    /** Holds the client context. */
    void *clientContext;
} NvMediaISCAdvancedConfig;

/**
 * \brief Creates an object that describes a device and returns a handle to
 *  the object.
 * \param[in] rootDevice        A pointer to the root device created with
 *                               NvMediaISCRootDeviceCreate().
 * \param[in] deviceAddressList A pointer to a list of I2C device addresses
 *                               corresponding to this NvMediaISCDevice object.
 * \param[in] numDevices        Number of I2C addresses in @a deviceAddressList.
 * \param[in] deviceDriver      A pointer to a structure that defines the
 *                               behavior of the device.
 * \param[in] advancedConfig    A pointer to a structure that defines advanced
 *                               configuration information about the device.
 * \return device  The new device's handle if successful, or NULL otherwise.
 */
NvMediaISCDevice *
NvMediaISCDeviceCreate(
    NvMediaISCRootDevice *rootDevice,
    uint32_t *deviceAddressList,
    uint32_t numDevices,
    NvMediaISCDeviceDriver *deviceDriver,
    NvMediaISCAdvancedConfig *advancedConfig
);

/**
 * \brief Destroys the object that describes an ISC device.
 * \param[in] device Handle to the device to destroy.
 */
void
NvMediaISCDeviceDestroy(
    NvMediaISCDevice *device
);

/**
 * \brief Performs a read operation over I2C.
 *
 * For safety use cases, application software must perform a read followed by
 * a read back of the same register locations using %NvMediaISCDeviceRead()
 * and verify that the two values match.
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
NvMediaISCDeviceRead(
    NvMediaISCDevice *device,
    uint32_t deviceIndex,
    uint32_t regLength,
    uint8_t *regData,
    uint32_t dataLength,
    uint8_t *data
);

/**
 * \brief Performs a write operation over I2C.
 *
 * For safety use cases, application software must perform a write using
 * %NvMediaISCDeviceWrite() followed by a read back of the same register
 * locations using NvMediaISCDeviceRead() and verify that the write was
 * successful.
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
NvMediaISCDeviceWrite(
    NvMediaISCDevice *device,
    uint32_t deviceIndex,
    uint32_t dataLength,
    const uint8_t *data
);

 /**
 * \hideinitializer
 * \brief Holds the ISC Module ISP configuration.
 */
typedef struct NvMediaISCModuleConfig {
    /** Holds the camera module name. */
    char cameraModuleCfgName[128];
    /**
     * Holds the camera-specific configuration string.
     * \deprecated  ISP can now produce two simultaneous outputs, making this
     *  member unncessary.
     */
    const char *cameraModuleConfigPass1;
    /**
     * Holds the camera-specific configuration string.
     * \deprecated  ISP can now produce two simultaneous outputs, making this
     *  member unncessary.
     */
    const char *cameraModuleConfigPass2;
} NvMediaISCModuleConfig;

/**
 * \brief Gets the module ISP configuration.
 * \param[in] device A pointer to the device to use.
 * \param[out] moduleConfig A pointer to the module ISP configuration.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more
 *           pointer parameters was NULL.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the device driver
 *           does not support this functionality.
 * \retval  NVMEDIA_STATUS_ERROR indicates that any other error occurred.
 */
NvMediaStatus
NvMediaISCGetModuleConfig(
    NvMediaISCDevice *device,
    NvMediaISCModuleConfig *moduleConfig
);

/**
 * \brief  Holds the range of a sensor attribute.
 */
typedef struct NvMediaISCAttrRange {
    /**
     * Holds the sensor attribute's minimum value.
     */
    float_t  min;

    /**
     * Holds the sensor attribute's maximum value.
     */
    float_t  max;

} NvMediaISCAttrRange;

/**
 * \brief  Holds the sensor attributes.
 */
typedef struct NvMediaISCSensorAttributes {
    /**
     * Holds the name attribute. If not supported, set to NULL.
     */
    char  sensorName[NVMEDIA_ISC_MAX_SENSOR_NAME_LENGTH];

    /**
     * Holds the CFA attribute. If not supported, set to zero.
     */
    uint32_t  sensorCFA;

    /**
     * Holds the fuse ID attribute. If not supported, set to NULL.
     */
    char sensorFuseId[NVMEDIA_ISC_MAX_FUSE_ID_LENGTH];

    /**
     * Holds the number of active exposures attribute. Range is
     * [1, \ref NVMEDIA_ISC_MAX_EXPOSURES]
     */
    uint8_t  numActiveExposures;

    /**
     * Holds the sensor exposure ranges for active exposures.
     */
    NvMediaISCAttrRange sensorExpRange[NVMEDIA_ISC_MAX_EXPOSURES];

    /**
     * Holds the sensor gain ranges for active exposures.
     */
    NvMediaISCAttrRange sensorGainRange[NVMEDIA_ISC_MAX_EXPOSURES];

    /**
     * Holds the sensor white balance ranges for active exposures.
     */
    NvMediaISCAttrRange sensorWhiteBalanceRange[NVMEDIA_ISC_MAX_EXPOSURES];

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
    float_t  sensorGainFactor[NVMEDIA_ISC_MAX_EXPOSURES];

    /**
     * Holds the number of frame report bytes supported by the sensor. If not
     * supported, set to zero.
     * Supported values : [1 , NVMEDIA_ISC_MAX_FRAME_REPORT_BYTES]
     */
    uint32_t  numFrameReportBytes;

} NvMediaISCSensorAttributes;

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
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more
 *           pointer parameters was NULL.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the device driver
 *           does not support this functionality.
 * \retval  NVMEDIA_STATUS_ERROR indicates that any other error occurred.
 */
NvMediaStatus
NvMediaISCGetSensorAttributes(
    NvMediaISCDevice *device,
    NvMediaISCSensorAttributes *sensorAttr,
    const size_t sensorAttrStructSize);

/**
 * \brief  Holds sensor exposure information.
 */
typedef struct NvMediaISCExposure {
    /**
     * Holds a flag which enables or disables the exposure block.
     */
    NvMediaBool expTimeValid;

    /**
     * Holds exposure time for each active exposure, in seconds. The array has
     * \ref NvMediaISCSensorAttributes.numActiveExposures elements.
     */
    float_t exposureTime[NVMEDIA_ISC_MAX_EXPOSURES];

    /**
     * Holds a flag which enables or disables the sensor gain block.
     */
    NvMediaBool gainValid;

    /**
     * Holds sensor a gain value for each active gain. The array has
     * \ref NvMediaISCSensorAttributes.numActiveExposures elements.
     */
    float_t sensorGain[NVMEDIA_ISC_MAX_EXPOSURES];

} NvMediaISCExposure;


/**
 * \brief  Holds the sensor white balance gain structure.
 */
typedef struct NvMediaISCWhiteBalance {
    /**
     * Holds a flag which enables or disables the white balance gain block.
     */
    NvMediaBool wbValid;

    /**
     * Holds sensor white balance gain values for active
     * exposures, in R Gr Gb B order.
     *
     * @a wbGain has \ref NvMediaISCSensorAttributes.numActiveExposures
     * elements. Each element is a
     * @a value array, which has \ref NVM_ISC_MAX_COLOR_COMPONENT elements.
     */
    struct {
        float_t value[NVM_ISC_MAX_COLOR_COMPONENT];
    } wbGain[NVMEDIA_ISC_MAX_EXPOSURES];

} NvMediaISCWhiteBalance;

/**
 * \brief  Holds the sensor report frame report structure.
 */
typedef struct NvMediaISCFrameReport {
    /**
     * Holds a flag which enables or disables frame report block.
     */
    NvMediaBool frameReportValid;

    /**
     * Holds the number of active frame report bytes. Range is
     * [1, NVMEDIA_ISC_MAX_FRAME_REPORT_BYTES].
     */
    uint8_t numBytes;

    /**
     * Holds the values of active frame report bytes. Array indexes must be
     * in the range [0, (@a numBytes-1)].
     */
    uint8_t sensorframeReport[NVMEDIA_ISC_MAX_FRAME_REPORT_BYTES];

} NvMediaISCFrameReport;

/**
 * \brief  Holds the sensor companding piecewise linear (PWL) structure.
 */
typedef struct NvMediaISCPWL {
    /**
     * Holds a flag which enables or disables the sensor PWL block.
     */
    NvMediaBool  pwlValid;

    /**
     * Holds the number of active PWL knee points. Must be in the range
     * [1, \ref NVMEDIA_ISC_MAX_PWL_KNEEPOINTS].
     */
    uint8_t numKneePoints;

    /**
     * Holds the values of active PWL knee points. Array indexes must be in the
     * range [0, (@a numKneePoints-1)].
     */
    NvMediaPointDouble kneePoints[NVMEDIA_ISC_MAX_PWL_KNEEPOINTS];

} NvMediaISCPWL;

/**
 * \brief  Holds the sensor temperature structure.
 */
typedef struct NvMediaISCTemperature {
    /**
     * Holds a flag which enables or disables the sensor temperature block.
     */
    NvMediaBool  tempValid;

    /**
     * Holds the number of active temperatures. Must be in the range
     * [1, \ref NVMEDIA_ISC_MAX_NUM_TEMPERATURES].
     */
    uint8_t numTemperatures;

    /**
     * Holds the values of active sensor temperatures in degrees Celsius. Array
     * indexes must be in the range [0, (@a numTemperatures-1)].
     */
    float_t sensorTempCelsius[NVMEDIA_ISC_MAX_NUM_TEMPERATURES];

} NvMediaISCTemperature;

/**
 * \brief  Holds the sensor CRC structure.
 */
typedef struct NvMediaISCCRC {
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

} NvMediaISCCRC;


/**
 * \brief  Holds the sensor frame sequence number structure.
 */
typedef struct NvMediaISCFrameSeqNum {
    /**
     * Holds a flag which enables OR DISABLES the frame sequence number block.
     */
    NvMediaBool  frameSeqNumValid;

    /**
     * Holds the sensor frame sequence number value.
     */
    uint64_t frameSequenceNumber;

} NvMediaISCFrameSeqNum;


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
typedef struct NvMediaISCSensorControl {
    /**
     * Holds the number of sensor contexts to activate.
     * A sensor context is a mode of operation, supported by some sensors,
     * in which multiple set of settings (contexts) are programmed and
     * the sensor toggles between them at run time.
     *
     * Must be in the range [1, NVMEDIA_ISC_MAX_SENSOR_CONTEXTS].
     * For sensors that do support sensor context, set to 1.
     */
    uint8_t                 numSensorContexts;

    /**
     * Holds the sensor exposure settings to set for each context.
     */
    NvMediaISCExposure       exposureControl[NVMEDIA_ISC_MAX_SENSOR_CONTEXTS];

    /**
     * Holds the sensor white balance settings to set for each context.
     */
    NvMediaISCWhiteBalance   wbControl[NVMEDIA_ISC_MAX_SENSOR_CONTEXTS];

    /**
     * Holds the sensor frame report value to be programmed.
     * Sensor frame report control enables you to program
     * custom user information per frame into sensor scratch space (registers)
     * provided by the sensor for private use.
     */
    NvMediaISCFrameReport    frameReportControl;

} NvMediaISCSensorControl;


/**
 * \brief Sets sensor control parameters.
 *
 * This function enables you to control sensor
 * image settings like exposure time, sensor gain, and white balance gain.
 * All parameters provided to this function are applied together at a frame boundary
 * through ISC's "group hold" functionality, if supported by the sensor.
 *
 * \note This function invokes the device driver function specified by the
 *  call to SetSensorControls().
 *
 * \param[in] device  A pointer to the sensor control device in use.
 * \param[in] sensorControl  A pointer to a sensor control structure for @a device.
 * \param[in] sensrCtrlStructSize Size of the @a sensorControl structure.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more
 *           pointer parameters was NULL.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the device driver
 *           does not support this functionality.
 * \retval  NVMEDIA_STATUS_ERROR indicates that any other error occurred.
 */
NvMediaStatus
NvMediaISCSetSensorControls(
    NvMediaISCDevice *device,
    const NvMediaISCSensorControl *sensorControl,
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
typedef struct NvMediaISCEmbeddedDataInfo {

    /**
     * Holds the parsed embedded data frame number of exposures for the
     * captured frame.
     */
    uint32_t                     numExposures;

    /**
     * Holds the parsed embedded data sensor exposure info for the
     * captured frame.
     */
    NvMediaISCExposure           sensorExpInfo;

    /**
     * Holds the parsed embedded data sensor white balance info for the
     * captured frame.
     */
    NvMediaISCWhiteBalance       sensorWBInfo;

    /**
     * Holds the parsed embedded data sensor PWL info for the captured frame.
     */
    NvMediaISCPWL                sensorPWLInfo;

    /**
     * Holds the parsed embedded data sensor CRC info for the captured frame.
     */
    NvMediaISCCRC                sensorCRCInfo;

    /**
     * Holds the parsed embedded data frame report info for the captured frame.
     */
    NvMediaISCFrameReport        sensorReportInfo;

    /**
     * Holds the parsed embedded data sensor temperature info for the
     * captured frame.
     */
    NvMediaISCTemperature        sensorTempInfo;

    /**
     * Holds parsed embedded data frame sequence number info for the
     * captured frame.
     */
    NvMediaISCFrameSeqNum        frameSeqNumInfo;
} NvMediaISCEmbeddedDataInfo;


/*
 * \brief  Holds the sensor embedded data chunk structure.
 */
typedef struct NvMediaISCEmbeddedDataChunk {
    /**
     * Holds the line length of an embedded chunk, in bytes.
     */
    uint32_t lineLength;

    /**
     * Holds a pointer to the data chunk.
     */
    const uint8_t *lineData;
} NvMediaISCEmbeddedDataChunk;


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
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that one or more
 *           pointer parameters was NULL or invalid.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the device driver
 *           does not support this functionality.
 * \retval  NVMEDIA_STATUS_ERROR indicates that any other error occurred.
 */
NvMediaStatus
NvMediaISCParseEmbedDataInfo(
    NvMediaISCDevice *device,
    const NvMediaISCEmbeddedDataChunk *embeddedTopDataChunk,
    const NvMediaISCEmbeddedDataChunk *embeddedBotDataChunk,
    const size_t embeddedDataChunkStructSize,
    NvMediaISCEmbeddedDataInfo *embeddedDataInfo,
    const size_t dataInfoStructSize);

/** \brief  Set sensor to characterization mode.
 *
 * This function configures
 * the sensor for characterization. Sensor characterization provides optimal
 * parameters, corresponding to the sensor's physical and functional
 * characteristics, for image processing.
 *
 * Sensor characterization for High Dynamic Range (HDR) sensors with
 * multiple exposures (T1, T2,... Tn) requires separately characterizing
 * individual exposures for some sensors. This function can configure
 * the sensor to capture each exposure separately, if required by
 * sensor characterization. It changes sensor static attributes like
 * @a numActiveExposures, @a sensorExpRange, and @a sensorGainRange,
 * and so must be called during sensor initialization.
 *
 * To characterize sensor exposure number @a n,
 * where n = {1,2,3, … , N} for an N-exposure HDR sensor, set
 * @a expNo to @a n. For a non-HDR sensor, set @a expNo to 1.
 *
 * \param[in]  device   A pointer to the sensor control device in use.
 * \param[in]  expNo    Sensor exposure number to be used for characterization.
 *                       Must be in the range
 *                       [0, (\ref NVMEDIA_ISC_MAX_EXPOSURES - 1)].
 *                       For a non-HDR sensor, must be set to 1.
 * \retval  NVMEDIA_STATUS_OK indicates that the operation was successful.
 * \retval  NVMEDIA_STATUS_BAD_PARAMETER indicates that @a device was NULL or
 *  invalid.
 * \retval  NVMEDIA_STATUS_NOT_SUPPORTED indicates that the device driver does
 *  not support this functionality.
 * \retval  NVMEDIA_STATUS_ERROR indicates that any other error occurred.
 */
NvMediaStatus
NvMediaISCSetSensorCharMode(
    NvMediaISCDevice *device,
    uint8_t expNo);


/**
 * \brief Defines Exposure mode.
 * \deprecated  Use the NvMediaISCSetSensorCharMode() parameter @a expNo
 *  instead.
 */
typedef enum {
    /**  Specifies long exposure mode. */
    NVMEDIA_ISC_EXPOSURE_MODE_LONG,
    /**  Specifies short exposure mode. */
    NVMEDIA_ISC_EXPOSURE_MODE_SHORT,
    /**  Specifies very short exposure mode. */
    NVMEDIA_ISC_EXPOSURE_MODE_VERY_SHORT,
    /**  Specifies the exposure mode count. */
    NVMEDIA_ISC_EXPOSURE_MODE_MAX
} NvMediaISCExposureMode;

/**
 * \brief  Holds exposure control information.
 * \deprecated  Use \ref NvMediaISCSensorControl instead.
 */
typedef struct NvMediaISCExposureControl {
    /** Holds exposure time for each exposure mode. */
    struct {
        float_t value;
        NvMediaBool valid;
    } exposureTime[NVMEDIA_ISC_EXPOSURE_MODE_MAX];
    /** Holds sensor gain for each exposure mode. */
    struct {
        float_t value;
        NvMediaBool valid;
    } sensorGain[NVMEDIA_ISC_EXPOSURE_MODE_MAX];
    uint8_t sensorFrameId;
} NvMediaISCExposureControl;

/**
 * \brief  Holds the white balance control structure.
 * \deprecated  Use \ref NvMediaISCSensorControl instead.
 */
typedef struct NvMediaISCWBGainControl {
    /** Holds white balance for each exposure mode
        in R Gr Gb B order. */
    struct {
        float_t value[4];
        NvMediaBool valid;
    } wbGain[NVMEDIA_ISC_EXPOSURE_MODE_MAX];
} NvMediaISCWBGainControl;

/**
 * \brief  Holds the embedded data buffer structure.
 * \deprecated  Use the new structure \ref NvMediaISCEmbeddedDataChunk instead.
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
} NvMediaISCEmbeddedDataBuffer;

/**
 * \brief  Holds the embedded data structure.
 * \deprecated  Use the new structure \ref NvMediaISCEmbeddedDataInfo instead.
 */
typedef struct NvMediaISCEmbeddedData {
    NvMediaISCEmbeddedDataBuffer top;
    NvMediaISCEmbeddedDataBuffer bottom;
    /** Holds parsed exposure control. */
    NvMediaISCExposureControl exposureControl;
    /** Holds parsed WB gains in R Gr Gb B order. */
    NvMediaISCWBGainControl wbControl;
    /** Holds parsed frame counter. */
    uint32_t frameSequenceNumber;
} NvMediaISCEmbeddedData;


/**
* \hideinitializer
* \brief Defines ISC sensor attributes.
* \deprecated  Use the new function NvMediaISCGetSensorAttributes() instead.
*/
typedef enum {
    /** Specifies a unique ID per instance of camera module.
      *
      * Data type: char[32] */
    NVMEDIA_ISC_SENSOR_ATTR_FUSE_ID,
    /** Specifies minimum possible gain values.
      *
      * Data type: float_t[NVMEDIA_ISC_EXPOSURE_MODE_MAX] */
    NVMEDIA_ISC_SENSOR_ATTR_GAIN_MIN,
    /** Specifies maximum possible gain values.
      *
      * Data type: float_t[NVMEDIA_ISC_EXPOSURE_MODE_MAX] */
    NVMEDIA_ISC_SENSOR_ATTR_GAIN_MAX,
    /** Specifies minimum possible exposure time values in seconds.
      *
      * Data type: float_t[NVMEDIA_ISC_EXPOSURE_MODE_MAX] */
    NVMEDIA_ISC_SENSOR_ATTR_ET_MIN,
    /** Specifies maximum possible exposure time values in seconds.
      *
      * Data type: float_t[NVMEDIA_ISC_EXPOSURE_MODE_MAX] */
    NVMEDIA_ISC_SENSOR_ATTR_ET_MAX,
    /** Specifies fine integration time values in seconds.
      *
      * Data type: float_t[NVMEDIA_ISC_EXPOSURE_MODE_MAX] */
    NVMEDIA_ISC_SENSOR_ATTR_ET_FINE,
    /** Specifies exposure time step values in seconds.
      *
      * Data type: double[NVMEDIA_ISC_EXPOSURE_MODE_MAX] */
    NVMEDIA_ISC_SENSOR_ATTR_ET_STEP,
    /** Specifies maximum possible HDR ratio value.
      *
      * Data type: uint32_t */
    NVMEDIA_ISC_SENSOR_ATTR_HDR_MAX,
    /** Specifies gain factor between the exposures.
      * If non zero, possible factors are 1, x, 1/x.
      *
      * Data type: float_t */
    NVMEDIA_ISC_SENSOR_ATTR_GAIN_FACTOR,
    /** Specifies sensitivity ratios for different exposures.
      * The ratios are normalized to the maximum sensitivity,
      * so that all ratios are in the range [0.0, 1.0].
      *
      * Data type: float_t[NVMEDIA_ISC_EXPOSURE_MODE_MAX] */ NVMEDIA_ISC_SENSOR_ATTR_QE_PIXEL_RATIO,
    /** Specifies frames per second.
      *
      * Data type: float_t */
    NVMEDIA_ISC_SENSOR_ATTR_FRAME_RATE,
    /** Specifies active number of exposures.
      *
      * Data type: uint32_t */
    NVMEDIA_ISC_SENSOR_ATTR_NUM_EXPOSURES,
} NvMediaISCSensorAttrType;

/*@} <!-- Ends isc_device_api ISC Device --> */
/*
 * \defgroup history_isc History
 * Provides change history for the NvMedia Image Sensor Control API.
 *
 * \section history_isc Version History
 *
 * <b> Version 1.0 </b> April 1, 2014
 * - Initial release
 *
 * <b> Version 1.1 </b> April 3, 2015
 * - Add mode in \ref NvMediaISCEmbeddedData
 *
 * <b> Version 1.2 </b> April 16, 2015
 * - Add new port config macro \ref ISC_ROOT_DEVICE_CFG
 * - The \a advancedConfig attribute is being used in \ref NvMediaISCDeviceCreate
 * - Add new callback function \ref NvMediaISCRootDeviceRegisterCallback to handle device errors
 *
 * <b> Version 1.3 </b> April 24, 2015
 * - Moved the \a regLength and \a dataLength attributes from \ref NvMediaISCAdvancedConfig
 *   in \ref NvMediaISCDeviceDriver.
 * - Add macro \ref ADV_CONFIG_INIT
 *
 * <b> Version 1.4 </b> April 28, 2015
 * - Add hdrRatio in \ref NvMediaISCExposureControl structure
 * - Add conversionGain in \ref NvMediaISCEmbeddedData structure
 *
 * <b> Version 1.5 </b> July 13, 2015
 * - Add exposureMidpointTime in \ref NvMediaISCEmbeddedData structure
 * - Add NVMEDIA_ISC_I2C_SIMULATOR in \ref NVMEDIA_ISC_I2C_SIMULATOR
 * - Add new function \ref NvMediaISCRootDevicePowerControl and \ref NvMediaISCPowerItems
 * - Modify the input parameter cbFunc of NvMediaISCRootDeviceRegisterCallback
 * - Add function \ref NvMediaISCGetErrorStatus to get error information from device
 * - Add \ref NvMediaISCSensorProperties structure
 * - Add \ref NvMediaISCGetSensorProperties to get sensor properties
 *
 * <b> Version 1.6 </b> September 9, 2015
 * - Add SensorMode in \ref NvMediaISCExposureControl structure
 *
 * <b> Version 1.7 </b> September 21, 2015
 * - Increase I2C ports to support 12 I2C busses
 *
 * <b> Version 1.8 </b> December 09, 2015
 * - Add channelGainRatio for sensors which only allow same or fixed ratio gain in exposure channels
 *
 * <b> Version 1.9 </b> January 22, 2016
 * - Add NvMediaISCRootDeviceGetPowerStatus function
 *
 * <b> Version 1.10 </b> May 5, 2016
 * - Add NvMediaISCModuleConfig structure and NvMediaISCGetModuleConfig function.
 *
 * <b> Version 1.11 </b> May 11, 2016
 * - Add \ref NvMediaISCCheckVersion API
 *
 * <b> Version 1.12 </b> June 7, 2016
 * - Add NVMEDIA_ISC_SIMULATOR_ADDRESS to use for ISC simulator devices
 *
 * <b> Version 1.13 </b> June 20, 2016
 * - Add frameSequenceNumber in \ref NvMediaISCEmbeddedData
 *
 * <b> Version 1.14 </b> July 6, 2016
 * - Add ISC_ROOT_DEVICE_CFG_EX macro
 *
 * <b> Version 1.15 </b> July 8, 2016
 * - Add \ref NvMediaISCSetBracketedExposure new API for settings bracketed exposure
 *
 * <b> Version 1.16 </b> Oct 10, 2016
 * - Add sensor attributes \ref NvMediaISCSensorAttrType & new API \ref NvMediaISCGetSensorAttr
 *
 * <b> Version 1.17 </b> November 21, 2016
 * - Add \ref NvMediaISCRootDeviceEnableSync new API for enabling/disabling PWM
 * - Add \ref NvMediaISCRootDeviceSetSyncConfig new API for setting PWM's frequency and duty
 * - Add \ref NvMediaISCRootDeviceGetSyncConfig new API for getting PWM's frequency and duty
 *
 * <b> Version 1.18 </b> Jan 5, 2017
 * - Add \ref NvMediaISCSetCompandingCurve API for setting companding curve
 *
 * <b> Version 1.19 </b> Jan 27, 2017
 * - Add \ref Add const modifier to data parameter in NvmediaISCSupportFuntion Write.
 *
 * <b> Version 2.00 </b> April 19, 2017
 * - Deprecated NvMediaISCDeviceType and EnableLink from \ref NvMediaISCDeviceDriver.
 * - Updated to use stdint.h
 * - Replaced float with float_t
 * - Removed unused input parameter from NvMediaISCRootDeviceCreate
 * - Deprecated queueElementsNumber and queueElementSize from \ref NvMediaISCRootDeviceCreate
 * - Deprecated unused or not supported APIs as below list.
 *     NvMediaISCRootDeviceDoTransaction
 *     NvMediaISCRootDeviceSync
 *     NvMediaISCUpdateDeviceAddress
 *     NvMediaISCSetDeviceLinkMode
 *     NvMediaISCGetLinkStatus
 *     NvMediaISCSetPriority
 * - NvMediaISCGetLinkStatus is replaced by \ref NvMediaISCCheckLink
 * - Updated NvMediaISCCheckLink to have one more argument to specify which link
 *   to enable as optional.
 * - Renamed ISC_ROOT_DEVICE_CFG_EX to ISC_SLV_RDEV_CFG
 * - Renamed ISC_ROOT_DEVICE_CFG to ISC_RDEV_CFG
 * - Deprecated parentDevice, instanceNumber from NvMediaISCDeviceCreate
 * - Deprecated NvMediaISCGetSensorProperties, replaced by \ref NvMediaISCGetSensorAttr
 *
 * <b> Version 2.1 </b> May 12, 2017
 * - Added \ref NvMediaISCGetVersion API to get the version of NvMedia ISC library
 * - NvMediaISCCheckVersion is deprecated. Use NvMediaISCGetVersion() instead
 *
 * <b> Version 2.2 </b> May 23, 2017
 * - Deprecated NvMediaISCSensorMode
 * - Deprecated mode & exposureMidpointTime from \ref NvMediaISCEmbeddedData
 * - Deprecated sensorMode & hdrRatio from \ref NvMediaISCExposureControl
 * - Replaced exposure in \ref NvMediaISCEmbeddedData with \ref NvMediaISCExposureControl
 * - Replaced wbGains in \ref NvMediaISCEmbeddedData with \ref NvMediaISCWBGainControl
 *
 * <b> Version 2.3 </b> May 26, 2017
 * - Add device list for ISC handle creation
 *
 * <b> Version 2.4 </b> Feb 26, 2018
 * - Add ISC_RDEV_CFG_EX to have an option to disable power control in \ref NvMediaISCRootDeviceCreate
 *
 * <b> Version 2.5 </b> April 30, 2018
 * - Add sensor characterization attributes \ref NvMediaISCSensorCharAttr
 * - Add new APIs \ref NvMediaISCSetCharacterizationAttr \ref NvMediaISCGetCharacterizationAttr
 *
 * <b> Version 2.6 </b> September 24, 2018
 * - Added \ref NvMediaISCExposure \ref NvMediaISCWhiteBalance
 *   \ref NvMediaISCCRC \ref NvMediaISCFrameSeqNum \ref NvMediaISCFrameReport
 *   \ref NvMediaISCPWL and \ref NvMediaISCTemperature
 * - Added \ref NvMediaISCSensorControl \ref NvMediaISCEmbeddedDataChunk
 *   \ref NvMediaISCEmbeddedDataInfo and \ref NvMediaISCSensorAttributes
 * - Add new APIs \ref NvMediaISCGetSensorAttributes \ref NvMediaISCParseEmbedDataInfo
 *   \ref NvMediaISCSetSensorControls \ref NvMediaISCSetSensorCharMode
 *
 * <b> Version 2.7 </b> Dec 29, 2018
 * - Add new APIs \r NvMediaISCWaitForError \ref NvMediaISCAbortWaitForError
 *
 * <b> Version 2.8 </b> Feb 25, 2019
 * - Remove NvMediaISCRootDeviceRegisterCallback API
 *
 * <b> Version 2.9 </b> March 25, 2019
 * - Remove NvMediaISCSetExposure, NvMediaISCSetBracketedExposure,
 *   NvMediaISCSetWBGain, NvMediaISCSetCompandingCurve,NvMediaISCGetSensorFrameId,
 *   NvMediaISCGetTemperature, NvMediaISCParseEmbeddedData, NvMediaISCGetSensorAttr,
 *   NvMediaISCSetCharacterizationAttr, NvMediaISCGetCharacterizationAttr
 *
 * <b> Version 3.0 </b> April 3, 2019
 * - Remove NvMediaISCTransactionHandle, NvMediaISCDriverHandle and NvMediaISCSupportFunctions
 * - Change all ISC APIs to use only NvMediaISCDevice handle
 * - Add NvMediaISCDeviceRead and NvMediaISCDeviceWrite
 * - Add support for per device simulator mode
 * - Change lineData member type in \ref NvMediaISCEmbeddedDataChunk to 'const uint8_t *'
 * - Change sensorCFA member type in \ref NvMediaISCSensorAttributes to 'uint32_t'
 * - Remove the following members from NvMediaISCDeviceDriver
 *    CheckPresence
 *    SetDeviceConfig
 *    ReadRegister
 *    WriteRegister
 *    WriteParameters
 *    ReadParameters
 *    DumpRegisters
 *    GetErrorStatus
 *    CheckLink
 * - Remove the following APIs
 *    NvMediaISCCheckPresence
 *    NvMediaISCCheckLink
 *    NvMediaISCSetDefaults
 *    NvMediaISCSetDeviceConfig
 *    NvMediaISCReadParameters
 *    NvMediaISCWriteParameters
 *    NvMediaISCGetErrorStatus
 *    NvMediaISCReadRegister
 *    NvMediaISCWriteRegister
 *    NvMediaISCDumpRegisters
 *
 */

/*@} <!-- Ends nvmedia_isc_api Image Sensor Control --> */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* _NVMEDIA_ISC_H */
