/*
 * Copyright (c) 2017-2019, NVIDIA CORPORATION. All rights reserved. All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */


/**
 * \file
 * \brief <b> NVIDIA Media Interface: Core </b>
 *
 * @b Description: This file contains the Core data types and API.
 */

#ifndef NVMEDIA_CORE_H
#define NVMEDIA_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdint.h>
#include <time.h>

#if !defined(NVM_DEPRECATED)
    #if defined(__GNUC__) && (__GNUC__ >= 4) && (__GNUC_MINOR__ >= 6)
        /*
         * deprecated as build time warnings to prompt developers to migrate
         * from older API to new one gradually. Should be removed once API
         * transition is done(ie: no warnings).
         */

        #pragma GCC diagnostic warning "-Wdeprecated-declarations"
        #define NVM_DEPRECATED_MSG(fmt) __attribute__((deprecated(fmt)))
    #else
        #define NVM_DEPRECATED
        #define NVM_DEPRECATED_MSG(fmt) NVM_DEPRECATED
    #endif
#else
    #define NVM_DEPRECATED_MSG(fmt) NVM_DEPRECATED
#endif

/**
 * \defgroup nvmedia_common_top Common Declarations
 * \ingroup nvmedia_top
 *
 * \brief Defines basic types used for video and images throughout the
 *  NvMedia API.
 */

/**
 * \defgroup basic_api_top Basic NvMedia Types and Structures
 * \ingroup nvmedia_common_top
 *
 * \brief Defines basic types used throughout the NvMedia API.
 * @{
 */

/** \brief Release major version number. */
#define NVMEDIA_RELEASE_VERSION_MAJOR   2
/** \brief Release minor version number. */
#define NVMEDIA_RELEASE_VERSION_MINOR   0

/** \brief Core major version number. */
#define NVMEDIA_CORE_VERSION_MAJOR   1
/** \brief Core minor version number. */
#define NVMEDIA_CORE_VERSION_MINOR   14

/** \hideinitializer \brief A true \ref NvMediaBool value. */
#define NVMEDIA_TRUE  (0 == 0)
/** \hideinitializer \brief A false \ref NvMediaBool value. */
#define NVMEDIA_FALSE (0 == 1)

/**
 * \brief A boolean value, holding \ref NVMEDIA_TRUE or \ref
 * NVMEDIA_FALSE.
 */
typedef uint32_t NvMediaBool;

/**
 * \brief Holds the media time in timespec format as defined by the POSIX specification.
 */
typedef struct timespec NvMediaTime;

/**
 * \brief Media global time, measured in microseconds.
 */
typedef uint64_t NvMediaGlobalTime;

/** \brief Defines clock base for NvMediaTime.
 */
typedef enum {
    /** \hideinitializer \brief Specifies that PTP clock is used
     for base time calculation. */
    NVMEDIA_TIME_BASE_CLOCK_PTP = 0,
    /** \hideinitializer \brief Specifies that kernel monotonic clock is used
     for base time calculation. */
    NVMEDIA_TIME_BASE_CLOCK_MONOTONIC,
    /** \hideinitializer \brief Specifies that a user defined clock is used
     for base time calculation. */
    NVMEDIA_TIME_BASE_CLOCK_USER_DEFINED,
} NvMediaTimeBase;

/** \brief Defines color standards.
 */
typedef enum {
    /** \hideinitializer \brief Specifies ITU BT.601 color standard. */
    NVMEDIA_COLOR_STANDARD_ITUR_BT_601,
    /** \hideinitializer \brief Specifies ITU BT.709 color standard. */
    NVMEDIA_COLOR_STANDARD_ITUR_BT_709,
    /** \hideinitializer \brief Specifies SMTE 240M color standard. */
    NVMEDIA_COLOR_STANDARD_SMPTE_240M,
    /** \hideinitializer \brief Specifies ITU BT.601 color standard extended
     range. */
    NVMEDIA_COLOR_STANDARD_ITUR_BT_601_ER,
    /** \hideinitializer \brief Specifies ITU BT.709 color standard extended
     range. */
    NVMEDIA_COLOR_STANDARD_ITUR_BT_709_ER
} NvMediaColorStandard;

/**
 * \brief Holds a rectangular region of a surface.
 *
 * The co-ordinates are top-left inclusive, bottom-right
 * exclusive.
 *
 * The NvMedia co-ordinate system has its origin at the top left corner
 * of a surface, with x and y components increasing right and
 * down.
 */
typedef struct {
    /*! Left X co-ordinate. Inclusive. */
    uint16_t x0;
    /*! Top Y co-ordinate. Inclusive. */
    uint16_t y0;
    /*! Right X co-ordinate. Exclusive. */
    uint16_t x1;
    /*! Bottom Y co-ordinate. Exclusive. */
    uint16_t y1;
} NvMediaRect;

/**
 * \brief Defines the location of a point on a two-dimensional object.
 */
typedef struct {
    /*! Holds the horizontal location of the point. */
    int32_t x;
    /*! Holds the vertical location of the point. */
    int32_t y;
} NvMediaPoint;

/**
 * \brief Defines the float-precision location of a point on a two-dimensional
 *  object.
 */
typedef struct {
    /*! Holds the horizontal location of the point. */
    float_t x;
    /*! Holds the vertical location of the point. */
    float_t y;
} NvMediaPointFloat;

/**
 * \brief Defines the double-precision location of a point on a two-dimensional
 *  object.
 */
typedef struct {
    /*! Holds the horizontal location of the point. */
    double_t x;
    /*! Holds the vertical location of the point. */
    double_t y;
} NvMediaPointDouble;

/**
 * \hideinitializer
 * \brief Defines all possible error codes.
 */
typedef enum {
    /** Specifies that the operation completed successfully
     (with no error). */
    NVMEDIA_STATUS_OK = 0,
    /** Specifies that a bad parameter was passed. */
    NVMEDIA_STATUS_BAD_PARAMETER = 1,
    /** Specifies that the operation has not finished yet. */
    NVMEDIA_STATUS_PENDING = 2,
    /** Specifies that the operation timed out. */
    NVMEDIA_STATUS_TIMED_OUT = 3,
    /** Specifies that the process is out of memory. */
    NVMEDIA_STATUS_OUT_OF_MEMORY = 4,
    /** Specifies that a component requred by the function call is not
     initialized. */
    NVMEDIA_STATUS_NOT_INITIALIZED = 5,
    /** Specifies that the requested operation is not supported. */
    NVMEDIA_STATUS_NOT_SUPPORTED = 6,
    /** Specifies a catch-all error, used when no other error code applies. */
    NVMEDIA_STATUS_ERROR = 7,
    /** Specifies that no operation is pending. */
    NVMEDIA_STATUS_NONE_PENDING = 8,
    /** Specifies insufficient buffering. */
    NVMEDIA_STATUS_INSUFFICIENT_BUFFERING = 9,
    /** Specifies that the size of an object passed to a function was
     invalid. */
    NVMEDIA_STATUS_INVALID_SIZE = 10,
    /** Specifies that a library's version is incompatible with the
     application. */
    NVMEDIA_STATUS_INCOMPATIBLE_VERSION = 11,
    /** Specifies that the operation entered an undefined state. */
    NVMEDIA_STATUS_UNDEFINED_STATE = 13,
    /** Specifies an error from Permanent Fault Software Diagnostic. */
    NVMEDIA_STATUS_PFSD_ERROR = 14,
} NvMediaStatus;

/**
 * \brief Holds status of latest operation for NvMedia managed data structure.
 *
 * Data structures include images and arrays.
 */
typedef struct {
    /*! Holds actual status - \ref NvMediaStatus. */
    NvMediaStatus status;
    /*! Holds timestamp of end of operation. */
    uint64_t endTimestamp;
} NvMediaTaskStatus;

/**
 * \defgroup version_info_api Version Information
 *
 * Provides version information for the NvMedia library.
 * @{
 */

/**
 * Holds NvMedia version information.
 */
typedef struct {
    /*! Holds NvMedia major version number. */
    uint8_t major;
    /*! Holds NvMedia minor version number.*/
    uint8_t minor;
} NvMediaVersion;

/*
 ******* Definitions ******************
 * SOFFence - Start of frame \ref NvSciSyncFence. An NvSciSyncFence
 *            whose expiry indicates that the processing has started.
 * EOFFence - End of frame NvSciSyncFence. An NvSciSyncFence
 *            whose expiry indicates that the processing is done.
 * PREFence - An NvSciSyncFence on which the start of processing is
 *            blocked until the expiry of the fence.
 **************************************
 */

/**
 * \brief NvMedia NvSciSync Client Type
 */
typedef enum {
    /** An NvMedia component acts as a signaler. */
    NVMEDIA_SIGNALER,
    /** An NvMedia component acts as a waiter. */
    NVMEDIA_WAITER,
    /** An NvMedia component acts as a signaler and waiter also for the same
     \ref NvSciSyncObj. */
    NVMEDIA_SIGNALER_WAITER
} NvMediaNvSciSyncClientType;

/**
 * \brief Defines NvMedia \ref NvSciSyncObj types.
 */
typedef enum {
    /** Specifies an NvSciSyncObj type for which an NvMedia component acts
     as a waiter. */
    NVMEDIA_PRESYNCOBJ,
    /** Specifies an NvSciSyncObj type for which an NvMedia component acts
     as a signaler, signaling EOFFence. */
    NVMEDIA_EOFSYNCOBJ,
    /** Specifies an NvSciSyncObj type for which an NvMedia component acts
     as a signaler, signaling SOFFence. */
    NVMEDIA_SOFSYNCOBJ,
    /** Specifies an NvSciSyncObj type for which an NvMedia component acts
     both as a signaler, signaling EOFFence, and as a waiter.
       Use this type in use cases where a EOFfence from an NvMedia component
       handle in one iteration is used as a PREfence for the same
       handle in the next iteration. */
    NVMEDIA_EOF_PRESYNCOBJ,
    /** Specifies an NvSciSyncObj type for which an NvMedia component acts
     as a signaler, signaling SOFFence, as a waiter.
       Use this type in use cases where a SOFfence from an NvMedia component
       handle in one iteration is used as a PREfence for the same
       handle in the next iteration. */
    NVMEDIA_SOF_PRESYNCOBJ

} NvMediaNvSciSyncObjType;

/*
 * \brief Defines all possible access modes.
 */
typedef enum {
    /** Specifies read-only access mode. */
    NVMEDIA_ACCESS_MODE_READ,
    /** Specifies read/write access mode. */
    NVMEDIA_ACCESS_MODE_READ_WRITE,
} NvMediaAccessMode;

/**
 * @brief Gets the release version information for the NvMedia library.
 * @param[in, out] version A valid non-NULL pointer to a \ref NvMediaVersion structure
 *                      to be filled by the function.
 * @retval NVMEDIA_STATUS_OK if the operation was successful
 * @retval NVMEDIA_STATUS_BAD_PARAMETER if @a version was invalid.
 */
NvMediaStatus
NvMediaReleaseGetVersion(
    NvMediaVersion *version
);

/**
 * \brief Gets the core version information for the NvMedia library.
 * \param[in, out] version A valid non-NULL pointer to a \ref NvMediaVersion structure
 *                      to be filled by the function.
 * @retval NVMEDIA_STATUS_OK if the operation was successful
 * @retval NVMEDIA_STATUS_BAD_PARAMETER if @a version was invalid.
 */
NvMediaStatus
NvMediaCoreGetVersion(
    NvMediaVersion *version
);

/** @} <!-- Ends version_info_api Version Information sub-group -> */

/**
 * \defgroup device_api Device
 *
 * Manages \ref NvMediaDevice objects, which are the root of the Nvmedia object
 * system.
 *
 * A caller can use an
 * NvMediaDevice object to create objects of all other types. See
 * the sections describing the other object types for details
 * on object creation.
 * @{
 */

/**
 * \brief  An opaque handle representing an NvMediaDevice object.
 */
typedef struct NvMediaDevice NvMediaDevice;

/**
 * @brief Creates an instance of the NvMediaDevice structure
 *
 * Memory will be allocated for a NvMediaDevice structure and private SOC specific NvMedia
 * information will be saved in the created NvMediaDevice.
 *
 * @retval NvMediaDevice Valid non-NULL pointer to the new device if successful
 * @retval NULL incase of failure
 */
NvMediaDevice *
NvMediaDeviceCreate(
    void
);

/**
 * \brief Destroys an NvMediaDevice instance.
 *
 * Memory allocated for NvMediaDevice using \ref NvMediaDeviceCreate will be freed and the
 * instance of NvMediaDevice structure will be destroyed.
 *
 * \param[in] device A valid non-NULL pointer to the device to be destroyed.
 */
void
NvMediaDeviceDestroy(
    NvMediaDevice *device
);

/** @} <!-- Ends device_api Device sub-group -> */

/** @} <!-- Ends basic_api_top group Basic NvMedia Types and Structures --> */

/*
 * \defgroup history_nvmedia_core History
 * Provides change history for the NvMedia Common Types.
 *
 * \section history_nvmedia_core Version History
 *
 * <b> Version 1.0 </b> March 21, 2017
 * - Initial release
 *
 * <b> Version 1.1 </b> April 18, 2017
 * - NVMEDIA_VERSION_MAJOR is renamed to NVMEDIA_CORE_VERSION_MAJOR.
 * - NVMEDIA_VERSION_MINOR is renamed to NVMEDIA_CORE_VERSION_MINOR.
 * - NvMediaBool is now changed from "int" to "uint32_t" type.
 * - NvMediaRect is now changed from "unsigned short" to "uint16_t".
 * - All NvMedia data types are moved to standard data types from <stdint.h>
 * - NvMediaVersionInfo is now deprecated. Use module specific GetVersion()
     API to query the module versions.
 * - NvMediaGetVersionInfo is now deprecated. Use NvMediaCoreGetVersion()
     or NvMediaReleaseGetVersion()
 * - NvMediaCheckVersion is now deprecated. Applications are expected
     to check their version using above GetVersion() APIs.
 * - NVMEDIA_SET_VERSION macro is now deprecated.
 *
 * <b> Version 1.2 </b> May 4, 2017
 * - Added \ref NvMediaROI and \ref NvMediaTaskStatus.
 *
 * <b> Version 1.3 </b> May 17, 2017
 * - Added macros to generate build warnings for deprecated APIs
 * - Changed the size of \ref NvMediaPoint members
 *
 * <b> Version 1.4 </b> September 14, 2017
 * - Added \ref NvMediaTimeBase
 *
 * <b> Version 1.5 </b> December 12, 2017
 * - Deprecated the following palette related APIs:
 *   NvMediaPaletteCreate
 *   NvMediaPaletteDestroy
 *   NvMediaPaletteLoad
 *
 * <b> Version 1.6 </b> Sep 12, 2018
 * - Add \ref NvMediaPointDouble
 *
 * <b> Version 1.7 </b> Dec 12, 2018
 * - MISRA Rule 21.1 and 21.2 violations are fixed
 *
 * <b> Version 1.8 </b> Feb 4, 2019
 * - Removed \ref NvMediaROI
 * - Moved \ref NvMediaColor to nvmedia_vmp.h
 * - Changed \ref NvMediaDevice type from void to struct
 *
 * <b> Version 1.9 </b> March 7, 2019
 * - Added \ref NvMediaNvSciSyncClientType and \ref NvMediaNvSciSyncObjType.
 *
 * <b> Version 1.10 </b> March 18, 2019
 * - Added /ref NvMediaAccessMode
 *
 * <b> Version 1.11 </b> March 25, 2019
 * - Removed enum value NVMEDIA_STATUS_CANCELLED from \ref NvMediaStatus
 *
 * <b> Version 1.12 </b> April 2, 2019
 * - Added enum value NVMEDIA_STATUS_PFSD_ERROR to \ref NvMediaStatus
 *
 * <b> Version 1.13 </b> August 1, 2019
 * - Added manual enumeration for backward compatibility to \ref NvMediaStatus
 *
 * <b> Version 1.14 </b> November 06, 2019
 * - Add \ref NvMediaPointFloat
 *
 */

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_CORE_H */
