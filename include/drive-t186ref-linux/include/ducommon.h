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
 * @brief Common definitions and declarations for NVIDIA DRIVE™ Update.
 */

#ifndef DUCOMMON_H_
#define DUCOMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <assert.h>

/*!
 * @defgroup du_common_group Common Definitions
 *
 * Common definitions and declarations for NVIDIA DRIVE Update
 *
 * @ingroup drive_update_top
 * @{
 */

#define DU_UID_COMMON       (0U)
/*! Defines the transport unit id. */
#define DU_UID_TRANSPORT    (1U)
/*! Defines the link unit id. */
#define DU_UID_LINK         (2U)
/*! Defines the installer unit id. */
#define DU_UID_INSTALLER    (3U)
/*! Defines the client unit id. */
#define DU_UID_CLIENT       (4U)
/*! Defines the update unit id. */
#define DU_UID_NVUPDATE     (5U)
#define DU_UID_MASTER       (6U)

/*! Place-holder for cxx plugin unit ids (1xx) */
#define DU_UID_COMPAT       (101U)

/*!
 * Drive Update error codes.
 *
 * The error code type is defined as an unsigned 32-bit integer. There are two
 * classes of error codes: common codes and unit specific codes. The common
 * error codes can be applied to multiple units, while the unit specific error
 * codes is applied to one certain unit only.
 */
typedef uint32_t DU_RCODE;

/*!
 * Helper macro to define a unique error code, in which @c uid is the unit id,
 * @c suid is the sub-unit id and @c err is an integer code.
 */
#define DU_ECODE(uid, suid, err) duErrCode(uid, suid, err)

/*! Generates a unique error code. */
static inline DU_RCODE duErrCode(uint32_t uid, uint32_t suid, uint32_t err)
{
    return (DU_RCODE)((((uid) & (uint32_t) (0x000000FF)) << (uint32_t) 24U) | \
           (((suid) & (uint32_t) (0x0000000F)) << (uint32_t) 20U) | \
           ((err)   & (uint32_t) (0x000FFFFF)));
}

/*! Gets the unit id. */
#define DU_ECODE_GET_UID(rcode)  (((rcode) >> 24U) & 0xFF)
/*! Gets the sub-unit. */
#define DU_ECODE_GET_SUID(rcode) (((rcode) >> 20U) & 0xF)
/*! Gets the integer error code. */
#define DU_ECODE_GET_ERR(rcode)  ((rcode) & 0xFFFFF)

/*! \brief Defines the success code. */
#define DU_OK                 (0U)

// Common error codes used by shared components only
#define DUCOMMON_ECODE(err)            DU_ECODE(DU_UID_COMMON, 0, (err))

#define DUCOMMON_ERR_GENERIC           DUCOMMON_ECODE(1)
#define DUCOMMON_ERR_INVALID_ARGUMENT  DUCOMMON_ECODE(2)
#define DUCOMMON_ERR_NOT_FOUND         DUCOMMON_ECODE(3)
#define DUCOMMON_ERR_BUFFER_TOO_SMALL  DUCOMMON_ECODE(4)
#define DUCOMMON_ERR_SYNTAX            DUCOMMON_ECODE(5)

/*! \brief Defines the default long string buffer size. */
#define DU_STR_LONG_BUF_SIZE  (128U)

/*! \brief Defines the default short string buffer size. */
#define DU_STR_SHORT_BUF_SIZE (32U)

/*! \brief Defines the max path size. */
#define DU_PATH_MAX           (512U)

/*! \brief Defines the array size. */
#define DU_ARRAY_SIZE(x)      (sizeof(x)/sizeof((x)[0]))

/*! \brief Defines the buffer size of 1KB. */
#define DU_1KB   (1024U)
/*! \brief Defines the buffer size of 2KB. */
#define DU_2KB   (2U * DU_1KB)
/*! \brief Defines the buffer size of 4KB. */
#define DU_4KB   (4U * DU_1KB)
/*! \brief Defines the buffer size of 8KB. */
#define DU_8KB   (8U * DU_1KB)
/*! \brief Defines the buffer size of 16KB. */
#define DU_16KB  (16U * DU_1KB)
/*! \brief Defines the buffer size of 32KB. */
#define DU_32KB  (32U * DU_1KB)
/*! \brief Defines the buffer size of 64KB. */
#define DU_64KB  (64U * DU_1KB)
/*! \brief Defines the buffer size of 128KB. */
#define DU_128KB (128U * DU_1KB)
/*! \brief Defines the buffer size of 256KB. */
#define DU_256KB (256U * DU_1KB)
/*! \brief Defines the buffer size of 512KB. */
#define DU_512KB (512U * DU_1KB)
/*! \brief Defines the buffer size of 1MB. */
#define DU_1MB   (1024U * DU_1KB)

/*! \brief Defines a macro for asserting failures. */
#define DU_CT_ASSERT(x)       _Static_assert(x, "DU_CT_ASSERT failed")
/*! \brief Defines a macro for asserts. */
#ifdef NDEBUG
#define DU_ASSERT(x)
#else
#define DU_ASSERT(x)          assert(x)
#endif

/*! \brief Defines the run level. */
typedef enum DU_RUN_LEVEL
{
    /*! \brief Dormant run level (default). */
    RL_DORMANT = 0U,
    /*! \brief Monitor run level. */
    RL_MONITOR,
    /*! \brief Telecast run level. */
    RL_TELECAST,
    /*! \brief Transfer run level. */
    RL_XFER,
    /*! \brief Background apply run level. */
    RL_BG_APPLY,
    /*! \brief Full apply run level. */
    RL_FULL_APPLY,
    /*! \brief Unrestricted run level. */
    RL_UNRESTRICTED,
    /*! \brief Invalid run level. */
    RL_INVALID,
} DU_RUN_LEVEL;

/*! \brief Defines the max size of the run level string. */
#define DU_RUNLEVEL_STR_MAX_SIZE      (16U)
/** @} */

#ifdef __cplusplus
}
#endif

#endif // DUCOMMON_H_
