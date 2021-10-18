/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

/*!
 * @file  dubhc_plugin.h
 * @brief Data structures and definitions for Dubhc plugin
 */

#ifndef DUBHC_SERVER_H_
#define DUBHC_SERVER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------ Drive Update Includes --------------------------- */
#include "ducommon.h"
#include "dulink.h"
#include "duplugin.h"

/* ------------------------ System Includes --------------------------------- */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

/* ------------------------ Defines ----------------------------------------- */
// Used to init DU Link connection
#define DUBHC_PLUGIN_NAME           "bhc"
#define DUBHC_PLUGIN_DUTR_TR_TYPE   (DUTR_TR_TYPE_TCP)
#define DUBHC_PLUGIN_DUTR_SEC_TYPE  (DUTR_SEC_TYPE_NONE)
#define DUBHC_PLUGIN_CONN_TYPE      (DULINK_CONNECT_UPLINK)
#define DUBHC_PLUGIN_REMOTE_PORT    (5008)
#define DUBHC_PLUGIN_LOCAL_PORT     (0)
#define DUBHC_PLUGIN_REMOTE_IP      "127.0.0.1"
#define DUBHC_PLUGIN_LOCAL_IP       "127.0.0.1"

// Path to DU Master used for registration
#define DUMASTER_PATH                 "/master"

#define NODE_RESULT                   ("result")

#define NOTIFY_NODE_RESULT            ("result.notify")

#define MAX_CMD_LEN                   (512)
#define MAX_CMD_TOKENS                (20)

#define DUBHC_ERR_GENERIC           (1)
#define DUBHC_ERR_INVALID_ARGUMENT  (2)
#define DUBHC_ERR_NOT_FOUND         (3)
#define DUBHC_ERR_BUFFER_TOO_SMALL  (4)

// Possible results
enum DUBHC_RESULT
{
    RESULT_UNKNOWN = 0U,
    RESULT_PASS,
    RESULT_FAIL,
    RESULT_MAX
};

/* ------------------------  Data Structures -------------------------------- */
typedef struct DU_BHC_PLUGIN DU_BHC_PLUGIN, *PDU_BHC_PLUGIN;

struct DU_BHC_PLUGIN
{
    /// Mutex lock for shared resourses
    pthread_mutex_t mutex;
    /// Current command
    char cmd[MAX_CMD_LEN];
    /// Current state
    uint8_t state;
    char stateStr[DU_STR_SHORT_BUF_SIZE];
    /// Result
    uint8_t result;
    char resultStr[DU_STR_SHORT_BUF_SIZE];
    /// Current runlevel
    uint8_t currentRl;
    /// Pending runlevel
    uint8_t pendingRl;
    /// Most recently requested rl
    uint8_t requestedRl;
};

/// Global sample plugin object;
extern DU_BHC_PLUGIN dubhc_plugin;

/* ------------------------ Callbacks --------------------------------------- */
DU_RCODE requestedRLCB
(
    const char          *pRequestPath,
    const char          *pOriginPath,
    void                *pCtx,
    uint64_t             offset,
    uint64_t             length,
    void                *pBuf,
    DULINK_CB_OPERATION  operation,
    uint64_t            *pRetVal
);

DU_RCODE cmdCB
(
    const char          *pRequestPath,
    const char          *pOriginPath,
    void                *pCtx,
    uint64_t             offset,
    uint64_t             length,
    void                *pBuf,
    DULINK_CB_OPERATION  operation,
    uint64_t            *pRetVal
);

/* ------------------------ Public Functions -------------------------------- */
/*!
 * Set du-bhc plugin result to result. Mutex should be held before calling
 * this function
 *
 * @param[in,out] pPlugin
 *      Pointer to dubhc plugin data structure
 *
 * @param[in] result
 *      Result to be set
 */
void dubhcPluginSetResult
(
    PDU_BHC_PLUGIN pPlugin,
    uint8_t        result
);

/*!
 * Initialize dubhc plugin data structures and export DU Link Nodes
 *
 * @param[in] pPlugin
 *      Pointer to uninitialized bhc plugin data structure
 *
 * @return DU_OK
 *      Initialized successfully
 * @return BHC_ERR_GENERIC
 *      Failed to initialize
 */
DU_RCODE dubhcPluginInit
(
    PDU_BHC_PLUGIN pPlugin
);

/*!
 * Main loop for dubhc plugin
 *
 * @param[in] pPlugin
 *      Pointer to dubhc plugin data structure
 *
 * @return DU_OK
 *      Exiting normally
 * @return BHC_ERR_GENERIC
 *      Error happened
 */
DU_RCODE dubhcPluginRun
(
    PDU_BHC_PLUGIN pPlugin
);

/*!
 * Run Boot Health tests
 *
 * @param[in] pPlugin
 *          Pointer to dubhc plugin data structure
 *
 * @return void
 *
 */
void BootHealthCheck
(
    PDU_BHC_PLUGIN pPlugin
);

/*!
 * lock mutex lock and assert no error has occurred
 *
 * @param[in] pMutex
 *          pointer to mutex to lock
 *
 * @return void
 *
 */
static inline void
duMutexLock
(
    pthread_mutex_t *pMutex
)
{
    int ret = pthread_mutex_lock(pMutex);
    if (ret != 0)
    {
        fprintf(stderr, "Mutex lock failed, (%s)\n", strerror(ret));
        DU_ASSERT(0);
    }
}

/*!
 * Unlock mutex lock and assert no error has occurred
 *
 * @param[in] pMutex
 *          Pointer to mutex to unlock
 *
 * @return void
 *
 */
static inline void
duMutexUnlock
(
    pthread_mutex_t *pMutex
)
{
    int ret = pthread_mutex_unlock(pMutex);
    if (ret != 0)
    {
        fprintf(stderr, "Mutex unlock failed, (%s)\n", strerror(ret));
        DU_ASSERT(0);
    }
}
/*!
 * @brief Convert a run level string to a run level value
 *
 * @param[in] pRunlevelStr: run level string
 *
 * @return value of run level
 *          Success
 *
 * @return RL_INVALID
 *          Fail
 */
static inline DU_RUN_LEVEL
runlevelStrToUint
(
    char *pRunlevelStr
)
{
    uint32_t i;
    char runLevelStr[][DU_RUNLEVEL_STR_MAX_SIZE] =
        {
            "RL_DORMANT",
            "RL_MONITOR",
            "RL_TELECAST",
            "RL_XFER",
            "RL_BG_APPLY",
            "RL_FULL_APPLY",
            "RL_UNRESTRICTED"
        };

    if (pRunlevelStr == NULL)
    {
        return RL_INVALID;
    }

    for (i = 0; i <= RL_UNRESTRICTED; i++)
    {
        if (strcmp(pRunlevelStr, runLevelStr[i]) == 0)
        {
            return i;
        }
    }

    return RL_INVALID;
}
/*!
 * @brief Convert a run level value to a run level str
 *
 * @param[in] runlevel: run level value
 *
 * @param[out] pBuf: buf to get run level string
 *
 * @param[in] bufSize: size of buf
 *
 * @return the actually size of run level string
 *          Success
 *
 * @return 0
 *          Fail
 */
static inline uint64_t
runlevelUintToStr
(
    DU_RUN_LEVEL runlevel,
    char        *pBuf,
    uint64_t     bufSize
)
{
    char runLevelStr[][DU_RUNLEVEL_STR_MAX_SIZE] =
        {
            "RL_DORMANT",
            "RL_MONITOR",
            "RL_TELECAST",
            "RL_XFER",
            "RL_BG_APPLY",
            "RL_FULL_APPLY",
            "RL_UNRESTRICTED"
        };

    if (pBuf == NULL || bufSize < DU_RUNLEVEL_STR_MAX_SIZE ||
        runlevel >= DU_ARRAY_SIZE(runLevelStr))
    {
        return 0;
    }

    memset(pBuf, 0, bufSize);
    memcpy(pBuf, runLevelStr[runlevel], DU_RUNLEVEL_STR_MAX_SIZE);

    return strlen(pBuf) + 1;
}

#ifdef __cplusplus
}
#endif
#endif
