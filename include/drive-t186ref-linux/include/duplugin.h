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
 * @file  duplugin.h
 * @brief Defines valid DUV3 plugin types and their states, also some shared
 * common helpers
 */

#ifndef DUPLUGIN_H_
#define DUPLUGIN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "dulink.h"
#include <pthread.h>

/* ------------------------ Defines ----------------------------------------- */

// Path to DU Master used for registration
#define DUMASTER_PATH                 "/master"

/// Defines for all valid plugin types
#define PLUGIN_TYPE_INSTALLER         "installer"
#define PLUGIN_TYPE_CONTROLLER        "controller"
#define PLUGIN_TYPE_CONTENT_PROVIDER  "content_provider"
#define PLUGIN_TYPE_FILE_TRANSFORM    "file_transform"
#define PLUGIN_TYPE_VALIDATOR         "validator"
#define PLUGIN_TYPE_METADATA_PROVIDER "metadata_provider"
#define PLUGIN_TYPE_PERS_CTX_STORE    "persistent_ctx_store"

/// File names to be exported in by all plugins
#define NODE_PLUGIN_TYPE              "plugin-type"
#define NODE_REQUESTED_RL             "requested_rl"
#define NODE_CURRENT_RL               "current_rl"
#define NODE_STATE                    "state"
#define NODE_STATE_LIST               "state.list"
#define NODE_PENDING_RL               "pending_rl"

/// File names exported by > 1 but not all plugin types
#define NODE_PROGRESS                 "progress"
#define NODE_CMD                      "cmd"
#define NODE_CMD_LIST                 "cmd.list"


/// Notify Nodes names to be exported in by all plugins
#define NOTIFY_NODE_CURRENT_RL        "current_rl.notify"
#define NOTIFY_NODE_STATE             "state.notify"
#define NOTIFY_NODE_PENDING_RL        "pending_rl.notify"

/// Notify Node names exported by > 1 but not all plugin types
#define NOTIFY_NODE_PROGRESS          "progress.notify"

/// Read only file attribute
#define READ_ONLY_ATTR { \
    .type = DULINK_NODE_TYPE_FILE, \
    .numACL = 1, \
    .acl = \
    { \
        [0].srcElement = "/*", \
        [0].allowMask = DULINK_PERMISSION_READ, \
        [0].denyMask = DULINK_PERMISSION_WRITE \
    } \
}

/// Read only dir
#define READ_ONLY_DIR_ATTR { \
    .type = DULINK_NODE_TYPE_DIRECTORY, \
    .numACL = 1, \
    .acl = \
    { \
        [0].srcElement = "/*", \
        [0].allowMask = DULINK_PERMISSION_READ, \
        [0].denyMask = DULINK_PERMISSION_WRITE \
    } \
}

/// Read/write allowed file
#define READ_WRITE_ATTRIBUTE { \
    .type = DULINK_NODE_TYPE_FILE, \
    .numACL = 1, \
    .acl = \
    { \
        [0].srcElement = "/*", \
        [0].allowMask = DULINK_PERMISSION_WRITE | DULINK_PERMISSION_READ, \
        [0].denyMask = DULINK_PERMISSION_NONE \
    } \
}

/// Write only file
#define WRITE_ONLY_ATTRIBUTE { \
    .type = DULINK_NODE_TYPE_FILE, \
    .numACL = 1, \
    .acl = \
    { \
        [0].srcElement = "/*", \
        [0].allowMask = DULINK_PERMISSION_WRITE, \
        [0].denyMask = DULINK_PERMISSION_READ \
    } \
}

/// Master can read/write, all others have no access
#define MASTER_ONLY_RW_ATTRIBUTE { \
    .type = DULINK_NODE_TYPE_FILE, \
    .numACL = 2, \
    .acl = \
    { \
        [0].srcElement = "/*", \
        [0].allowMask = DULINK_PERMISSION_NONE, \
        [0].denyMask = DULINK_PERMISSION_NONE, \
        [1].srcElement = "/master", \
        [1].allowMask = DULINK_PERMISSION_READ | DULINK_PERMISSION_WRITE, \
        [1].denyMask = DULINK_PERMISSION_NONE \
    } \
}

/// Master/Compat can read/write, all others have no access
#define MASTER_COMPAT_ONLY_CMP_RW_ATTRIBUTE { \
    .type = DULINK_NODE_TYPE_FILE, \
    .numACL = 3, \
    .acl = \
    { \
        [0].srcElement = "/*", \
        [0].allowMask = DULINK_PERMISSION_NONE, \
        [0].denyMask = DULINK_PERMISSION_NONE, \
        [1].srcElement = "/master", \
        [1].allowMask = DULINK_PERMISSION_READ | DULINK_PERMISSION_WRITE, \
        [1].denyMask = DULINK_PERMISSION_NONE, \
        [2].srcElement = "/compatchecker", \
        [2].allowMask = DULINK_PERMISSION_READ | DULINK_PERMISSION_WRITE, \
        [2].denyMask = DULINK_PERMISSION_NONE \
    } \
}

/// Only master can write but not read, all others have no access
#define MASTER_ONLY_WRITE_ATTRIBUTE { \
    .type = DULINK_NODE_TYPE_FILE, \
    .numACL = 2, \
    .acl = \
    { \
        [0].srcElement = "/*", \
        [0].allowMask = DULINK_PERMISSION_NONE, \
        [0].denyMask = DULINK_PERMISSION_READ, \
        [1].srcElement = "/master", \
        [1].allowMask = DULINK_PERMISSION_WRITE, \
        [1].denyMask = DULINK_PERMISSION_READ \
    } \
}

/* ------------------------ Type Definitions -------------------------------- */
enum DUPLUGIN_TYPE
{
    CONTROLLER = 0U,
    INSTALLER,
    CONTENT_PROVIDER,
    FILE_TRANSFORM,
    VALIDATOR,
    METADATA_PROVIDER,
    DUPLUGIN_TYPE_MAX
};
typedef enum DUPLUGIN_TYPE DUPLUGIN_TYPE;

typedef struct DULINK_EXPORT_REQS DULINK_EXPORT_REQS, *PDULINK_EXPORT_REQS;
typedef struct CTX_LOCK_STR CTX_LOCK_STR, *PCTX_LOCK_STR;
typedef struct CTX_LOCK_UINT8 CTX_LOCK_UINT8, *PCTX_LOCK_UINT8;
typedef struct CTX_LIST_STR CTX_LIST_STR, *PCTX_LIST_STR;

///
/// For defining static tables of DU Link node details. Tables may be iterated
/// through for simplicity in exporting all nodes owned by a component
///
struct DULINK_EXPORT_REQS
{
    /// DU Link path to be exported
    const char        *pPath;
    /// Callback function to export
    const DULINK_CB    cb;
    /// DU Link Attribute of node being exported
    DULINK_ATTR        attr;
    /// Context for to callback function
    const void        *pCtx;
};

///
/// Data structure to enable access in a DU Link callback to a string requiring
/// thread access protection
///
struct CTX_LOCK_STR
{
    /// String to be passed to callback context
    char *pStr;
    /// Will be locked before reading from pStr
    pthread_mutex_t *pMutex;
};

///
/// Data structure to enable access in a DU Link callback to a uint8_t requiring
/// thread access protection
///
struct CTX_LOCK_UINT8
{
    /// Integer to be passed to callback context
    uint8_t *pInt;
    /// Will be locked before reading from pStr
    pthread_mutex_t *pMutex;
};

/// Data structure to enable reading a list of values in a DU Link callback
struct CTX_LIST_STR
{
    /// Number of items in the list
    const uint8_t size;
    /// Pointer to array of strings to iterate
    char const **ppStrList;
};

/* ------------------------ Common Helper Functions ------------------------- */
/*!
 * Export DU Link directories, files and notify nodes. This function takes a
 * list of DULINK_EXPORT_REQS for each type and will export the corresponding
 * DU Link node type based on which list it is in
 *
 * @param[in] pDulinkDirs
 *      List of export requirements for DU Link directories. If NULL, numDirs
 *      must be 0
 * @param[in] numDirs
 *      Number of dirs in pDulinkDirs
 * @param[in] pDulinkFiles
 *      List of export requirements for DU Link files. If NULL, numFiles must
 *      be 0
 * @param[in] numFiles
 *      Number of files in pDulinkFiles
 * @param[in] pDulinkNotifyNodes
 *      List of export requirements for DU Link notify nodes. If NULL,
 *      numNotifyNodes must be 0
 * @param[in] numNotifyNodes
 *      Number of notify nodes in pDulinkNotifyNodes
 *
 * @return DU_OK
 *      All nodes exported successfully
 * @return DUCOMMON_ERR_GENERIC
 *      Failed to export DU Link Nodes
 */
DU_RCODE exportDULinkNodes
(
    const DULINK_EXPORT_REQS *pDulinkDirs,
    uint8_t                   numDirs,
    const DULINK_EXPORT_REQS *pDulinkFiles,
    uint8_t                   numFiles,
    const DULINK_EXPORT_REQS *pDulinkNotifyNodes,
    uint8_t                   numNotifyNodes
);

/*!
 * Register a DU Link plugin to DU Master. This will tell DU Master to set up
 * listeners for progress.notify, current_rl.notify, pending_rl.notify, and
 * state.notify on the plugin registering to master.
 *
 * @param[in] pMasterPath
 *      DU Link path to DU Master element
 *
 * @return DU_OK
 *      Successfully registered to DU Master
 * @return DUCOMMON_ERR_INVALID_ARGUMENT
 *      Invalid path to DU Master
 * @return DUCOMMON_ERR_GENERIC
 *      Failed to register to DU Master
 */
DU_RCODE registerToMaster
(
    const char *pMasterPath
);

/*!
 * Replace $CONTENT_ROOT$ in path string to actual content
 *
 * @param[in,out] pStr
 *      Path string to process
 * @param[in] pContRootVal
 *      Actual content root value
 * @param[in] bufLen
 *      Maximum length of input/output buffer
 *
 * @return DU_OK
 *      String successfully converted
 * @return DUCOMMON_ERR_BUFFER_TOO_SMALL
 *      Buffer size not enough for actual content
 *
 */
DU_RCODE replaceContentRoot
(
    char       *pStr,
    const char *pContRootVal,
    uint32_t    bufLen
);

/* ------------------------ Shared Callbacks -------------------------------- */
/*!
 * Callback for simply reading the value stored in a particular string with
 * no additional functionality required. Context passed to this callback
 * should be of type LOCK_STR_CTX. In case the mutex in the passed structure
 * is not NULL, the mutex will be locked surrounding any reading of the
 * string value
 *
 * Used for the following callbacks:
 * plugin-type, state, progress, ver/installer, last_deployed/metadata,
 * last_deployed/result
 *
 * WRITE operations are not supported by this callback
 *
 * @param[in] pRequestPath
 *          Full path of node being accessed
 * @param[in] pOriginPath
 *          Full path of element originating the request
 * @param[in] pCtx
 *          Pointer to CTX_LOCK_STR data structure
 * @param[in] offset
 *          Should always be 0, otherwise error
 * @param[in] length
 *          Length of buffer to use for reading
 * @param[in] pBuf
 *          Pointer to buffer for storing data
 * @param[in] operation
 *          IO operation, it must be one of:
 *          DULINK_CB_READ
 *          DULINK_CB_SIZE
 * @param[out] pRetVal
 *          for operation DULINK_CB_READ, Actual bytes read to buffer
 *          for operation DULINK_CB_SIZE, length of the string
 *
 * @return DU_OK
 *          Upon success
 * @return DULINK_CB_ERR_INVALID_ARGUMENT
 *          Invalid operation or offset
 * @return DULINK_CB_ERR_UNKNOWN
 *          Not enough space to store value in buffer
 */
DU_RCODE readOnlyStringCB
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

/*!
 * Callback for read-only .list nodes. This callback will iterate through a
 * data structure of type CTX_LIST_STR and return a NULL separated list
 * of strings containing all values in the list
 *
 * Used for the following callbacks:
 * cmd.list, state.list
 *
 * WRITE operations are not supported by this callback
 *
 * @param[in] pRequestPath
 *          Full path of node being accessed
 * @param[in] pOriginPath
 *          Full path of element originating the request
 * @param[in] pCtx
 *          Pointer to initialized CTX_LIST_STR data structure
 * @param[in] offset
 *          Should always be 0, otherwise error
 * @param[in] length
 *          Length of buffer to use for reading
 * @param[in] pBuf
 *          Pointer to buffer for storing data
 * @param[in] operation
 *          IO operation, it must be one of:
 *          DULINK_CB_READ
 *          DULINK_CB_SIZE
 * @param[out] pRetVal
 *          for operation DULINK_CB_READ, Actual bytes read to buffer
 *          for operation DULINK_CB_SIZE, length of the string
 *
 * @return DU_OK
 *          Upon success
 * @return DULINK_CB_ERR_INVALID_ARGUMENT
 *          Invalid operation or offset
 * @return DULINK_CB_ERR_UNKNOWN
 *          Not enough space to store value in buffer
 */
DU_RCODE listNodeCB
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

/*!
 * Callback for read-only runlevel nodes. This callback will return a string
 * representation of runlevel when reading a node's either current or
 * pending runlevel
 *
 * Used for the following callbacks:
 * current_rl, pending_rl
 *
 * WRITE operations are not supported by this callback
 *
 * @param[in] pRequestPath
 *          Full path of node being accessed
 * @param[in] pOriginPath
 *          Full path of element originating the request
 * @param[in] pCtx
 *          Pointer to uint8_t storing runlevel
 * @param[in] offset
 *          Should always be 0, otherwise error
 * @param[in] length
 *          Length of buffer to use for reading
 * @param[in] pBuf
 *          Pointer to buffer for storing data
 * @param[in] operation
 *          IO operation, it must be one of:
 *          DULINK_CB_READ
 *          DULINK_CB_SIZE
 * @param[out] pRetVal
 *          for operation DULINK_CB_READ, Actual bytes read to buffer
 *          for operation DULINK_CB_SIZE, length of the string
 *
 * @return DU_OK
 *          Upon success
 * @return DULINK_CB_ERR_INVALID_ARGUMENT
 *          Invalid operation or offset
 * @return DULINK_CB_ERR_UNKNOWN
 *          Not enough space to store value in buffer
 */
DU_RCODE readOnlyRunlevelCB
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

#ifdef __cplusplus
}
#endif

#endif
