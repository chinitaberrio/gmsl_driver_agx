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
 * @brief Command and Control Client Library
 *
 * Control and interact using 'libnvducc'.
 */

#ifndef DUCC_H_
#define DUCC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ducommon.h"
#include "dutransport.h"

/*!
 * @defgroup du_cc_group Command and Control API
 *
 * This file declares APIs for the DRIVE Update library 'libnvducc.so'.
 *
 * @ingroup drive_update_top
 * @{
 */
/* ---------------------- Types --------------------------------------------- */
/*!
 * Defines the return codes of DRIVE Update C&C library function calls.
 */
typedef uint32_t DUCC_Error;
#define DUCC_Success                    (0x00000000)
#define ERR_NotImplemented              (0x00010000)
#define ERR_InvalidArgument             (0x00010001)
#define ERR_NoPendingIMG                (0x00010002)
#define ERR_PathNotExist                (0x00010003)
#define ERR_OperationNotPermitted       (0x00010004)
#define ERR_BufferTooSmall              (0x00010005)
#define ERR_OperationRestrictedByConfig (0x00010006)
#define ERR_Busy                        (0x00010007)
#define ERR_UnableConnect               (0x00020000)
#define ERR_ConnectionLost              (0x00020001)
#define ERR_NoActiveConnection          (0x00020002)
#define ERR_InvalidResponse             (0x00020003)

/*!
 * Type of pending actions defined by DRIVE Update C&C interface.
 */
typedef uint32_t DUCC_PendingAction;
#define PA_NONE       (0)
#define PA_RUNLEVEL   (1)
#define PA_USERACTION (2)

/*!
 * States defined by DRIVE Update C&C interface.
 */
typedef uint32_t DUCC_State;
#define STATE_DORMANT            (0)
#define STATE_NO_CONNECTIVITY    (1)
#define STATE_UP_TO_DATE         (2)
#define STATE_UPDATE_AVAILABLE   (3)
#define STATE_UPDATE_IN_PROGRESS (4)
#define STATE_UPDATE_FAILED      (5)
#define STATE_FATAL_ERROR        (6)
#define NUM_DUCC_STATES          (7)

/*!
 * \brief Defines the locales used to get content.
 */
typedef uint32_t DUCC_Locale;
/*! \brief Defines the US locale. */
#define DUCC_LOC_US (0)

/*! \brief Defines the image installation states. */
typedef uint32_t DUCC_Image;
/*! \brief Defines the currently installed image state. */
#define DUCC_IMG_CURRENT (0)
/*! \brief Defines the image pending installation state. */
#define DUCC_IMG_PENDING (1)

/*! \brief Indicates no progress or progress not defined. */
#define PR_NONE      (101)
/*! \brief Indicates progress number when it reaches 100%. */
#define PR_MAX       (100)
/*! \brief Defines the max length of the file path. */
#define FILEPATH_MAX (512)

/*!
 * Default option for user actions, which will accept any terms or otherwise
 * continue the OTA installation process whenever applicable.
 */
#define DUCC_USERACTION_DEFAULT_SELECTION (0)

/*!
 * Defines the DRIVE Update C&C handle.
 */
typedef struct DUCC *PDUCC;

/*!
 * Defines the OTA status structure returned from library function call.
 */
typedef struct DUCC_Status
{
    DUCC_State         state;
    uint32_t           progress;
    DUCC_PendingAction pendingAction;
    uint32_t           pendingActionArgs;
} DUCC_Status;

/*!
 * @brief Callback function when status changed.
 *
 * @param[in] pObj
 *          Pointer to the object registered during initialization.
 *
 * @param[in] pStatus
 *          Current DRIVE Update status.
 *
 * @return None
 */
typedef void (*DUCCCallback)(void *pObj,
                             const DUCC_Status *pStatus);

typedef struct optionEntry
{
    uint32_t userActionID;
    uint32_t contentOffset;
    uint32_t contentLen;
} optionEntry;

/*!
 * Buffer for storing options.
 *
 * The following describes the memory layout of the @c optionBuffer structure:
 * @code
 * +-------------------+
 * |    numEntries     |
 * +-------------------+
 * |                   |
 * |   OptionEntry[]   |
 * |                   |
 * +-------------------+
 * |                   |
 * |                   |
 * |      char[]       |
 * |                   |
 * |                   |
 * +-------------------+
 * |      Unused       |
 * +-------------------+
 * @endcode
 *
 * It contains header of each individual entry at top of the buffer, then area
 * of string which would be pointed by option entry header using offset bytes
 * Since option buffer is caller allocated, it may contains unused bytes at end
 * of the buffer. The structure layout would be initialized by library code.
 *
 * To obtain size needed for buffer, pass NULL to corresponding API call and a
 * minimal size would be filled in size parameter. The minimal size would also
 * be filled in size parameter if the buffer size is too small, under this case
 * ERR_BufferTooSmall would be returned.
 *
 * A null byte in string area does not mark the end of that string.
 */
typedef struct optionBuffer
{
    uint32_t     numEntries;
    optionEntry *pEntry;
} optionBuffer;

typedef struct versionEntry
{
    uint32_t nameOffset;
    uint32_t nameLen;
    uint32_t versionOffset;
    uint32_t versionLen;
} versionEntry;

/*!
 * Refer to @ref optionBuffer struct memory layout.
 */
typedef struct versionBuffer
{
    uint32_t      numEntries;
    versionEntry *pEntry;
} versionBuffer;

typedef struct infoEntry
{
    uint32_t nameOffset;
    uint32_t nameLen;
    uint32_t contentOffset;
    uint32_t contentLen;
} infoEntry;

/*!
 * Refer to @ref optionBuffer struct memory layout.
 */
typedef struct infoBuffer
{
    uint32_t   numEntries;
    infoEntry *pEntry;
} infoBuffer;

/* -------------------- Function Declarations ------------------------------ */
/*!
 * @brief Initialize C&C connection to the DRIVE Update Master.
 *
 * This function should be only called once before closing the connection.
 * This function is not thread safe.
 *
 * @param[in] trType
 *          Type of back-end transport protocol used.
 *
 * @param[in] seType
 *          Type of security protocol.
 *
 * @param[in] pTrParam
 *          Pointer to the argument of selected transport protocol. For instance,
 *          if @a trType is @ref DUTR_TR_TYPE_IVC, then it shall point to the structure
 *          @ref DUTR_IVC_PARAM.
 *
 * @param[in] pSeParam
 *          Pointer to the argument of selected security protocol. For instance,
 *          if @a seType is @ref DUTR_SEC_TYPE_TLS, then it shall point to the structure
 *          @ref DUTR_SEC_TLS_PARAM.
 *
 * @param[out] ppDucc
 *          Pointer to initialized DUCC handle.
 *
 * @return @ref DUCC_Success
 *          Upon success.
 *
 * @return ERR_*
 *          Error has occurred.
 */
DUCC_Error DUCC_Init
(
    DUTR_TR_TYPE            trType,
    DUTR_SEC_TYPE           seType,
    const PDUTR_TR_PARAM    pTrParam,
    const PDUTR_SEC_PARAM   pSeParam,
    PDUCC                  *ppDucc
);

/*!
 * @brief Return current C&C API version.
 *
 * Currently, this API will be returning version 1.
 *
 * @param[in] pDucc
 *          Pointer to DUCC handle.
 *
 * @param[out] pVersion
 *          Returned version number, this variable is caller allocated.
 *
 * @return DUCC_Success
 *          Upon success.
 *
 * @return ERR_*
 *          Other errors.
 */
DUCC_Error DUCC_Get_API_Version
(
    PDUCC     pDucc,
    uint32_t *pVersion
);

/*!
 * @brief Return current active run level of DRIVE Update.
 *
 * This may be different from most recently requested run level.
 *
 * @param[in] pDucc
 *          Pointer to DUCC handle.
 *
 * @param[out] pRunLevel
 *          Returned run level, this variable is caller allocated.
 *
 * @return @ref DUCC_Success
 *          Upon success.
 *
 * @return ERR_*
 *          Other errors.
 */
DUCC_Error DUCC_Get_Current_RunLevel
(
    PDUCC          pDucc,
    DU_RUN_LEVEL  *pRunLevel
);

/*!
 * @brief Request DRIVE Update to switch to another run level.
 *
 * This function is non-blocking. It doesn't ensure switch to requested run
 * level immediately.
 *
 * @param[in] pDucc
 *          Pointer to DUCC handle.
 *
 * @param[in] reqRunLevel
 *          Requested run level.
 *
 * @return @ref DUCC_Success
 *          Upon success.
 *
 * @return ERR_*
 *          Other errors.
 */
DUCC_Error DUCC_Request_RunLevel
(
    PDUCC         pDucc,
    DU_RUN_LEVEL  reqRunLevel
);

/*!
 * @brief Retrieve current DRIVE Update status.
 *
 * Notice if progress is returned as PR_NONE it means the progress is undefined.
 * Otherwise, it should be an unsigned int between 0 and PR_MAX.
 *
 * @param[in] pDucc
 *          Pointer to DUCC handle.
 *
 * @param[out] pStatus
 *          Current DRIVE Update status, this struct is caller allocated.
 *
 * @return @ref DUCC_Success
 *          Upon success.
 *
 * @return ERR_*
 *          Other errors.
 */
DUCC_Error DUCC_Get_Current_Status
(
    PDUCC        pDucc,
    DUCC_Status *pStatus
);

/*!
 * @brief Register a callback function to be informed when every time DRIVE
 * Update status changes.
 *
 * @param[in] pDucc
 *          Pointer to DUCC handle.
 *
 * @param[in] pObj
 *          User defined object, would be sent to callback function.
 *
 * @param[in] callback
 *          Callback function.
 *
 * @return @ref DUCC_Success
 *          Upon success.
 *
 * @return ERR_*
 *          Other errors.
 */
DUCC_Error DUCC_Register_Status_Update_Callback
(
    PDUCC        pDucc,
    void        *pObj,
    DUCCCallback callback
);

/*!
 * @brief Retrieves the (optionally) localized release notes.
 *
 * Release note is for either currently installed image or the image pending
 * installation.
 *
 * @param[in] pDucc
 *          Pointer to DUCC handle.
 *
 * @param[in] image
 *          Specifies which release notes to be retrieved.
 *
 * @param[in] locale
 *          Specifies which locale of release notes to be retrieved.
 *          Currently only DUCC_LOC_US is supported.
 *
 * @param[in] pPath
 *          Path to a folder where the release notes content should be placed.
 *
 * @param[out] pFilename
 *          Caller allocated buffer to hold the name of the top level file to be
 *          displayed for release note. It should at least has FILEPATH_MAX
 *          in size.
 *
 * @return @ref DUCC_Success
 *          Upon success.
 *
 * @return ERR_*
 *          Other errors.
 */
DUCC_Error DUCC_Get_ReleaseNotes_Content
(
    PDUCC        pDucc,
    DUCC_Image   image,
    DUCC_Locale  locale,
    const char  *pPath,
    char        *pFilename
);

/*!
 * @brief Retrieves the content of the user action (such as EULA) to be shown to
 * the user, as well as possible choices.
 *
 * @param[in] pDucc
 *          Pointer to DUCC handle.
 *
 * @param[in] userActionID
 *          Pending user action id mentioned by Get_CurrentStatus.
 *
 * @param[in] locale
 *          Specifies which locale of user action to be retrieved.
 *          Currently only DUCC_LOC_US is supported.
 *
 * @param[in] pPath
 *          Path to a folder where the user action file should be placed
 *
 * @param[out] pFilename
 *          Caller allocated buffer to hold the name of the top level file to be
 *          displayed for user action. It should at least has FILEPATH_MAX
 *          in size.
 *
 * @param[in, out] pOptionBuf
 *          Caller allocated buffer to hold possible user actions. It would
 *          contain a series of optionEntry to get possible selection id and
 *          its display.
 *
 *          Can be NULL to get requested buffer size.
 *
 * @param[in, out] pOptionBufsize
 *          Size of passed in buffer. If pOptionBuf is passed as NULL or this
 *          number is too small, the minimal size of buffer needed is returned.
 *
 * @return @ref DUCC_Success
 *          Upon success.
 *
 * @return @ref ERR_BufferTooSmall
 *          pOptionBuf is passed as NULL or pOptionBufsize was too small.
 *
 * @return ERR_*
 *          Other errors.
 */
DUCC_Error DUCC_Get_UserAction_Content
(
    PDUCC         pDucc,
    uint32_t      userActionID,
    DUCC_Locale   locale,
    const char   *pPath,
    char         *pFilename,
    optionBuffer *pOptionBuf,
    uint32_t     *pOptionBufsize
);

/*!
 * @brief Submit back the selection user made.
 *
 * @param[in] pDucc
 *          Pointer to DUCC handle.
 *
 * @param[in] userActionID
 *          Pending user action id mentioned by Get_CurrentStatus.
 *
 * @param[in] userSelectionID
 *          Selection Id user chosen.
 *
 * @return @ref DUCC_Success
 *          Upon success.
 *
 * @return ERR_*
 *          Other errors.
 */
DUCC_Error DUCC_Submit_UserAction
(
    PDUCC    pDucc,
    uint32_t userActionID,
    uint32_t userSelectionID
);

/*!
 * @brief Retrieves all known version information about currently installed
 * image or the image pending installation.
 *
 * @param[in] pDucc
 *          Pointer to DUCC handle.
 *
 * @param[in] image
 *          Specifies which version info to be retrieved.
 *
 * @param[in, out] pVersionBuf
 *          Caller allocated buffer to hold retrieved version information. It
 *          would contain a series of versionEntry to get possible version
 *          name and its value.
 *
 *          Can be NULL to get requested buffer size.
 *
 * @param[in, out] pVersionBufsize
 *          Size of passed in buffer. If @a pVersionBuf is passed as NULL or this
 *          number is too small, the minimal size of buffer needed is returned.
 *
 * @return @ref DUCC_Success
 *          Upon success.
 *
 * @return @ref ERR_BufferTooSmall
 *          @a pVersionBuf is passed as NULL or pVersionBufsize was too small.
 *
 * @return DUCC_*
 *          Other errors
 */
DUCC_Error DUCC_Get_Versions
(
    PDUCC          pDucc,
    DUCC_Image     image,
    versionBuffer *pVersionBuf,
    uint32_t      *pVersionBufsize
);

/*!
 * @brief Retrieves additional information available via C&C.
 *
 * @param[in] pDucc
 *          Pointer to DUCC handle.
 *
 * @param[in] pGroupName
 *          String which identifies the set of information being returned.
 *
 * @param[in, out] pInfoBuf
 *          Caller allocated buffer to hold retrieved additional information. It
 *          would contain a series of infoEntry to get possible information
 *          name and its value.
 *
 *          Can be NULL to get requested buffer size.
 *
 * @param[in, out] pInfoBufsize
 *          Size of passed in buffer. If pInfoBuf is passed as NULL or this
 *          number is too small, the minimal size of buffer needed is returned.
 *
 * @return @ref DUCC_Success
 *          Upon success.
 *
 * @return @ref ERR_BufferTooSmall
 *          pInfoBuf is passed as NULL or pInfoBufsize was too small.
 *
 * @return DUCC_*
 *          Other errors.
 */
DUCC_Error DUCC_Get_AdditionalInfo
(
    PDUCC       pDucc,
    const char *pGroupName,
    infoBuffer *pInfoBuf,
    uint32_t   *pInfoBufsize
);

/*!
 * @brief Close an active C&C connection.
 *
 * @param[in] pDucc
 *          Pointer to DUCC handle.
 *
 * @return @ref DUCC_Success
 *          Upon success.
 *
 * @return DUCC_*
 *          Other errors.
 */
DUCC_Error DUCC_Close
(
    PDUCC pDucc
);
/** @} */
#ifdef __cplusplus
}
#endif

#endif
