/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION. All rights reserved. All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation. Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

#ifndef _NVEEPROM_H_
#define _NVEEPROM_H_

#include "nvmedia_isc.h"
#include "devblk_cdi.h"
#include "nveeprom_spec.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void     NvEEPROMManager;
typedef uint32_t NvEEPROMManagerParamId;

typedef enum {
    /* The parameter is present on the EEPROM device and contains valid data */
    NVEEPROM_PARAM_STATUS_OK,
    /* The parameter is not present on the EEPROM device */
    NVEEPROM_PARAM_STATUS_NOT_FOUND,
    /* Space is allocated for the parameter on the EEPROM device, but it
       is marked as 'invalid'.  This means that either:
         1. The parameter was never written to.
         2. The parameter was corrupted at some point and was never rewritten. */
    NVEEPROM_PARAM_STATUS_INVALID,
    /* The parameter is present but is corrupted */
    NVEEPROM_PARAM_STATUS_CORRUPTED
} NvEEPROMManagerParamStatus;

/*************************************************************************
 *   Function Name: NvEEPROMManagerCreate
 *   Description:
 *       Allocate memory for and initialize the NvEEPROMManager data
 *       structure. This reads the entire EEPROM device and caches its
 *       contents. It also builds a table to the location of all of the
 *       groups stored in the EEPROM so they can be easily found. If no
 *       writable section is detected after the factory blob, the writable
 *       section is initialized and written to the EEPROM device.
 *
 *       This API sets the NvEEPROMManager context to use the NvMediaISC library
 *       implementation.
 *
 *   Inputs:
 *        device   - Pointer to the NvMediaISC device for the EEPROM.
 *        flags    - Reserved for future use. This should be set to 0 or
 *                   behavior may change in future version.
 *
 *   Outputs:
 *        N/A
 *
 *   Return Data:
 *       Pointer to the newly created NvEEPROMManager. If there is an error
 *       during the initialization process, NULL will be returned instead.
 *
 *   Notes:
 *       It is assumed that the image module to which the EEPROM device
 *       belongs has already been configured so that it is possible to
 *       communicate with the EEPROM device. It is also assumed that the
 *       image module is not currently capturing frames.
 *
 *************************************************************************/
NvEEPROMManager*
NvEEPROMManagerCreate(
    NvMediaISCDevice *device,
    uint32_t flags);

/*************************************************************************
 *   Function Name: NvEEPROMManagerDevBlkCreate
 *   Description:
 *       Allocate memory for and initialize the NvEEPROMManager data
 *       structure. This reads the entire EEPROM device and caches its
 *       contents. It also builds a table to the location of all of the
 *       groups stored in the EEPROM so they can be easily found. If no
 *       writable section is detected after the factory blob, the writable
 *       section is initialized and written to the EEPROM device.
 *
 *       This API sets the NvEEPROMManager context to use the SIPL DevBlkCDI
 *       library implementation.
 *
 *   Inputs:
 *        device   - Pointer to the DevBlkCDI device for the EEPROM.
 *        flags    - Reserved for future use. This should be set to 0 or
 *                   behavior may change in future version.
 *
 *   Outputs:
 *        N/A
 *
 *   Return Data:
 *       Pointer to the newly created NvEEPROMManager. If there is an error
 *       during the initialization process, NULL will be returned instead.
 *
 *   Notes:
 *       It is assumed that the image module to which the EEPROM device
 *       belongs has already been configured so that it is possible to
 *       communicate with the EEPROM device. It is also assumed that the
 *       image module is not currently capturing frames.
 *
 *************************************************************************/
NvEEPROMManager*
NvEEPROMManagerDevBlkCreate(
    DevBlkCDIDevice *device,
    uint32_t flags);

/*************************************************************************
 *   Function Name:  NvEEPROMManagerValidate
 *   Description:
 *        Check the status of each parameter in parameterList.  Write the
 *        status to the corresponding index in the statusList.
 *
 *        If both parameterList and statusList are set to NULL and numParams
 *        is set to zero, this function will instead check to see if all of the
 *        mandatory parameters are present and valid.  If they are, it will
 *        return NVMEDIA_STATUS_OK.
 *
 *   Inputs:
 *        eepromManager - A pointer to the EEPROM Manager object to verify
 *        parameterList - An array of parameters to verify.
 *        numParams     - The number of elements in the parameterList.
 *                        statusList must be large enough to hold this many
 *                        elements as well.
 *
 *   Outputs:
 *        statusList    - An array to write the status of the parameters to.
 *                        An element of this array corresponds to the
 *                        parameter with the same index in the parameterList.
 *
 *   Return Data:
 *        If using this API with parameterList, the following return codes
 *        are possible:
 *        NVMEDIA_STATUS_OK - The statusList was successfully updated to reflect
 *        the status of the parameters specified in parameterList.
 *        NVMEDIA_STATUS_BAD_PARAMTER - An invalid parameter was passed in.
 *        NVMEDIA_STATUS_ERROR - Any other error was encountered
 *
 *        If using this API to check the mandatory parameters (parameterList
 *        and statusList are NULL while numParams is 0), then the following
 *        return codes are possible:
 *        NVMEDIA_STATUS_OK - All of the mandatory parameters are present and
 *        valid.
 *        NVMEDIA_STATUS_ERROR - At least one of the mandatory parameters is
 *        not present or is corrupted.
 *
 *
 *************************************************************************/
NvMediaStatus
 NvEEPROMManagerValidate(
    NvEEPROMManager *eepromManager,
    const NvEEPROMManagerParamId *parameterList,
    NvEEPROMManagerParamStatus *statusList,
    uint32_t numParams);

 /*************************************************************************
 *   Function Name: NvEEPROMManagerRead
 *   Description:
 *       Read data associated with a parameter stored on the EEPROM device and
 *       write it to the provided buffer.
 *
 *       If the parameter is not present, does not contain valid data, or is
 *       corrupted, then this API call will fail, and no data will be written
 *       to the provided buffer.
 *
 *   Inputs:
 *        eepromManager - A pointer to the EEPROM Manager object to read from
 *        paramId       - The ID of the parameter to be read
 *        size          - The number of bytes that are to be read from the
 *                        parameter.  This must be greater than zero and
 *                        less than or equal to the maximum size of the
 *                        parameter.
 *
 *   Outputs:
 *        buffer        - A pointer to the buffer that the data is to be read
 *                        to. It must be at least 'size' bytes long.
 *
 *   Return Data:
 *       If the read is successful, NVMEDIA_STATUS_OK will be returned.
 *       If any of the parameters passed were invalid, then
 *       NVMEDIA_STATUS_BAD_PARAMETER will be returned.
 *       If a different error occurs, NVMEDIA_STATUS_ERROR will be
 *       returned, and a message describing the error will be printed to
 *       the debug log file.
 *
 *   Notes:
 *       This reads from the copy of the EEPROM data in local memory, so
 *       this can be done, even if the ISC device is in a state where it
 *       cannot be read from.
 *
 *************************************************************************/
NvMediaStatus
NvEEPROMManagerRead(
    NvEEPROMManager *eepromManager,
    NvEEPROMManagerParamId id,
    void *buffer,
    uint32_t size);

/*************************************************************************
 *   Function Name: NvEEPROMManagerWrite
 *   Description:
 *       Write a writable parameter to the EEPROM device. If the parameter
 *       is not present in the writable section, this append it to the end
 *       of the writable section. If the parameter is already present,
 *       it will be overwritten.
 *
 *       If the 'size' provided is less than the maximum size of the parameter
 *       with the given ID, space will still be allocated in the EEPROM for
 *       the maximum size of the parameter.  The bytes following the first
 *       'size' bytes will have an undefined value.
 *
 *   Inputs:
 *        eepromManager - A pointer to the EEPROM Manager object to write to
 *        paramId       - The ID of the parameter to be written
 *        buffer        - A pointer to the buffer that contains the data to
 *                        be written. It must be at least 'size' bytes long.
 *        size          - The number of bytes that are to be written to the
 *                        parameter.  This must be greater than zero and
 *                        less than or equal to the maximum size of the
 *                        parameter.
 *
 *   Outputs:
 *        N/A
 *
 *   Return Data:
 *       If the write is successful, NVMEDIA_STATUS_OK will be returned.
 *       If any of the parameters passed were invalid, then
 *       NVMEDIA_BAD_PARAMETER will be returned.
 *       If a different error occurs, NVMEDIA_STATUS_ERROR will be
 *       returned, and a message describing the error will be printed to
 *       the debug log file.
 *
 *   Notes:
 *       It is assumed that the image module to which the EEPROM device
 *       belongs has already been configured so that it is possible to
 *       communicate with the EEPROM device. It is also assumed that the
 *       image module is not currently capturing frames.
 *
 *************************************************************************/
NvMediaStatus
NvEEPROMManagerWrite(
    NvEEPROMManager *eepromManager,
    NvEEPROMManagerParamId id,
    const void *buffer,
    uint32_t size);

 /*************************************************************************
 *   Function Name: NvEEPROMManagerDestroy
 *   Description:
 *       Free the memory used by the EEPROM Manager
 *
 *   Inputs:
 *        eepromManager - A pointer to the EEPROM Manager to be deinitialized
 *
 *   Outputs:
 *        N/A
 *
 *   Return Data:
 *       N/A
 *
 *   Notes:
 *       This does not destroy the ISC device associated with the EEPROM.
 *
 *************************************************************************/
void
NvEEPROMManagerDestroy(
    NvEEPROMManager *eepromManager);

#ifdef __cplusplus
}
#endif

#endif /*_NVEEPROM_H_*/
