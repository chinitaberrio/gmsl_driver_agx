/*
 * Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

/**
 * Description: This file declares interfaces for libnvtegrahv.
 *
 * Return Values:
 * 	NvHvCheckOsNative:
 * 		 1 -> Native
 *		 0 -> Non Native
 *		-1 -> cannot determine
 *	NvHvGetOsVmId:
 *		!0 -> Fail
 *		 0 -> Success, non native
 *	NvHvGetDisplayVmId:
 *		!0 -> Fail
 *		 0 -> Success, non-native
 *      NvHvGetSysinfoAddr:
 *              !0 -> Fail
 *               0 -> Success. Argument will contain sysinfo phy addr
 */

#ifndef INCLUDED_NV_TEGRA_HV_H
#define INCLUDED_NV_TEGRA_HV_H

#if defined(__cplusplus)
extern "C"
{
#endif

#include <stdint.h>

/**
 * @defgroup nvhv_user_api_group NVHV::User interface
 * @ingroup qnx_top_group
 * @{
 */

#define BUF_SIZE 	20 /* Max digits for vmid */
#define DISPLAY_VM 	"DISPLAY_VM" /* environ var that stores display vmid */

/** @cond HIDDEN_SYMBOLS */
/**
 * @brief function pointer to NvHvGetOsVmId
 */
typedef int (*PfnNvHvGetOsVmId)(unsigned int *vmid);
/**
 * @brief function pointer to NvHvCheckOsNative
 */
typedef int (*PfnNvHvCheckOsNative)(void);
/**
 * @brief function pointer to NvHvGetSysinfoAddr
 */
typedef int (*PfnNvHvGetSysinfoAddr)(uint64_t *addr);
/** @endcond */

/**
 * @brief API to check whether running on Hypervisor or Native
 *
 * @return
 * @retval 0 Running on hypervisor
 * @retval 1 Native
 * @retval -1 Failed
 *
 * @note
 * - Allowed context for the API call
 *  - Interrupt: No
 *  - Signal handler: No
 * - Is thread safe: Yes
 * - Required Privileges: NIL
 * - API Group
 *  - Initialization: Yes
 *  - Run time: Yes
 *  - De-initialization: No
 *
 * SWUD_ID: QNXBSP_NVHV_NVTEGRAHV_LIB_01
 */
int NvHvCheckOsNative(void);

/**
 * @brief API to get VM ID
 *
 * @param[out] vmid Guest VM ID
 *
 * @return
 * @retval EOK Success
 * @retval ENODEV Invalid file descriptor
 * @retval EINVAL Invalid argument
 * @retval Others Other error values returned from QNX standard API
 *
 * @note
 * - Allowed context for the API call
 *  - Interrupt: No
 *  - Signal handler: No
 * - Is thread safe: Yes
 * - Required Privileges: NIL
 * - API Group
 *  - Initialization: Yes
 *  - Run time: Yes
 *  - De-initialization: No
 *
 * SWUD_ID: QNXBSP_NVHV_NVTEGRAHV_LIB_02
 */
int NvHvGetOsVmId(unsigned int *vmid);

/**
 * @brief API to get IPA to HV system info
 *
 * @param[out] addr IPA to HV system info
 *
 * @return
 * @retval EOK Success
 * @retval ENODEV Invalid file descriptor
 * @retval EINVAL Invalid argument
 * @retval Others Other error values returned from QNX standard API
 *
 * @note
 * - Allowed context for the API call
 *  - Interrupt: No
 *  - Signal handler: No
 * - Is thread safe: Yes
 * - Required Privileges: NIL
 * - API Group
 *  - Initialization: Yes
 *  - Run time: Yes
 *  - De-initialization: No
 *
 * SWUD_ID: QNXBSP_NVHV_NVTEGRAHV_LIB_03
 */
int NvHvGetSysinfoAddr(uint64_t *addr);

#if (NV_IS_SAFETY == 0)
/**
 * @brief function pointer to NvHvGetDisplayVmId
 */
typedef int (*PfnNvHvGetDisplayVmId)(unsigned int *dpvmid);
int NvHvGetDisplayVmId(unsigned int *dpvmid);
#endif

/**
 * @brief enum for OS type
 */
typedef enum
{
    /** OS is running as non native */
    Os_Non_Native = 0,
    /** OS is running as Native */
    Os_Native,
    /** OS detection failed */
    Os_Detection_Failed = -1
} NvOsType;

/** @} */ /* nvhv_user_api_group */

#if defined(__cplusplus)
}
#endif  /* __cplusplus */

#endif
