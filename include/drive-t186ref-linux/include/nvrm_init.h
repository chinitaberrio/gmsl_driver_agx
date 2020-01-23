/*
 * Copyright (c) 2007-2018 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_nvrm_init_H
#define INCLUDED_nvrm_init_H


#if defined(__cplusplus)
extern "C"
{
#endif


/** @file
 * @brief <b>NVIDIA Driver Development Kit:
 *     Resource Manager %Initialization API</b>
 *
 * @b Description: Declares the Resource Manager (RM) initialization API.
 */

/** @defgroup nvrm_init RM Initialization
 *
 * This file declares the Resource Manager initialization API.
 * @ingroup nvddk_rm
 *  @{
 */

#include "nvcommon.h"
#include "nverror.h"

/**
 * An opaque handle to an RM device.
 */

typedef struct NvRmDeviceRec *NvRmDeviceHandle;


// XXX We should probably get rid of this and just use NvU32.  It's rather
// difficult to explain what exactly NvRmPhysAddr is.  Also, what if some units
// are upgraded to do 64-bit addressing and others remain 32?  Would we really
// want to increase NvRmPhysAddr to NvU64 across the board?
//
// Another option would be to put the following types in nvcommon.h:
//   typedef NvU32 NvPhysAddr32;
//   typedef NvU64 NvPhysAddr64;
// Using these types would then be purely a form of documentation and nothing
// else.
//
// This header file is a somewhat odd place to put this type.  Putting it in
// memmgr would be even worse, though, because then a lot of header files would
// all suddenly need to #include nvrm_memmgr.h just to get the NvRmPhysAddr
// type.  (They already all include this header anyway.)

/**
 * A physical address type sized such that it matches the addressing support of
 * the hardware modules with which RM typically interfaces. May be smaller than
 * an ::NvOsPhysAddr.
 */

typedef NvU32 NvRmPhysAddr;

/** Macro to detect presence of NvRmOpenNew/NvRmClose API definition in
 * the header itself.
 */
#define NVRM_OPEN_CLOSE_DEFINED

/**
 * Deprecated API.
 */
static NvError NV_UNUSED NvRmOpenNew(NvRmDeviceHandle *pHandle) {
	*pHandle = (NvRmDeviceHandle)1;
	return NvSuccess;
}

/**
 * Deprecated API.
 */
static void NV_UNUSED NvRmClose(NvRmDeviceHandle NV_UNUSED hDevice) { }

/** @} */

#if defined(__cplusplus)
}
#endif

#endif
