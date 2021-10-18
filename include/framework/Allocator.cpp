/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2019-2020 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <memory>
#include <dw/core/DynamicMemory.h>

/////////////////////////////////////////////////////////////////////////////////////////
/// This file replaces the default C++ new/delete operators.
/// We will be using dwDynamicMemory API for allocations. Per default it uses posix_memalign()
/// to ensure that all allocations are 16-byte aligned. Any allocator which respects
/// alignment requirements can be used for application code using DriveWorks library.
///
/// This is needed because STL in C++14 does not provide aligned allocations.
/// For example: dwVector4f requires 16-byte alignment
///              but std::vector<dwVector4f> does not align by default.
///
/// No header file is needed. Simply linking this file into the executable/library
/// will replace the operators. Note that these operators should only be replaced
/// once. If a different operator new/delete is needed, do not link this file and
/// ensure that the other operator new returns aligned data.
/////////////////////////////////////////////////////////////////////////////////////////

void* operator new(size_t size)
{
    return dwDynamicMemory_malloc(size);
}
void* operator new(std::size_t size, const std::nothrow_t&) noexcept
{
    return dwDynamicMemory_malloc(size);
}

void operator delete(void* ptr) noexcept
{
    dwDynamicMemory_free(ptr);
}
void operator delete(void* ptr, const std::nothrow_t&)noexcept
{
    dwDynamicMemory_free(ptr);
}
void operator delete(void* ptr, size_t) noexcept
{
    dwDynamicMemory_free(ptr);
}
void operator delete(void* ptr, std::size_t, const std::nothrow_t&)noexcept
{
    dwDynamicMemory_free(ptr);
}
