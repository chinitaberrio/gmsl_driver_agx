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
// Copyright (c) 2015-2019 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef COMMON_CAMERAFRAMEPIPELINE_HPP_
#define COMMON_CAMERAFRAMEPIPELINE_HPP_

// Driveworks
#include <dw/core/Context.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/Image.h>

// C++ Std
#include <memory>
#include <vector>
#include <type_traits>
#include <chrono>
#include <thread>

// Common
#include <framework/Checks.hpp>
#include <framework/SimpleStreamer.hpp>

namespace dw_samples
{
namespace common
{

//-------------------------------------------------------------------------------
/**
* Class to process camera frame. Supports streaming and converting
* (once, in that order) so the returned image is in the expected format.
* The real type matches the type requested in the output properties.
*/
class CameraFramePipeline
{
public:
    CameraFramePipeline(const dwImageProperties& inputImageProperties,
                        dwCameraOutputType outputType,
                        dwContextHandle_t ctx);

    virtual ~CameraFramePipeline();

    /// Sets up output properties. Initializes streamers and converters
    /// if required.
    virtual void setOutputProperties(const dwImageProperties& outputImageProperties);

    /// Performs single camera frame processing.
    virtual void processFrame(dwCameraFrameHandle_t cameraFrame);

    /// Acquire the latest processed frame. Only valid when GL
    /// output is enabled.
    dwImageHandle_t getFrameRgba();
    dwImageHandle_t getFrame();

    bool isRgbaOutputEnabled() const { return m_imageRgba != DW_NULL_HANDLE; }

    /// Enables conversion to rgba format
    void enableRgbaOutput();

    const dwImageProperties& getImageProperties() const { return m_inputImgProps; }
    virtual const dwImageProperties& getOutputProperties() const { return m_outputImgProps; }
    dwImageProperties getRgbaOutputProperties() const;

    /// virtual method indicating softISP is present, always false for SimpleCamera
    virtual bool isSoftISPEnabled() const { return false; }

    virtual void reset();

protected:
    dwContextHandle_t m_ctx;

    dwCameraOutputType m_outputType;
    dwImageProperties m_inputImgProps;
    dwImageProperties m_outputImgProps;

    std::unique_ptr<SimpleImageStreamer<>> m_streamer;
    dwImageHandle_t m_converter = nullptr;

    dwImageHandle_t m_image     = nullptr;
    dwImageHandle_t m_imageRgba = nullptr;
};
}
}

#endif
