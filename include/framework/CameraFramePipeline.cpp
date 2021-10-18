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

#include "CameraFramePipeline.hpp"

#include <framework/Log.hpp>

namespace dw_samples
{
namespace common
{

///////////////////////////////////////////////////////////////////////////////
/// CameraFramePipeline
///////////////////////////////////////////////////////////////////////////////

CameraFramePipeline::CameraFramePipeline(const dwImageProperties& inputImageProperties,
                                         dwCameraOutputType outputType,
                                         dwContextHandle_t ctx)
    : m_ctx(ctx)
    , m_outputType(outputType)
    , m_inputImgProps(inputImageProperties)
    , m_outputImgProps(inputImageProperties)
{
}

CameraFramePipeline::~CameraFramePipeline()
{
    if (m_converter != DW_NULL_HANDLE)
        dwImage_destroy(m_converter);

    if (m_imageRgba != DW_NULL_HANDLE)
        dwImage_destroy(m_imageRgba);
}

void CameraFramePipeline::setOutputProperties(const dwImageProperties& outputImageProperties)
{
    m_outputImgProps        = outputImageProperties;
    m_outputImgProps.width  = m_inputImgProps.width;
    m_outputImgProps.height = m_inputImgProps.height;

    if (m_inputImgProps.type != m_outputImgProps.type)
    {
        m_streamer.reset(new SimpleImageStreamer<>(m_inputImgProps, m_outputImgProps.type, 60000, m_ctx));
    }

    if (m_inputImgProps.format != m_outputImgProps.format)
    {
        dwImage_create(&m_converter, m_outputImgProps, m_ctx);
    }
}

void CameraFramePipeline::enableRgbaOutput()
{
    const dwImageProperties& outProps = getOutputProperties();

    dwImageProperties propsRgba{};

    propsRgba.format       = DW_IMAGE_FORMAT_RGBA_UINT8;
    propsRgba.type         = outProps.type;
    propsRgba.width        = outProps.width;
    propsRgba.height       = outProps.height;
    propsRgba.memoryLayout = outProps.memoryLayout;

    CHECK_DW_ERROR(dwImage_create(&m_imageRgba, propsRgba, m_ctx));
}

dwImageProperties CameraFramePipeline::getRgbaOutputProperties() const
{
    dwImageProperties props{};
    dwImage_getProperties(&props, m_imageRgba);
    return props;
}

dwImageHandle_t CameraFramePipeline::getFrame()
{
    return m_image;
}

dwImageHandle_t CameraFramePipeline::getFrameRgba()
{
    if (!isRgbaOutputEnabled())
    {
        logWarn("CameraFramePipeline: RGBA output is not enabled. Did you forget to call enableRgbaOutput()?\n");
    }
    return m_imageRgba;
}

void CameraFramePipeline::processFrame(dwCameraFrameHandle_t cameraFrame)
{
    dwImageHandle_t img;
    CHECK_DW_ERROR(dwSensorCamera_getImage(&img, m_outputType, cameraFrame));

    m_image = img;
    if (m_streamer)
    {
        m_image = m_streamer->post(img);
    }

    if (m_converter)
    {
        dwStatus res = dwImage_copyConvert(m_converter, m_image, m_ctx);
        if (res != DW_SUCCESS)
        {
            logError("CameraFramePipeline: error converting image: %s\n", dwGetStatusName(res));
        }
        m_image = m_converter;
    }

    // Rgba & OpenGL
    // If SoftIsp pipeline is enabled it needs to be processed first
    if (!isSoftISPEnabled())
    {
        if (isRgbaOutputEnabled())
        {
            dwImage_copyConvert(m_imageRgba, m_image, m_ctx);
        }
    }
}

void CameraFramePipeline::reset()
{
    if (m_streamer)
    {
        m_streamer->release();
    }
}
}
}
