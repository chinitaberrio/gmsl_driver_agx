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

#include "SimpleCamera.hpp"

#include <framework/Log.hpp>

namespace dw_samples
{
namespace common
{

SimpleCamera::SimpleCamera(dwSALHandle_t sal,
                           dwContextHandle_t ctx)
    : m_ctx(ctx)
    , m_sal(sal)
    , m_pendingFrame(nullptr)
    , m_started(false)
{
}

SimpleCamera::SimpleCamera(const dwSensorParams& params,
                           dwSALHandle_t sal,
                           dwContextHandle_t ctx,
                           dwCameraOutputType outputType)
    : SimpleCamera(sal, ctx)
{
    dwImageProperties imageProperties;

    createSensor(imageProperties, params, outputType);

    m_framePipeline.reset(new CameraFramePipeline(imageProperties, outputType, ctx));
}

SimpleCamera::SimpleCamera(const dwImageProperties& outputProperties,
                           const dwSensorParams& params,
                           dwSALHandle_t sal,
                           dwContextHandle_t ctx,
                           dwCameraOutputType outputType)
    : SimpleCamera(params, sal, ctx, outputType)
{
    m_framePipeline->setOutputProperties(outputProperties);
}

SimpleCamera::~SimpleCamera()
{
    if (m_pendingFrame)
        releaseFrame();

    if (m_sensor)
    {
        if (m_started)
            dwSensor_stop(m_sensor);
        dwSAL_releaseSensor(m_sensor);
    }
}

void SimpleCamera::createSensor(dwImageProperties& imageProps, const dwSensorParams& params, dwCameraOutputType outputType)
{
    CHECK_DW_ERROR(dwSAL_createSensor(&m_sensor, params, m_sal));
    CHECK_DW_ERROR(dwSensorCamera_getSensorProperties(&m_cameraProperties, m_sensor));
    CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&imageProps, outputType, m_sensor));

    log("SimpleCamera: Camera image: %ux%u\n", imageProps.width, imageProps.height);
}

void SimpleCamera::setOutputProperties(const dwImageProperties& outputProperties)
{
    m_framePipeline->setOutputProperties(outputProperties);
}

dwImageHandle_t SimpleCamera::readFrame()
{
    if (!m_started)
    {
        CHECK_DW_ERROR(dwSensor_start(m_sensor));
        m_started = true;
    }

    if (m_pendingFrame)
        releaseFrame();

    dwStatus status = dwSensorCamera_readFrameNew(&m_pendingFrame, 1000000, m_sensor);

    if (status == DW_END_OF_STREAM)
    {
        log("SimpleCamera: Camera reached end of stream.\n");
        return nullptr;
    }
    else if (status == DW_NOT_READY)
    {
        while (status == DW_NOT_READY)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            status = dwSensorCamera_readFrameNew(&m_pendingFrame, 1000000, m_sensor);
        }
    }
    else if (status != DW_SUCCESS)
    {
        throw std::runtime_error("Error reading from camera");
    }

    m_framePipeline->processFrame(m_pendingFrame);
    if (isGLOutputEnabled())
    {
        m_imageRgbaGL = m_streamerGL->post(m_framePipeline->getFrameRgba());
    }

    return m_framePipeline->getFrame();
}

bool SimpleCamera::enableSeeking(size_t& frameCount, dwTime_t& startTimestamp, dwTime_t& endTimestamp)
{
    // Try to get seek range of a sensor
    dwStatus res = dwSensor_getSeekRange(&frameCount, &startTimestamp, &endTimestamp, m_sensor);
    if (res != DW_SUCCESS)
    {
        // Seek table has not been created. Trying to create here.
        res = dwSensor_createSeekTable(m_sensor);
        if (res != DW_SUCCESS)
        {
            logError("SimpleCamera: Error creating index table: %s. Seeking is not available.\n", dwGetStatusName(res));
            return false;
        }

        CHECK_DW_ERROR_MSG(dwSensor_getSeekRange(&frameCount, &startTimestamp, &endTimestamp, m_sensor),
                           "Cannot obtain seek range from the camera.");
    }

    return true;
}

void SimpleCamera::seekToTime(dwTime_t timestamp)
{
    dwStatus res = dwSensor_seekToTime(timestamp, m_sensor);

    if (res != DW_SUCCESS)
    {
        logError("SimpleCamera: seek to time failed with %s.\n", dwGetStatusName(res));
    }
}

void SimpleCamera::seekToFrame(size_t frameIdx)
{
    dwStatus res = dwSensor_seekToEvent(frameIdx, m_sensor);

    if (res != DW_SUCCESS)
    {
        logError("SimpleCamera: seek to frame failed with %s.\n", dwGetStatusName(res));
    }
}

dwImageHandle_t SimpleCamera::getFrameRgbaGL()
{
    if (!isGLOutputEnabled())
    {
        logWarn("SimpleCamera: GL output is not enabled. Did you forget to call enableGLOutput()?\n");
    }
    return m_imageRgbaGL;
}

void SimpleCamera::releaseFrame()
{
    if (m_pendingFrame)
    {
        CHECK_DW_ERROR(dwSensorCamera_returnFrame(&m_pendingFrame));
        m_pendingFrame = nullptr;
    }
}

void SimpleCamera::resetCamera()
{
    m_framePipeline->reset();
    releaseFrame();
    CHECK_DW_ERROR(dwSensor_reset(m_sensor));
}

void SimpleCamera::enableGLOutput()
{
    if (!m_framePipeline->isRgbaOutputEnabled())
    {
        m_framePipeline->enableRgbaOutput();
    }

    dwImageProperties props;
    dwImage_getProperties(&props, m_framePipeline->getFrameRgba());

    m_streamerGL.reset(new SimpleImageStreamerGL<>(props, 60000, m_ctx));
}
}
}
