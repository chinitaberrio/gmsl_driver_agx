/*
 * This code has been modified from Nvidia SDK
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  All rights reserved.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*
  This program requires Nvidia SDK installed
  Modified from Nvidia SDK - Camera gmsl and others (see Readme)
  Author: Punnu Phairatt
  Initial Date: 10/05/18
*/

#ifndef _DRIVE_WORKS_API_H_
#define _DRIVE_WORKS_API_H_

#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <memory>

#include <signal.h>
#include <iostream>
#include <cstring>
#include <thread>
#include <queue>

#ifdef LINUX
#include <execinfo.h>
#include <unistd.h>
#endif

#include <functional>
#include <list>
#include <iomanip>

#include <chrono>
#include <mutex>
#include <condition_variable>

#include <dw/core/Context.h>
#include <dw/core/Logger.h>
#include <dw/core/VersionCurrent.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/SensorSerializer.h>

#include <dw/image/Image.h>
#include <dw/interop/streamer/ImageStreamer.h>

#include <drive-t186ref-linux/include/nvmedia_image.h>
#include <drive-t186ref-linux/include/nvmedia_ijpe.h>

#include "cv_connection.hpp"
#include "DeviceArguments.hpp"
#include <folly/ProducerConsumerQueue.h>


namespace DriveWorks {
  struct Camera {
    dwSensorHandle_t sensor;
    uint32_t numSiblings;
    uint32_t width;
    uint32_t height;
    dwImageStreamerHandle_t streamer;
    std::queue<dwImageHandle_t *> rgbaPool;
    std::vector<dwImageHandle_t> frameRGBA;
    std::queue<uint8_t *> jpegPool;
    std::vector<NvMediaIJPE *> jpegEncoders;
  };

  struct ImageConfigPub {
    uint32_t image_width;
    uint32_t image_height;
    uint32_t buffer_size;
    bool is_compressed;
    uint32_t jpeg_quality;
    std::string camerainfo_folder;
  };

  class DriveWorksApi {
  public:
    explicit DriveWorksApi(const DeviceArguments &arguments,
                           const ImageConfigPub &pub_image_config);

    void stopCameras();

    bool isCamReady();

    bool isShutdownCompleted();

    uint32_t getNumPort();

    std::vector<uint32_t> getCameraPort();

  private:
    static void InitializeContextHandle(dwContextHandle_t &context_handle);

    static void InitializeSalHandle(dwSALHandle_t &sal_handle,
                                    const dwContextHandle_t &context_handle);

    void initSensors(std::vector<Camera> *cameras,
                     uint32_t *numCameras,
                     dwSALHandle_t sal,
                     DeviceArguments &arguments);

    void initFramesStart();

    void initFrameImage(Camera *cameraSensor);

    void startCameraPipline();

    void WorkerPortPipeline(Camera *cameraSensor, uint32_t port, dwContextHandle_t sdk);

    dwStatus captureCamera(dwImageHandle_t *frameNVMrgba,
                           dwSensorHandle_t cameraSensor, uint32_t port,
                           uint32_t sibling,
                           uint8_t *jpeg_image, NvMediaIJPE *jpegEncoder);

    void releaseCameras(Camera *cameraSensor);

    void releaseSDK();


  private:
    bool gTakeScreenshot = true;
    int gScreenshotCount = 0;
    bool is_running_{false};
    bool g_exitCompleted = false;
    bool g_initState = false;
    uint32_t g_numCameras;
    uint32_t g_numPort;
    std::vector<uint32_t> g_numCameraPort;
    const uint32_t max_jpeg_bytes = 3 * 1290 * 1208;

    DeviceArguments device_arguments_;
    ImageConfigPub pub_image_config_;
    std::vector<Camera> cameras;
    bool eof;
    dwContextHandle_t context_handle_ = DW_NULL_HANDLE;
    dwSALHandle_t sal_handle_ = DW_NULL_HANDLE;
    std::vector<std::vector<dwImageHandle_t *>> g_frameRGBAPtr;
    std::vector<std::vector<uint8_t *>> g_frameJPGPtr;
    std::vector<std::vector<uint32_t>> g_compressedSize;

  };

};


#endif
