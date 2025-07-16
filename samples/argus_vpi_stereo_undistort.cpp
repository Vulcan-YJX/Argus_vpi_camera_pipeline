/*
 * Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "opencv2/opencv.hpp"
#include "ArgusHelpers.h"
#include "EGLGlobal.h"
#include "Error.h"
#include "Thread.h"
#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <map>
#include <bitset>
#include <Argus/Ext/SyncSensorCalibrationData.h>
#include <Argus/Ext/SensorTimestampTsc.h>
#include <EGLStream/NV/ImageNativeBuffer.h>

#include <vpi/Event.h>
#include <vpi/Image.h>
#include <vpi/Stream.h>
#include <vpi/algo/Remap.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/LensDistortionModels.h>


class shutdown;
using namespace Argus;
using namespace EGLStream;


#define CHECK_STATUS(STMT)                                    \
    do                                                        \
    {                                                         \
        VPIStatus status = (STMT);                            \
        if (status != VPI_SUCCESS)                            \
        {                                                     \
            char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH];       \
            vpiGetLastStatusMessage(buffer, sizeof(buffer));  \
            std::ostringstream ss;                            \
            ss << vpiStatusGetName(status) << ": " << buffer; \
            throw std::runtime_error(ss.str());               \
        }                                                     \
    } while (0);

namespace ArgusSamples
{

/*
 * This sample opens sessions based on the number of stereo/sensor modules
 * connected. Each module can have 1 sensor or multiple (2) sensors connected.
 * The processing of the images happens in the worker thread, while the main
 * app thread is used to drive the captures.
 */

// Constants.
static const Size2D<uint32_t> STREAM_SIZE(1920, 1200);

// Debug print macros.
#define printf(...) printf("PRODUCER: " __VA_ARGS__)
#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)

// For stereo camera, maximum number of devices supported in a single session is 2
enum maxCamDevice
{
    LEFT_CAM_DEVICE  = 0,
    RIGHT_CAM_DEVICE = 1,
    MAX_CAM_DEVICE = 2
};

static const float FRAMERATE_DEFAULT = 30.0f;

// forward declaration
class SyncStereoConsumerThread;

#define MAX_MODULE_STRING 32

#define MAX_MODULE_COUNT 2

#define MAX_HAWK_MODULE_COUNT 4

#define MIN_MODULE_DEVICE_COUNT 1

#define MaxEGLStreamBuffers 12

// Threshold to detect out of sync detection
#define SYNC_THRESHOLD_TIME_US 100.0f

typedef struct
{
    char moduleName[MAX_MODULE_STRING];
    int camDevice[MAX_CAM_DEVICE];
    OutputStream *stream[MAX_CAM_DEVICE];
    CaptureSession *captureSession;
    OutputStreamSettings *streamSettings;
    Request *request;
    bool isCaptureSessionActive;
    SyncStereoConsumerThread *syncStereoConsumer;
    int sensorCount;
    bool initialized;
    VPICameraIntrinsic VPI_K[MAX_CAM_DEVICE] = {};
    VPIPolynomialLensDistortionModel VPI_distModel[MAX_CAM_DEVICE] = {};
    VPIPayload remap[MAX_CAM_DEVICE] = {};
    VPIWarpMap map[MAX_CAM_DEVICE] = {};
} ModuleInfo;


/*******************************************************************************
 * Argus disparity class
 * This class will analyze frames from 2 synchronized sensors
 ******************************************************************************/
class SyncStereoConsumerThread : public Thread
{
public:
    explicit SyncStereoConsumerThread(ModuleInfo *modInfo,
                                      uint16_t* sessionMask,
                                      int cameraDevices_size)
    {
        cameraDevices_size_ = cameraDevices_size;
        m_modInfo_ = modInfo;
        asyncCount = 0;
        sessionsMask = sessionMask;
        m_leftStream = modInfo->stream[LEFT_CAM_DEVICE];
        camDevices.push_back(modInfo->camDevice[0]);
        if (modInfo->sensorCount > 1) {
            m_rightStream = modInfo->stream[RIGHT_CAM_DEVICE];
            camDevices.push_back(modInfo->camDevice[1]);
        }
        else
            m_rightStream = NULL;

        strcpy(m_moduleName, modInfo->moduleName);
    }
    ~SyncStereoConsumerThread()
    {
        CONSUMER_PRINT("DESTRUCTOR  ... \n");
    }

private:
    /** @name Thread methods */
    /**@{*/
    virtual bool threadInitialize();
    virtual bool threadExecute();
    virtual bool threadShutdown();
    /**@}*/

    /* Assumption: We only have a Left-Right pair.
     * OutputStream and FrameConsumer should be created to a vector of
     * MAX_CAM_DEVICE size.
     */
    OutputStream *m_leftStream; // left stream tied to sensor index 0 and is used for autocontrol.
    OutputStream *m_rightStream; // right stream tied to sensor index 1.

    char m_moduleName[MAX_MODULE_STRING];
    UniqueObj<FrameConsumer> m_leftConsumer;
    UniqueObj<FrameConsumer> m_rightConsumer;
    uint64_t asyncCount;
    uint64_t syncCount;
    std::vector<int> camDevices;
    uint16_t* sessionsMask;
    ModuleInfo *m_modInfo_;
    int cameraDevices_size_ = 0;
};

ICaptureSession* g_iCaptureSession[MAX_MODULE_COUNT];

bool SyncStereoConsumerThread::threadInitialize()
{
    CONSUMER_PRINT("Creating FrameConsumer for left stream\n");
    m_leftConsumer = UniqueObj<FrameConsumer>(FrameConsumer::create(m_leftStream));
    if (!m_leftConsumer)
        printf("Failed to create FrameConsumer for left stream");

    if (m_rightStream)
    {
        CONSUMER_PRINT("Creating FrameConsumer for right stream\n");
        m_rightConsumer = UniqueObj<FrameConsumer>(FrameConsumer::create(m_rightStream));
        if (!m_rightConsumer)
            printf("Failed to create FrameConsumer for right stream");
    }

    return true;
}

bool SyncStereoConsumerThread::threadExecute()
{
    IEGLOutputStream *iLeftStream = interface_cast<IEGLOutputStream>(m_leftStream);
    IFrameConsumer* iFrameConsumerLeft = interface_cast<IFrameConsumer>(m_leftConsumer);

    IFrameConsumer* iFrameConsumerRight = NULL;
    if (m_rightStream)
    {
        IEGLOutputStream *iRightStream = interface_cast<IEGLOutputStream>(m_rightStream);
        iFrameConsumerRight = interface_cast<IFrameConsumer>(m_rightConsumer);
        if (!iFrameConsumerRight)
        {
            printf("[%s]: Failed to get right stream cosumer\n", m_moduleName);
        }
        // Wait until the producer has connected to the stream.
        CONSUMER_PRINT("[%s]: Waiting until Argus producer is connected to right stream...\n",
            m_moduleName);
        if (iRightStream->waitUntilConnected() != STATUS_OK)
            printf("Argus producer failed to connect to right stream.");
        CONSUMER_PRINT("[%s]: Argus producer for right stream has connected; continuing.\n",
            m_moduleName);
    }

    // Wait until the producer has connected to the stream.
    CONSUMER_PRINT("[%s]: Waiting until Argus producer is connected to left stream...\n",
        m_moduleName);
    if (iLeftStream->waitUntilConnected() != STATUS_OK)
        printf("[%s]Argus producer failed to connect to left stream.\n", m_moduleName);
    CONSUMER_PRINT("[%s]: Argus producer for left stream has connected; continuing.\n",
        m_moduleName);

    unsigned long long tscTimeStampLeft = 0, tscTimeStampLeftNew = 0;
    unsigned long long frameNumberLeft = 0;
    unsigned long long tscTimeStampRight = 0, tscTimeStampRightNew = 0;
    unsigned long long frameNumberRight = 0;
    unsigned long long diff = 0;
    IFrame *iFrameLeft = NULL;
    IFrame *iFrameRight = NULL;
    Frame *frameleft = NULL;
    Frame *frameright = NULL;
    Ext::ISensorTimestampTsc *iSensorTimestampTscLeft = NULL;
    Ext::ISensorTimestampTsc *iSensorTimestampTscRight = NULL;
    bool leftDrop = false;
    bool rightDrop = false;
    asyncCount = 0;
    syncCount = 0;
    int m_dmabufLeft = -1, m_dmabufRight = -1;
    int stream_width = iLeftStream->getResolution().width();
    int stream_height = iLeftStream->getResolution().height();
    
    // int imgFlags = VPI_BACKEND_CPU | VPI_BACKEND_CUDA;
    VPIImage srcImage_l = NULL, srcImage_r = NULL;
    VPIImage dstImage_l = NULL, dstImage_r = NULL;
    VPIImage remapOut_l = NULL, remapOut_r = NULL;
    VPIEvent parentEvent = NULL;
    VPIImageData imgData_l = {}, imgData_r = {};
    VPIStream streams[2] = {};
    // VPIImage tmpIn = NULL;
    VPIImageData outData_l,outData_r;
    cv::Mat cvImage_l,cvImage_r;

    CHECK_STATUS(vpiImageCreate(stream_width, stream_height, VPI_IMAGE_FORMAT_BGR8, 0, &dstImage_l));
    CHECK_STATUS(vpiImageCreate(stream_width, stream_height, VPI_IMAGE_FORMAT_BGR8, 0, &dstImage_r));
    CHECK_STATUS(vpiImageCreate(stream_width, stream_height, VPI_IMAGE_FORMAT_NV12_ER, 0, &remapOut_l));
    CHECK_STATUS(vpiImageCreate(stream_width, stream_height, VPI_IMAGE_FORMAT_NV12_ER, 0, &remapOut_r));

    // Create 1 event for the parent image
    CHECK_STATUS(vpiEventCreate(0, &parentEvent));
  
    for (int i = 0; i < 2; ++i)
    {
        // Create 2 streams to execute algorithms on
        CHECK_STATUS(vpiStreamCreate(0, &streams[i]));
    }

    for(int i = 0 ;i < 2;i++){
        VPICameraExtrinsic X = {};
        X[0][0] = X[1][1] = X[2][2] = 1;
        m_modInfo_[0].map[i].grid.numHorizRegions  = 1;
        m_modInfo_[0].map[i].grid.numVertRegions   = 1;
        m_modInfo_[0].map[i].grid.regionWidth[0]   = stream_width;
        m_modInfo_[0].map[i].grid.regionHeight[0]  = stream_height;
        m_modInfo_[0].map[i].grid.horizInterval[0] = 1;
        m_modInfo_[0].map[i].grid.vertInterval[0]  = 1;
        CHECK_STATUS(vpiWarpMapAllocData(&m_modInfo_[0].map[i]));
        vpiWarpMapGenerateFromPolynomialLensDistortionModel(m_modInfo_[0].VPI_K[i], X, m_modInfo_[0].VPI_K[i], &m_modInfo_[0].VPI_distModel[i], &m_modInfo_[0].map[i]);
        CHECK_STATUS(vpiCreateRemap(VPI_BACKEND_CUDA, &m_modInfo_[0].map[i], &m_modInfo_[0].remap[i]));
        vpiWarpMapFreeData(&m_modInfo_[0].map[i]);
    }

    while (true)
    {
        if ((diff/1000.0f < SYNC_THRESHOLD_TIME_US) || leftDrop)
        {
            frameleft = iFrameConsumerLeft->acquireFrame();
            if (!frameleft)
                break;

            leftDrop = false;

            // Use the IFrame interface to print out the frame number/timestamp, and
            // to provide access to the Image in the Frame.
            iFrameLeft = interface_cast<IFrame>(frameleft);
            if (!iFrameLeft)
                printf("Failed to get left IFrame interface.");

            CaptureMetadata* captureMetadataLeft =
                    interface_cast<IArgusCaptureMetadata>(frameleft)->getMetadata();
            ICaptureMetadata* iMetadataLeft = interface_cast<ICaptureMetadata>(captureMetadataLeft);
            if (!captureMetadataLeft || !iMetadataLeft)
                printf("Cannot get metadata for frame left");

            if (iMetadataLeft->getSourceIndex() != LEFT_CAM_DEVICE)
                printf("Incorrect sensor connected to Left stream");

            iSensorTimestampTscLeft =
                                interface_cast<Ext::ISensorTimestampTsc>(captureMetadataLeft);
            if (!iSensorTimestampTscLeft)
                printf("failed to get iSensorTimestampTscLeft inteface");

            tscTimeStampLeftNew = iSensorTimestampTscLeft->getSensorSofTimestampTsc();
            frameNumberLeft = iFrameLeft->getNumber();
            EGLStream::Image *imageLeft = iFrameLeft->getImage();
            EGLStream::NV::IImageNativeBuffer *iNativeBuffer =
                Argus::interface_cast<EGLStream::NV::IImageNativeBuffer>(imageLeft);
            if (!iNativeBuffer)
                printf("IImageNativeBuffer not supported by Image.");
            if (m_dmabufLeft == -1)
            {
                m_dmabufLeft = iNativeBuffer->createNvBuffer(iLeftStream->getResolution(),
                                                        NVBUF_COLOR_FORMAT_NV12_ER,
                                                        NVBUF_LAYOUT_PITCH);
                if (m_dmabufLeft == -1)
                    printf("\tFailed to create NvBuffer\n");
            }
            else if (iNativeBuffer->copyToNvBuffer(m_dmabufLeft) != Argus::STATUS_OK)
            {
                printf("Failed to copy frame to NvBuffer.");
            }
            {
                // printf("\tcreate NvBuffer\n");
                NvBufSurface *nvbuf_surf = 0;
                if(NvBufSurfaceFromFd (m_dmabufLeft, (void**)(&nvbuf_surf)) < 0)
                {
                    printf("Failed to copy frame to NvBuffer.");
                }
                else
                {
                    if (nvbuf_surf->surfaceList[0].colorFormat == NVBUF_COLOR_FORMAT_NV12_ER){

                        NvBufSurfaceMapParams buffer_params;
                        NvBufSurfaceGetMapParams(nvbuf_surf, 0, &buffer_params);

                        imgData_l.bufferType = VPI_IMAGE_BUFFER_NVBUFFER;
                        imgData_l.buffer.fd = buffer_params.fd;
                        if(srcImage_l == nullptr){
                            CHECK_STATUS(vpiImageCreateWrapper(&imgData_l,nullptr,VPI_BACKEND_CUDA,&srcImage_l));
                        }else{
                            CHECK_STATUS(vpiImageSetWrapper(srcImage_l,&imgData_l));
                        }
                        CHECK_STATUS(vpiSubmitRemap(streams[0], VPI_BACKEND_CUDA, m_modInfo_[0].remap[1], srcImage_l, remapOut_l, VPI_INTERP_CATMULL_ROM,
                                                    VPI_BORDER_ZERO, 0));
                        CHECK_STATUS(vpiSubmitConvertImageFormat(streams[0], VPI_BACKEND_CUDA, remapOut_l, dstImage_l, NULL));
                        CHECK_STATUS(vpiEventRecord(parentEvent, streams[0]));
                        CHECK_STATUS(vpiStreamWaitEvent(streams[1], parentEvent));
                    }
                }
            }

        }

        if (m_rightStream && ((diff/1000.0f < SYNC_THRESHOLD_TIME_US) || rightDrop))
        {
            frameright = iFrameConsumerRight->acquireFrame();
            if (!frameright)
                break;

            rightDrop = false;

            // Use the IFrame interface to print out the frame number/timestamp, and
            // to provide access to the Image in the Frame.
            iFrameRight = interface_cast<IFrame>(frameright);
            if (!iFrameRight)
                printf("Failed to get right IFrame interface.");

            CaptureMetadata* captureMetadataRight =
                    interface_cast<IArgusCaptureMetadata>(frameright)->getMetadata();
            ICaptureMetadata* iMetadataRight = interface_cast<ICaptureMetadata>(captureMetadataRight);
            if (!captureMetadataRight || !iMetadataRight)
            {
                printf("Cannot get metadata for frame right");
            }
            if (iMetadataRight->getSourceIndex() != RIGHT_CAM_DEVICE)
                printf("Incorrect sensor connected to Right stream");

            iSensorTimestampTscRight =
                                interface_cast<Ext::ISensorTimestampTsc>(captureMetadataRight);
            if (!iSensorTimestampTscRight)
                printf("failed to get iSensorTimestampTscRight inteface");

            tscTimeStampRightNew = iSensorTimestampTscRight->getSensorSofTimestampTsc2();
            frameNumberRight = iFrameRight->getNumber();

            EGLStream::Image *imageRight = iFrameRight->getImage();
            EGLStream::NV::IImageNativeBuffer *iNativeBuffer_r =
                Argus::interface_cast<EGLStream::NV::IImageNativeBuffer>(imageRight);
            if (!iNativeBuffer_r)
                printf("IImageNativeBuffer not supported by Image.");
            if (m_dmabufRight == -1)
            {
                m_dmabufRight = iNativeBuffer_r->createNvBuffer(iLeftStream->getResolution(),
                                                        NVBUF_COLOR_FORMAT_NV12_ER,
                                                        NVBUF_LAYOUT_PITCH);
                if (m_dmabufRight == -1)
                    printf("\tFailed to create NvBuffer\n");
            }
            else if (iNativeBuffer_r->copyToNvBuffer(m_dmabufRight) != Argus::STATUS_OK)
            {
                printf("Failed to copy frame to NvBuffer.");
            }
            {
                // printf("\t Right NvBuffer\n");
                NvBufSurface *nvbuf_surf = 0;
                if(NvBufSurfaceFromFd (m_dmabufRight, (void**)(&nvbuf_surf)) < 0)
                {
                    printf("Failed to copy frame to NvBuffer.");

                }else
                {
                    if (nvbuf_surf->surfaceList[0].colorFormat == NVBUF_COLOR_FORMAT_NV12_ER){

                        NvBufSurfaceMapParams buffer_params;
                        NvBufSurfaceGetMapParams(nvbuf_surf, 0, &buffer_params);

                        imgData_r.bufferType = VPI_IMAGE_BUFFER_NVBUFFER;
                        imgData_r.buffer.fd = buffer_params.fd;
                        if(srcImage_r == nullptr){
                            CHECK_STATUS(vpiImageCreateWrapper(&imgData_r,nullptr,VPI_BACKEND_CUDA,&srcImage_r));
                        }else{
                            CHECK_STATUS(vpiImageSetWrapper(srcImage_r,&imgData_r));
                        }
                        CHECK_STATUS(vpiSubmitRemap(streams[1], VPI_BACKEND_CUDA, m_modInfo_[0].remap[0], srcImage_r, remapOut_r, VPI_INTERP_CATMULL_ROM,
                            VPI_BORDER_ZERO, 0));

                        CHECK_STATUS(vpiSubmitConvertImageFormat(streams[1], VPI_BACKEND_CUDA, remapOut_r, dstImage_r, NULL));
                        CHECK_STATUS(vpiStreamSync(streams[0]));
                        CHECK_STATUS(vpiStreamSync(streams[1]));

                        // CHECK_STATUS(vpiImageLockData(dstImage_l, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &outData_l));
                        // CHECK_STATUS(vpiImageLockData(dstImage_r, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &outData_r));

                        // CHECK_STATUS(vpiImageDataExportOpenCVMat(outData_l, &cvImage_l));
                        // CHECK_STATUS(vpiImageDataExportOpenCVMat(outData_r, &cvImage_r));
                        // cv::imwrite("vpi1.png", cvImage_l);
                        // cv::imwrite("vpi2.png", cvImage_r);
                        // CHECK_STATUS(vpiImageUnlock(dstImage_l));
                        // CHECK_STATUS(vpiImageUnlock(dstImage_r));

                    }
                }
            }
        }
        tscTimeStampLeft = tscTimeStampLeftNew;
        if (m_rightStream)
        {
            tscTimeStampRight = tscTimeStampRightNew;
        }
        else
            tscTimeStampRight = tscTimeStampLeft;

        diff = llabs(tscTimeStampLeft - tscTimeStampRight);

        CONSUMER_PRINT("[%s]: left and right tsc timestamps (us): { %llu %llu }, difference (us): %f and frame number: { %llu %llu }\n",
            m_moduleName,
            tscTimeStampLeft/1000, tscTimeStampRight/1000,
            diff/1000.0f,
            frameNumberLeft, frameNumberRight);

        if (diff/1000.0f > SYNC_THRESHOLD_TIME_US)
        {
            // check if we heave to drop left frame i.e. re-acquire
            if (tscTimeStampLeft < tscTimeStampRight)
            {
                leftDrop = true;
                printf("CONSUMER:[%s]: number { %llu %llu } out of sync detected with diff %f us left is ahead *********\n",
                    m_moduleName, frameNumberLeft, frameNumberRight, diff/1000.0f );
                iFrameLeft->releaseFrame();
            }
            else
            {
                rightDrop = true;
                printf("CONSUMER:[%s]: number { %llu %llu } out of sync detected with diff %f us right is ahead *********\n",
                    m_moduleName, frameNumberLeft, frameNumberRight, diff/1000.0f );
                iFrameRight->releaseFrame();
            }
            asyncCount++;
            continue;
        }

        CONSUMER_PRINT("[%s] Synchronized frames captured count %ld.\n", m_moduleName, syncCount++);
        iFrameLeft->releaseFrame();

        if (m_rightStream)
        {
            iFrameRight->releaseFrame();
        }
    }

    CONSUMER_PRINT("No more frames. Cleaning up.\n");

    PROPAGATE_ERROR(requestShutdown());

    CONSUMER_PRINT("shutDown done\n");

    for (int i = 0; i < 2; ++i)
    {
        if (streams[i] != NULL)
        {
            vpiStreamSync(streams[i]);
        }
    }

    for (int i = 0; i < 2; ++i)
    {
        vpiStreamDestroy(streams[i]);
    }
    vpiEventDestroy(parentEvent);
    vpiImageDestroy(srcImage_l);
    vpiImageDestroy(srcImage_r);
    vpiImageDestroy(dstImage_l);
    vpiImageDestroy(dstImage_r);

    return true;
}

bool SyncStereoConsumerThread::threadShutdown()
{
    CONSUMER_PRINT("threadShutdown----------\n");
    CONSUMER_PRINT("[%s] Asynchronized frames captured count %ld.\n", m_moduleName, asyncCount);
    return true;
}

static void SyncStereoCalibrationData(
    const Ext::ISyncSensorCalibrationData *iSyncSensorCalibrationData, ModuleInfo& moduleInfo, int sensor_id)
{
    Size2D<uint32_t> ImageSize = iSyncSensorCalibrationData->getImageSizeInPixels();
    printf(" Image size = %d, %d\n", ImageSize.width(), ImageSize.height());

    Point2D<float> FocalLength = iSyncSensorCalibrationData->getFocalLength();
    printf(" Focal Length = %f, %f\n", FocalLength.x(), FocalLength.y());

    Point2D<float> PrincipalPoint = iSyncSensorCalibrationData->getPrincipalPoint();
    printf(" Principal Point = %f, %f\n", PrincipalPoint.x(), PrincipalPoint.y());

    float K_temp[2][3] = {
        {FocalLength.x(), 0.0, FocalLength.y()},
        {0.0, PrincipalPoint.x(), PrincipalPoint.y()}
    };
    printf(" sensor_id %d \n",sensor_id);
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            moduleInfo.VPI_K[sensor_id][i][j] = K_temp[i][j];
        }
    }
    float Skew = iSyncSensorCalibrationData->getSkew();
    printf(" Skew = %f\n", Skew);

    MappingType FishEyeMappingType = iSyncSensorCalibrationData->getFisheyeMappingType();
    printf(" Fish Eye mapping type = %s\n", FishEyeMappingType.getName());

    DistortionType LensDistortionType = iSyncSensorCalibrationData->getLensDistortionType();
    printf(" Lens Distortion type = %s\n", LensDistortionType.getName());

    uint32_t RadialCoeffsCount =
                        iSyncSensorCalibrationData->getRadialCoeffsCount(LensDistortionType);
    printf(" Radial coeffs count = %d\n", RadialCoeffsCount);

    std::vector<float> k;
    iSyncSensorCalibrationData->getRadialCoeffs(&k, LensDistortionType);

    moduleInfo.VPI_distModel[sensor_id].k1 = k[0];
    moduleInfo.VPI_distModel[sensor_id].k2 = k[1];
    moduleInfo.VPI_distModel[sensor_id].k3 = k[2];
    moduleInfo.VPI_distModel[sensor_id].k4 = k[3];
    moduleInfo.VPI_distModel[sensor_id].k5 = k[4];
    moduleInfo.VPI_distModel[sensor_id].k6 = k[5];
    printf(" Radial coefficients = ");
    for (uint32_t idx = 0; idx < k.size(); idx++)
    {
        printf("%f ", k[idx]);
    }

    uint32_t TangentialCoeffsCount =
        iSyncSensorCalibrationData->getTangentialCoeffsCount();
    printf(" Tangential coeffs count = %d\n", TangentialCoeffsCount);

    std::vector<float> p;
    iSyncSensorCalibrationData->getTangentialCoeffs(&p);

    moduleInfo.VPI_distModel[sensor_id].p1 = p[0];
    moduleInfo.VPI_distModel[sensor_id].p2 = p[1];
    printf(" Tangential coefficients = ");
    for (uint32_t idx = 0; idx < p.size(); idx++)
    {
        printf("%f ", p[idx]);
    }

    Point3D<float> rot3d = iSyncSensorCalibrationData->getRotationParams();
    printf(" rot3d x, y, z{%f, %f, %f}\n", rot3d.x(), rot3d.y(), rot3d.z());

    Point3D<float> translation = iSyncSensorCalibrationData->getTranslationParams();
    printf(" translation 3d x, y, z{%f, %f, %f}\n",
        translation.x(), translation.y(), translation.z());

    char moduleSerialNumber[MAX_MODULE_STRING];
    iSyncSensorCalibrationData->getModuleSerialNumber(
        moduleSerialNumber, sizeof(moduleSerialNumber));

    printf(" moduleSerialNumber %s\n", moduleSerialNumber);

    bool isImu = iSyncSensorCalibrationData->isImuSensorAvailable();
    if (isImu)
    {
        printf(" For IMU sensors \n");

        Point3D<float> linearAccBias = iSyncSensorCalibrationData->getLinearAccBias();
        printf("linearAccBias 3d x, y, z{%f, %f, %f}\n", linearAccBias.x(), linearAccBias.y(), linearAccBias.z());

        Point3D<float> angularVelocityBias = iSyncSensorCalibrationData->getAngularVelocityBias();
        printf("angularVelocityBias 3d x, y, z{%f, %f, %f}\n", angularVelocityBias.x(), angularVelocityBias.y(), angularVelocityBias.z());

        Point3D<float> gravityAcc = iSyncSensorCalibrationData->getGravityAcc();
        printf("gravityAcc 3d x, y, z{%f, %f, %f}\n", gravityAcc.x(), gravityAcc.y(), gravityAcc.z());

        Point3D<float> imuRotation = iSyncSensorCalibrationData->getImuRotationParams();
        printf("ImuRotation 3d x, y, z{%f, %f, %f}\n", imuRotation.x(), imuRotation.y(), imuRotation.z());

        Point3D<float> imuTranslationParams = iSyncSensorCalibrationData->getImuTranslationParams();
        printf("imuTranslationParams 3d x, y, z{%f, %f, %f}\n", imuTranslationParams.x(), imuTranslationParams.y(), imuTranslationParams.z());

        float updateRate = iSyncSensorCalibrationData->getUpdateRate();
        printf("updateRate %f", updateRate);

        float LinearAccNoiseDensity = iSyncSensorCalibrationData->getLinearAccNoiseDensity();
        printf("LinearAccNoiseDensity %f", LinearAccNoiseDensity);

        float LinearAccRandomWalk = iSyncSensorCalibrationData->getLinearAccRandomWalk();
        printf("LinearAccRandomWalk %f", LinearAccRandomWalk);

        float AngularVelNoiseDensity = iSyncSensorCalibrationData->getAngularVelNoiseDensity();
        printf("AngularVelNoiseDensity %f", AngularVelNoiseDensity);

        float AngularVelRandomWalk = iSyncSensorCalibrationData->getAngularVelRandomWalk();
        printf("AngularVelRandomWalk %f", AngularVelRandomWalk);
        printf("\n\n");
    }
}

static bool execute()
{
    ModuleInfo moduleInfo[MAX_MODULE_COUNT];
    int moduleCount = 0;
    int hawkModuleCount = 0;
    uint16_t sessionMask = 0;

    memset(&moduleInfo, 0, MAX_MODULE_COUNT*sizeof(ModuleInfo));
    for (int i = 0; i < MAX_MODULE_COUNT; i++)

        moduleInfo[i].initialized = false;

    // Initialize the Argus camera provider.
    UniqueObj<CameraProvider> cameraProvider(CameraProvider::create());

    // Get the ICameraProvider interface from the global CameraProvider.
    ICameraProvider *iCameraProvider = interface_cast<ICameraProvider>(cameraProvider);
    if (!iCameraProvider)
        printf("Failed to get ICameraProvider interface");
    printf("Argus Version: %s\n", iCameraProvider->getVersion().c_str());

    // Get the camera devices.
    std::vector<CameraDevice*> cameraDevices;
    iCameraProvider->getCameraDevices(&cameraDevices);
    if (cameraDevices.size() < 2)
        printf("Must have at least 2 sensors available");

    /**
     * For multiple HAWK modules, we need to map the available sensors
     * to identify which sensor belongs to which HAWK module.
     * In case we have non-HAWK modules, the sensors would be mapped accordingly.
     * Current assumption is that each HAWK module has only 2 sensors and
     * each non-HAWK modulMIN_MODULE_DEVICE_COUNTe has only a single sensor.
     *
     */

    char syncSensorId[MAX_MODULE_STRING];
    for (uint32_t i = 0; i < cameraDevices.size(); i++)
    {
        // printf("cameraDevices.size() : %d \n", i);
        Argus::ICameraProperties *iCameraProperties =
                        Argus::interface_cast<Argus::ICameraProperties>(cameraDevices[i]);
        if (!iCameraProperties)
            printf("Failed to get cameraProperties interface");

        printf("getSensorPlacement for sensor %d is %s\n",
                i, iCameraProperties->getSensorPlacement().getName());
        const Ext::ISyncSensorCalibrationData* iSyncSensorCalibrationData =
            interface_cast<const Ext::ISyncSensorCalibrationData>(cameraDevices[i]);
        if (iSyncSensorCalibrationData)
        {
            iSyncSensorCalibrationData->getSyncSensorModuleId(
                        syncSensorId, sizeof(syncSensorId));
            printf("Found : %s\n", syncSensorId);

            for (int j = 0; j <= moduleCount; j++)
            {
                if (strcmp(syncSensorId, moduleInfo[j].moduleName))
                {
                    if (moduleInfo[j].initialized == false)
                    {
                        strcpy(moduleInfo[j].moduleName, syncSensorId);
                        moduleInfo[j].initialized = true;
                        moduleInfo[j].camDevice[moduleInfo[j].sensorCount++] = i;
                    }
                    else
                    {
                        continue;
                    }

                    moduleCount++;
                    break;
                }
                else
                {
                    moduleInfo[j].camDevice[moduleInfo[j].sensorCount++] = i;
                    hawkModuleCount++;
                    break;
                }
            }
        }
    }

    if ((moduleCount > MAX_MODULE_COUNT) || (hawkModuleCount > MAX_HAWK_MODULE_COUNT))
        printf("Failed to get right stream cosumer\n");

    printf("Total Module Count is %d\n", moduleCount);

    iCameraProvider->setSyncSensorSessionsCount(0, 0);

    int sensor_count = 0;
    int total_SensorCount = 0;
    for (int i = 0; i < moduleCount; i++)
    {
        sensor_count = moduleInfo[i].sensorCount;
        total_SensorCount += sensor_count;
        if (sensor_count >= MIN_MODULE_DEVICE_COUNT && sensor_count <= MAX_CAM_DEVICE) {
            printf("/**************************/\n");

            printf("Identified stereo camera module %s with the following %d sensors connected:\n",
                moduleInfo[i].moduleName, moduleInfo[i].sensorCount);
            for (int j = 0; j < moduleInfo[i].sensorCount; j++) {
                printf("%d\n", moduleInfo[i].camDevice[j]);
            }
            printf("/**************************/\n");
        }
    }

    int session_count = 0;
    for (int i = 0; i < moduleCount; i++)
    {
        moduleInfo[i].isCaptureSessionActive = false;
        if (moduleInfo[i].sensorCount < MIN_MODULE_DEVICE_COUNT ||
            moduleInfo[i].sensorCount > MAX_CAM_DEVICE) {
            continue;
        }

        std::vector <CameraDevice*> lrCameras;

        // Group camera devices to identify no. of sessions to be created
        for (int j = 0; j < moduleInfo[i].sensorCount; j++)
        {
            lrCameras.push_back(cameraDevices[moduleInfo[i].camDevice[j]]);
            printf("Session[%d] : add camera device %d\n",
                session_count, moduleInfo[i].camDevice[j]);
        }

        /**
         * Create the capture session for each set of camera devices identified above,
         * Each session will comprise of two devices (for now) in case of HAWK module.
         * AutoControl will be based on what the 1st device sees.
         * In case of non-HAWK module, there will be a single session for single camera device.
         *
         */
        moduleInfo[i].captureSession = iCameraProvider->createCaptureSession(lrCameras);
        g_iCaptureSession[i] = interface_cast<ICaptureSession>(moduleInfo[i].captureSession);
        if (!g_iCaptureSession[i])
            printf("Failed to get capture session interface");
        moduleInfo[i].isCaptureSessionActive = true;
        /**
         * Create stream settings object and set settings common to both streams in case of HAWK module.
         * Else single stream will be created for non-HAWK module.
         *
         */
        moduleInfo[i].streamSettings =
            g_iCaptureSession[i]->createOutputStreamSettings(STREAM_TYPE_EGL);
        IOutputStreamSettings* iStreamSettings =
            interface_cast<IOutputStreamSettings>(moduleInfo[i].streamSettings);
        IEGLOutputStreamSettings* iEGLStreamSettings =
            interface_cast<IEGLOutputStreamSettings>(moduleInfo[i].streamSettings);
        if (!iStreamSettings || !iEGLStreamSettings)
            printf("Failed to create OutputStreamSettings");
        iEGLStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
        iEGLStreamSettings->setResolution(STREAM_SIZE);
        iEGLStreamSettings->setMetadataEnable(true);
        iEGLStreamSettings->setMode(EGL_STREAM_MODE_FIFO);
        iEGLStreamSettings->setFifoLength(MaxEGLStreamBuffers);

        // Create EGL streams based on stream settings created above for HAWK/non-HAWK modules.
        for (int a = 0; a < moduleInfo[i].sensorCount; a++)
        {
            printf("Creating stream[%d].\n", a);
            iStreamSettings->setCameraDevice(lrCameras[a]);
            moduleInfo[i].stream[a] = g_iCaptureSession[i]->createOutputStream(
                                            moduleInfo[i].streamSettings);
        }

        for (uint32_t j = 0; j < lrCameras.size(); j++)
        {
            const Ext::ISyncSensorCalibrationData *iSyncSensorCalibrationData =
                              interface_cast<const Ext::ISyncSensorCalibrationData>(lrCameras[j]);
            if (iSyncSensorCalibrationData)
            {
                printf("\nCalibration data of sensor %d (device %d) of session %d:\n",
                    j, moduleInfo[i].camDevice[j], i);
                SyncStereoCalibrationData(iSyncSensorCalibrationData,moduleInfo[i],moduleInfo[i].camDevice[j]);
            }
        }

    }

    // SyncStereoPerfThread* perfThread = new SyncStereoPerfThread(total_SensorCount, &sessionMask);

    IRequest *g_iRequest[MAX_MODULE_COUNT];
    for (int i = 0; i < moduleCount; i++)
    {
        if (!moduleInfo[i].isCaptureSessionActive)
            continue;

        printf("Launching syncsensor consumer\n");
        moduleInfo[i].syncStereoConsumer = new SyncStereoConsumerThread(&moduleInfo[i],&sessionMask,cameraDevices.size());
        PROPAGATE_ERROR(moduleInfo[i].syncStereoConsumer->initialize());
        PROPAGATE_ERROR(moduleInfo[i].syncStereoConsumer->waitRunning());

        // Create a request
        printf("creating request[%d] iCaptureSession %p and captureSeesion %p+++++\n",
            i, (void*)g_iCaptureSession[i],  (void*)moduleInfo[i].captureSession);
        moduleInfo[i].request = g_iCaptureSession[i]->createRequest();
        printf("creating g_request[%d] for module %d done\n", i, i);
        g_iRequest[i] = interface_cast<IRequest>(moduleInfo[i].request );
        printf("creating g_iRequest[%d] for module %d interface \n", i, i);
        if (!g_iRequest[i])
            printf("Failed to create Request");

        // Enable output streams based on EGL streams created above for HAWK/non-HAWK modules.
        for (int a = 0; a < moduleInfo[i].sensorCount; a++)
        {
            printf("Enable stream[%d].\n", a);
            g_iRequest[i]->enableOutputStream(moduleInfo[i].stream[a]);
        }
    }

    // Submit capture for the specified time.
    printf("Starting capture requests \n");
    for (int i = 0; i < moduleCount; i++)
    {
        if (g_iCaptureSession[i]->repeat(moduleInfo[i].request) != Argus::STATUS_OK)
            printf("Failed to start capture request for module %d \n",i);
    }

    // Wait for specified time (second).
    sleep(2000);

    for (int i = 0; i < moduleCount; i++)
    {
        if (moduleInfo[i].sensorCount < MIN_MODULE_DEVICE_COUNT ||
            moduleInfo[i].sensorCount > MAX_CAM_DEVICE) {
            continue;
        }

        // Stop the capture requests and wait until they are complete.
        g_iCaptureSession[i]->stopRepeat();
    }

    for (int i = 0; i < moduleCount; i++)
    {
        g_iCaptureSession[i]->waitForIdle();

        // Destroy the output streams to end the consumer thread.
        printf("Captures complete, disconnecting producer: %d\n", i);
        for (int a = 0; a < moduleInfo[i].sensorCount; a++)
        {
            moduleInfo[i].stream[a]->destroy();
        }

        moduleInfo[i].request->destroy();
        moduleInfo[i].streamSettings->destroy();
        moduleInfo[i].captureSession->destroy();

        // Wait for the consumer thread to complete.
        printf("Wait for consumer thread to complete\n");
        PROPAGATE_ERROR(moduleInfo[i].syncStereoConsumer->shutdown());
        if (moduleInfo[i].syncStereoConsumer)
        {
            delete(moduleInfo[i].syncStereoConsumer);
            moduleInfo[i].syncStereoConsumer = NULL;
        }
    }

    // Shut down Argus.
    cameraProvider.reset();

    printf("Done -- exiting.\n");
    return true;
}

}; // namespace ArgusSamples

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    if (!ArgusSamples::execute())
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}
