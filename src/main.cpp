// Copyright (c) 2025 VulcanYJX
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include "ArgusHelpers.h"
#include "CommonOptions.h"
#include <Argus/Ext/SyncSensorCalibrationData.h>

#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>

#include <vpi/Image.h>
#include <vpi/Stream.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/algo/ConvertImageFormat.h>
#include "opencv2/opencv.hpp"

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

#define EXIT_IF_NULL(val,msg)   \
        {if (!val) {printf("%s\n",msg); return EXIT_FAILURE;}}
#define EXIT_IF_NOT_OK(val,msg) \
        {if (val!=Argus::STATUS_OK) {printf("%s\n",msg); return EXIT_FAILURE;}}

#define MAX_MODULE_STRING 32
using namespace Argus;


static void SyncStereoCalibrationData(
    const Ext::ISyncSensorCalibrationData *iSyncSensorCalibrationData)
{
    Size2D<uint32_t> ImageSize = iSyncSensorCalibrationData->getImageSizeInPixels();
    printf("Image size = %d, %d\n", ImageSize.width(), ImageSize.height());

    Point2D<float> FocalLength = iSyncSensorCalibrationData->getFocalLength();
    printf("Focal Length = %f, %f\n", FocalLength.x(), FocalLength.y());

    Point2D<float> PrincipalPoint = iSyncSensorCalibrationData->getPrincipalPoint();
    printf("Principal Point = %f, %f\n", PrincipalPoint.x(), PrincipalPoint.y());

    float Skew = iSyncSensorCalibrationData->getSkew();
    printf("Skew = %f\n", Skew);

    MappingType FishEyeMappingType = iSyncSensorCalibrationData->getFisheyeMappingType();
    printf("Fish Eye mapping type = %s\n", FishEyeMappingType.getName());

    DistortionType LensDistortionType = iSyncSensorCalibrationData->getLensDistortionType();
    printf("Lens Distortion type = %s\n", LensDistortionType.getName());

    uint32_t RadialCoeffsCount =
                        iSyncSensorCalibrationData->getRadialCoeffsCount(LensDistortionType);
    printf("Radial coeffs count = %d\n", RadialCoeffsCount);

    std::vector<float> k;
    iSyncSensorCalibrationData->getRadialCoeffs(&k, LensDistortionType);

    printf("Radial coefficients = ");
    for (uint32_t idx = 0; idx < k.size(); idx++)
    {
        printf("%f ", k[idx]);
    }

    uint32_t TangentialCoeffsCount =
        iSyncSensorCalibrationData->getTangentialCoeffsCount();
    printf("Tangential coeffs count = %d\n", TangentialCoeffsCount);

    std::vector<float> p;
    iSyncSensorCalibrationData->getTangentialCoeffs(&p);

    printf("Tangential coefficients = ");
    for (uint32_t idx = 0; idx < p.size(); idx++)
    {
        printf("%f ", p[idx]);
    }

    Point3D<float> rot3d = iSyncSensorCalibrationData->getRotationParams();
    printf("rot3d x, y, z{%f, %f, %f}\n", rot3d.x(), rot3d.y(), rot3d.z());

    Point3D<float> translation = iSyncSensorCalibrationData->getTranslationParams();
    printf("translation 3d x, y, z{%f, %f, %f}\n",
        translation.x(), translation.y(), translation.z());

    char moduleSerialNumber[MAX_MODULE_STRING];
    iSyncSensorCalibrationData->getModuleSerialNumber(
        moduleSerialNumber, sizeof(moduleSerialNumber));

    printf("moduleSerialNumber %s\n", moduleSerialNumber);

    bool isImu = iSyncSensorCalibrationData->isImuSensorAvailable();
    if (isImu)
    {
        printf("For IMU sensors \n");

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


int main(int argc, char** argv)
{
    ArgusSamples::CommonOptions options(basename(argv[0]),
                                        ArgusSamples::CommonOptions::Option_D_CameraDevice |
                                        ArgusSamples::CommonOptions::Option_M_SensorMode);
    if (!options.parse(argc, argv))
        return EXIT_FAILURE;
    if (options.requestedExit())
        return EXIT_SUCCESS;

    const uint64_t FIVE_SECONDS_IN_NANOSECONDS = 5000000000;

    /*
     * Set up Argus API Framework, identify available camera devices, and create
     * a capture session for the first available device
     */

    Argus::UniqueObj<Argus::CameraProvider> cameraProvider(Argus::CameraProvider::create());

    Argus::ICameraProvider *iCameraProvider =
        Argus::interface_cast<Argus::ICameraProvider>(cameraProvider);
    EXIT_IF_NULL(iCameraProvider, "Cannot get core camera provider interface");
    printf("Argus Version: %s\n", iCameraProvider->getVersion().c_str());

    Argus::CameraDevice *device = ArgusSamples::ArgusHelpers::getCameraDevice(
            cameraProvider.get(), options.cameraDeviceIndex());
    printf("Device : %d\n", options.cameraDeviceIndex());
    Argus::ICameraProperties *iCameraProperties =
        Argus::interface_cast<Argus::ICameraProperties>(device);
    if (!iCameraProperties)
    {
        REPORT_ERROR("Failed to get ICameraProperties interface");
        return EXIT_FAILURE;
    }

    const Argus::Ext::ISyncSensorCalibrationData * iSyncSensorCalibrationData =
        Argus::interface_cast<const Argus::Ext::ISyncSensorCalibrationData>(device);
    char syncSensorId[MAX_MODULE_STRING];
    if (iSyncSensorCalibrationData)
    {
        iSyncSensorCalibrationData->getSyncSensorModuleId(syncSensorId, sizeof(syncSensorId));
        SyncStereoCalibrationData(iSyncSensorCalibrationData);
        printf("Found : %s\n", syncSensorId);
        // iCameraProvider->setSyncSensorSessionsCount(0, 0);
    }

    Argus::SensorMode* sensorMode = ArgusSamples::ArgusHelpers::getSensorMode(
            device, options.sensorModeIndex());
    Argus::ISensorMode *iSensorMode =
        Argus::interface_cast<Argus::ISensorMode>(sensorMode);
    if (!iSensorMode)
    {
        REPORT_ERROR("Failed to get sensor mode interface");
        return EXIT_FAILURE;
    }

    printf("Capturing from device %d using sensor mode %d (%dx%d)\n",
           options.cameraDeviceIndex(), options.sensorModeIndex(),
           iSensorMode->getResolution().width(), iSensorMode->getResolution().height());

    Argus::Status status;
    Argus::UniqueObj<Argus::CaptureSession> captureSession(
        iCameraProvider->createCaptureSession(device, &status));
    EXIT_IF_NOT_OK(status, "Failed to create capture session");

    Argus::ICaptureSession *iSession =
        Argus::interface_cast<Argus::ICaptureSession>(captureSession);
    EXIT_IF_NULL(iSession, "Cannot get Capture Session Interface");

    /*
     * Creates the stream between the Argus camera image capturing
     * sub-system (producer) and the image acquisition code (consumer).  A consumer object is
     * created from the stream to be used to request the image frame.  A successfully submitted
     * capture request activates the stream's functionality to eventually make a frame available
     * for acquisition.
     */

    Argus::UniqueObj<Argus::OutputStreamSettings> streamSettings(
        iSession->createOutputStreamSettings(Argus::STREAM_TYPE_EGL));

    Argus::IEGLOutputStreamSettings *iEGLStreamSettings =
        Argus::interface_cast<Argus::IEGLOutputStreamSettings>(streamSettings);
    EXIT_IF_NULL(iEGLStreamSettings, "Cannot get IEGLOutputStreamSettings Interface");
    iEGLStreamSettings->setPixelFormat(Argus::PIXEL_FMT_YCbCr_420_888);
    iEGLStreamSettings->setResolution(iSensorMode->getResolution());
    iEGLStreamSettings->setMetadataEnable(true);

    Argus::UniqueObj<Argus::OutputStream> stream(
        iSession->createOutputStream(streamSettings.get()));
    EXIT_IF_NULL(stream, "Failed to create EGLOutputStream");

    Argus::UniqueObj<EGLStream::FrameConsumer> consumer(
        EGLStream::FrameConsumer::create(stream.get()));

    EGLStream::IFrameConsumer *iFrameConsumer =
        Argus::interface_cast<EGLStream::IFrameConsumer>(consumer);
    EXIT_IF_NULL(iFrameConsumer, "Failed to initialize Consumer");

    Argus::UniqueObj<Argus::Request> request(
        iSession->createRequest(Argus::CAPTURE_INTENT_STILL_CAPTURE));

    Argus::IRequest *iRequest = Argus::interface_cast<Argus::IRequest>(request);
    EXIT_IF_NULL(iRequest, "Failed to get capture request interface");

    status = iRequest->enableOutputStream(stream.get());
    EXIT_IF_NOT_OK(status, "Failed to enable stream in capture request");

    Argus::ISourceSettings *iSourceSettings =
        Argus::interface_cast<Argus::ISourceSettings>(request);
    EXIT_IF_NULL(iSourceSettings, "Failed to get source settings request interface");
    iSourceSettings->setSensorMode(sensorMode);

    uint32_t requestId = iSession->capture(request.get());
    EXIT_IF_NULL(requestId, "Failed to submit capture request");

    Argus::UniqueObj<EGLStream::Frame> frame(
        iFrameConsumer->acquireFrame(FIVE_SECONDS_IN_NANOSECONDS, &status));

    EGLStream::IFrame *iFrame = Argus::interface_cast<EGLStream::IFrame>(frame);
    EXIT_IF_NULL(iFrame, "Failed to get IFrame interface");

    EGLStream::Image *image = iFrame->getImage();
    EXIT_IF_NULL(image, "Failed to get Image from iFrame->getImage()");

    EGLStream::NV::IImageNativeBuffer *iNativeBuffer =
            Argus::interface_cast<EGLStream::NV::IImageNativeBuffer>(image);
    if (!iNativeBuffer)
        printf("IImageNativeBuffer not supported by Image.");
    int m_dmabuf = -1;
    if (m_dmabuf == -1)
    {
        m_dmabuf = iNativeBuffer->createNvBuffer(iSensorMode->getResolution(),
                                                NVBUF_COLOR_FORMAT_BGRA,
                                                NVBUF_LAYOUT_PITCH);
        if (m_dmabuf == -1){
          printf("\tFailed to create NvBuffer\n");
        }
        else{
          printf("\tcreate NvBuffer\n");
          NvBufSurface *nvbuf_surf = 0;
          int ret = 0;
          ret = NvBufSurfaceFromFd (m_dmabuf, (void**)(&nvbuf_surf));
          if (nvbuf_surf->surfaceList[0].colorFormat == NVBUF_COLOR_FORMAT_BGRA){

            NvBufSurfaceMapParams buffer_params;
            NvBufSurfaceGetMapParams(nvbuf_surf, 0, &buffer_params);

            VPIStream stream;
            CHECK_STATUS(vpiStreamCreate(0, &stream));
            VPIImage srcImage   = NULL;
            VPIImageData imgData = {};
            imgData.bufferType = VPI_IMAGE_BUFFER_NVBUFFER;
            imgData.buffer.fd = buffer_params.fd;
            CHECK_STATUS(vpiImageCreateWrapper(&imgData,nullptr,VPI_BACKEND_CUDA,&srcImage));
            VPIImage dstImage;
            int width = iSensorMode->getResolution().width();
            int height = iSensorMode->getResolution().height();
            CHECK_STATUS(vpiImageCreate(width, height, VPI_IMAGE_FORMAT_BGR8, 0, &dstImage));

            CHECK_STATUS(vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, srcImage, dstImage, NULL));
            
            CHECK_STATUS(vpiStreamSync(stream));

            VPIImageData outData;
            CHECK_STATUS(vpiImageLockData(dstImage, VPI_LOCK_READ,VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &outData));
            cv::Mat cvImage;
            CHECK_STATUS(vpiImageDataExportOpenCVMat(outData, &cvImage));
            cv::imwrite("vpi.png", cvImage);
          }

        }
    }
    // else if (iNativeBuffer->copyToNvBuffer(m_dmabuf) != Argus::STATUS_OK)
    // {
    //     printf("Failed to copy frame to NvBuffer.");
    // }

    // Shut down Argus.
    cameraProvider.reset();

    return EXIT_SUCCESS;
}
