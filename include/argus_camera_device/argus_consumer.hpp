// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARGUS_CAMERA_DEVICE__ARGUS_CONSUMER_HPP_
#define ARGUS_CAMERA_DEVICE__ARGUS_CONSUMER_HPP_

#include "EGLGlobal.h"
#include "Error.h"
#include "Thread.h"
#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include "NvBufSurface.h"

#include <map>
#include <bitset>
#include <tuple>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Argus/Ext/SyncSensorCalibrationData.h>
#include <Argus/Ext/SensorTimestampTsc.h>

#include <NvJpegEncoder.h>
#include "mqtt/async_client.h"
#include "opencv2/opencv.hpp"


namespace ArgusSamples
{

#define MAX_MODULE_STRING 32

#define MAX_MODULE_COUNT 8

#define MAX_HAWK_MODULE_COUNT 4

#define MIN_MODULE_DEVICE_COUNT 1

#define MaxEGLStreamBuffers 12

// Threshold to detect out of sync detection
#define SYNC_THRESHOLD_TIME_US 100.0f

// Constants.
static const Argus::Size2D<uint32_t> STREAM_SIZE(960, 600);

struct mqtt_info
{
    bool img_transmission = false;
    std::string broker_ip = "ws://127.0.0.1:1883";
    std::string user_name = "tita";
    std::string passwd = "apollo";
    std::string image_topic = "tita/remote/img";
    std::string client_id = "";
};

static Argus::Size2D<uint32_t> PREVIEW_SIZE (640, 480);
#define JPEG_BUFFER_SIZE    (PREVIEW_SIZE.area() * 3 / 2)

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


typedef struct
{
    char moduleName[MAX_MODULE_STRING];
    int camDevice[MAX_CAM_DEVICE];
    Argus::OutputStream *stream[MAX_CAM_DEVICE];
    Argus::CaptureSession *captureSession;
    Argus::OutputStreamSettings *streamSettings;
    Argus::Request *request;
    bool isCaptureSessionActive;
    SyncStereoConsumerThread *syncStereoConsumer;
    int sensorCount;
    bool initialized;
} ModuleInfo;

struct CameraArgV {
    std::string distortion_model;
    int height;
    int width;
};

struct CameraCalibT {
    CameraArgV image_argv;
    cv::Mat K_intrinsic;
    cv::Mat T_translation;
    cv::Mat R_rectification;
    cv::Mat D_distortion;
    cv::Mat P_projection;

    CameraCalibT() {
      K_intrinsic = cv::Mat::zeros(3, 3, CV_64F);
      D_distortion = cv::Mat::zeros(1, 8, CV_64F);
    }
};
/*******************************************************************************
 * Argus disparity class
 * This class will analyze frames from 2 synchronized sensors
 ******************************************************************************/
class SyncStereoConsumerThread : public Thread
{
public:
    explicit SyncStereoConsumerThread(ModuleInfo *modInfo,
                                      mqtt_info &mqtt_config):mqtt_config_(mqtt_config),client(mqtt_config.broker_ip, mqtt_config.client_id),
                                      matTuple_(std::make_tuple(
                                        cv::Mat::zeros(cv::Size(960, 600), CV_8UC3),
                                        cv::Mat::zeros(cv::Size(960, 600), CV_8UC3)))
    {
        m_leftStream = modInfo->stream[LEFT_CAM_DEVICE];
        camDevices.push_back(modInfo->camDevice[0]);
        if (modInfo->sensorCount > 1) {
            m_rightStream = modInfo->stream[RIGHT_CAM_DEVICE];
            camDevices.push_back(modInfo->camDevice[1]);
        }
        else
            m_rightStream = NULL;

        strcpy(m_moduleName, modInfo->moduleName);

        try
        {
            mqtt::connect_options connOpts;
            connOpts.set_clean_session(true);
            connOpts.set_user_name(mqtt_config_.user_name);
            connOpts.set_password(mqtt_config_.passwd);

            std::cout << "Connecting to MQTT broker..." << std::endl;
            client.connect(connOpts)->wait();
            std::cout << "Connected to MQTT broker" << std::endl;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            mqtt_config_.img_transmission = false;
        }        

    }

    void setImgRemoteMark(bool setMark)
    {
        mqtt_config_.img_transmission = setMark;
    }

    std::tuple<cv::Mat, cv::Mat> get_stereo_img();

    ~SyncStereoConsumerThread()
    {
        printf("DESTRUCTOR  ... \n");
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
    Argus::OutputStream *m_leftStream; // left stream tied to sensor index 0 and is used for autocontrol.
    Argus::OutputStream *m_rightStream; // right stream tied to sensor index 1.

    int dump_dmabuffers(int dmabuf_fd,cv::Mat& image_out);

    bool processV4L2Fd(int32_t fd);

    char m_moduleName[MAX_MODULE_STRING];
    Argus::UniqueObj<EGLStream::FrameConsumer> m_leftConsumer;
    Argus::UniqueObj<EGLStream::FrameConsumer> m_rightConsumer;
    std::vector<int> camDevices;
    std::string namespace_ = "";
    NvJPEGEncoder *m_JpegEncoder;
    unsigned char *m_OutputBuffer;
    mqtt_info mqtt_config_;
    mqtt::async_client client;
    std::tuple<cv::Mat, cv::Mat> matTuple_;
    int m_dmabuf = -1;
    int m2_dmabuf = -1;
    int m3_dmabuf = -1;

};

}; // namespace ArgusSamples


#endif  // ARGUS_CAMERA_DEVICE__ARGUS_CONSUMER_HPP_
