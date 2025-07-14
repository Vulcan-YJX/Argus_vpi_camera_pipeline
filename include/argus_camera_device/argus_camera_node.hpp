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

#ifndef ARGUS_CAMERA_DEVICE__ARGUS_CAMERA_NODE_HPP_
#define ARGUS_CAMERA_DEVICE__ARGUS_CAMERA_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <tuple>

#include "cv_bridge/cv_bridge.h"
#include "argus_camera_device/argus_consumer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tita_utils/topic_names.hpp"
namespace tita
{

using namespace ArgusSamples;

class ArgusCamNode : public rclcpp::Node
{
public:
  explicit ArgusCamNode(const rclcpp::NodeOptions & options);
  ~ArgusCamNode();
  
private:

  void SyncStereoCalibrationData(
    const Argus::Ext::ISyncSensorCalibrationData *iSyncSensorCalibrationData,sensor_msgs::msg::CameraInfo& cam_info);

  void publishCameraInfo();

  void publishCameraImg();

  sensor_msgs::msg::CameraInfo lcamera_info_msg_;
  sensor_msgs::msg::CameraInfo rcamera_info_msg_;


  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
  rclcpp::TimerBase::SharedPtr info_timer_;
  rclcpp::TimerBase::SharedPtr img_timer_;
  std::string namespace_ = "";

  Argus::UniqueObj<Argus::CameraProvider> cameraProvider;
  Argus::ICaptureSession* g_iCaptureSession[MAX_MODULE_COUNT];
  ArgusSamples::ModuleInfo moduleInfo[MAX_MODULE_COUNT];
  int moduleCount = 0;
  int hawkModuleCount = 0;
  uint16_t sessionMask = 0;
  mqtt_info mqtt_config_;
  
};
}

#endif  // ARGUS_CAMERA_DEVICE__ARGUS_CAMERA_NODE_HPP_
