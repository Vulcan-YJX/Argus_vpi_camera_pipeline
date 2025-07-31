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

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

// 只为测试性能占用，全部使用一套参数
const cv::Size image_size(1920, 1200);
const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
                               964.431946, 0.000000, 951.792114,
                               0.000000, 964.806641, 582.802063,
                               0.000000, 0.000000, 1.000000);

const cv::Mat distortion_coefficients = (cv::Mat_<double>(1, 8) << 
                                         0.854475, 0.144149, 0.001531, 1.227936, 
                                         0.364355, 0.020056, -0.000219, 0.000041);

int main() {
    const int num_cameras = 8;
    std::vector<cv::VideoCapture> cameras(num_cameras);
    std::vector<std::string> pipelines(num_cameras);

    for (int i = 0; i < num_cameras; ++i) {
        pipelines[i] = "nvarguscamerasrc sensor_id=" + std::to_string(i) +
                       " ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1200, format=(string)NV12, "
                       "framerate=(fraction)30/1 ! queue ! nvvidconv ! "
                       "video/x-raw, width=(int)1920, height=(int)1200, format=(string)BGRx "
                       "! queue ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

        cameras[i].open(pipelines[i], cv::CAP_GSTREAMER);
        if (!cameras[i].isOpened()) {
            std::cerr << "Failed to open camera " << i << std::endl;
            return -1;
        }
    }

    std::cout << "All cameras opened successfully!" << std::endl;

    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(
        camera_matrix, distortion_coefficients, cv::Mat(), camera_matrix,
        image_size, CV_16SC2, map1, map2);

    while (true) {
        for (int i = 0; i < num_cameras; ++i) {
            cv::Mat frame, undistorted_frame;
            if (!cameras[i].read(frame)) {
                std::cerr << "Failed to capture frame from camera " << i << std::endl;
                continue;
            }

            if (frame.size() != image_size) {
                std::cerr << "Frame size does not match calibration data for camera " << i << std::endl;
                continue;
            }

            cv::remap(frame, undistorted_frame, map1, map2, cv::INTER_LINEAR);
            std::cout << "Camera " << i << ": " << frame.cols << "x" << frame.rows << std::endl;

            // std::string window_name = "Undistorted Camera " + std::to_string(i);
            // cv::imshow(window_name, undistorted_frame);
        }

        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // 释放资源
    for (int i = 0; i < num_cameras; ++i) {
        cameras[i].release();
    }
    // cv::destroyAllWindows();

    return 0;
}