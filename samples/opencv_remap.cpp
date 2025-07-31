#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

// 定义相机参数
const cv::Size image_size(1920, 1200); // 图像尺寸
const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
                               964.431946, 0.000000, 951.792114,
                               0.000000, 964.806641, 582.802063,
                               0.000000, 0.000000, 1.000000);

const cv::Mat distortion_coefficients = (cv::Mat_<double>(1, 8) << 
                                         0.854475, 0.144149, 0.001531, 1.227936, 
                                         0.364355, 0.020056, -0.000219, 0.000041);

int main() {
    const int num_cameras = 8; // 摄像头数量
    std::vector<cv::VideoCapture> cameras(num_cameras);
    std::vector<std::string> pipelines(num_cameras);

    // 初始化摄像头的 GStreamer 管道
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

    // 初始化存储去畸变结果的映射表
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(
        camera_matrix, distortion_coefficients, cv::Mat(), camera_matrix,
        image_size, CV_16SC2, map1, map2);

    // 开始读取视频流
    while (true) {
        for (int i = 0; i < num_cameras; ++i) {
            cv::Mat frame, undistorted_frame;

            // 从摄像头读取帧
            if (!cameras[i].read(frame)) {
                std::cerr << "Failed to capture frame from camera " << i << std::endl;
                continue;
            }

            // 确保图像尺寸与标定数据一致
            if (frame.size() != image_size) {
                std::cerr << "Frame size does not match calibration data for camera " << i << std::endl;
                continue;
            }

            // 去畸变处理
            cv::remap(frame, undistorted_frame, map1, map2, cv::INTER_LINEAR);
            std::cout << "Camera " << i << ": " << frame.cols << "x" << frame.rows << std::endl;

            // 显示去畸变后的结果
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