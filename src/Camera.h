//
// Created by freek on 08/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_CAMERA_H
#define TECHNICAL_CHALLENGE_2019_CAMERA_H

#include <opencv2/videoio.hpp>
#include <chrono>
#include <iostream>

class Camera {
public:
    explicit Camera(); // constructor
    bool captureImage();
    bool working;
    cv::VideoCapture* cap;
    int frameCounter;
    bool captureSuccess;

    cv::Mat cameraImageBGR;

    std::chrono::steady_clock::time_point startFrameTime;
};


#endif //TECHNICAL_CHALLENGE_2019_CAMERA_H
