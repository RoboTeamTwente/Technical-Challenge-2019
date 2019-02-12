//
// Created by freek on 08/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_CAMERA_H
#define TECHNICAL_CHALLENGE_2019_CAMERA_H


#include <opencv2/videoio.hpp>

class Camera {
public:
    explicit Camera(); // constructor
    bool captureImage();
    cv::VideoCapture cap;
    int frameCounter;
    std::chrono::time_point startFrameTime;
    cv::Mat cameraImageBGR;
};


#endif //TECHNICAL_CHALLENGE_2019_CAMERA_H
