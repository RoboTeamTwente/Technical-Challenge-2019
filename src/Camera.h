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
    bool working;
    cv::VideoCapture cap;
    int frameCounter;
    bool captureSuccess;

    cv::Mat cameraImageBGR;

    // TODO fix time_point error
    std::chrono::time_point startFrameTime;
};


#endif //TECHNICAL_CHALLENGE_2019_CAMERA_H
