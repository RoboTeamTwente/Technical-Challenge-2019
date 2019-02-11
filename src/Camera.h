//
// Created by freek on 08/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_CAMERA_H
#define TECHNICAL_CHALLENGE_2019_CAMERA_H


#include <opencv2/videoio.hpp>

class Camera {
public:
    Camera(const std::chrono::time_point &startFrameTime); // constructor
    bool captureImage(cv::Mat inputMat);
    cv::VideoCapture cap;
    int previousCameraBallX;
    int previousCameraBallY;
    int frameCounter;
    std::chrono::time_point startFrameTime;

};


#endif //TECHNICAL_CHALLENGE_2019_CAMERA_H
