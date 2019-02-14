//
// Created by freek on 08/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_BALLFINDER_H
#define TECHNICAL_CHALLENGE_2019_BALLFINDER_H

#include <chrono>
#include <opencv2/core/mat.hpp>
#include <numeric>
#include "Constants.h"
#include "Main.h"

#include "Settings.h"
#include <vector>
#include <algorithm>
#include <opencv2/core/types_c.h>
#include <cxcore.hpp>
#include "CircularBuffer.h"

class Interface;
class ImageProcessor;
class Camera;

class BallFinder {
public:

    float ballDistanceFromCamera;
    cv::Point2f topDownBallPoint;

    CircularBuffer<cv::Point2f>* topDownBallPointHistory;
    cv::Point_<float> topDownBallPointHistorySum;

    cv::Point2f topDownBallMeanPoint;

    std::chrono::steady_clock::time_point endFrameTime;

    static cv::Point_<float> ballVelocityVectorAsPoint;

    cv::Point_<float> interceptPos;

    float ballSpeed;
    int ballPixelsFromCenterX;
    float ballAngleInCameraPlane;
    float ballAngleInCameraPlaneDegrees;

    cv::Point2f previousTopDownBallMeanPoint;

    explicit BallFinder(); // CONSTRUCTOR
    void findTopDownBallPoint(ImageProcessor imageProcessorObject);
    void findMeanOfBallPoints();
    void findBallSpeedVector(Camera cameraObject);
    void findBallInterceptionVector();
};

#include "Interface.h"
#endif //TECHNICAL_CHALLENGE_2019_BALLFINDER_H
