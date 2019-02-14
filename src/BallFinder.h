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
#include "ImageProcessor.h"
#include "Settings.h"
#include <vector>
#include <algorithm>
#include <opencv2/core/types_c.h>
#include <cxcore.hpp>

class BallFinder {
public:

    std::vector<double> timeVector;
    cv::Point2f topDownBallPoint;

    CircularBuffer<cv::Point2f>* topDownBallPointHistory;
    cv::Point_<float> topDownBallPointHistorySum;

    cv::Point2f topDownBallMeanPoint;

    cv::Point_<float> ballVelocityVectorAsPoint;

    float ballSpeed;
    int ballPixelsFromCenterX;
    float ballAngleInCameraPlane;
    float ballAngleInCameraPlaneDegrees;

    explicit BallFinder(); // CONSTRUCTOR
    void findTopDownBallPoint(ImageProcessor imageProcessorObject);
    void findMeanOfBallPoints();
};


#endif //TECHNICAL_CHALLENGE_2019_BALLFINDER_H
