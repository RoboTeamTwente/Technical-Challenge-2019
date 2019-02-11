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

class BallFinder {
public:
    void findBall(int cameraObject.frameCounter, float x, float y, const std::chrono::time_point &cameraObject.startFrameTime, float oneradius,
                 const cv::Scalar_<double> &color, int currentX, :cv::Point_<float> &topDownBallMeanPoint, cv::Mat &imgThresholded,
    float &ballSpeed, cv::Mat &topDown);
    std::vector<double> timeVector;
    float topDownBallX;
    float topDownBallY;
    cv::Point2f topDownBallMeanPoint;
};


#endif //TECHNICAL_CHALLENGE_2019_BALLFINDER_H
