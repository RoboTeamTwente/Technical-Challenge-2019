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

class BallFinder {
public:
    explicit BallFinder(); // CONSTRUCTOR
    void findBall();
    std::vector<double> timeVector;
    float topDownBallX;
    float topDownBallY;
    cv::Point2f topDownBallMeanPoint;
    cv::Point_<float> ballVelocityVectorAsPoint;
    float ballSpeed;
};


#endif //TECHNICAL_CHALLENGE_2019_BALLFINDER_H
