//
// Created by freek on 08/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_BALLFINDER_H
#define TECHNICAL_CHALLENGE_2019_BALLFINDER_H

#include <chrono>
#include <opencv2/core/mat.hpp>
#include <numeric>
#include <vector>
#include <algorithm>
#include <cxcore.hpp>

template <typename T>
class CircularBuffer;
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

    cv::Point_<float> ballVelocityVectorAsPoint;

    cv::Point_<float> interceptPos;

    float ballSpeed;
    int ballPixelsFromCenterX;
    float ballAngleInCameraPlane;
    float ballAngleInCameraPlaneDegrees;

    cv::Point2f previousTopDownBallMeanPoint;

    explicit BallFinder(); // CONSTRUCTOR
    void findTopDownBallPoint(const ImageProcessor &imageProcessorObject);
    void findMeanOfBallPoints();
    void findBallSpeedVector(Camera cameraObject);
    void findBallInterceptionVector();
};


#endif //TECHNICAL_CHALLENGE_2019_BALLFINDER_H
