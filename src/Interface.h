//
// Created by freek on 08/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_INTERFACE_H
#define TECHNICAL_CHALLENGE_2019_INTERFACE_H


#include <cv.hpp>
#include "Constants.h"
#include "ImageProcessor.h"
#include "BallFinder.h"
#include "Camera.h"


class Interface {
public:
    int LOW_HUE = 0;
    int HIGH_HUE = 5;

    int LOW_SATURATION = 126;
    int HIGH_SATURATION = 255;

    int LOW_VALUE = 92;
    int HIGH_VALUE = 255;

    // CONSTRUCTOR
    explicit Interface();
    cv::Mat cameraImageTrailOverlay;
    cv::Mat cameraImageContourOverlay;
    cv::Mat topDown;
    cv::Point_<float> topDownBallPos;
    // TODO some stuff below to constructor
    cv::Point_<float> topDownCameraPositionPoint = cv::Point_<float>(100, 240);
    cv::Scalar_<double> orange = cv::Scalar_<double>(2, 106, 253);
    cv::Scalar_<double> bluegray = cv::Scalar_<double>(255, 120, 120);
    float line1x = 540 * cos(-0.5 * Constants::HORIZONTAL_FOV_RADIANS);
    float line1y = 540 * sin(-0.5 * Constants::HORIZONTAL_FOV_RADIANS);

    float line2x = 540 * cos(0.5 * Constants::HORIZONTAL_FOV_RADIANS);
    float line2y = 540 * sin(0.5 * Constants::HORIZONTAL_FOV_RADIANS);

    // TODO why does this reference to ImageProcessor not work?
    void drawContourAndBallTrailOnCameraView(Camera cameraObject, ImageProcessor imageProcessorObject);
    void drawTopDownView(BallFinder ballFinderObject, ImageProcessor imageProcessorObject);
    void displayMatsAndDrawText(Camera cameraObject, ImageProcessor imageProcessorObject, BallFinder ballFinderObject);

};


#endif //TECHNICAL_CHALLENGE_2019_INTERFACE_H
