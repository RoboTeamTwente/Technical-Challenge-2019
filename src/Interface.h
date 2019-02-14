//
// Created by freek on 08/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_INTERFACE_H
#define TECHNICAL_CHALLENGE_2019_INTERFACE_H


#include <cv.hpp>
#include "Constants.h"
#include "Camera.h"
#include "ImageProcessor.h"
#include "BallFinder.h"


class Interface {
public:
    int LOW_HUE = 0;
    int HIGH_HUE = 5;

    int LOW_SATURATION = 126;
    int HIGH_SATURATION = 255;

    int LOW_VALUE = 92;
    int HIGH_VALUE = 255;

    // CONSTRUCTOR

    cv::Mat cameraImageTrailOverlay;
    cv::Mat cameraImageContourOverlay;
    cv::Mat topDownDrawingMat;
    cv::Point_<float> topDownBallPositionForDrawing;

    cv::Point_<float> topDownCameraPositionPoint;
    cv::Scalar_<double> orange;
    cv::Scalar_<double> bluegray;
    float line1y;
    float line1x;

    float line2y;
    float line2x;

    explicit Interface();
    // TODO why does this reference to ImageProcessor not work?
    void drawContourAndBallTrailOnCameraView(Camera cameraObject, ImageProcessor imageProcessorObject);
    void drawTopDownView(BallFinder ballFinderObject, ImageProcessor imageProcessorObject);
    void displayMatsAndDrawText(Camera cameraObject, ImageProcessor imageProcessorObject, BallFinder ballFinderObject);

};


#endif //TECHNICAL_CHALLENGE_2019_INTERFACE_H
