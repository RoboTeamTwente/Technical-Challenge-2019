//
// Created by freek on 08/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_INTERFACE_H
#define TECHNICAL_CHALLENGE_2019_INTERFACE_H


#include <cv.hpp>
#include <iostream>

class Camera;
class BallFinder;
class ImageProcessor;

class Interface {


public:
    int LOW_HUE = 0;// 0;
    int HIGH_HUE = 9;//5;

    int LOW_SATURATION = 50;
    int HIGH_SATURATION = 255;

    int LOW_VALUE = 200;// 92;
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

    void drawContourAndBallTrailOnCameraView(Camera cameraObject, ImageProcessor imageProcessorObject);
    void drawTopDownView(BallFinder ballFinderObject, ImageProcessor imageProcessorObject);
    void displayMatsAndDrawText(Camera cameraObject, ImageProcessor imageProcessorObject, BallFinder ballFinderObject);

    cv::Point_<float> interceptPosForDrawing;
};


#endif //TECHNICAL_CHALLENGE_2019_INTERFACE_H
