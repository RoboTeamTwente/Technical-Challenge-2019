//
// Created by freek on 08/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H
#define TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H


#include <opencv2/core/mat.hpp>
#include "Interface.h"
#include "Camera.h"

class ImageProcessor {
public:
    void imageConversion(Camera &cameraObject, Interface &interfaceObject);

    void findBallContour();
    std::vector<std::vector<cv::Point>> contours; // Could be a field
    std::vector<std::vector<cv::Point>> contoursAsPolygonsVector; // Could be a field
    cv::Mat cameraImageThresholded; // Could be a field
    cv::Point2f cameraImageBallCenterPoint;
    float cameraImageBallRadius;
    std::vector<cv::Point2f> cameraImageBallCenterHistory;
};


#endif //TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H
