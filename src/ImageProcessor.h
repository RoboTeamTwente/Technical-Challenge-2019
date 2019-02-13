#ifndef TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H
#define TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H


#include <opencv2/core/mat.hpp>
#include "Interface.h"
#include "Camera.h"
#include "Settings.h"

class ImageProcessor {
public:
    ImageProcessor(); // constructor
    void imageConversion(Camera &cameraObject, Interface &interfaceObject);

    void findBallContour();
    std::vector<std::vector<cv::Point>> contours; // Could be a field
    std::vector<std::vector<cv::Point>> contoursAsPolygonsVector; // Could be a field
    cv::Mat cameraImageThresholded; // Could be a field
    cv::Point2f cameraImageBallCenterPoint;
    float cameraImageBallRadius;
    CircularBuffer* cameraImageBallCenterHistory;
    cv::Mat imgHSV;
    // TODO move some stuff below to constructor
    double largestContourArea;
    cv::Scalar colors[3];
    std::vector<cv::Point> largestContourAsPolygon;
    colors[0] = cv::Scalar(255, 0, 0);
    colors[1] = cv::Scalar(0, 255, 0);
    colors[2] = cv::Scalar(0, 0, 255);
    std::vector<cv::Point> largestContour;

};


#endif //TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H
