#ifndef TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H
#define TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H


#include <opencv2/core/mat.hpp>
#include "Interface.h"
#include "Camera.h"
#include "Settings.h"

class ImageProcessor {
public:
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> contoursAsPolygonsVector;
    cv::Mat cameraImageThresholded;
    cv::Point2f cameraImageBallCenterPoint;
    float cameraImageBallRadius;
    CircularBuffer<cv::Point2f>* cameraImageBallCenterHistory;
    cv::Mat imgHSV;
    double largestContourArea;
    cv::Scalar colors[3];
    std::vector<cv::Point> largestContourAsPolygon;
    std::vector<cv::Point> largestContour;
    std::vector<cv::Point2f>(30) circularBufferInput = {};

    ImageProcessor() : cameraImageBallCenterHistory(std::vector<cv::Point2f>);
    void imageConversion(Camera &cameraObject, Interface &interfaceObject);

    void findBallContour();
};


#endif //TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H
