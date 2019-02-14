#ifndef TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H
#define TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H


#include <opencv2/core/mat.hpp>

#include <opencv2/imgproc.hpp>
#include "Camera.h"
#include "Settings.h"

#include "CircularBuffer.h"
class Interface;


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


    ImageProcessor();
    void imageConversion(Camera cameraObject, Interface interfaceObject);

    void findBallContour();
};

#include "Interface.h"
#endif //TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H
