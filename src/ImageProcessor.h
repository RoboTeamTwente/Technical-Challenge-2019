//
// Created by freek on 08/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H
#define TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H


#include <opencv2/core/mat.hpp>
#include "Interface.h"

class ImageProcessor {
public:
    cv::Mat imageConversion(cv::Mat cameraImageBGR, Interface interfaceObj);

    void findBallContour(const cv::Mat &cameraImageThresholded, cv::Point2f &cameraImageBallCenterPoint,
                         float &cameraImageBallRadius);
};


#endif //TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H
