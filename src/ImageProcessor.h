//
// Created by freek on 08/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H
#define TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H


#include <opencv2/core/mat.hpp>
#include "Interface.h"

class ImageProcessor {
public:
    cv::Point_<float> onecenter;
    float oneradius;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> contours_poly;
    std::vector<cv::Point> largest_contour;
    int largest_area;
    cv::Mat imgHSV;
    cv::Mat cameraImageThresholded;


};



// TODO move this
template<typename T, typename A>
void circularPush(std::vector<T, A> vec, T element) {
    std::_V2::rotate(vec.rbegin(), vec.rbegin() + 1, vec.rend());
    // replace first element
    vec[0] = element;
}

#endif //TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H
