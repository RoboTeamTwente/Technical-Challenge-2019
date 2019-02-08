//
// Created by freek on 08/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H
#define TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H


#include <opencv2/core/mat.hpp>

class ImageProcessor {
public:
    cv::Mat convertThresholded(cv::Mat imgBGR) {
// TODO implement
// TODO pass interface obj here by reference?
    }
};


template<typename T, typename A>
void circularPush(std::vector<T, A> vec, T element);

template<typename T, typename A>
void circularPush(std::vector<T, A> vec, T element) {
    std::_V2::rotate(vec.rbegin(), vec.rbegin() + 1, vec.rend());
    // replace first element
    vec[0] = element;
}

#endif //TECHNICAL_CHALLENGE_2019_IMAGEPROCESSOR_H
