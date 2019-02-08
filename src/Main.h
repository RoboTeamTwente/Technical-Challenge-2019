// Main.h
//

#ifndef TECHNICAL_CHALLENGE_2019_MAIN_H
#define TECHNICAL_CHALLENGE_2019_MAIN_H

#include "Constants.h"
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "Interface.h"

#include <chrono>
#include <opencv2/imgproc.hpp>

template<typename T, typename A>
void circularPush(std::vector<T, A> vec, T element);

extern std::vector<double> timeVector;
extern std::vector<cv::Point2f> pointVector;

int main(int argc, char **argv);

template<typename T, typename A>
void circularPush(std::vector<T, A> vec, T element) {
    std::rotate(vec.rbegin(), vec.rbegin() + 1, vec.rend());
    // replace first element
    vec[0] = element;
}

#endif // TECHNICAL_CHALLENGE_2019_MAIN_H
