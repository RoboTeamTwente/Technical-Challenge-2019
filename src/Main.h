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

extern std::vector<double> timeVector;
extern std::vector<cv::Point2f> pointVector;

int main(int argc, char **argv);

#endif // TECHNICAL_CHALLENGE_2019_MAIN_H
