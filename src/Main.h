// Main.h
//

#ifndef TECHNICAL_CHALLENGE_2019_MAIN_H
#define TECHNICAL_CHALLENGE_2019_MAIN_H


#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include <chrono>
#include <opencv2/imgproc.hpp>

#include "Constants.h"
#include "ImageProcessor.h"
#include "Interface.h"
#include "BallFinder2.cpp"
#include "Camera.h"
#include "TestClass.h"

extern std::vector<double> timeVector;
extern std::vector<cv::Point2f> pointVector;

int main(int argc, char **argv);

#endif
