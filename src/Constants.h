//
// Created by freek on 08/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_CONSTANTS_H
#define TECHNICAL_CHALLENGE_2019_CONSTANTS_H

#include <cmath>

namespace Constants{

    // focal length calculation adapted from https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
    float BALL_RADIUS_PIXELS = 41.6; //pixels
    float REAL_DISTANCE = 30.0; // cm
    float REAL_RADIUS = 4.2; //cm

    // HFOV calculation from https://www.mvteamcctv.com/news/Do-you-know-what-s-the-mean-of-FOV-HFOV-VFOV-DFOV-for-security-cameras.html
    float HALF_MEASURING_WIDTH = 30 / 2;
    float MEASURING_DISTANCE = 30;
    float HORIZONTAL_FOV_RADIANS = 2 * atan(HALF_MEASURING_WIDTH / MEASURING_DISTANCE); // 0.927295 = 53.13 degrees

    float FOCAL_LENGTH = (BALL_RADIUS_PIXELS * REAL_DISTANCE) / REAL_RADIUS;
};


#endif //TECHNICAL_CHALLENGE_2019_CONSTANTS_H
