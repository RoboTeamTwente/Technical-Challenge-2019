#ifndef TECHNICAL_CHALLENGE_2019_CONSTANTS_H
#define TECHNICAL_CHALLENGE_2019_CONSTANTS_H

namespace Constants {

    const int ROBOT_ID = 2;

    const float BALL_RADIUS_PIXELS = 41.6;


    const float REAL_DISTANCE = 30.0;


    const float REAL_RADIUS = 4.2;


    const float HALF_MEASURING_WIDTH = 30 / 2;


    const float MEASURING_DISTANCE = 30;


    const float HORIZONTAL_FOV_RADIANS = 2 * std::atan(HALF_MEASURING_WIDTH / MEASURING_DISTANCE);


    const float FOCAL_LENGTH = (BALL_RADIUS_PIXELS * REAL_DISTANCE) / REAL_RADIUS;
}

#endif