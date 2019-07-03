#ifndef TECHNICAL_CHALLENGE_2019_CONSTANTS_H
#define TECHNICAL_CHALLENGE_2019_CONSTANTS_H

namespace Constants {

    const int ROBOT_ID = 3;

    const float BALL_RADIUS_PIXELS = 60;// 41.6;
    const float REAL_DISTANCE = 30.0;
    const float REAL_RADIUS = 4.2;

    const float FOCAL_LENGTH = (BALL_RADIUS_PIXELS * REAL_DISTANCE) / REAL_RADIUS;


    const float HALF_MEASURING_WIDTH = 9; // 30 / 2; // For camera FOV width


    const float MEASURING_DISTANCE = 23; //30; // For camera FOV width


    const float HORIZONTAL_FOV_RADIANS = 2 * std::atan(HALF_MEASURING_WIDTH / MEASURING_DISTANCE);

}

#endif