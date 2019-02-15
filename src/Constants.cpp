// Constants.cpp
//

#include "Constants.h"

namespace Constants {

    float BALL_RADIUS_PIXELS = 41.6;


    float REAL_DISTANCE = 30.0;


    float REAL_RADIUS = 4.2;


    float HALF_MEASURING_WIDTH = 30 / 2;


    float MEASURING_DISTANCE = 30;


    float HORIZONTAL_FOV_RADIANS = 2 * std::atan(HALF_MEASURING_WIDTH / MEASURING_DISTANCE);


    float FOCAL_LENGTH = (BALL_RADIUS_PIXELS * REAL_DISTANCE) / REAL_RADIUS;
}

