//
// Created by freek on 08/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_INTERFACE_H
#define TECHNICAL_CHALLENGE_2019_INTERFACE_H


#include <cv.hpp>
#include "Constants.h"

class Interface {
public:
    int LOW_HUE = 0;
    int HIGH_HUE = 5;

    int LOW_SATURATION = 126;
    int HIGH_SATURATION = 255;

    int LOW_VALUE = 92;
    int HIGH_VALUE = 255;

    // CONSTRUCTOR
    explicit Interface();
};


#endif //TECHNICAL_CHALLENGE_2019_INTERFACE_H
