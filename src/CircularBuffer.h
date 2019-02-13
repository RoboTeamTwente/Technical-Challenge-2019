//
// Created by freek on 12/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_CIRCULARBUFFER_H
#define TECHNICAL_CHALLENGE_2019_CIRCULARBUFFER_H

#include <vector>
#include <algorithm>

class CircularBuffer {
public:


    int currentElement;
    template<typename T>
    explicit CircularBuffer();


    template<typename T>
    std::vector<T> internalVector;
};




#endif //TECHNICAL_CHALLENGE_2019_CIRCULARBUFFER_H
