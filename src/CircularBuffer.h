//
// Created by freek on 12/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_CIRCULARBUFFER_H
#define TECHNICAL_CHALLENGE_2019_CIRCULARBUFFER_H

#include <vector>
#include <algorithm>
#include <opencv2/core/types.hpp>
template<typename T>
class CircularBuffer {
private:
    std::vector<T> internalVector;
    int indexOffset;
public:
    void moveIndex(int amount);
    T get(int relativeIndex);
    void set(T element, int relativeIndex);
    explicit CircularBuffer(std::vector<T> inputVector);
    void circularPush(T element);
    std::vector<T> getVector();
};




#endif //TECHNICAL_CHALLENGE_2019_CIRCULARBUFFER_H
