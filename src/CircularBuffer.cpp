#include <opencv2/core/types.hpp>
#include "CircularBuffer.h"

template<typename T>
CircularBuffer<T>::CircularBuffer(std::vector<T> inputVector) {
    internalVector = inputVector; // apparently copies the vector
    indexOffset = 0;
}

template<typename T>
void CircularBuffer<T>::moveIndex(int amount) {
    if (indexOffset + amount >= 0) {
        indexOffset = (indexOffset + amount) % internalVector.size();
    } else {
        while (indexOffset + amount < 0) {
            indexOffset += internalVector.size();
        }
        indexOffset = (indexOffset + amount) % internalVector.size();
    }
}


template<typename T>
void CircularBuffer<T>::circularPush(T element) {
    moveIndex(-1);
    set(element, 0);
}

template<typename T>
T CircularBuffer<T>::get(int relativeIndex) {
    return internalVector.at((relativeIndex + indexOffset) % internalVector.size());
}

template<typename T>
void CircularBuffer<T>::set(T element, int relativeIndex) {
    internalVector.at((relativeIndex + indexOffset) % internalVector.size()) = element;
}

template<typename T>
std::vector<T> CircularBuffer<T>::getVector() {
    return internalVector;
}

template class CircularBuffer<cv::Point2f>;


