#include "CircularBuffer.h"

template<typename T>
CircularBuffer::CircularBuffer(std::vector<T>* inputVector) {

    internalVector = inputVector->copy();
    currentElement = 0;

}
// 2 fields: the vector, and an index
// method circular push should replace current value at index, and move index forward by 1 (except at end)
// is much more efficient than rotating vector constantly


// TODO do not rotate vector but keep an internal pointer
template<typename T>
void circularPush(std::vector<T> vec, T element) {
    std::rotate(vec.rbegin(), vec.rbegin() + 1, vec.rend());
    // replace first element
    vec[0] = element;
}


