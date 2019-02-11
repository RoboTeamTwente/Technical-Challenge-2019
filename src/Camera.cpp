//
// Created by freek on 08/02/19.
//

#include "Main.h"
#include "Camera.h"


bool Camera::captureImage(cv::Mat inputMat){
    // START IMAGE CAPTURE //

    startFrameTime = std::chrono::high_resolution_clock::now();

    // get image from camera and save to cameraImageBGR
    bool captureSuccess = cap.read(inputMat);

    // check if this worked
    if (!captureSuccess) {
        std::cout << "Cannot read a frame from video stream" << std::endl;

    } else {
        frameCounter++;

    }
    return captureSuccess;
    // END IMAGE CAPTURE //
}

// TODO ?? what to do here
Camera::Camera() {
    // INIT CAMERA
    Camera cameraObj = Camera(); // Starts camera
    previousCameraBallX = -1;
    previousCameraBallY = -1;
    cameraObject.frameCounter = 0;

    std::cout << "init camera";
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cout << "webcam failure; is another openCV program running?" << std::endl;
        return -1;
    }
}



}