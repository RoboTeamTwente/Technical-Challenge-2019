//
// Created by freek on 08/02/19.
//

#include "Main.h"
#include "Camera.h"


bool Camera::captureImage(){
    startFrameTime = std::chrono::high_resolution_clock::now();

    // get image from camera and save to the Mat
    bool captureSuccess = cap.read(cameraImageBGR);

    // check if this worked
    if (!captureSuccess) {
        std::cout << "Cannot read a frame from video stream" << std::endl;

    } else {
        frameCounter++;

    }
    return captureSuccess;
    // END IMAGE CAPTURE //
}


Camera::Camera() {
    // INIT CAMERA
    Camera cameraObj = Camera(); // Starts camera
    frameCounter = 0;

    std::cout << "init camera";
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cout << "webcam failure; is another openCV program running?" << std::endl;
        working=false;
    } else {
        working=true;
    }

}



