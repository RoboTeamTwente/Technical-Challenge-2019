//
// Created by freek on 08/02/19.
//

#include "Main.h"
#include "Camera.h"

bool Camera::captureImage(){
    startFrameTime = std::chrono::steady_clock::now();
    // TODO implement a circular buffer for frame times to replace frameDurationInSeconds which is not that accurate

    // get image from camera and save to the Mat

    captureSuccess = cap.read(cameraImageBGR);

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



