//
// Created by freek on 08/02/19.
//


#include <QtCore/QTime>
#include "Camera.h"

bool Camera::captureImage(){
    startFrameTime = std::chrono::steady_clock::now();

    // get image from camera and save to the Mat

    int delay=0;
    QTime h;
    while(delay<15)// This is to get the latest image, not from buffer
    {
        h.start();//Start the chronometer
        captureSuccess = cap->read(cameraImageBGR); //Declarations not included in this example
        delay=h.elapsed(); //Compute delay in ms since the start commande
    }



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
    cap = new cv::VideoCapture(1);
    if (!cap->isOpened()) {
        std::cout << "webcam failure; is another openCV program running?" << std::endl;
        working=false;
    } else {
        cap->set(cv::CAP_PROP_BUFFERSIZE, 1);
    cap->set(cv::CAP_PROP_FPS, 25);

        working=true;
    }

}



