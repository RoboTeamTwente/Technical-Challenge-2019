//
// Created by freek on 08/02/19.
//

#include "Main.h"
#include "Camera.h"

void cameraInit() {
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


Camera::Camera(const std::chrono::time_point &startFrameTime) : startFrameTime(startFrameTime) {}
}
    circle(contourImage, onecenter, (int) oneradius, color, 2, 8, 0);

    // END CONTOUR, BALL DRAWING //


    // DRAWING BALL TRAIL START START //

    // refresh trail every 30 frames
    if (cameraObject.frameCounter % 30 == 1) {
        imgLines = cv::Mat::zeros(cameraImageBGR.size(), CV_8UC3);
    }
    int currentY = onecenter.y;

    if (previousX >= 0 && previousY >= 0 && currentX >= 0 && currentY >= 0) {

        line(imgLines, cv::Point(currentX, currentY), cv::Point(previousX, previousY), cv::Scalar(255, 0, 0),
             10);
        // drawing blue line on original image
    }

    previousX = currentX;
    previousY = currentY;

    // DRAWING BALL TRAIL END //

}