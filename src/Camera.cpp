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
    frameCounter = 0;

    std::cout << "init camera";
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cout << "webcam failure; is another openCV program running?" << std::endl;
        return -1;
    }
}

void drawContourAndBallTtrail(int previousX, int previousY, int frameCounter,
                              const std::vector<std::vector<cv::Point>> &contours, const cv::Mat &contourImage,
                              const std::vector<std::vector<cv::Point>> &contours_poly,
                              const cv::cv::Point_<float> &onecenter, float oneradius, cv::Mat &imgBGR,
                              cv::Mat &imgLines, cv::Mat &imgThresholded, cv::Scalar &color, int &currentX) {

    color = cv::Scalar(255, 255, 255);
    currentX = onecenter.x;// START CONTOUR, BALL DRAWING //
    cv::Mat drawing = cv::Mat::zeros(imgThresholded.size(), CV_8UC3);;
    for (int i = 0; i < contours.size(); i++) {
        drawContours(drawing, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

    }
    circle(contourImage, onecenter, (int) oneradius, color, 2, 8, 0);

    // END CONTOUR, BALL DRAWING //


    // DRAWING BALL TRAIL START START //

    // refresh trail every 30 frames
    if (frameCounter % 30 == 1) {
        imgLines = cv::Mat::zeros(imgBGR.size(), CV_8UC3);
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