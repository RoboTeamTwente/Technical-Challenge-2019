//
// Created by freek on 08/02/19.
//

#include "Interface.h"

// CONSTRUCTOR
Interface::Interface() {
    // Open control window
    cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create control window
    cvCreateTrackbar("LowH", "Control", &LOW_HUE, 179); //Hue
    cvCreateTrackbar("HighH", "Control", &HIGH_HUE, 179);

    cvCreateTrackbar("LowS", "Control", &LOW_SATURATION, 255); //Saturation
    cvCreateTrackbar("HighS", "Control", &HIGH_SATURATION, 255);

    cvCreateTrackbar("LowV", "Control", &LOW_VALUE, 255); //Value
    cvCreateTrackbar("HighV", "Control", &HIGH_VALUE, 255);
    cv::moveWindow("Control", 500, 500);

}

void Interface::drawContourAndBallTrailOnCameraView(Camera cameraObject, ImageProcessor imageProcessorObject) {
    // TODO define previousX, previousY

    cameraImageContourOverlay = cv::Mat::zeros(imageProcessorObject.cameraImageThresholded.size(), CV_8UC3);

    // draw imageProcessorObject.contours on image
    for (int i = 0; i < imageProcessorObject.contours.size(); i++) {
        cv::drawContours(cameraImageContourOverlay, imageProcessorObject.contoursAsPolygonsVector, i, cv::Scalar(255, 255, 255), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    }

    // draw circle of ball on image
    circle(cameraImageContourOverlay, imageProcessorObject.cameraImageBallCenterPoint, (int) imageProcessorObject.cameraImageBallRadius, color, 2, 8, 0);

    // DRAWING BALL TRAIL START START //

    // TODO replace by vector of size 30 or so, and circularPush it each time

    // refresh trail every 30 frames
    if (cameraObject.frameCounter % 30 == 1) {
        cameraImageTrailOverlay = cv::Mat::zeros(imageProcessorObject.cameraImageThresholded.size(), CV_8UC3);
    }


    if (previousX >= 0 && previousY >= 0 && imageProcessorObject.cameraImageBallCenterPoint.x >= 0 && imageProcessorObject.cameraImageBallCenterPoint.y >= 0) {

        line(cameraImageTrailOverlay, cv::Point(imageProcessorObject.cameraImageBallCenterPoint.x, imageProcessorObject.cameraImageBallCenterPoint.y), cv::Point(previousX, previousY), cv::Scalar(255, 0, 0),
             10);
        // drawing blue line on original image
    }

    previousX = imageProcessorObject.cameraImageBallCenterPoint.x;
    previousY = imageProcessorObject.cameraImageBallCenterPoint.y;

    // DRAWING BALL TRAIL END //
}

void Interface::drawTopDownView() {

    // DRAWING TOP DOWN MAP STUFF //


    cv::Point_<float> cameraXandY(100, 240);



    float line1x = 540 * cos(-0.5 * Constants::HORIZONTAL_FOV_RADIANS);
    float line1y = 540 * sin(-0.5 * Constants::HORIZONTAL_FOV_RADIANS);

    float line2x = 540 * cos(0.5 * Constants::HORIZONTAL_FOV_RADIANS);
    float line2y = 540 * sin(0.5 * Constants::HORIZONTAL_FOV_RADIANS);

    line(topDown, cv::Point2i(100 + line1x, 240 + line1y), cv::Point2i(cameraXandY.x, cameraXandY.y),
         cv::Scalar<double>(255, 255, 255), 1);
    line(topDown, cv::Point2i(100 + line2x, 240 + line2y), cv::Point2i(cameraXandY.x, cameraXandY.y),
         cv::Scalar<double>(255, 255, 255), 1);

    cv::Scalar_<double> orange = cv::Scalar_<double>(2, 106, 253);
    cv::Scalar_<double> bluegray = cv::Scalar_<double>(255, 120, 120);

    cv::Point_<float> topDownBallPos;
    topDownBallPos.x = 100 + topDownBallMeanPoint.x * 5;
    topDownBallPos.y = 240 + topDownBallMeanPoint.y * 5;

    circle(topDown, topDownBallPos, (int) 5, orange, 2, 8, 0); // draw orange ball

    line(topDown, topDownBallPos, (topDownBallPos + (speedPoint * 1)), orange, 2); //speed line

    // END OF TOP DOWN INIT //
}

void Interface::displayMatsAndDrawText(Camera cameraObject, ImageProcessor imageProcessorObject) {
    // START DISPLAY MATS //
    cv::Mat cameraImageForDisplay;

    imshow("Thresholded Image", imageProcessorObject.cameraImageThresholded);
    cv::moveWindow("Thresholded Image", 0, 0);
    cameraImageForDisplay = cameraObject.cameraImageBGR + cameraImageTrailOverlay + cameraImageContourOverlay;
    imshow("Original", cameraObject.cameraImageBGR);
    cv::moveWindow("Original", 0, 600);
    imshow("Top down view", topDown);
    cv::moveWindow("Top down view", 800, 600);

    // END DISPLAY MATS //

    // START TOPDOWN TEXT DRAWING

    cv::String text1 = "x=" + std::__cxx11::to_string(meanPoint.x);
    putText(topDown, text1, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv::LINE_AA);

    cv::String text2 = "y=" + std::__cxx11::to_string(meanPoint.y);
    putText(topDown, text2, cv::Point(10, 40 * 2), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2,
            cv::LINE_AA);

    cv::String text3 = "speed=" + std::__cxx11::to_string(ballSpeed);
    putText(topDown, text3, cv::Point(10, Settings::IMAGE_HEIGHT - 40), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2,
            cv::LINE_AA);


    // END TOPDOWN TEXT DRAWING //


}