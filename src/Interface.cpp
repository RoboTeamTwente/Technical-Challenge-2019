#include "Interface.h"
#include "BallFinder.h"
#include "ImageProcessor.h"

#include "Camera.h"

#include "CircularBuffer.h"
#include "Constants.h"
#include "Settings.h"

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

    topDownCameraPositionPoint = cv::Point_<float>(100, 240);
    line1x = 540 * cos(-0.5 * Constants::HORIZONTAL_FOV_RADIANS);

    line1y = 540 * sin(-0.5 * Constants::HORIZONTAL_FOV_RADIANS);


    line2x = 540 * cos(0.5 * Constants::HORIZONTAL_FOV_RADIANS);
    line2y = 540 * sin(0.5 * Constants::HORIZONTAL_FOV_RADIANS);
    orange = cv::Scalar_<double>(2, 106, 253);
    bluegray = cv::Scalar_<double>(255, 120, 120);

}

void Interface::drawContourAndBallTrailOnCameraView(Camera cameraObject, ImageProcessor imageProcessorObject) {

    cameraImageContourOverlay = cv::Mat::zeros(imageProcessorObject.cameraImageThresholded.size(), CV_8UC3);
    cameraImageTrailOverlay = cv::Mat::zeros(imageProcessorObject.cameraImageThresholded.size(), CV_8UC3);

    // draw imageProcessorObject.contours on image
    for (int i = 0; i < imageProcessorObject.contours.size(); i++) {
        cv::drawContours(cameraImageContourOverlay, imageProcessorObject.contoursAsPolygonsVector, i, cv::Scalar(255, 255, 255), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    }

    // draw circle of ball on image
    circle(cameraImageContourOverlay, imageProcessorObject.cameraImageBallCenterPoint, (int) imageProcessorObject.cameraImageBallRadius, cv::Scalar(255,255,255), 2, 8, 0);

    // DRAWING BALL TRAIL START START //

    for (int i=0; i<imageProcessorObject.cameraImageBallCenterHistory->getVector().size()-1; i++) {
        cv::line(cameraImageTrailOverlay,imageProcessorObject.cameraImageBallCenterHistory->get(i),
                imageProcessorObject.cameraImageBallCenterHistory->get(i+1),cv::Scalar(255, 0, 0),10);
    }

    // DRAWING BALL TRAIL END //
}

void Interface::drawTopDownView(BallFinder ballFinderObject, ImageProcessor imageProcessorObject) {

    // DRAWING TOP DOWN MAP STUFF //
    topDownDrawingMat = cv::Mat::zeros(imageProcessorObject.cameraImageThresholded.size(), CV_8UC3);


    line(topDownDrawingMat, cv::Point2i(100 + line1x, 240 + line1y), cv::Point2i(topDownCameraPositionPoint.x, topDownCameraPositionPoint.y),
         cv::Scalar(255, 255, 255), 1);
    line(topDownDrawingMat, cv::Point2i(100 + line2x, 240 + line2y), cv::Point2i(topDownCameraPositionPoint.x, topDownCameraPositionPoint.y),
         cv::Scalar(255, 255, 255), 1);

    topDownBallPositionForDrawing.x = 100 + ballFinderObject.topDownBallMeanPoint.x * 5;
    topDownBallPositionForDrawing.y = 240 + ballFinderObject.topDownBallMeanPoint.y * 5;

    circle(topDownDrawingMat, topDownBallPositionForDrawing, (int) 5, orange, 2, 8, 0); // draw orange ball

    line(topDownDrawingMat, topDownBallPositionForDrawing, (topDownBallPositionForDrawing + (ballFinderObject.ballVelocityVectorAsPoint * 1)), orange, 2); //speed line

    // END OF TOP DOWN INIT //

    // drawing intercept line

    line(topDownDrawingMat, topDownCameraPositionPoint, topDownCameraPositionPoint + (ballFinderObject.interceptPos * 1), bluegray, 2); //speed line

}

void Interface::displayMatsAndDrawText(Camera cameraObject, ImageProcessor imageProcessorObject, BallFinder ballFinderObject) {
    // START DISPLAY MATS //
    cv::Mat cameraImageForDisplay;

    imshow("Thresholded Image", imageProcessorObject.cameraImageThresholded);
    cv::moveWindow("Thresholded Image", 0, 0);
    cameraImageForDisplay = cameraObject.cameraImageBGR + cameraImageTrailOverlay + cameraImageContourOverlay;
    imshow("Original", cameraObject.cameraImageBGR);
    cv::moveWindow("Original", 0, 600);
    imshow("Top down view", topDownDrawingMat);
    cv::moveWindow("Top down view", 800, 600);

    // END DISPLAY MATS //

    // START TOPDOWN TEXT DRAWING

    cv::String text1 = "x=" + std::__cxx11::to_string(ballFinderObject.topDownBallMeanPoint.x);
    putText(topDownDrawingMat, text1, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv::LINE_AA);

    cv::String text2 = "y=" + std::__cxx11::to_string(ballFinderObject.topDownBallMeanPoint.y);
    putText(topDownDrawingMat, text2, cv::Point(10, 40 * 2), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2,
            cv::LINE_AA);

    cv::String text3 = "speed=" + std::__cxx11::to_string(ballFinderObject.ballSpeed);
    putText(topDownDrawingMat, text3, cv::Point(10, Settings::IMAGE_HEIGHT - 40), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2,
            cv::LINE_AA);



    // END TOPDOWN TEXT DRAWING //

    // write text in terminal
    std::cout << "distance=" << ballFinderObject.ballDistanceFromCamera << ", angle=" << ballFinderObject.ballAngleInCameraPlaneDegrees << std::endl;
    std::cout << "x=" << ballFinderObject.topDownBallMeanPoint.x << ", y=" << ballFinderObject.topDownBallMeanPoint.y << std::endl;

    std::cout << "ballspeed in cm/s:" << ballFinderObject.ballSpeed << std::endl;

}