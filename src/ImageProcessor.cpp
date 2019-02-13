#include "ImageProcessor.h"
#include "Interface.h"
#include "CircularBuffer.h"

ImageProcessor::ImageProcessor() {
    cameraImageBallCenterHistory = new CircularBuffer(new std::vector<cv::Point2f>(30));

}

void ImageProcessor::imageConversion(Camera &cameraObject, Interface &interfaceObject) {
    cv::Mat imgHSV;

    cvtColor(cameraObject.cameraImageBGR, imgHSV, cv::COLOR_BGR2HSV);
    //Convert the captured frame from BGR to HSV
    inRange(imgHSV, cv::Scalar(interfaceObject.LOW_HUE, interfaceObject.LOW_SATURATION, interfaceObject.LOW_VALUE),
            cv::Scalar(interfaceObject.HIGH_VALUE, interfaceObject.HIGH_SATURATION, interfaceObject.HIGH_VALUE),
            cameraImageThresholded);

    //morphological opening
    erode(cameraImageThresholded, cameraImageThresholded,
          getStructuringElement(cv::MORPH_ELLIPSE,
                                cv::Size(Settings::MORPHOLOGICAL_OPENING_SIZE, Settings::MORPHOLOGICAL_OPENING_SIZE)));
    dilate(cameraImageThresholded, cameraImageThresholded,
           getStructuringElement(cv::MORPH_ELLIPSE,
                                 cv::Size(Settings::MORPHOLOGICAL_OPENING_SIZE, Settings::MORPHOLOGICAL_OPENING_SIZE)));


}

void ImageProcessor::findBallContour() {
    // init variables
    cv::Scalar colors[3];
    double largestContourArea = 0;
    colors[0] = cv::Scalar(255, 0, 0);
    colors[1] = cv::Scalar(0, 255, 0);
    colors[2] = cv::Scalar(0, 0, 255);
    std::vector<cv::Point> largestContour;

    // clone this image
    // TODO Find out if removing cloning is safe to speed up performance
    cv::Mat cameraImageThresholdedClone = cameraImageThresholded.clone();

    // find contours, store in contours vector
    cv::findContours(cameraImageThresholdedClone, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // make an image for displaying the contours
    cv::Mat contourImage(cameraImageThresholdedClone.size(), CV_8UC3, cv::Scalar(0, 0, 0));



    // loop through all contours
    for (size_t idx = 0; idx < contours.size(); idx++) {

        // draw contour on contourImage
        drawContours(contourImage, contours, idx, colors[idx % 3]);

        // calculate area of contour
        double contourArea = contourArea(contours[idx], false);

        // if it's the largest area so far:
        if (contourArea > largestContourArea) {
            largestContourArea = contourArea;
            largestContour = contours[idx];
        }
    }

    std::vector<cv::Point> largestContourAsPolygon;

    // convert contour to a polygon and store it in contoursAsPolygonsVector
    approxPolyDP(cv::Mat(largestContour), largestContourAsPolygon, 3, true);
    // find minimum enclosing circle of contour polygon and store in cameraImageBallCenterPoint and cameraImageBallRadius
    minEnclosingCircle(largestContourAsPolygon, cameraImageBallCenterPoint, cameraImageBallRadius);

    // TODO Check if this is not a null pointer exception

    if (cameraImageBallCenterHistory->internalVector.empty()){
        // Set all values to initial value
        cameraImageBallCenterHistory = std::vector<cv::Point2f>(30);
        std::fill(cameraImageBallCenterHistory.begin(), cameraImageBallCenterHistory.end(), cameraImageBallCenterPoint);
    }


}


