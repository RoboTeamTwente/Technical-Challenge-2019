#include "ImageProcessor.h"
#include "Interface.h"
#include "Camera.h"

#include "CircularBuffer.h"
#include "Constants.h"
#include "Settings.h"

ImageProcessor::ImageProcessor(){


    colors[0] = cv::Scalar(255, 0, 0);
    colors[1] = cv::Scalar(0, 255, 0);
    colors[2] = cv::Scalar(0, 0, 255);
    cameraImageBallCenterHistory = nullptr;
}

void ImageProcessor::imageConversion(Camera cameraObject, Interface interfaceObject) {


    cvtColor(cameraObject.cameraImageBGR, imgHSV, cv::COLOR_BGR2HSV);
    //Convert the captured frame from BGR to HSV
    inRange(imgHSV, cv::Scalar(interfaceObject.LOW_HUE, interfaceObject.LOW_SATURATION, interfaceObject.LOW_VALUE),
            cv::Scalar(interfaceObject.HIGH_HUE, interfaceObject.HIGH_SATURATION, interfaceObject.HIGH_VALUE),
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

    largestContourArea = 0;

    // find contours, store in contours vector
    cv::findContours(cameraImageThresholded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // make an image for displaying the contours
    cv::Mat contourImage(cameraImageThresholded.size(), CV_8UC3, cv::Scalar(0, 0, 0));

    // loop through all contours
    for (size_t idx = 0; idx < contours.size(); idx++) {

        // draw contour on contourImage
        drawContours(contourImage, contours, idx, colors[idx % 3]);

        // calculate area of contour
        double contourArea = cv::contourArea(contours[idx], false);

        // if it's the largest area so far:
        if (contourArea > largestContourArea) {
            largestContourArea = contourArea;
            largestContour = contours[idx];
        }
    }



    // convert contour to a polygon and store it in contoursAsPolygonsVector
    approxPolyDP(cv::Mat(largestContour), largestContourAsPolygon, 3, true);
    // find minimum enclosing circle of contour polygon and store in cameraImageBallCenterPoint and cameraImageBallRadius
    minEnclosingCircle(largestContourAsPolygon, cameraImageBallCenterPoint, cameraImageBallRadius);

    // TODO segfault happens here because this is always false?
    if (cameraImageBallCenterHistory == nullptr){

        std::vector<cv::Point2f> inputVector(30);
        // Set all values to initial value
        std::fill(inputVector.begin(), inputVector.end(), cameraImageBallCenterPoint);


        cameraImageBallCenterHistory = new CircularBuffer<cv::Point2f>(inputVector);

    } else {
        cameraImageBallCenterHistory->circularPush(cameraImageBallCenterPoint);
    }


}


