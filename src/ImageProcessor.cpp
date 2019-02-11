//
// Created by freek on 08/02/19.
//

#include "ImageProcessor.h"
#include "Interface.h"

cv::Mat ImageProcessor::imageConversion(cv::Mat cameraImageBGR, Interface interfaceObj) {
    cv::Mat imgHSV;
    cv::Mat cameraImageThresholded; // Could be a field

    cvtColor(cameraImageBGR, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    inRange(imgHSV, cv::Scalar(interfaceObj.LOW_HUE, interfaceObj.LOW_SATURATION, interfaceObj.LOW_VALUE),
            cv::Scalar(interfaceObj.HIGH_VALUE, interfaceObj.HIGH_SATURATION, interfaceObj.HIGH_VALUE),
            cameraImageThresholded);

    // some filtering

    //morphological opening
    erode(cameraImageThresholded, cameraImageThresholded,
          getStructuringElement(cv::MORPH_ELLIPSE,
                                cv::Size(MORPHOLOGICAL_OPENING_SIZE, MORPHOLOGICAL_OPENING_SIZE)));
    dilate(cameraImageThresholded, cameraImageThresholded,
           getStructuringElement(cv::MORPH_ELLIPSE,
                                 cv::Size(MORPHOLOGICAL_OPENING_SIZE, MORPHOLOGICAL_OPENING_SIZE)));

    return cameraImageThresholded;
}

void ImageProcessor::findBallContour(const cv::Mat &cameraImageThresholded, cv::Point2f &cameraImageBallCenterPoint,
                                     float &cameraImageBallRadius) {
    // init variables
    cv::Scalar colors[3];
    double largest_area = 0;
    colors[0] = cv::Scalar(255, 0, 0);
    colors[1] = cv::Scalar(0, 255, 0);
    colors[2] = cv::Scalar(0, 0, 255);
    std::vector<std::vector<cv::Point>> contours; // Could be a field
    std::vector<std::vector<cv::Point>> contoursAsPolygonsVector; // Could be a field
    std::vector<cv::Point> largest_contour; // Could be a field

    // clone this image
    // TODO Find out if removing cloning is safe to speed up performance
    cv::Mat cameraImageThresholdedClone = cameraImageThresholded.clone();

    // find contours, store in contours vector
    cv::findContours(cameraImageThresholdedClone, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // make an image for displaying the contours
    cv::Mat contourImage(cameraImageThresholdedClone.size(), CV_8UC3, cv::Scalar(0, 0, 0));

    // vector used for storing contours in polygon form
    contoursAsPolygonsVector = std::vector<std::vector<cv::Point>>(contours.size());

    // loop through all contours
    for (size_t idx = 0; idx < contours.size(); idx++) {

        // draw contour on contourImage
        drawContours(contourImage, contours, idx, colors[idx % 3]);

        // calculate area of contour
        double a = contourArea(contours[idx], false);

        // if it's the largest area so far:
        if (a > largest_area) {

            largest_area = a;
            largest_contour = contours[idx];

            // convert contour to a polygon and store it in contoursAsPolygonsVector
            approxPolyDP(cv::Mat(contours[idx]), contoursAsPolygonsVector[idx], 3, true);

            // find minimum enclosing circle of contour polygon and store in cameraImageBallCenterPoint and cameraImageBallRadius
            minEnclosingCircle((cv::Mat) contoursAsPolygonsVector[idx], cameraImageBallCenterPoint,
                               cameraImageBallRadius);


        }
    }
}


