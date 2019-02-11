//
// Created by freek on 08/02/19.
//

#include "ImageProcessor.h"
#include "Interface.h"





void ImageProcessor::imageConversion(cv::Mat cameraImageBGR, Interface interfaceObj) {// START IMAGE CONVERSION //
//    largest_contour = std::vector<cv::Point>();
    largest_area = 0;
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

    // END IMAGE CONVERSION //

    // TODO find out whether to return imageThresholded or keep it as a field
}
void ImageProcessor::imageContourFinding(){
    // START CONTOUR FINDING //
    cv::Mat contourOutput = cameraImageThresholded.clone();
    cv::findContours(contourOutput, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


    cv::Mat contourImage(contourOutput.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Scalar colors[3];
    colors[0] = cv::Scalar(255, 0, 0);
    colors[1] = cv::Scalar(0, 255, 0);
    colors[2] = cv::Scalar(0, 0, 255);

    std::vector<std::vector<cv::Point>> contours_poly(contours.size());
    for (size_t idx = 0; idx < contours.size(); idx++) {

        drawContours(contourImage, contours, idx, colors[idx % 3]);
        // TODO find difference between drawContours here and 20 lines below

        double a = contourArea(contours[idx], false);  //  Find the area of contour
        if (a > largest_area) {

            largest_area = a;

            largest_contour = contours[idx];
            approxPolyDP(cv::Mat(contours[idx]), contours_poly[idx], 3, true);

            minEnclosingCircle((cv::Mat) contours_poly[idx], onecenter, oneradius);


        }
    }

    // TODO what to return? largest contour?
    // END CONTOUR FINDING //

}