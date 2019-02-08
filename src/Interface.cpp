//
// Created by freek on 08/02/19.
//

#include "Interface.h"

cv::Mat &displayMatsAndDrawText(cv::Mat &imgBGR, const cv::Mat &imgLines, const cv::Point2f &meanPoint,
                                const cv::Mat &imgThresholded, const cv::Mat &contourImage, float ballSpeed,
                                const cv::Mat &topDown);

    Interface::Interface() {
    {
        cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create control window
        cvCreateTrackbar("LowH", "Control", &LOW_HUE, 179); //Hue
        cvCreateTrackbar("HighH", "Control", &HIGH_HUE, 179);

        cvCreateTrackbar("LowS", "Control", &LOW_SATURATION, 255); //Saturation
        cvCreateTrackbar("HighS", "Control", &HIGH_SATURATION, 255);

        cvCreateTrackbar("LowV", "Control", &LOW_VALUE, 255); //Value
        cvCreateTrackbar("HighV", "Control", &HIGH_VALUE, 255);
        cv::moveWindow("Control", 500, 500);
    }
    }

    cv::Mat &displayMatsAndDrawText(cv::Mat &imgBGR, const cv::Mat &imgLines, const cv::Point2f &meanPoint,
                                const cv::Mat &imgThresholded, const cv::Mat &contourImage, float ballSpeed,
                                const cv::Mat &topDown) {// START DISPLAY MATS //

    imshow("Thresholded Image", imgThresholded);
    cv::moveWindow("Thresholded Image", 0, 0);
    imgBGR = imgBGR + imgLines + contourImage;
    imshow("Original", imgBGR);
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
    putText(topDown, text3, cv::Point(10, IMAGE_HEIGHT - 40), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2,
            cv::LINE_AA);


    // END TOPDOWN TEXT DRAWING //



    return imgBGR;
}