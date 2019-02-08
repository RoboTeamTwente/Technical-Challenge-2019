//
// Created by freek on 08/02/19.
//

#include "Interface.h"

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