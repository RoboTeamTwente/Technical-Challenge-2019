// Main.cpp
//

#include "Main.h"
#include "ImageProcessor.h"

std::vector<double> timeVector;
std::vector<cv::Point2f> pointVector;

int main(int argc, char **argv) {
    auto startTime = std::chrono::high_resolution_clock::now();



    // INIT CAMERA
    //Camera cameraObj = Camera(); // Starts camera
    Interface interfaceObj; // Starts interface





    int previousX = -1;
    int previousY = -1;
    int frameCounter = 0;

    cv::Mat imgBGR;
    cv::Mat imgLines;
    float x = -1;
    float y = -1;
    cv::Point2f meanPoint;
    meanPoint = cv::Point2f(-1, -1);

    // START CAMERA INIT //

    std::cout << "init camera";
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cout << "webcam failure; is another openCV program running?" << std::endl;
        return -1;
    }

    // END CAMERA INIT //

    // START LOOP
    while (true) {
        frameCounter++;
        std::vector<cv::Point> largest_contour;
        int largest_area = 0;


        auto startFrameTime = std::chrono::high_resolution_clock::now();






        // START IMAGE CAPTURE //

        bool captureSuccess = cap.read(imgBGR);
        if (!captureSuccess) {
            std::cout << "Cannot read a frame from video stream" << std::endl;
            break;
        }

        //


        // END IMAGE CAPTURE //

        // START IMAGE CONVERSION //

        cv::Mat imgHSV;
        cv::cvtColor(imgBGR, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        cv::Mat imgThresholded;

        cv::inRange(imgHSV, cv::Scalar(interfaceObj.iLowH, interfaceObj.iLowS, interfaceObj.iLowV),
                    cv::Scalar(interfaceObj.iHighH, interfaceObj.iHighS, interfaceObj.iHighV), imgThresholded);

        // some filtering

        //morphological opening
        erode(imgThresholded, imgThresholded,
              getStructuringElement(cv::MORPH_ELLIPSE,
                                    cv::Size(MORPHOLOGICAL_OPENING_SIZE, MORPHOLOGICAL_OPENING_SIZE)));
        dilate(imgThresholded, imgThresholded,
               getStructuringElement(cv::MORPH_ELLIPSE,
                                     cv::Size(MORPHOLOGICAL_OPENING_SIZE, MORPHOLOGICAL_OPENING_SIZE)));

        // END IMAGE CONVERSION //


        // START CONTOUR FINDING //
        std::vector<std::vector<cv::Point> > contours;
        cv::Mat contourOutput = imgThresholded.clone();
        cv::findContours(contourOutput, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


        cv::Mat contourImage(contourOutput.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Scalar colors[3];
        colors[0] = cv::Scalar(255, 0, 0);
        colors[1] = cv::Scalar(0, 255, 0);
        colors[2] = cv::Scalar(0, 0, 255);

        std::vector<std::vector<cv::Point>> contours_poly(contours.size());
        cv::cv::Point_<float> onecenter;
        float oneradius;

        for (size_t idx = 0; idx < contours.size(); idx++) {

            cv::drawContours(contourImage, contours, idx, colors[idx % 3]);
            // TODO find difference between drawContours here and 20 lines below

            double a = contourArea(contours[idx], false);  //  Find the area of contour
            if (a > largest_area) {

                largest_area = a;

                largest_contour = contours[idx];
                cv::approxPolyDP(cv::Mat(contours[idx]), contours_poly[idx], 3, true);

                cv::minEnclosingCircle((cv::Mat) contours_poly[idx], onecenter, oneradius);


            }
        }

        // END CONTOUR FINDING //

        // START CONTOUR, BALL DRAWING //
        cv::Mat drawing = cv::Mat::zeros(imgThresholded.size(), CV_8UC3);;
        cv::Scalar color = cv::Scalar(255, 255, 255);

        for (int i = 0; i < contours.size(); i++) {
            cv::drawContours(drawing, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

        }
        cv::circle(contourImage, onecenter, (int) oneradius, color, 2, 8, 0);

        // END CONTOUR, BALL DRAWING //


        // DRAWING BALL TRAIL START START //

        // refresh trail every 30 frames
        if (frameCounter % 30 == 1) {
            imgLines = cv::Mat::zeros(imgBGR.size(), CV_8UC3);
        }

        int currentX = onecenter.x;
        int currentY = onecenter.y;

        if (previousX >= 0 && previousY >= 0 && currentX >= 0 && currentY >= 0) {

            cv::line(imgLines, cv::Point(currentX, currentY), cv::Point(previousX, previousY), cv::Scalar(255, 0, 0),
                     10);
            // drawing blue line on original image
        }

        previousX = currentX;
        previousY = currentY;

        // DRAWING BALL TRAIL END //
        float ballSpeed;
        cv::Mat topDown;
        findBall(frameCounter, x, y, startFrameTime, oneradius, color, currentX, meanPoint, imgThresholded, ballSpeed,
                 topDown);


        // START DISPLAY MATS //

        cv::imshow("Thresholded Image", imgThresholded);
        cv::moveWindow("Thresholded Image", 0, 0);
        imgBGR = imgBGR + imgLines + contourImage;
        cv::imshow("Original", imgBGR);
        cv::moveWindow("Original", 0, 600);
        cv::imshow("Top down view", topDown);
        cv::moveWindow("Top down view", 800, 600);

        // END DISPLAY MATS //

        // START TOPDOWN TEXT DRAWING

        cv::String text1 = "x=" + std::to_string(meanPoint.x);
        cv::putText(topDown, text1, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv::LINE_AA);

        cv::String text2 = "y=" + std::to_string(meanPoint.y);
        cv::putText(topDown, text2, cv::Point(10, 40 * 2), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2,
                    cv::LINE_AA);

        cv::String text3 = "speed=" + std::to_string(ballSpeed);
        cv::putText(topDown, text3, cv::Point(10, IMAGE_HEIGHT - 40), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2,
                    cv::LINE_AA);


        // END TOPDOWN TEXT DRAWING //



        // END OF LOOP //

        if (cv::waitKey(30) == 27) {
            std::cout << "esc key pressed; ending program" << std::endl;
            break;
        }

    }


    return 0;

}

