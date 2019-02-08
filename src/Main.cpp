// Main.cpp
//

#include "Main.h"
#include "BallFinder.h"


BallFinder bf = BallFinder();

TestClass t();

std::vector<double> timeVector;
std::vector<cv::Point2f> pointVector;
cv::Mat imgThresholded;
cv::Mat contourImage;
// TODO make startFrameTime not auto
auto startFrameTime;
float topDownBallX;
float topDownBallY;

// anything below should stay here
cv::Mat imgBGR;
cv::Mat imgLines;
cv::Point2f topDownBallMeanPoint;

int main(int argc, char **argv) {
    auto startTime = std::chrono::high_resolution_clock::now();

    topDownBallX = -1;
    topDownBallY = -1;
    topDownBallMeanPoint = cv::Point2f(-1, -1);
    Interface interfaceObj; // Starts interface



    // END CAMERA INIT //

    // START LOOP
    while (true) {



        // TODO move this specification?
        largest_contour = nullptr;
        largest_area = 0;
        startFrameTime = std::chrono::high_resolution_clock::now();

        // START IMAGE CAPTURE //

        bool captureSuccess = cap.read(imgBGR);
        if (!captureSuccess) {
            std::cout << "Cannot read a frame from video stream" << std::endl;
            continue;
        }

        frameCounter++;
        // END IMAGE CAPTURE //


        // TODO specify these to be null here
//        cv::Mat imgThresholded;
//        std::vector<std::vector<cv::Point>> contours;
//        cv::Mat contourImage;
//        std::vector<std::vector<cv::Point>> contours_poly;

        cv::Point_<float> onecenter;
        float oneradius;

        imageConversionAndContourFinding(interfaceObj, imgBGR, largest_contour, largest_area, imgThresholded, contours,
                                         onecenter, oneradius);
        cv::Scalar color;
        int currentX;
        drawContourAndBallTtrail(previousCameraBallX, previousCameraBallY, frameCounter, contours, contourImage, contours_poly,
                                 onecenter, oneradius,
                                 imgBGR, imgLines, imgThresholded, color, currentX);


        float ballSpeed;
        cv::Mat topDown;
        findBall(frameCounter, topDownBallX, topDownBallY, startFrameTime, oneradius, color, currentX, topDownBallMeanPoint, imgThresholded, ballSpeed,
                 topDown);
        imgBGR = displayMatsAndDrawText(imgBGR, imgLines, topDownBallMeanPoint, imgThresholded, contourImage, ballSpeed,
                                        topDown);



        // END OF LOOP //

        if (cv::waitKey(30) == 27) {
            std::cout << "esc key pressed; ending program" << std::endl;
            break;
        }

    }


    return 0;

}

