// Main.cpp
//

#include "Main.h"
#include "BallFinder.h"

// TODO move this
//std::vector<cv::Point2f> pointVector;
//
//cv::Mat cameraImageThresholded;
//cv::Mat contourImage;

//cv::Mat imgLines;


// anything below should stay here
cv::Mat cameraImageBGR;

int main(int argc, char **argv) {


    // Initializing objects
    Interface interfaceObj; // Starts interface
    ImageProcessor imageProcessorObject;
    Camera cameraObject;
    BallFinder ballFinderObject;

    // TODO what to do with this?
    // TODO make not auto
    auto startTime = std::chrono::high_resolution_clock::now();



    // END CAMERA INIT //

    // START LOOP
    while (true) {
        bool captureSuccess = cameraObject.captureImage(cameraImageBGR);
        if (!captureSuccess) {
            continue;
        }

        imageProcessorObject.imageConversionAndContourFinding(cameraImageBGR);

        interfaceObj.drawContourAndBallTtrail(cameraObject.previousCameraBallX, cameraObject.previousCameraBallY,
                                 cameraObject.frameCounter, contours, contourImage, contours_poly,
                                 onecenter, oneradius,
                                 cameraImageBGR, imgLines, cameraImageThresholded, color, currentX);


        float ballSpeed;
        cv::Mat topDown;
        findBall(cameraObject.frameCounter, topDownBallX, topDownBallY, cameraObject.startFrameTime, oneradius, color,
                 currentX, topDownBallMeanPoint, cameraImageThresholded, ballSpeed,
                 topDown);
        cameraImageBGR = displayMatsAndDrawText(cameraImageBGR, imgLines, topDownBallMeanPoint, cameraImageThresholded, contourImage, ballSpeed,
                                        topDown);



        // END OF LOOP //

        if (cv::waitKey(30) == 27) {
            std::cout << "esc key pressed; ending program" << std::endl;
            break;
        }

    }


    return 0;

}

