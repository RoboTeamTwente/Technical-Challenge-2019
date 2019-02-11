#include "Main.h"

int main(int argc, char **argv) {
    // Initializing objects
    Interface interfaceObj;
    ImageProcessor imageProcessorObject;
    Camera cameraObject;
    BallFinder ballFinderObject;

    // TODO what to do with this?
    // TODO make not auto
    auto startTime = std::chrono::high_resolution_clock::now();

    // START LOOP
    while (true) {
        bool captureSuccess = cameraObject.captureImage(cameraImageBGR);
        if (!captureSuccess) {
            continue;
        }

        imageProcessorObject.imageConversion(cameraImageBGR, interfaceObj);

        imageProcessorObject.contourFinding();
        interfaceObj.drawContourAndBallTtrail(cameraObject.previousCameraBallX, cameraObject.previousCameraBallY,
                                              cameraObject.frameCounter, contours, contourImage, contours_poly,
                                              onecenter, oneradius,
                                              cameraImageBGR, imgLines, cameraImageThresholded, color, currentX);



        ballFinderObject.findBall(cameraObject.frameCounter, topDownBallX, topDownBallY, cameraObject.startFrameTime, oneradius, color,
                 currentX, topDownBallMeanPoint, cameraImageThresholded, ballSpeed,
                 topDown);

        interfaceObj.displayMatsAndDrawText(cameraImageBGR, imgLines, topDownBallMeanPoint, cameraImageThresholded,
                                                contourImage, ballSpeed,
                                                topDown);



        // END OF LOOP //

        if (cv::waitKey(30) == 27) {
            std::cout << "esc key pressed; ending program" << std::endl;
            break;
        }

    }


    return 0;

}

