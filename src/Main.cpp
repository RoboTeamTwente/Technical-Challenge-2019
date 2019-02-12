#include "Main.h"

int main(int argc, char **argv) {
    // Initializing objects
    Interface interfaceObject;
    ImageProcessor imageProcessorObject;
    Camera cameraObject;
    BallFinder ballFinderObject;


    cv::Mat cameraImageThresholded;

    cv::Point2f cameraImageBallCenterPoint;
    float cameraImageBallRadius;



    // START LOOP
    while (true) {
        bool captureSuccess = cameraObject.captureImage();
        if (!captureSuccess) {
            continue;
        }

        // Convert camera image to single channel image with only the ball remaining in it
        imageProcessorObject.imageConversion(cameraObject, interfaceObject);

        // Find ball center and radius on camera image, and store it in the variables created for this
        // TODO maybe return as a boolean, and if failed continue
        imageProcessorObject.findBallContour();

        interfaceObj.drawContourAndBallTrail();



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

