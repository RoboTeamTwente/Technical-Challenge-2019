#include "Main.h"

int main(int argc, char **argv) {
    // Initializing objects
    Interface interfaceObject;
    ImageProcessor imageProcessorObject;
    Camera cameraObject;
    BallFinder ballFinderObject;

    if (!cameraObject.working) {
        return -1;
    }

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



        ballFinderObject.findBall();

        if (Settings::ENABLE_DRAWING) {
            interfaceObject.drawContourAndBallTrailOnCameraView();

            interfaceObject.drawTopDownView();

            interfaceObject.displayMatsAndDrawText();
        }



        // END OF LOOP //

        // TODO improve performance by changing waitkey
        if (cv::waitKey(30) == 27) {
            std::cout << "esc key pressed; ending program" << std::endl;
            break;
        }

    }


    return 0;

}

