#include "Main.h"
#include "Interface.h"
#include "Camera.h"
#include "ImageProcessor.h"
#include "BallFinder.h"
#include "Constants.h"
#include "Settings.h"
#include "Connection.h"

int main(int argc, char **argv) {
    // Initializing objects
    Interface interfaceObject;
    ImageProcessor imageProcessorObject;
    Camera cameraObject;
    BallFinder ballFinderObject;
    Connection connectionObject("/dev/ttyACM0");

    if (!cameraObject.working) {
        return -1;

    }

    // START LOOP
    while (true) {
        bool captureSuccess = cameraObject.captureImage();
        if (!captureSuccess) {
            std::cout << "Camera not available" << std::endl;
            break;
        }

        // Convert camera image to single channel image with only the ball remaining in it
        imageProcessorObject.imageConversion(cameraObject, interfaceObject);

        // Find ball center and radius on camera image, and store it in the variables created for this
        bool ballFindSuccess = imageProcessorObject.findBallContour();
        if (!ballFindSuccess) {

            if (cameraObject.frameCounter % 10 == 0 ) {
                connectionObject.sendStopCommand();
            }
            continue;
        }

        ballFinderObject.findTopDownBallPoint(imageProcessorObject);

        ballFinderObject.findMeanOfBallPoints();
        ballFinderObject.findBallSpeedVector(cameraObject);
        ballFinderObject.findBallInterceptionVector();

        if (Settings::ENABLE_DRAWING) {
            interfaceObject.drawContourAndBallTrailOnCameraView(cameraObject, imageProcessorObject);
            interfaceObject.drawTopDownView(ballFinderObject, imageProcessorObject);
            interfaceObject.displayMatsAndDrawText(cameraObject, imageProcessorObject, ballFinderObject);
        }

        if (Settings::ENABLE_CONNECTION){
            int velocity= ballFinderObject.topDownBallMeanPoint.x*10;
            std::cout << velocity << std::endl;
            int angle = std::acos(ballFinderObject.topDownBallMeanPoint.y);

            if ((connectionObject.lastVelocity!=velocity || connectionObject.lastAngle!=angle) || (cameraObject.frameCounter % 10 == 0 )) {
                connectionObject.sendMoveCommand(velocity, angle);
            }
        }
        // END OF LOOP //

        // TODO improve performance by removing waitkey
        // TODO Move entire interface to another thread
        // TODO also move USB Connection to other thread and use a queue

        if (cv::waitKey(30) == 27) {
            std::cout << "esc key pressed; ending program" << std::endl;
            break;
        }

    }


    return 0;

}

