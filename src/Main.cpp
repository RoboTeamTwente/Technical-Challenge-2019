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
    Connection connectionObject("/dev/ttyAMA0", 115200);

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

        connectionObject.sendCommand();

        // END OF LOOP //

        // TODO improve performance by removing waitkey
        // TODO Move entire interface to another thread

        if (cv::waitKey(30) == 27) {
            std::cout << "esc key pressed; ending program" << std::endl;
            break;
        }

    }


    return 0;

}

