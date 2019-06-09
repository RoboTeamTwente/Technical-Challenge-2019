#include "Main.h"
#include "Interface.h"
#include "Camera.h"
#include "ImageProcessor.h"
#include "BallFinder.h"
#include "Constants.h"
#include "Settings.h"
#include "Connection.h"
#include "Publisher.h"

int main(int argc, char **argv) {
    // Initializing objects
    Interface interfaceObject;
    ImageProcessor imageProcessorObject;
    Camera cameraObject;
    BallFinder ballFinderObject;
    Connection connectionObject("/dev/ttyACM0");
    Control control;
    Publisher publisher(control);

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

            // TODO if ball was very close recently
            //      turn dribbler on. Go forwards 0.3m, go back 0.7m
            //  else
            //      send stop command

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
            // TODO update robot object! Get newest values

            float velocity= ballFinderObject.topDownBallMeanPoint.x;
            std::cout << velocity << std::endl;
            float angle = std::atan2(ballFinderObject.topDownBallMeanPoint.y,ballFinderObject.topDownBallMeanPoint.x);
            publisher.command = control.makeCommand(ballFinderObject.topDownBallMeanPoint.x, ballFinderObject.topDownBallMeanPoint.y, angle);
            publisher.skillpublishRobotCommand();


//            if ((connectionObject.lastVelocity!=velocity || connectionObject.lastAngle!=angle) || (cameraObject.frameCounter % 10 == 0 )) {
//                connectionObject.sendMoveCommand(velocity, angle);
//            }
        }
        // END OF LOOP //


\

        // TODO software should listen to robot STOP commands etc

        if (cv::waitKey(30) == 27) {
            std::cout << "esc key pressed; ending program" << std::endl;
            break;
        }

    }


    return 0;

}

