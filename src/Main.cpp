#include "Main.h"
#include "Interface.h"
#include "Camera.h"
#include "ImageProcessor.h"
#include "BallFinder.h"
#include "Constants.h"
#include "Settings.h"
#include "Connection.h"
#include "Publisher.h"
#include <ros/ros.h>
#include <roboteam_msgs/RefereeCommand.h>
#include <roboteam_msgs/RefereeData.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "technical_challenge_2019");
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
    while (ros::ok()) {
        ros::spinOnce();
        bool captureSuccess = cameraObject.captureImage();
        if (!captureSuccess) {
            std::cout << "Camera not available" << std::endl;
            break;
        } else {
//            cameraObject.cap->set(CV_CAP_PROP_FPS, 25);
        }

        // Convert camera image to single channel image with only the ball remaining in it
        imageProcessorObject.imageConversion(cameraObject, interfaceObject);

        // Find ball center and radius on camera image, and store it in the variables created for this
        bool ballFindSuccess = imageProcessorObject.findBallContour();


        ballFinderObject.findTopDownBallPoint(imageProcessorObject);

        ballFinderObject.findMeanOfBallPoints();
        ballFinderObject.findBallSpeedVector(cameraObject);
        ballFinderObject.findBallInterceptionVector();


        if (Settings::ENABLE_DRAWING) {

            if (ballFindSuccess) {
                interfaceObject.drawContourAndBallTrailOnCameraView(cameraObject, imageProcessorObject);
                interfaceObject.drawTopDownView(ballFinderObject, imageProcessorObject);
                interfaceObject.displayMatsAndDrawText(cameraObject, imageProcessorObject, ballFinderObject);
            }

            if (cv::waitKey(40) == 27) { // TODO improve performance here
                std::cout << "esc key pressed; ending program" << std::endl;
                break;
            }
        }



        if (Settings::ENABLE_CONNECTION ){
            // TODO update robot object! Get newest values

            if  (publisher.refereeCommand==roboteam_msgs::RefereeCommand::NORMAL_START || publisher.refereeCommand==roboteam_msgs::RefereeCommand::FORCE_START){

    //            float velocity= ballFinderObject.topDownBallMeanPoint.x;
    //            std::cout << velocity << std::endl;
                if (!ballFindSuccess || imageProcessorObject.cameraImageBallRadius < 5) {

                    // TODO if ball was very close recently
                    //
                    //  else
                    //      send stop command
                    if (ballFinderObject.topDownBallMeanPoint.x < 20 && control.sentZero == false) { // If ball within 30 cm
                        control.takeBallGoBackwards(publisher);
                        control.sentZero = true;

                    } else {

                            publisher.command = control.makeSimpleCommand(0, 0, 0);
                            publisher.skillpublishRobotCommand(control);
                            control.sentZero = true;

                    }



                } else {

                    control.sentZero = false;
                    float meanAngle = std::atan2(ballFinderObject.topDownBallMeanPoint.y, ballFinderObject.topDownBallMeanPoint.x);
                    publisher.command = control.makeSimpleCommand(ballFinderObject.topDownBallMeanPoint.x, ballFinderObject.topDownBallMeanPoint.y, meanAngle);
                    publisher.skillpublishRobotCommand(control);
                }
            } else {
                publisher.command = control.makeSimpleCommand(0, 0, 0);

                control.sentZero = true;
                publisher.skillpublishRobotCommand(control);
            }



//            if ((connectionObject.lastVelocity!=velocity || connectionObject.lastAngle!=angle) || (cameraObject.frameCounter % 10 == 0 )) {
//                connectionObject.sendMoveCommand(velocity, angle);
//            }
        }
        // END OF LOOP //


\

        // TODO software should listen to robot STOP commands etc


    }


    return 0;

}

