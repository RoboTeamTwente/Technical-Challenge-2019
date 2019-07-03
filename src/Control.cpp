//
// Created by freek on 06/06/19.
//


#include <roboteam_msgs/RobotCommand.h>
#include <opencv2/core/types.hpp>
#include "Control.h"
#include "src/lib/Robot.h"
#include <roboteam_utils/Vector2.h>
#include <chrono>
#include <thread>
#include "lib/Robot.h"
#include "Constants.h"
#include "lib/pid.h"
#include "Settings.h"
#include "Publisher.h"

Control::Control() {
    forwardPID = new PID(1,0,0,0);
    sidewaysPID = new PID(0.1,0,0,0);
    rotationPID = new PID(1,0,0,0);
    lastBallSeenTime = std::chrono::steady_clock::now();
}


roboteam_msgs::RobotCommand Control::limitRobotCommand(roboteam_msgs::RobotCommand command) {
    if (command.x_vel > Settings::MAX_VEL) {
        command.x_vel = Settings::MAX_VEL;
    }
    if (command.x_vel < -Settings::MAX_VEL) {
        command.x_vel = -Settings::MAX_VEL;
    }

    if (command.y_vel > Settings::MAX_VEL) {
        command.y_vel = Settings::MAX_VEL;
    }

    if (command.y_vel < -Settings::MAX_VEL) {
        command.y_vel = -Settings::MAX_VEL;
    }

    if (command.w > Settings::MAX_VEL) {
        command.w = Settings::MAX_VEL;
    }
    if (command.w < -Settings::MAX_VEL) {
        command.w = -Settings::MAX_VEL;
    }
    command.w = 0; // TODO change this after Simen fixes angle stuff
    command.y_vel=0;
    return command;
}

roboteam_msgs::RobotCommand Control::takeBallGoBackwards(Publisher publisher){
    roboteam_msgs::RobotCommand command;
    command.x_vel = 0.5;
    command.y_vel = 00;
    command.dribbler = 26;
    publisher.command = command;
    publisher.skillpublishRobotCommand(*this);
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));

    roboteam_msgs::RobotCommand command2;
    command2.x_vel = 0;
    command2.y_vel = 0;
    command2.dribbler = 31;
    publisher.command = command2;
    publisher.skillpublishRobotCommand(*this);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    roboteam_msgs::RobotCommand command3;
    command3.x_vel = -0.3;
    command3.y_vel = 0;
    command3.dribbler = 31;
    publisher.command = command3;
    publisher.skillpublishRobotCommand(*this);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));


}

roboteam_msgs::RobotCommand Control::makeSimpleCommand(float x, float y, float angle) {
    if (x !=0) {robotXdist = x/100;} else {robotXdist = 0;}
    if (y !=0) {robotYdist = -y/100;} else {robotYdist = 0;}
    if (angle !=0) {robotAngle = -angle;} else {robotAngle = 0;}

    roboteam_msgs::RobotCommand command;
    command.use_angle = 1;
    command.id = Constants::ROBOT_ID;
    command.geneva_state = 3;
    command.x_vel = robotXdist;
    command.y_vel = robotYdist;
    command.w=robotAngle;

    return limitRobotCommand(command);



}


