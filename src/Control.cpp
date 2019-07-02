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

//    if (command.w > Settings::MAX_VEL) {
//        command.w = Settings::MAX_VEL;
//    }
//    if (command.w < -Settings::MAX_VEL) {
//        command.w = -Settings::MAX_VEL;
//    }
    command.w = 0; // TODO change this after Simen fixes angle stuff

    return command;
}

roboteam_msgs::RobotCommand Control::takeBallGoBackwards(Publisher publisher){
    roboteam_msgs::RobotCommand command;
    command.x_vel = 0.5;
    command.dribbler = 26;
    publisher.command = command;
    publisher.skillpublishRobotCommand(*this);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    roboteam_msgs::RobotCommand command2;
    command2.dribbler = 31;
    publisher.command = command2;
    publisher.skillpublishRobotCommand(*this);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    roboteam_msgs::RobotCommand command3;
    command3.x_vel = -0.3;
    command3.dribbler = 31;
    publisher.command = command3;
    publisher.skillpublishRobotCommand(*this);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));


}

roboteam_msgs::RobotCommand Control::makeSimpleCommand(float x, float y, float angle) {
    robotYdist = -x;
    robotXdist = -y;
    robotAngle = -angle;

    if (robotXdist < 0.5) {
        ballIsClose = true;
    } else {
        ballIsClose = false;
    }
}


