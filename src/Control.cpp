//
// Created by freek on 06/06/19.
//


#include <roboteam_msgs/RobotCommand.h>
#include <opencv2/core/types.hpp>
#include "Control.h"
#include "src/lib/Robot.h"
#include <roboteam_utils/Vector2.h>
#include <chrono>
#include "lib/Robot.h"
#include "Constants.h"
#include "lib/pid.h"
#include "Settings.h"

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

roboteam_msgs::RobotCommand Control::makeSimpleCommand(float x, float y, float angle) {
    robotYdist= -x;
    robotXdist= -y;
    robotAngle = -angle;



    roboteam_msgs::RobotCommand command;
    command.use_angle = 0;

    command.id = Constants::ROBOT_ID;
    if (angle > 0.13 || angle < -0.13) {
        // We must rotate
        command.x_vel = 0;
        command.y_vel = 0;
        command.w = robotAngle;




    } else {
        command.x_vel = forwardPID->getOutput(robotXdist);
        command.y_vel = 0;
        command.w = 0;


        // TODO check if command.y_vel=y is nice
    }
    command = limitRobotCommand(command);

    return command;
}


