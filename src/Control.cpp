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


roboteam_msgs::RobotCommand Control::limitRobotCommand(roboteam_msgs::RobotCommand command) {
    if (command.x_vel > 0.5) {
        command.x_vel = 0.5;
    }
    if (command.y_vel > 0.5) {
        command.y_vel = 0.5;
    }
    return command;
}

roboteam_msgs::RobotCommand Control::makeSimpleCommand(float x, float y, float angle) {
    robotYdist= -x;
    robotXdist= -y;
    robotAngle = -angle;


    roboteam_msgs::RobotCommand command;
    command.id = Constants::ROBOT_ID;
    if (angle > 0.13 || angle < -0.13) {
        // We must rotate
        command.x_vel = 0;
        command.y_vel = 0;
        command.w = robotAngle;




    } else {
        command.x_vel = robotXdist;
        command.y_vel = 0;
        command.w = 0;


        // TODO check if command.y_vel=y is nice
    }
    command = limitRobotCommand(command);
    return command;
}

