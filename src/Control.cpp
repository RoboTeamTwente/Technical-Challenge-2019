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




roboteam_msgs::RobotCommand Control::makeSimpleCommand(float x, float y, float angle) {
    robotYdist= -x;
    robotXdist= -y;
    robotAngle = -angle;


    roboteam_msgs::RobotCommand command;
    command.id = Constants::ROBOT_ID;
    if (angle > 2 || angle < -2) {
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
    return command;
}

