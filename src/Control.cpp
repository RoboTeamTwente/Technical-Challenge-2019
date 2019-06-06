//
// Created by freek on 06/06/19.
//

#include <roboteam_msgs/RobotCommand.h>
#include "Control.h"

roboteam_msgs::RobotCommand Control::limitRobotCommand(roboteam_msgs::RobotCommand command) {
    auto limitedVel = Vector2(command.x_vel, command.y_vel);
    limitedVel = control::ControlUtils::velocityLimiter(limitedVel);
    limitedVel = control::ControlUtils::accelerationLimiter(limitedVel, robot->getPidPreviousVel(), command.w);
    robot->setPidPreviousVel(limitedVel);

    command.x_vel = limitedVel.x;
    command.y_vel = limitedVel.y;
    return command;
}