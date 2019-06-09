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


roboteam_msgs::RobotCommand Control::limitRobotCommand(roboteam_msgs::RobotCommand command, rtt::ai::world::Robot* robot) {

    auto limitedVel = rtt::Vector2(command.x_vel, command.y_vel);
    float speed = sqrt(pow(limitedVel.x,2)+pow(limitedVel.y,2));
    if (speed > MAX_SPEED){
        limitedVel.x *= MAX_SPEED/speed;
        limitedVel.y *= MAX_SPEED/speed;
    }
    robot->setPidPreviousVel(limitedVel);

    command.x_vel = limitedVel.x;
    command.y_vel = limitedVel.y;
    return command;
}

roboteam_msgs::RobotCommand Control::makeSimpleCommand(float x, float y, float angle) {
    roboteam_msgs::RobotCommand command;
    if (angle < 2 || angle > -2) {
        auto time_now = std::chrono::steady_clock::now();
        auto time_diff = time_now - last_rotation_send_time;

        // Every second...
        int timeDifference = std::chrono::duration_cast<std::chrono::milliseconds>(time_diff).count();
        if (timeDifference > 1000) {
            // We must rotate
            last_rotation_send_time = std::chrono::steady_clock::now();
            float absoluteAngle = previousAngle + angle;
            previousAngle = absoluteAngle;
            command.x_vel = 0;
            command.y_vel = 0;
            command.id = Constants::ROBOT_ID;
            command.w = absoluteAngle;


        }

    } else {
        command.x_vel = x;

        // TODO check if command.y_vel=y is nice
    }
    return command;
}

roboteam_msgs::RobotCommand Control::makeCommand(float x, float y, float angle, rtt::ai::world::Robot* robot) {

    roboteam_msgs::RobotCommand command;

    // get current robot angular velocity, normal velocity
    rtt::Angle &currentAngle = robot->angle;
//    robot->setDribblerState() // value between 1 and 30

    return command;
}

