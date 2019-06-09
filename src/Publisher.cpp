//
// Created by freek on 05/06/19.
//

#include "Publisher.h"
#include <cmath>
#include <iostream>
#include <roboteam_msgs/RobotCommand.h>
#include <rosconsole/macros_generated.h>
#include <ros/node_handle.h>
#include "src/lib/Robot.h"
#include "Control.h"

Publisher::Publisher(Control inputControl){
    control = &inputControl;
};





void Publisher::refreshRobotCommand() {
    roboteam_msgs::RobotCommand emptyCmd;
    emptyCmd.use_angle = 1;
    emptyCmd.id = robot ? robot->id : -1;
    emptyCmd.geneva_state = 3;
    command = emptyCmd;
}

void Publisher::ioManagerPublishRobotCommand() {

    if (! control->paused) {
        if (true) { // TODO remove if?

            // the geneva cannot be received from world, so we set it when it gets sent.

            if (robot) {
                robot->setGenevaState(command.geneva_state);
                robot->setDribblerState(command.dribbler);
            }
            // sometimes trees are terminated without having a role assigned.
            // It is then possible that a skill gets terminated with an empty robot: and then the id can be for example -1.
            if (command.id >= 0 && command.id < 16) {
                robotCommandPublisher.publish(command); //
            }
        }
        else {
            ROS_ERROR("Joystick demo has the robot taken over ID:   %s", std::to_string(command.id).c_str());
        }
    }
    else {
        ROS_ERROR("HALT!");
    }
    refreshRobotCommand();
}

void Publisher::skillpublishRobotCommand() { // this one calls the iomanager one

    ros::NodeHandle nh;
    std::string ourSideParam;
    nh.getParam("our_side", ourSideParam);

    limitRobotCommand();

    if (std::isnan(command.x_vel) || std::isnan(command.y_vel)) {
        std::cout << "ERROR: x or y vel in command is NAN in Skill " << "technical challenge" << "!" << std::endl;
    }
    if (command.id == -1) {
        if (robot && robot->id != -1) {
            command.id = robot->id;
            ioManagerPublishRobotCommand(); // We default to our robots being on the left if parameter is not set
        }
    } else {
        ioManagerPublishRobotCommand(); // We default to our robots being on the left if parameter is not set
    }
    // refresh the robotcommand after it has been sent

}