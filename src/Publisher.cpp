//
// Created by freek on 05/06/19.
//
#include <cmath>
#include <iostream>
#include <roboteam_msgs/RobotCommand.h>
#include <rosconsole/macros_generated.h>
#include <ros/node_handle.h>
#include <roboteam_msgs/RefereeData.h>
#include <mutex>
#include "src/lib/Robot.h"
#include "Control.h"
#include "Publisher.h"
#include "Settings.h"
#include "Constants.h"



Publisher::Publisher(Control inputControl){
    control = &inputControl;
    if (Settings::ENABLE_CONNECTION){
        robotCommandPublisher = nodeHandle.advertise<roboteam_msgs::RobotCommand>("robotcommands", 100);
        this->subscribeToRefereeData();

        this->refereeCommand=roboteam_msgs::RefereeCommand::STOP;
    }
};

void Publisher::subscribeToRefereeData() {
    this->refereeSubscriber = nodeHandle.subscribe<roboteam_msgs::RefereeData>(
            "vision_refbox", //vision_referee or vision_refbox
            100,
            &Publisher::handleRefereeData,
            this,
            ros::TransportHints().reliable().tcpNoDelay()
    );
}


void Publisher::handleRefereeData(const roboteam_msgs::RefereeDataConstPtr &refData) {
//    std::cout << "HANDLING REF DATA" << std::endl;
    this->refDataMsg = *refData;
    if (refDataMsg.command.command==roboteam_msgs::RefereeCommand::NORMAL_START || refDataMsg.command.command==roboteam_msgs::RefereeCommand::FORCE_START
    || refDataMsg.command.command==roboteam_msgs::RefereeCommand::STOP || refDataMsg.command.command==roboteam_msgs::RefereeCommand::HALT) {
        this->refereeCommand = refDataMsg.command.command;

    }
}


void Publisher::refreshRobotCommand() {
    roboteam_msgs::RobotCommand emptyCmd;
    emptyCmd.use_angle = 1;
    emptyCmd.id = Constants::ROBOT_ID;
    emptyCmd.geneva_state = 3;
    command = emptyCmd;
}

void Publisher::ioManagerPublishRobotCommand() {

    if (! control->paused) {
        if (command.id >= 0 && command.id < 16) {
            robotCommandPublisher.publish(command); //
        }
    }
    else {
        ROS_ERROR("HALT!");
    }
//    refreshRobotCommand(); // TODO fix this later
}

void Publisher::skillpublishRobotCommand(Control control) { // this one calls the iomanager one
    std::cout << " sending x vel: " << command.x_vel << std::endl;
    ros::NodeHandle nh;
    std::string ourSideParam;
    nh.getParam("our_side", ourSideParam);

//    control.limitRobotCommand();

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