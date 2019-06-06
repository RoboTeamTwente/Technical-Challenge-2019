//
// Created by freek on 05/06/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_PUBLISHER_H
#define TECHNICAL_CHALLENGE_2019_PUBLISHER_H

#include <ros/publisher.h>
#include <roboteam_msgs/RobotCommand.h>

class Publisher{
    ros::Publisher robotCommandPublisher;
    roboteam_msgs::RobotCommand command;

    void SkillpublishRobotCommand();

    void ioManagerPublishRobotCommand();

    void refreshRobotCommand();

    void limitRobotCommand();
};

#endif //TECHNICAL_CHALLENGE_2019_PUBLISHER_H
