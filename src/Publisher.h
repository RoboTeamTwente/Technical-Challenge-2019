//
// Created by freek on 05/06/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_PUBLISHER_H
#define TECHNICAL_CHALLENGE_2019_PUBLISHER_H

class Control;


#include <ros/publisher.h>
#include <roboteam_msgs/RobotCommand.h>
#include <src/lib/Robot.h>
#include <ros/node_handle.h>
#include "Control.h"



class Publisher{
public:
    explicit Publisher(Control inputControl);// inputControl);
    Control* control;


    ros::NodeHandle nodeHandle;
    ros::Publisher robotCommandPublisher;
    roboteam_msgs::RobotCommand command;

    rtt::ai::world::Robot* robot;

    void skillpublishRobotCommand(Control control);

    void ioManagerPublishRobotCommand();

    void refreshRobotCommand();

    void limitRobotCommand();

//    RobotCommand_()
//            : id(0)
//            , active(false)
//            , x_vel(0.0)
//            , y_vel(0.0)
//            , w(0.0)
//            , use_angle(false)
//            , dribbler(false)
//            , kicker(false)
//            , kicker_forced(false)
//            , kicker_vel(0.0)
//            , chipper(false)
//            , chipper_forced(false)
//            , chipper_vel(0.0)
//            , geneva_state(0)  {

};

#endif //TECHNICAL_CHALLENGE_2019_PUBLISHER_H
