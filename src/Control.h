//
// Created by freek on 06/06/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_CONTROL_H
#define TECHNICAL_CHALLENGE_2019_CONTROL_H


#include <src/lib/Robot.h>
#include <chrono>

class Control {
public:
    bool paused;
    void limitRobotCommand();
    float MAX_SPEED = 3;
    float previousAngle = 0;



    roboteam_msgs::RobotCommand limitRobotCommand(roboteam_msgs::RobotCommand command, rtt::ai::world::Robot *robot);




    std::chrono::steady_clock::time_point last_rotation_send_time;

    roboteam_msgs::RobotCommand makeSimpleCommand(float x, float y, float angle);

    float robotYdist;
    float robotXdist;
    float robotAngle;
};


#endif //TECHNICAL_CHALLENGE_2019_CONTROL_H
