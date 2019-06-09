//
// Created by freek on 06/06/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_CONTROL_H
#define TECHNICAL_CHALLENGE_2019_CONTROL_H


#include <src/lib/Robot.h>
#include <chrono>
#include <src/lib/pid.h>

class Control {
public:
    explicit Control();
    bool paused;
    float MAX_SPEED = 3;
    float previousAngle = 0;

    PID* forwardPID;





    std::chrono::steady_clock::time_point last_rotation_send_time;

    roboteam_msgs::RobotCommand makeSimpleCommand(float x, float y, float angle);

    float robotYdist;
    float robotXdist;
    float robotAngle;

    roboteam_msgs::RobotCommand limitRobotCommand(roboteam_msgs::RobotCommand command);
};


#endif //TECHNICAL_CHALLENGE_2019_CONTROL_H
