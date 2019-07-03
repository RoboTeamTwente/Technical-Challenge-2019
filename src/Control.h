//
// Created by freek on 06/06/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_CONTROL_H
#define TECHNICAL_CHALLENGE_2019_CONTROL_H


#include <src/lib/Robot.h>
#include <chrono>
#include <src/lib/pid.h>

class Publisher;

#include "Publisher.h"

class Control {
public:
    explicit Control();
    bool paused;
    bool ballIsClose = false;
    float MAX_SPEED = 3;
    float previousAngle = 0;

    PID* forwardPID;
    PID* sidewaysPID;
    PID* rotationPID;





    std::chrono::steady_clock::time_point last_rotation_send_time;

    roboteam_msgs::RobotCommand makeSimpleCommand(float x, float y, float angle);

    float robotYdist;
    float robotXdist;
    float robotAngle;

    roboteam_msgs::RobotCommand limitRobotCommand(roboteam_msgs::RobotCommand command);

    bool sentZero;
    std::chrono::steady_clock::time_point lastBallSeenTime;
    roboteam_msgs::RobotCommand takeBallGoBackwards(Publisher publisher);

    bool sentRotation;
};


#endif //TECHNICAL_CHALLENGE_2019_CONTROL_H
