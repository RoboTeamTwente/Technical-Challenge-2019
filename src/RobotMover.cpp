////
//// Created by freek on 18/02/19.
////
//
//#include <rosconsole/macros_generated.h>
//#include <roboteam_msgs/RobotCommand.h>
//#include <roboteam_msgs/robot.h>
//#include
//#include "RobotMover.h"
//#include <roboteam_utils/vector2.h>
//
//void sendMoveCommand2() {
//    if (! checkTargetPos(targetPos)) {
//        ROS_ERROR("Target position is not correct GoToPos");
//        return;
//    }
//
//    roboteam_msgs::RobotCommand command;
//    // TODO import robot
//    command.id = robot->id;
//    command.use_angle = 1;
//
//    command.w = static_cast<float>(deltaPos.angle());
//    Vector2 deltaPosUnit = deltaPos.normalize();
//
//    command.x_vel = static_cast<float>(deltaPosUnit.x*speed);// abs(angularVel)/(abs(angularVel)-1);
//    command.y_vel = static_cast<float>(deltaPosUnit.y*speed);
//    publishRobotCommand(command);
//    commandSend = true;
//}
//
///// Get an update on the skill
//bt::Node::Status GoToPos::onUpdate() {
//    roboteam_msgs::RobotCommand command;
//    //reset velocity and angle commands
//    command.x_vel = 0;
//    command.y_vel = 0;
//    command.w = 0;
//
//    Status gtpStatus = gtpUpdate();
//    if (gtpStatus != Status::Running) return gtpStatus;
//
//    // check targetPos
//    if ((targetPos - robot->pos).length2() < errorMargin*errorMargin) {
//        // check targetAngle
//        if (targetAngle == 0.0 || abs(targetAngle - robot->angle) < angleErrorMargin) {
//            return Status::Success;
//        }
//    }
//    if (command.x_vel == 0 || command.y_vel == 0 || command.w == 0) {
//        control::PosVelAngle pva = posController->getPosVelAngle(robot, targetPos, targetAngle);
//
//        // set robotcommands if they have not been set yet in gtpUpdate()
//        command.x_vel = command.x_vel == 0 ? pva.vel.x : command.x_vel;
//        command.y_vel = command.y_vel == 0 ? pva.vel.y : command.y_vel;
//        command.w = command.w == 0 ? static_cast<float>(pva.angle) : command.w;
//    }
//
//    publishRobotCommand();
//    return Status::Running;
//}
///// Check the progress the robot made a9nd alter the currentProgress
////GoToPos::Progression GoToPos::checkProgression() {
////    double maxMargin = 0.2;                        // max offset or something.
////    if (deltaPos.length() >= maxMargin) return ON_THE_WAY;
////    else return DONE;
//}
//// createRobotCommand
//// use robothub's packing::createLowLevelRobotCommand()
//// createRobotPacket
