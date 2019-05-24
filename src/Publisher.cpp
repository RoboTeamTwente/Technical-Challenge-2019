//
// Created by freek on 21/05/19.
//

#include "Publisher.h"
#include <string>
#include <signal.h>
#include <chrono>
#include <memory>
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/GeometryData.h"
#include "roboteam_msgs/RefereeData.h"
#include "roboteam_msgs/GameEvent.h"

//#include "net/robocup_ssl_client.h"
#include "roboteam_utils/messages_robocup_ssl_wrapper.pb.h"
#include "roboteam_utils/messages_robocup_ssl_wrapper_legacy.pb.h"

#include "roboteam_utils/normalize.h"
#include "roboteam_utils/constants.h"
//#include "roboteam_utils/Vector2.h


namespace rtt {

    /**
     * Converts a protoBuf DetectionFrame to the ROS version.
     */
    roboteam_msgs::DetectionFrame generate_detection_frame() {
        roboteam_msgs::DetectionFrame rosFrame;


        rosFrame.frame_number = protoFrame.frame_number();
        rosFrame.t_capture = protoFrame.t_capture();
        rosFrame.t_sent = protoFrame.t_sent();
        rosFrame.camera_id = protoFrame.camera_id();

        // std::cout << "Vision balls: " << protoFrame.balls().size() << "\n";
        // std::cout << "Vision camera id: " << protoFrame.camera_id() << "\n";

        for (int i = 0; i < protoFrame.balls().size(); ++i) {
            SSL_DetectionBall protoBall = protoFrame.balls().Get(i);
            roboteam_msgs::DetectionBall rosBall = convert_detection_ball(protoBall);
            rosFrame.balls.push_back(rosBall);
        }

        for (int i = 0; i < protoFrame.robots_yellow().size(); ++i) {
            SSL_DetectionRobot protoBot = protoFrame.robots_yellow().Get(i);
            roboteam_msgs::DetectionRobot rosBot = convert_detection_robot(protoBot);

            if (us_is_yellow) {
                rosFrame.us.push_back(rosBot);
            } else {
                rosFrame.them.push_back(rosBot);
            }
        }

        for (int i = 0; i < protoFrame.robots_blue().size(); ++i) {
            SSL_DetectionRobot protoBot = protoFrame.robots_blue().Get(i);
            roboteam_msgs::DetectionRobot rosBot = convert_detection_robot(protoBot);

            if (us_is_yellow) {
                rosFrame.them.push_back(rosBot);
            } else {
                rosFrame.us.push_back(rosBot);
            }
        }

        return rosFrame;
    }


    /**
     * Converts a protoBuf DetectionBall to the ROS version.
     */
    roboteam_msgs::DetectionBall generate_detection_ball (SSL_DetectionBall protoBall) {
        roboteam_msgs::DetectionBall rosBall;

        rosBall.confidence = protoBall.confidence();
        rosBall.existence = protoBall.area();
        rosBall.pos.x = mm_to_m(protoBall.x());
        rosBall.pos.y = mm_to_m(protoBall.y());
        rosBall.z = mm_to_m(protoBall.z());
        rosBall.pixel_pos.x = protoBall.pixel_x();
        rosBall.pixel_pos.x = protoBall.pixel_y();

        return rosBall;
    }


    /**
     * Converts a protoBuf DetectionRobot to the ROS version.
     */
    roboteam_msgs::DetectionRobot generate_robot(SSL_DetectionRobot protoBot) {
        roboteam_msgs::DetectionRobot rosBot;

        rosBot.confidence = protoBot.confidence();
        rosBot.robot_id = protoBot.robot_id();
        rosBot.pos.x = mm_to_m(protoBot.x());
        rosBot.pos.y = mm_to_m(protoBot.y());
        rosBot.orientation = protoBot.orientation();
        rosBot.pixel_pos.x = protoBot.pixel_x();
        rosBot.pixel_pos.y = protoBot.pixel_y();
        rosBot.height = mm_to_m(protoBot.height());

        return rosBot;
    }

}

class Publisher {


////    constexpr int DEFAULT_VISION_PORT = 10006;
////    constexpr int DEFAULT_REFEREE_PORT = 10003;
////
////    const std::string SSL_VISION_SOURCE_IP = "224.5.23.2";
////    const std::string SSL_REFEREE_SOURCE_IP = "224.5.23.1";
////
////    const std::string LOCAL_SOURCE_IP = "127.0.0.1";
////
////    std::string VISION_SOURCE_IP = LOCAL_SOURCE_IP;
////    std::string REFEREE_SOURCE_IP = LOCAL_SOURCE_IP;
//
//// const std::string VISION_SOURCE_IP = SSL_VISION_SOURCE_IP;
//// const std::string REFEREE_SOURCE_IP = SSL_REFEREE_SOURCE_IP;
//
//// Our name as specified by ssl-refbox : https://github.com/RoboCup-SSL/ssl-refbox/blob/master/referee.conf
//    const std::string ROBOTEAM_TWENTE = "RoboTeam Twente";
//
//// Keeps track of which team is us.
//// True is yellow, false is blue.
//    bool us_is_yellow = true;
//// Wether to use the legacy SSL protobuf packets.
//    bool use_legacy_packets = false;
//    bool our_side_is_left = true;
//    bool normalize_field = true;
//
//// Field size used for calculating the transformation scale.
//// Is updated when a GeometryData packet arrives.
//    rtt::Vector2 field_size = rtt::Vector2(9, 12);  // Does this thing even do anything?
//
//    bool transform_field = false;
//    rtt::Vector2 transform_move = rtt::Vector2(0, 0);
//    rtt::Vector2 transform_scale = rtt::Vector2(1, 1);
//    float transform_top = 0;
//    float transform_bottom = 0;
//    float transform_left = 0;
//    float transform_right = 0;
//
//    bool transform_rotate_right_angle = false;
//
//    std::unique_ptr<RoboCupSSLClient> vision_client;
//    std::unique_ptr<RoboCupSSLClient> refbox_client;
//    boost::optional <roboteam_msgs::RefereeData> latestReferee;
//    boost::optional <roboteam_msgs::GeometryData> latestGeom;
//    boost::optional <roboteam_msgs::DetectionFrame> latestFrame;
//
//    boost::optional<int> lastKnownVisionPort, lastKnownRefereePort;
//
//namespace {
//
//    bool shuttingDown = false;
//
//    void sigIntHandler(int) {
//        shuttingDown = true;
//    }
//
//} // anonymous namespace

}
// Sends a SSL_DetectionFrame out through the supplied publisher.
void send_detection_frame(SSL_DetectionFrame detectionFrame, ros::Publisher publisher, bool normalize_field) {
    uint cam_id = detectionFrame.camera_id();

    // TODO: See if it is necessary to remove duplicate frames.
    // Are they even duplicate?

    // Convert the detection frame.
    roboteam_msgs::DetectionFrame frame = rtt::convert_detection_frame(detectionFrame, us_is_yellow);

    if (transform_field) {
        rtt::dropObjectsOutsideTransform(
                frame,
                field_size,
                transform_top,
                transform_right,
                transform_bottom,
                transform_left
        );

        rtt::transformDetectionFrame(frame, transform_move, transform_rotate_right_angle);
    }

    if (normalize_field && !our_side_is_left) {
        frame = rtt::normalizeDetectionFrame(frame);
    }

    // Publish the frame.
    publisher.publish(frame);
}

int main(int argc, char **argv) {

    std::vector<std::string> args(argv, argv + argc);

    // Init ros
    ros::init(argc, argv, "roboteam_msgs",
              ros::init_options::NoSigintHandler); // NoSigintHandler gives the ability to override ROS sigint handler
    // Flower power!
    signal(SIGINT, sigIntHandler);
    ros::NodeHandle n;

    // Run at 80 hz.
    int rate = 80;
    ros::Rate loop_rate(rate);
    ROS_INFO_STREAM("Running at " << rate << " Hz");

    // Create the publishers.
    // The `true` means that the messages will be latched.
    // When a new node subscribes, it will automatically get the latest message of the topic,
    // even if that was published a minute ago.
    ros::Publisher detection_pub = n.advertise<roboteam_msgs::DetectionFrame>("vision_detection", 1000, true);
    ros::Publisher geometry_pub = n.advertise<roboteam_msgs::GeometryData>("vision_geometry", 1000, true);
    ros::Publisher refbox_pub = n.advertise<roboteam_msgs::RefereeData>("vision_refbox", 1000, true);

    ROS_INFO("Publishing to 'vision_detection', 'vision_geometry', 'vision_refbox'");

    int initialVisionPort = DEFAULT_VISION_PORT;
    if (ros::param::has("vision_source_port")) {
        ros::param::get("vision_source_port", initialVisionPort);
    }

    int initialRefereePort = DEFAULT_REFEREE_PORT;
    if (ros::param::has("referee_source_port")) {
        ros::param::get("referee_source_port", initialRefereePort);
    }

    // If Serial, assume real game. If Grsim, assume local testing
    std::string robot_output_target = "grsim";
    if (ros::param::has("robot_output_target")) {
        ros::param::get("robot_output_target", robot_output_target);
    } else {
        ROS_WARN("parameter 'robot_output_target' not specified. Assuming grsim");
    }

    // Set ip addresses accordingly
    if (robot_output_target == "serial") {
        VISION_SOURCE_IP = SSL_VISION_SOURCE_IP;
        REFEREE_SOURCE_IP = SSL_REFEREE_SOURCE_IP;
    } else if (robot_output_target == "grsim") {
        VISION_SOURCE_IP = LOCAL_SOURCE_IP;
        REFEREE_SOURCE_IP = LOCAL_SOURCE_IP;
    } else {
        ROS_WARN_STREAM(
                "Parameter 'robot_output_target' has unknown value '" << robot_output_target << "'! Assuming grsim");
    }
    ROS_INFO_STREAM("Parameters set for '" << robot_output_target << "'");

    vision_client = std::make_unique<RoboCupSSLClient>(initialVisionPort, VISION_SOURCE_IP);
    refbox_client = std::make_unique<RoboCupSSLClient>(initialRefereePort, REFEREE_SOURCE_IP);

    ROS_INFO_STREAM("Vision  : " << VISION_SOURCE_IP << ":" << initialVisionPort);
    ROS_INFO_STREAM("Referee : " << REFEREE_SOURCE_IP << ":" << initialRefereePort);

    if (VISION_SOURCE_IP != SSL_VISION_SOURCE_IP)
        ROS_WARN_STREAM("Watch out! Current VISION_SOURCE_IP will not work for the competition! Remember to set it to "
                                << SSL_VISION_SOURCE_IP);
    if (REFEREE_SOURCE_IP != SSL_REFEREE_SOURCE_IP)
        ROS_WARN_STREAM("Watch out! Current REFEREE_SOURCE_IP will not work for the competition! Remember to set it to "
                                << SSL_REFEREE_SOURCE_IP);

    // Open the clients, blocking = false.
    vision_client->open(false);
    refbox_client->open(false);

    std::chrono::high_resolution_clock::time_point last_parameter_update_time = std::chrono::high_resolution_clock::now();

    update_parameters_from_ros();

    ROS_INFO("Vision ready");

    SSL_WrapperPacket vision_packet;
    RoboCup2014Legacy::Wrapper::SSL_WrapperPacket vision_packet_legacy;
    SSL_Referee refbox_packet;

    // int detectionPackets = 0;
    // using namespace std::chrono;
    // auto lastStatistics = std::chrono::high_resolution_clock::now();

    int vision_packets_received = 0;
    int referee_packets_received = 0;
    int last_received_gameEvent = 0;

    while (ros::ok() && !shuttingDown) {
        // If the vision client is running
        if (vision_client) {
            // Receive current version packets.
            while (vision_client->receive(vision_packet)) {

                // Detection frame.
                if (vision_packet.has_detection()) {
                    send_detection_frame(vision_packet.detection(), detection_pub, normalize_field);
                    vision_packets_received++;
                }

                if (vision_packet.has_geometry()) {
                    // Convert the geometry frame.
                    roboteam_msgs::GeometryData data = rtt::convert_geometry_data(vision_packet.geometry());

                    field_size.x = data.field.field_length;
                    field_size.y = data.field.field_width;

                    if (transform_field) {
                        rtt::scaleGeometryData(data, transform_scale);
                    }

                    // Publish the data.
                    geometry_pub.publish(data);
                }
            }
        } else {
            ROS_ERROR("roboteam_vision: Vision disconnected!");
            if (latestFrame) {
                detection_pub.publish(*latestFrame);
            }
            if (latestGeom) {
                geometry_pub.publish(*latestGeom);
            }
        }

        // If the referee client is running
        if (refbox_client) {
            while (refbox_client->receive(refbox_packet)) {
                bool shouldUpdateParameters = false;

                // Convert the referee data.
                roboteam_msgs::RefereeData data = rtt::convert_referee_data(refbox_packet, us_is_yellow);

                if (data.gameEvent.event != 0 && data.gameEvent.event != last_received_gameEvent) {
                    last_received_gameEvent = data.gameEvent.event;
                    ROS_INFO_STREAM("Game event occured! event=" << gameEventToString(last_received_gameEvent).c_str()
                                                                 << ", message='" << data.gameEvent.message
                                                                 << "', team:bot=" << data.gameEvent.originator.team
                                                                 << ":" << data.gameEvent.originator.botId);
                }

                // === Check if our color has changed === //
                // If the name of our team is not "RoboTeam Twente", but the opponents team is, we are set to the wrong color
                if (data.us.name != ROBOTEAM_TWENTE && data.them.name == ROBOTEAM_TWENTE) {
                    ROS_WARN_STREAM("The name of our team is not " << ROBOTEAM_TWENTE
                                                                   << " ! The name of their team is! Switching colors..");
                    // If we are yellow, switch to blue. If we are blue, switch to yellow.
                    rtt::set_PARAM_OUR_COLOR(us_is_yellow ? "blue" : "yellow");
                    shouldUpdateParameters = true;
                } else
                    // If none of the teams are named "RoboTeam Twente"
                if (data.us.name != ROBOTEAM_TWENTE && data.them.name != ROBOTEAM_TWENTE) {
                    ROS_DEBUG_STREAM_THROTTLE(1, "None of the teams are named " << ROBOTEAM_TWENTE);
                }

                // === Check if our side has changed === //
                // blueTeamOnPositiveHalf=true means "the blue team will have it's goal on the positive x-axis of the ssl-vision coordinate system" : https://github.com/RoboCup-SSL/ssl-refbox/blob/master/referee.proto
                if (our_side_is_left == (us_is_yellow ^ data.blueTeamOnPositiveHalf)) {
                    ROS_WARN("We are playing on the wrong side! Switching sides..");
                    // If we are left, switch to right. If we are right, switch to left
                    rtt::set_PARAM_OUR_SIDE(our_side_is_left ? "right" : "left");
                    shouldUpdateParameters = true;
                }

                // Update the ros_parameters if we changed color or sides
                if (shouldUpdateParameters) {
                    update_parameters_from_ros();
                }

                if (transform_field) {
                    rtt::transformRefereeData(data, transform_move, transform_rotate_right_angle);
                }

                if (normalize_field && !our_side_is_left) {
                    data = rtt::normalizeRefereeData(data);
                }

                // Publish the data
                refbox_pub.publish(data);
                referee_packets_received++;
            }
        } else {
            ROS_WARN("roboteam_vision: Referee Disconnected!");
            if (latestReferee) {
                refbox_pub.publish(*latestReferee);
            }
        }

        // ---- Param updates ----

        auto time_now = std::chrono::high_resolution_clock::now();
        auto time_diff = time_now - last_parameter_update_time;

        // Every second...
        int timeDifference = std::chrono::duration_cast<std::chrono::milliseconds>(time_diff).count();
        if (timeDifference > 10000) {
            last_parameter_update_time = time_now;

            update_parameters_from_ros();

            ROS_INFO("Vision  packets Hz : %.2f", vision_packets_received * (1000.0 / timeDifference));
            ROS_INFO("Referee packets Hz : %.2f", referee_packets_received * (1000.0 / timeDifference));
            vision_packets_received = 0;
            referee_packets_received = 0;
        }

        // ---- /Param updates ----

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Shutting down vision..");
    // Destructors do not call close properly. Just to be sure we do.
    vision_client->close();
    refbox_client->close();

    // This is needed because we use our own SIGINT handler!
    // See crashtest.cpp for details
    ros::shutdown();

    return 0;
}

};