////
//// Created by freek on 21/05/19.
////
//
//#include "Camera.h"
//#include "WorldState.h"
//#include <string>
//#include <signal.h>
//#include <chrono>
//#include <memory>
//#include "ros/ros.h"
//#include "std_srvs/Empty.h"
//#include "roboteam_msgs/DetectionFrame.h"
//#include "roboteam_msgs/GeometryData.h"
//#include "roboteam_msgs/RefereeData.h"
//#include "roboteam_msgs/GameEvent.h"
//
//#include "lib/robocup_ssl_client.h"
//#include "roboteam_utils/messages_robocup_ssl_wrapper.pb.h"
//#include "roboteam_utils/messages_robocup_ssl_wrapper_legacy.pb.h"
//
//#include "roboteam_utils/normalize.h"
//#include "roboteam_utils/constants.h"
//#include "ImageProcessor.h"
//#include "BallFinder.h"
//#include "roboteam_utils/Vector2.h"
//#include "roboteam_vision/convert/convert_referee.h"
//#include "roboteam_vision/roboteam_vision.h"
//#include "roboteam_vision/transform.h"
//
//
//
//
////namespace rtt {
//
///**
// * Converts a protoBuf DetectionFrame to the ROS version.
// */
//
//
//
//
///**
// * Converts a protoBuf DetectionRobot to the ROS version.
// */
//
//
//
//
//
//
////}
//
//class Publisher {
//
//    const int DEFAULT_VISION_PORT = 10006;
//    const int DEFAULT_REFEREE_PORT = 10003;
//
//    const std::string SSL_VISION_SOURCE_IP = "224.5.23.2";
//    const std::string SSL_REFEREE_SOURCE_IP= "224.5.23.1";
//
//    const std::string LOCAL_SOURCE_IP = "127.0.0.1";
//
//    std::string VISION_SOURCE_IP  = LOCAL_SOURCE_IP;
//    std::string REFEREE_SOURCE_IP = LOCAL_SOURCE_IP;
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
//    boost::optional<roboteam_msgs::RefereeData> latestReferee;
//    boost::optional<roboteam_msgs::GeometryData> latestGeom;
//    boost::optional<roboteam_msgs::DetectionFrame> latestFrame;
////
//    boost::optional<int> lastKnownVisionPort, lastKnownRefereePort;
////
//
//
//
//// Sends a SSL_DetectionFrame out through the supplied publisher.
//    void update_parameters_from_ros() {
//
//        if (rtt::has_PARAM_NORMALIZE_FIELD()) {
//            rtt::get_PARAM_NORMALIZE_FIELD(normalize_field);
//        } else {
//            rtt::set_PARAM_NORMALIZE_FIELD(true);
//        }
//
//        // Check the ROS parameters for which side we are on
//        if (rtt::has_PARAM_OUR_SIDE()) {
//            std::string our_side;
//            rtt::get_PARAM_OUR_SIDE(our_side);
//
//            if (our_side == "left") {
//                if(!our_side_is_left) ROS_INFO_STREAM("Changed side to left");
//                our_side_is_left = true;
//            } else if (our_side == "right") {
//                if(our_side_is_left) ROS_INFO_STREAM("Changed side to right");
//                our_side_is_left = false;
//            } else {
//                ROS_WARN_STREAM("Parameter for side invalid : '" << our_side << "' ! Defaulting to left");
//                rtt::set_PARAM_OUR_SIDE("left");
//            }
//        } else {
//            ROS_WARN_STREAM("Parameter for side not set! Defaulting to left");
//            rtt::set_PARAM_OUR_SIDE("left");
//        }
//
//        // Check the ROS parameters for which color we are
//        if (rtt::has_PARAM_OUR_COLOR()) {
//            std::string our_color;
//            rtt::get_PARAM_OUR_COLOR(our_color);
//
//            if (our_color == "yellow") {
//                if(!us_is_yellow) ROS_INFO_STREAM("Changed color to yellow");
//                us_is_yellow = true;
//            } else if (our_color == "blue") {
//                if(us_is_yellow) ROS_INFO_STREAM("Changed color to blue");
//                us_is_yellow = false;
//            } else {
//                ROS_WARN_STREAM("Parameter for color invalid : '" << our_color << "' ! Defaulting to yellow");
//                rtt::set_PARAM_OUR_COLOR("yellow");
//            }
//        } else {
//            ROS_WARN_STREAM("Parameter for side not set! Defaulting to yellow");
//            rtt::set_PARAM_OUR_COLOR("yellow");
//        }
//
//        // Check if we should use legacy packets
//        if (ros::param::has("use_legacy_packets")) {
//            bool _use_legacy_packets;
//            ros::param::get("use_legacy_packets", _use_legacy_packets);
//            if(_use_legacy_packets && !use_legacy_packets)
//                ROS_INFO_STREAM("Switching to using legacy packets");
//            use_legacy_packets = _use_legacy_packets;
//        } else {
//            ros::param::set("use_legacy_packets", false);
//        }
//
//
//        // ---- Transformation parameters ----
//
//        if (ros::param::has("transform_field/enabled")) {
//            ros::param::get("transform_field/enabled", transform_field);
//            ros::param::get("transform_field/rotate", transform_rotate_right_angle);
//
//            ros::param::get("transform_field/offset/top", transform_top);
//            ros::param::get("transform_field/offset/bottom", transform_bottom);
//            ros::param::get("transform_field/offset/left", transform_left);
//            ros::param::get("transform_field/offset/right", transform_right);
//
//            float new_width = field_size.x - (transform_left + transform_right);
//            float new_height = field_size.y - (transform_top + transform_bottom);
//
//            float relative_width = new_width / field_size.x;
//            float relative_height = new_height / field_size.y;
//
//            transform_move.x = (transform_right - transform_left) / 2;
//            transform_move.y = (transform_top - transform_bottom) / 2;
//
//            if (transform_rotate_right_angle) {
//                transform_scale.x = new_height / field_size.x;
//                transform_scale.y = new_width / field_size.y;
//            } else {
//                transform_scale.x = relative_width;
//                transform_scale.y = relative_height;
//            }
//        }
//
//        // ---- /Transformation parameters ----
//
//        // Vision
//        if (ros::param::has("vision_source_port")) {
//            int currentPort;
//            ros::param::get("vision_source_port", currentPort);
//            if (lastKnownVisionPort && currentPort != *lastKnownVisionPort) {
//                ROS_INFO_STREAM("vision port changed to " << currentPort);
//                // Vision port changed; reset the client
//                lastKnownVisionPort = currentPort;
//                if (vision_client) {
//                    vision_client->close();
//                    delete vision_client.release();
//                }
//                vision_client = std::make_unique<RoboCupSSLClient>(currentPort, VISION_SOURCE_IP);
//                vision_client->open(false);
//            } else if (!lastKnownVisionPort) {
//                lastKnownVisionPort = currentPort;
//            }
//        } else {
//            ros::param::set("vision_source_port", DEFAULT_VISION_PORT);
//        }
//
//        // Referee
//        if (ros::param::has("referee_source_port")) {
//            int currentPort;
//            ros::param::get("referee_source_port", currentPort);
//            if (lastKnownRefereePort && currentPort != *lastKnownRefereePort) {
//                ROS_INFO_STREAM("referee port changed to " << currentPort);
//                // Referee port changed; reset the client
//                lastKnownRefereePort = currentPort;
//                if (refbox_client) {
//                    refbox_client->close();
//                    delete refbox_client.release();
//                }
//                refbox_client = std::make_unique<RoboCupSSLClient>(currentPort, "224.5.23.1");
//                refbox_client->open(false);
//            } else if (!lastKnownRefereePort) {
//                lastKnownRefereePort = currentPort;
//            }
//        } else {
//            ros::param::set("referee_source_port", DEFAULT_REFEREE_PORT);
//        }
//    }
//
//
//
//    bool shuttingDown = false;
//
//    void sigIntHandler(int) {
//        shuttingDown = true;
//    }
//
//
//public:
//    Publisher(Camera camera, BallFinder ballFinder) { // constructor
//
//        std::vector<std::string> args(argv, argv + argc);
//
//        // Init ros
//        ros::init(argc, argv, "roboteam_msgs",
//                  ros::init_options::NoSigintHandler); // NoSigintHandler gives the ability to override ROS sigint handler
//        // TODO fix this sigIntHandler later
////    signal(SIGINT, sigIntHandler(SIGINT));
////    signal(SIGINT, sigIntHandler);
//        ros::NodeHandle n;
//
//        // Run at 80 hz.
//        int rate = 80;
//        ros::Rate loop_rate(rate);
//        ROS_INFO_STREAM("Running at " << rate << " Hz");
//
//        // Create the publishers.
//        // The `true` means that the messages will be latched.
//        // When a new node subscribes, it will automatically get the latest message of the topic,
//        // even if that was published a minute ago.
//        ros::Publisher detection_pub = n.advertise<roboteam_msgs::DetectionFrame>("vision_detection", 1000, true);
//        ros::Publisher geometry_pub = n.advertise<roboteam_msgs::GeometryData>("vision_geometry", 1000, true);
//        ros::Publisher refbox_pub = n.advertise<roboteam_msgs::RefereeData>("vision_refbox", 1000, true);
//
//        ROS_INFO("Publishing to 'vision_detection', 'vision_geometry', 'vision_refbox'");
//
//
//        int initialVisionPort = DEFAULT_VISION_PORT;
//        if (ros::param::has("vision_source_port")) {
//            ros::param::get("vision_source_port", initialVisionPort);
//        }
//
//        int initialRefereePort = DEFAULT_REFEREE_PORT;
//        if (ros::param::has("referee_source_port")) {
//            ros::param::get("referee_source_port", initialRefereePort);
//        }
//
//        // If Serial, assume real game. If Grsim, assume local testing
//        std::string robot_output_target = "grsim";
//        if (ros::param::has("robot_output_target")) {
//            ros::param::get("robot_output_target", robot_output_target);
//        } else {
//            ROS_WARN("parameter 'robot_output_target' not specified. Assuming grsim");
//        }
//
//        // Set ip addresses accordingly
//        if (robot_output_target == "serial") {
//            VISION_SOURCE_IP = SSL_VISION_SOURCE_IP;
//            REFEREE_SOURCE_IP = SSL_REFEREE_SOURCE_IP;
//        } else if (robot_output_target == "grsim") {
//            VISION_SOURCE_IP = LOCAL_SOURCE_IP;
//            REFEREE_SOURCE_IP = LOCAL_SOURCE_IP;
//        } else {
//            ROS_WARN_STREAM(
//                    "Parameter 'robot_output_target' has unknown value '" << robot_output_target
//                                                                          << "'! Assuming grsim");
//        }
//        ROS_INFO_STREAM("Parameters set for '" << robot_output_target << "'");
//
//
//        refbox_client = std::make_unique<RoboCupSSLClient>(initialRefereePort, REFEREE_SOURCE_IP);
//
//        ROS_INFO_STREAM("Referee : " << REFEREE_SOURCE_IP << ":" << initialRefereePort);
//
//        if (REFEREE_SOURCE_IP != SSL_REFEREE_SOURCE_IP)
//            ROS_WARN_STREAM(
//                    "Watch out! Current REFEREE_SOURCE_IP will not work for the competition! Remember to set it to "
//                            << SSL_REFEREE_SOURCE_IP);
//
//        // Open the clients, blocking = false.
//
//        refbox_client->open(false);
//
//        std::chrono::high_resolution_clock::time_point last_parameter_update_time = std::chrono::high_resolution_clock::now();
//
//        update_parameters_from_ros();
//
//        ROS_INFO("Vision ready");
//
//        SSL_WrapperPacket vision_packet;
//        RoboCup2014Legacy::Wrapper::SSL_WrapperPacket vision_packet_legacy;
//        SSL_Referee refbox_packet;
//
//        // int detectionPackets = 0;
//        // using namespace std::chrono;
//        // auto lastStatistics = std::chrono::high_resolution_clock::now();
//
//        int vision_packets_received = 0;
//        int referee_packets_received = 0;
//        int last_received_gameEvent = 0;
//
//    }
//    public void publisherLoop() {
//
//
//            publish_frame(camera, ballFinder, detection_pub);
//            // If the vision client is running
//            // If the referee client is running
//            if (refbox_client) {
//                while (refbox_client->receive(refbox_packet)) {
//                    bool shouldUpdateParameters = false;
//
//                    // Convert the referee data.
//                    roboteam_msgs::RefereeData data = rtt::convert_referee_data(refbox_packet, us_is_yellow);
//
//                    if (data.gameEvent.event != 0 && data.gameEvent.event != last_received_gameEvent) {
//                        last_received_gameEvent = data.gameEvent.event;
//                        ROS_INFO_STREAM(
//                                "Game event occured! event=" << gameEventToString(last_received_gameEvent).c_str()
//                                                             << ", message='" << data.gameEvent.message
//                                                             << "', team:bot=" << data.gameEvent.originator.team
//                                                             << ":" << data.gameEvent.originator.botId);
//                    }
//
//                    // === Check if our color has changed === //
//                    // If the name of our team is not "RoboTeam Twente", but the opponents team is, we are set to the wrong color
//                    if (data.us.name != ROBOTEAM_TWENTE && data.them.name == ROBOTEAM_TWENTE) {
//                        ROS_WARN_STREAM("The name of our team is not " << ROBOTEAM_TWENTE
//                                                                       << " ! The name of their team is! Switching colors..");
//                        // If we are yellow, switch to blue. If we are blue, switch to yellow.
//                        rtt::set_PARAM_OUR_COLOR(us_is_yellow ? "blue" : "yellow");
//                        shouldUpdateParameters = true;
//                    } else
//                        // If none of the teams are named "RoboTeam Twente"
//                    if (data.us.name != ROBOTEAM_TWENTE && data.them.name != ROBOTEAM_TWENTE) {
//                        ROS_DEBUG_STREAM_THROTTLE(1, "None of the teams are named " << ROBOTEAM_TWENTE);
//                    }
//
//                    // === Check if our side has changed === //
//                    // blueTeamOnPositiveHalf=true means "the blue team will have it's goal on the positive x-axis of the ssl-vision coordinate system" : https://github.com/RoboCup-SSL/ssl-refbox/blob/master/referee.proto
//                    if (our_side_is_left == (us_is_yellow ^ data.blueTeamOnPositiveHalf)) {
//                        ROS_WARN("We are playing on the wrong side! Switching sides..");
//                        // If we are left, switch to right. If we are right, switch to left
//                        rtt::set_PARAM_OUR_SIDE(our_side_is_left ? "right" : "left");
//                        shouldUpdateParameters = true;
//                    }
//
//                    // Update the ros_parameters if we changed color or sides
//                    if (shouldUpdateParameters) {
//                        update_parameters_from_ros();
//                    }
//
//                    if (transform_field) {
//                        rtt::transformRefereeData(data, transform_move, transform_rotate_right_angle);
//                    }
//
//                    if (normalize_field && !our_side_is_left) {
//                        data = rtt::normalizeRefereeData(data);
//                    }
//
//                    // Publish the data
//                    refbox_pub.publish(data);
//                    referee_packets_received++;
//                }
//            } else {
//                ROS_WARN("roboteam_vision: Referee Disconnected!");
//                if (latestReferee) {
//                    refbox_pub.publish(*latestReferee);
//                }
//            }
//
//            // ---- Param updates ----
//
//            auto time_now = std::chrono::high_resolution_clock::now();
//            auto time_diff = time_now - last_parameter_update_time;
//
//            // Every second...
//            int timeDifference = std::chrono::duration_cast<std::chrono::milliseconds>(time_diff).count();
//            if (timeDifference > 10000) {
//                last_parameter_update_time = time_now;
//
//                update_parameters_from_ros();
//
//                ROS_INFO("Vision  packets Hz : %.2f", vision_packets_received * (1000.0 / timeDifference));
//                ROS_INFO("Referee packets Hz : %.2f", referee_packets_received * (1000.0 / timeDifference));
//                vision_packets_received = 0;
//                referee_packets_received = 0;
//            }
//
//            // ---- /Param updates ----
//
//            ros::spinOnce();
//            loop_rate.sleep();
//        }
//
//        ROS_INFO("Shutting down vision..");
//        // Destructors do not call close properly. Just to be sure we do.
//        refbox_client->close();
//        // This is needed because we use our own SIGINT handler!
//        // See crashtest.cpp for details
//        ros::shutdown();
//
//        return 0;
//
//
//    };
//
//    roboteam_msgs::DetectionRobot generate_detection_robot() {
//        roboteam_msgs::DetectionRobot rosBot;
//
//        rosBot.confidence = 1;
//        rosBot.robot_id = 1;
//        rosBot.pos.x = 0; // TODO maybe make position 0,0 - ballPos
//        rosBot.pos.y = 0;
//        rosBot.orientation = 0;
//        rosBot.pixel_pos.x = 0;
//        rosBot.pixel_pos.y = 0;
//        rosBot.height = 0;
//
//        return rosBot;
//    }
//
///**
// * Converts a protoBuf DetectionBall to the ROS version.
// */
//    roboteam_msgs::DetectionBall generate_detection_ball(BallFinder ballFinder) {
//        roboteam_msgs::DetectionBall rosBall;
//
//        rosBall.confidence = 1;
//        rosBall.existence = 1; // TODO fix this
//        rosBall.pos.x = ballFinder.topDownBallMeanPoint.x;
//        rosBall.pos.y = ballFinder.topDownBallMeanPoint.y;
//        rosBall.z = 0;
//        rosBall.pixel_pos.x = ballFinder.topDownBallMeanPoint.x; // TODO Find out what to do with this
//        rosBall.pixel_pos.x = ballFinder.topDownBallMeanPoint.y;
//
//        return rosBall;
//    }
//
//    roboteam_msgs::DetectionFrame publish_frame(Camera camera, BallFinder ballFinder, ros::Publisher publisher) {
//        roboteam_msgs::DetectionFrame rosFrame;
//
//        long startFrameTimeValue = std::chrono::duration_cast<std::chrono::milliseconds>(
//                std::chrono::time_point_cast<std::chrono::milliseconds>(camera.startFrameTime).time_since_epoch()).count();
//        rosFrame.frame_number = camera.frameCounter;
//        rosFrame.t_capture = startFrameTimeValue;
//        rosFrame.t_sent = startFrameTimeValue;
//        rosFrame.camera_id = 0;
//
//        // std::cout << "Vision balls: " << protoFrame.balls().size() << "\n";
//        // std::cout << "Vision camera id: " << protoFrame.camera_id() << "\n";
//
//        roboteam_msgs::DetectionBall rosBall = generate_detection_ball(ballFinder);
//        rosFrame.balls.push_back(rosBall);
//
//
//        roboteam_msgs::DetectionRobot rosBot = generate_detection_robot();
//
//
//        rosFrame.us.push_back(rosBot);
//
//
//        publisher.publish(rosFrame);
//    }
//
//
//    //
//
//
//    //
//};
//
// // anonymous namespace
//
//
