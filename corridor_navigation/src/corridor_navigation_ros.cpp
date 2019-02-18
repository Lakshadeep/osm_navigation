#include "corridor_navigation/corridor_navigation_ros.h"

CorridorNavigationROS::CorridorNavigationROS(ros::NodeHandle& nh): nh_(nh)
{
    loadParameters(); 
    
    // subscribers
    gateway_detection_subscriber_ = nh_.subscribe(gateway_detection_topic_, 1, &CorridorNavigationROS::gatewayDetectionCallback, this);
    distance_monitor_subscriber_ = nh_.subscribe(distance_monitor_topic_, 1, &CorridorNavigationROS::distanceMonitorCallback, this);
    heading_monitor_subscriber_ = nh_.subscribe(heading_monitor_topic_, 1, &CorridorNavigationROS::headingMonitorCallback, this);
    desired_heading_subscriber_ = nh_.subscribe(desired_heading_topic_, 1, &CorridorNavigationROS::desiredHeadingCallback, this);

    // publishers
    nav2d_operator_publisher_ = nh_.advertise<nav2d_operator::cmd>(nav2d_operator_cmd_topic_, 1);

    // service clients
    heading_control_switch_service_client_ = nh_.serviceClient<heading_control::Switch>(heading_control_switch_);
    heading_monitor_reset_service_client_ = nh_.serviceClient<robot_heading_monitor::Reset>(reset_heading_monitor_service_);
    distance_monitor_reset_service_client_ = nh_.serviceClient<robot_distance_monitor::Reset>(reset_distance_monitor_service_);
}

CorridorNavigationROS::~CorridorNavigationROS()
{
}

void CorridorNavigationROS::run()
{
}

void CorridorNavigationROS::loadParameters()
{
    // detection
    std::string gateway_detection_topic;
    nh_.param<std::string>("gateway_detection_topic", gateway_detection_topic, "/gateways_detected");
    gateway_detection_topic_ = gateway_detection_topic;
    ROS_DEBUG("gateway_detection_topic: %s", gateway_detection_topic_.c_str());

    // monitors
    std::string distance_monitor_topic;
    nh_.param<std::string>("distance_monitor_topic", distance_monitor_topic, "/monitored_distance");
    distance_monitor_topic_ = distance_monitor_topic;
    ROS_DEBUG("distance_monitor_topic: %s", distance_monitor_topic_.c_str());

    std::string heading_monitor_topic;
    nh_.param<std::string>("heading_monitor_topic", heading_monitor_topic, "/monitored_heading");
    heading_monitor_topic_ = heading_monitor_topic;
    ROS_DEBUG("heading_monitor_topic: %s", heading_monitor_topic_.c_str());

    std::string reset_distance_monitor_service;
    nh_.param<std::string>("reset_distance_monitor_service", reset_distance_monitor_service, "/reset_distance_monitor");
    reset_distance_monitor_service_ = reset_distance_monitor_service;
    ROS_DEBUG("reset_distance_monitor_service: %s", reset_distance_monitor_service_.c_str());

    std::string reset_heading_monitor_service;
    nh_.param<std::string>("reset_heading_monitor_service", reset_heading_monitor_service, "/reset_heading_monitor");
    reset_heading_monitor_service_ = reset_heading_monitor_service;
    ROS_DEBUG("reset_heading_monitor_service: %s", reset_heading_monitor_service_.c_str());

    // low level control
    std::string heading_control_switch;
    nh_.param<std::string>("heading_control_switch", heading_control_switch, "/heading_control_switch");
    heading_control_switch_ = heading_control_switch ;
    ROS_DEBUG("heading_control_switch service: %s", heading_control_switch.c_str());

    std::string desired_heading_topic;
    nh_.param<std::string>("desired_heading_topic", desired_heading_topic, "/desired_heading");
    desired_heading_topic_ = desired_heading_topic;
    ROS_DEBUG("desired_heading_topic: %s", desired_heading_topic_.c_str());

    // navigation operator
    std::string nav2d_operator_cmd_topic;
    nh_.param<std::string>("nav2d_operator_cmd_topic", nav2d_operator_cmd_topic, "/cmd");
    nav2d_operator_cmd_topic_ = nav2d_operator_cmd_topic;
    ROS_DEBUG("nav2d_operator_cmd_topic: %s", nav2d_operator_cmd_topic_.c_str());

    // corridor navigation params
    double recovery_direction_threshold;
    nh_.param<double>("recovery_direction_threshold", recovery_direction_threshold, 0.5);
    corridor_navigation_.setRecoveryDirectionThreshold(recovery_direction_threshold);
    ROS_DEBUG("recovery_direction_threshold: %f", recovery_direction_threshold);

    double correction_direction_threshold;
    nh_.param<double>("correction_direction_threshold", correction_direction_threshold, 0.06);
    corridor_navigation_.setCorrectionDirectionThreshold(correction_direction_threshold);
    ROS_DEBUG("correction_direction_threshold: %f", correction_direction_threshold);
}

void CorridorNavigationROS::gatewayDetectionCallback(const gateway_msgs::Gateways::ConstPtr& msg)
{

}

void CorridorNavigationROS::distanceMonitorCallback(const std_msgs::Float32::ConstPtr& msg)
{

}

void CorridorNavigationROS::headingMonitorCallback(const std_msgs::Float32::ConstPtr& msg)
{

}

void CorridorNavigationROS::desiredHeadingCallback(const std_msgs::Float32::ConstPtr& msg)
{

}




