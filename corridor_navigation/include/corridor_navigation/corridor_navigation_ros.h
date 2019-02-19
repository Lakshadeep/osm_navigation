#ifndef CORRIDOR_NAVIGATION_ROS_H
#define CORRIDOR_NAVIGATION_ROS_H

#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <corridor_navigation/corridor_navigation.h>
#include <corridor_navigation/structs.h>

// ROS messages
#include <gateway_msgs/gateways.h>
#include <std_msgs/Float32.h>
#include <nav2d_operator/cmd.h>
#include <corridor_navigation_msgs/CorridorNavigationAction.h>

// ROS services
#include <heading_control/Switch.h>
#include <robot_distance_monitor/Reset.h>
#include <robot_heading_monitor/Reset.h>



class CorridorNavigationROS
{

public:
    // Constructor / destructor
    CorridorNavigationROS(ros::NodeHandle&);
    ~CorridorNavigationROS();
    void loadParameters();

    // Running
    void run();

private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber gateway_detection_subscriber_;
    ros::Subscriber distance_monitor_subscriber_;
    ros::Subscriber heading_monitor_subscriber_;

    ros::Publisher nav2d_operator_publisher_;
    ros::Publisher desired_heading_publisher_;

    ros::ServiceClient heading_control_switch_service_client_;
    ros::ServiceClient heading_monitor_reset_service_client_;
    ros::ServiceClient distance_monitor_reset_service_client_;

    actionlib::SimpleActionServer<corridor_navigation_msgs::CorridorNavigationAction> corridor_navigation_server_;

    // subscriber callbacks
    void gatewayDetectionCallback(const gateway_msgs::Gateways::ConstPtr& msg);
    void distanceMonitorCallback(const std_msgs::Float32::ConstPtr& msg);
    void headingMonitorCallback(const std_msgs::Float32::ConstPtr& msg);
    void desiredHeadingCallback(const std_msgs::Float32::ConstPtr& msg);

    // ROS service, topic names
    std::string gateway_detection_topic_;
    std::string distance_monitor_topic_;
    std::string heading_monitor_topic_;
    std::string desired_heading_topic_;
    std::string nav2d_operator_cmd_topic_;

    std::string heading_control_switch_service_;
    std::string reset_distance_monitor_service_;
    std::string reset_heading_monitor_service_;

    int controller_frequency_;

    // corridor navigation
    CorridorNavigation corridor_navigation_;

    // data caching
    
    // 1. detected features 
    Gateways detected_gateways_;

    // 2. monitors
    double monitored_distance_;
    double monitored_heading_;

    // action server callbacks
    void CorridorNavigationExecute(const corridor_navigation_msgs::CorridorNavigationGoalConstPtr& goal);

    // reset
    void resetMonitors();
    void reset();
};

#endif
