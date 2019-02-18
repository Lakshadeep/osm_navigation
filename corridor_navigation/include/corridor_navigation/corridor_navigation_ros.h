#ifndef CORRIDOR_NAVIGATION_ROS_H
#define CORRIDOR_NAVIGATION_ROS_H

#include <string>
#include <ros/ros.h>
#include <corridor_navigation/corridor_navigation.h>

#include <gateway_msgs/gateways.h>
#include <std_msgs/Float32.h>
#include <nav2d_operator/cmd.h>

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
    ros::Subscriber desired_heading_subscriber_;

    ros::Publisher nav2d_operator_publisher_;

    ros::ServiceClient heading_control_switch_service_client_;
    ros::ServiceClient heading_monitor_reset_service_client_;
    ros::ServiceClient distance_monitor_reset_service_client_;

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

    std::string heading_control_switch_;
    std::string reset_distance_monitor_service_;
    std::string reset_heading_monitor_service_;

    // corridor navigation
    CorridorNavigation corridor_navigation_;


};

#endif
