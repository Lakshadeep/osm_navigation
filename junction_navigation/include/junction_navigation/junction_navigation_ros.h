#ifndef JUNCTION_NAVIGATION_ROS_H
#define JUNCTION_NAVIGATION_ROS_H

#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <junction_navigation/junction_navigation.h>
#include <junction_navigation/structs.h>

// ROS messages
#include <gateway_msgs/Gateways.h>
#include <std_msgs/Float32.h>
#include <junction_navigation/JunctionNavigationAction.h>

// ROS services
#include <heading_control/Switch.h>
#include <robot_distance_monitor/Reset.h>
#include <robot_heading_monitor/Reset.h>

class JunctionNavigationROS
{

public:
    // Constructor / destructor
    JunctionNavigationROS(ros::NodeHandle&);
    ~JunctionNavigationROS();
    void loadParameters();

    void run();

private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber gateway_detection_subscriber_;
    ros::Subscriber distance_monitor_subscriber_;
    ros::Subscriber heading_monitor_subscriber_;

    ros::Publisher desired_heading_publisher_;
    ros::Publisher desired_velocity_publisher_;

    ros::ServiceClient heading_control_switch_service_client_;
    ros::ServiceClient heading_monitor_reset_service_client_;
    ros::ServiceClient distance_monitor_reset_service_client_;

    actionlib::SimpleActionServer<junction_navigation::JunctionNavigationAction> junction_navigation_server_;

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
    std::string desired_velocity_topic_;

    std::string heading_control_switch_service_;
    std::string reset_distance_monitor_service_;
    std::string reset_heading_monitor_service_;

    int controller_frequency_;

    // junction navigation
    JunctionNavigation junction_navigation_;

    // data caching
    
    // 1. detected features 
    Gateways detected_gateways_;

    // 2. monitors
    double monitored_distance_;
    double monitored_heading_;

    // action server callbacks
    void junctionNavigationExecute(const junction_navigation::JunctionNavigationGoalConstPtr& goal);

    // ROS related helper functions
    void resetMonitors();
    void resetHeadingMonitor();
    void resetDistanceMonitor();
    void reset();
    void enableHeadingController();
    void disableHeadingController();
};

#endif
