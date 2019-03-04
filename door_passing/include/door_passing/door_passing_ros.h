#ifndef DOOR_PASSING_ROS_H
#define DOOR_PASSING_ROS_H

#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <door_passing/door_passing.h>
#include <door_passing/structs.h>

// ROS messages
#include <gateway_msgs/Gateways.h>
#include <std_msgs/Float32.h>
#include <door_passing/DoorPassingAction.h>

// ROS services
#include <heading_control/Switch.h>
#include <robot_distance_monitor/Reset.h>
#include <robot_heading_monitor/Reset.h>
#include <motion_control/Switch.h>
#include <motion_control/Params.h>
#include <motion_control/DriveMode.h>

class DoorPassingROS
{

public:
    // Constructor / destructor
    DoorPassingROS(ros::NodeHandle&);
    ~DoorPassingROS();
    
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

    ros::ServiceClient motion_control_switch_service_client_;
    ros::ServiceClient motion_control_params_service_client_;
    ros::ServiceClient motion_control_drive_mode_service_client_;

    actionlib::SimpleActionServer<door_passing::DoorPassingAction> door_passing_server_;

    // subscriber callbacks
    void gatewayDetectionCallback(const gateway_msgs::Gateways::ConstPtr& msg);
    void distanceMonitorCallback(const std_msgs::Float32::ConstPtr& msg);
    void headingMonitorCallback(const std_msgs::Float32::ConstPtr& msg);
    void desiredHeadingCallback(const std_msgs::Float32::ConstPtr& msg);

    // action server callbacks
    void doorPassingExecute(const door_passing::DoorPassingGoalConstPtr& goal);

    // ROS service, topic names
    std::string gateway_detection_topic_;
    std::string distance_monitor_topic_;
    std::string heading_monitor_topic_;
    std::string desired_heading_topic_;
    std::string desired_velocity_topic_;

    std::string heading_control_switch_service_;
    std::string reset_distance_monitor_service_;
    std::string reset_heading_monitor_service_;
    std::string motion_control_switch_service_;
    std::string motion_control_params_service_;
    std::string motion_control_drive_mode_service_;

    // ROS related helper functions
    void resetMonitors();
    void resetHeadingMonitor();
    void resetDistanceMonitor();
    void reset();
    void enableHeadingController();
    void disableHeadingController();
    void enableMotionController();
    void disableMotionController();
    void setMotionControllerParams(double inflation_radius);
    void setMotionControllerDriveMode(int drive_mode);

    // door passing params
    int controller_frequency_;
    double velocity_;
    double laser_robot_center_offset_x_;

    // door passing
    DoorPassing door_passing_;

    // data caching
    
    // 1. detected features 
    Gateways detected_gateways_;

    // 2. monitors
    double monitored_distance_;
    double monitored_heading_;

    
};

#endif
