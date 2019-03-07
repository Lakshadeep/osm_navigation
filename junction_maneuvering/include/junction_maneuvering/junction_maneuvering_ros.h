#ifndef JUNCTION_MANEUVERING_ROS_H
#define JUNCTION_MANEUVERING_ROS_H

#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <junction_maneuvering/junction_maneuvering.h>
#include <junction_maneuvering/structs.h>

// ROS messages
#include <gateway_msgs/Gateways.h>
#include <std_msgs/Float32.h>
#include <junction_maneuvering/JunctionManeuveringAction.h>

// ROS services
#include <heading_control/Switch.h>
#include <robot_distance_monitor/Reset.h>
#include <robot_heading_monitor/Reset.h>
#include <motion_control/Switch.h>
#include <motion_control/Params.h>
#include <motion_control/DriveMode.h>

class JunctionManeuveringROS
{

public:
    // Constructor / destructor
    JunctionManeuveringROS(ros::NodeHandle&);
    ~JunctionManeuveringROS();
    
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

    actionlib::SimpleActionServer<junction_maneuvering::JunctionManeuveringAction> junction_maneuvering_server_;

    // subscriber callbacks
    void gatewayDetectionCallback(const gateway_msgs::Gateways::ConstPtr& msg);
    void distanceMonitorCallback(const std_msgs::Float32::ConstPtr& msg);
    void headingMonitorCallback(const std_msgs::Float32::ConstPtr& msg);
    void desiredHeadingCallback(const std_msgs::Float32::ConstPtr& msg);

    // action server callbacks
    void junctionManeuveringExecute(const junction_maneuvering::JunctionManeuveringGoalConstPtr& goal);

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

    // junction maneuvering
    JunctionManeuvering junction_maneuvering_;

    // data caching
    
    // 1. detected features 
    Gateways detected_gateways_;

    // 2. monitors
    double monitored_distance_;
    double monitored_heading_;

    
};

#endif
