#ifndef SYMBOLIC_NAVIGATION_ROS_H
#define SYMBOLIC_NAVIGATION_ROS_H

#include <string>
#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <symbolic_navigation/symbolic_navigation.h>
//#include <symbolic_navigation/structs.h>

// ROS messages
#include <symbolic_navigation/SymbolicNavigationAction.h>


class SymbolicNavigationROS
{

public:
    // Constructor / destructor
    SymbolicNavigationROS(ros::NodeHandle&);
    ~SymbolicNavigationROS();
    void loadParameters();
    void run();

private:
    // ROS
    ros::NodeHandle nh_;

    actionlib::SimpleActionServer<symbolic_navigation::SymbolicNavigationAction> symbolic_navigation_server_;

    int controller_frequency_;

    // symbolic navigation
    SymbolicNavigation symbolic_navigation_;


    // action server callbacks
    void SymbolicNavigationExecute(const symbolic_navigation::SymbolicNavigationGoalConstPtr& goal);
};

#endif
