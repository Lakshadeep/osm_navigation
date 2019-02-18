#ifndef CORRIDOR_NAVIGATION_ROS_H
#define CORRIDOR_NAVIGATION_ROS_H

#include <string>
#include <ros/ros.h>
#include <corridor_navigation/corridor_navigation.h>

class CorridorNavigationROS
{

public:
    // Constructor / destructor
    CorridorNavigationROS(ros::NodeHandle&);
    ~CorridorNavigationROS();
    // Running
    void run();

private:
    // ROS
    ros::NodeHandle nh_;

    // corridor navigation
    CorridorNavigation corridor_navigation_;


};

#endif
