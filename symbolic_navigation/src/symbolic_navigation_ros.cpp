#include "symbolic_navigation/symbolic_navigation_ros.h"

SymbolicNavigationROS::SymbolicNavigationROS(ros::NodeHandle& nh): nh_(nh), symbolic_navigation_server_(nh,"/symbolic_navigation_server",
  boost::bind(&SymbolicNavigationROS::SymbolicNavigationExecute, this, _1),false)
{
    loadParameters(); 
    
}

SymbolicNavigationROS::~SymbolicNavigationROS()
{
}

void SymbolicNavigationROS::run()
{
    symbolic_navigation_server_.start();
    ROS_INFO("Symbolic navigation action server started");
}

void SymbolicNavigationROS::SymbolicNavigationExecute(const symbolic_navigation::SymbolicNavigationGoalConstPtr& goal)
{
}

void SymbolicNavigationROS::loadParameters()
{
    // std::string navigation_signs_topic;
    // nh_.param<std::string>("navigation_signs_topic", navigation_signs_topic, "/navigation_signs_detected");
    // navigation_signs_topic_ = navigation_signs_topic;
    // ROS_DEBUG("navigation_signs_topic: %s", navigation_signs_topic_.c_str());
}
