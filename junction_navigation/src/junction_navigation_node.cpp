#include "junction_navigation/junction_navigation_ros.h"
#include <ros/console.h>

int main(int argc, char **argv)
{

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG("Starting junction navigation node");

    ros::init(argc, argv, "junction_navigation_node");
    ros::NodeHandle nh("~");
    JunctionNavigationROS junction_navigation_ros(nh);

    junction_navigation_ros.run();
    
    ros::spin();

    return 0;
}