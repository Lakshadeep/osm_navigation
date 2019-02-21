#include "corridor_navigation/corridor_navigation_ros.h"
#include <ros/console.h>

int main(int argc, char **argv)
{

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG("Starting corridor navigation node");

    ros::init(argc, argv, "corridor_navigation_node");
    ros::NodeHandle nh("~");
    CorridorNavigationROS corridor_navigation_ros(nh);

    corridor_navigation_ros.run();
    
    ros::spin();

    return 0;
}