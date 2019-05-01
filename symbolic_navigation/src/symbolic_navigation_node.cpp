#include "symbolic_navigation/symbolic_navigation_ros.h"
#include <ros/console.h>

int main(int argc, char **argv)
{

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG("Starting area navigation node");

    ros::init(argc, argv, "symbolic_navigation_node");
    ros::NodeHandle nh("~");
    SymbolicNavigationROS symbolic_navigation_ros(nh);

    symbolic_navigation_ros.run();
    
    ros::spin();

    return 0;
}