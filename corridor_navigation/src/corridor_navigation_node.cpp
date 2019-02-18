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

    double frequency;
    nh.param<double>("frequency", frequency, 10);
    ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
    ros::Rate rate(frequency);

    while (ros::ok())
    {
        corridor_navigation_ros.run();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}