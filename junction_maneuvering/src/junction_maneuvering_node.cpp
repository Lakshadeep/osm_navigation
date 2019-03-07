#include "junction_maneuvering/junction_maneuvering_ros.h"
#include <ros/console.h>

int main(int argc, char **argv)
{

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG("Starting junction maneuvering node");

    ros::init(argc, argv, "junction_maneuvering_node");
    ros::NodeHandle nh("~");
    JunctionManeuveringROS junction_maneuvering_ros(nh);

    junction_maneuvering_ros.run();
    
    ros::spin();

    return 0;
}