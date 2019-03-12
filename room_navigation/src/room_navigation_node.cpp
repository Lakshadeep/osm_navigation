#include "room_navigation/room_navigation_ros.h"
#include <ros/console.h>

int main(int argc, char **argv)
{

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG("Starting room navigation node");

    ros::init(argc, argv, "room_navigation_node");
    ros::NodeHandle nh("~");
    RoomNavigationROS room_navigation_ros(nh);

    room_navigation_ros.run();
    
    ros::spin();

    return 0;
}