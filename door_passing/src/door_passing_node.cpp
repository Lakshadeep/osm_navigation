#include "door_passing/door_passing_ros.h"
#include <ros/console.h>

int main(int argc, char **argv)
{

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG("Starting door passing node");

    ros::init(argc, argv, "door_passing_node");
    ros::NodeHandle nh("~");
    DoorPassingROS door_passing_ros(nh);

    door_passing_ros.run();
    
    ros::spin();

    return 0;
}