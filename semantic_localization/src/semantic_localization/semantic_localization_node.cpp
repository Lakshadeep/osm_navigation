#include "semantic_localization/semantic_localization.h"

boost::shared_ptr<SemanticLocalization> sl_ptr;

void sigintHandler(int sig)
{
    // Save latest pose as we're shutting down.
    sl_ptr->savePoseToServer();
    sl_ptr.reset();
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_localization");
    ros::NodeHandle nh;

    // Override default sigint handler
    signal(SIGINT, sigintHandler);

    // Make our node available to sigintHandler
    sl_ptr.reset(new SemanticLocalization());

    if (argc == 1)
    {
        // run using ROS input
        ros::spin();
    }

    // Without this, our boost locks are not shut down nicely
    sl_ptr.reset();

    // To quote Morgan, Hooray!
    return (0);
}