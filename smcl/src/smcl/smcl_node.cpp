#include "smcl/smcl.h"

boost::shared_ptr<SMCL> smcl_ptr;

void sigintHandler(int sig)
{
    // Save latest pose as we're shutting down.
    smcl_ptr->savePoseToServer();
    smcl_ptr.reset();
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smcl");
    ros::NodeHandle nh;

    // Override default sigint handler
    signal(SIGINT, sigintHandler);

    // Make our node available to sigintHandler
    smcl_ptr.reset(new SMCL());

    if (argc == 1)
    {
        // run using ROS input
        ros::spin();
    }

    // Without this, our boost locks are not shut down nicely
    smcl_ptr.reset();

    // To quote Morgan, Hooray!
    return (0);
}