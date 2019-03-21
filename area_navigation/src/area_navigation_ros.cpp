#include "area_navigation/area_navigation_ros.h"

AreaNavigationROS::AreaNavigationROS(ros::NodeHandle& nh): nh_(nh), area_navigation_server_(nh,"/area_navigation_server",
  boost::bind(&AreaNavigationROS::AreaNavigationExecute, this, _1),false), monitored_heading_(0), monitored_distance_(0)
{
    loadParameters(); 
    
    // subscribers
    navigation_signs_subscriber_ = nh_.subscribe(navigation_signs_topic_, 1, &AreaNavigationROS::navigationSignsCallback, this);
    distance_monitor_subscriber_ = nh_.subscribe(distance_monitor_topic_, 1, &AreaNavigationROS::distanceMonitorCallback, this);
    heading_monitor_subscriber_ = nh_.subscribe(heading_monitor_topic_, 1, &AreaNavigationROS::headingMonitorCallback, this);

    // publishers
    desired_heading_publisher_ = nh_.advertise<std_msgs::Float32>(desired_heading_topic_, 1);
    desired_velocity_publisher_ = nh_.advertise<std_msgs::Float32>(desired_velocity_topic_, 1);

    // service clients
    heading_control_switch_service_client_ = nh_.serviceClient<heading_control::Switch>(heading_control_switch_service_);
    heading_monitor_reset_service_client_ = nh_.serviceClient<robot_heading_monitor::Reset>(reset_heading_monitor_service_);
    distance_monitor_reset_service_client_ = nh_.serviceClient<robot_distance_monitor::Reset>(reset_distance_monitor_service_);

    motion_control_switch_service_client_ = nh_.serviceClient<motion_control::Switch>(motion_control_switch_service_);
    motion_control_params_service_client_ = nh_.serviceClient<motion_control::Params>(motion_control_params_service_);
    motion_control_drive_mode_service_client_ = nh_.serviceClient<motion_control::DriveMode>(motion_control_drive_mode_service_);
}

AreaNavigationROS::~AreaNavigationROS()
{
}

void AreaNavigationROS::run()
{
    area_navigation_server_.start();
    ROS_INFO("Area navigation action server started");
}

void AreaNavigationROS::AreaNavigationExecute(const area_navigation::AreaNavigationGoalConstPtr& goal)
{
    reset();
    if(area_navigation_.setGoal(goal->goal_type, navigation_signs_))
    {
        ros::Rate r(controller_frequency_);

        std_msgs::Float32 desired_direction_msg;
        std_msgs::Float32 desired_velocity_msg;
        area_navigation::AreaNavigationFeedback feedback;

        double desired_orientation;

        enableHeadingController();
        enableMotionController();
        setMotionControllerParams(0.5);
        setMotionControllerDriveMode(2);

        while(nh_.ok())
        {
            area_navigation_.updateGoalNavigationSign(navigation_signs_);
            if(area_navigation_server_.isPreemptRequested())
            {
                reset();
                disableHeadingController();
                disableMotionController();
                //notify the ActionServer that we've successfully preempted
                ROS_DEBUG_NAMED("area_navigation", "Area navigation preempting the current goal");
                area_navigation_server_.setPreempted();

                return;
            }
            else
            {
                feedback.remaining_distance = monitored_distance_;
                feedback.current_direction = monitored_heading_;

                /**
                Area navigation state machine
                --------------------------
                State -1: turn to desired direction
                State 0: travel desired distance
                State 1: success
                **/

                if(area_navigation_.getState() == -1)
                {
                    desired_direction_msg.data = 0;
                    desired_velocity_msg.data = area_navigation_.getNominalVelocity();

                    if(area_navigation_.isStateChanged(monitored_distance_, monitored_heading_, navigation_signs_))
                    {
                        resetHeadingMonitor();
                        resetDistanceMonitor();
                        setMotionControllerDriveMode(1);
                    }
                }
                else if (area_navigation_.getState() == 0)
                {
                    
                    if (area_navigation_.determineDirection(desired_orientation, monitored_heading_, navigation_signs_))
                        desired_direction_msg.data = desired_orientation;
                    else
                        desired_direction_msg.data = 0;

                    desired_velocity_msg.data = area_navigation_.getNominalVelocity();

                    if(area_navigation_.isStateChanged(monitored_distance_, monitored_heading_, navigation_signs_))
                    {
                        setMotionControllerDriveMode(0);
                        setMotionControllerParams(0.7);
                        disableHeadingController();
                        disableMotionController();
                        
                        reset();

                        area_navigation_server_.setSucceeded(area_navigation::AreaNavigationResult(), "Goal reached");
                        return;
                    }
                }
                
                desired_heading_publisher_.publish(desired_direction_msg);
                desired_velocity_publisher_.publish(desired_velocity_msg);


            }
            feedback.desired_direction = desired_orientation;
            feedback.state = area_navigation_.getState();
            area_navigation_server_.publishFeedback(feedback);  

            r.sleep();
        }
    }
    else
    {
        area_navigation_server_.setAborted(area_navigation::AreaNavigationResult(), "No navigation sign detected");
        return;
    }
    
}

void AreaNavigationROS::loadParameters()
{
    // detection
    std::string navigation_signs_topic;
    nh_.param<std::string>("navigation_signs_topic", navigation_signs_topic, "/navigation_signs_detected");
    navigation_signs_topic_ = navigation_signs_topic;
    ROS_DEBUG("navigation_signs_topic: %s", navigation_signs_topic_.c_str());

    // monitors
    std::string distance_monitor_topic;
    nh_.param<std::string>("distance_monitor_topic", distance_monitor_topic, "/monitored_distance");
    distance_monitor_topic_ = distance_monitor_topic;
    ROS_DEBUG("distance_monitor_topic: %s", distance_monitor_topic_.c_str());

    std::string heading_monitor_topic;
    nh_.param<std::string>("heading_monitor_topic", heading_monitor_topic, "/monitored_heading");
    heading_monitor_topic_ = heading_monitor_topic;
    ROS_DEBUG("heading_monitor_topic: %s", heading_monitor_topic_.c_str());

    std::string reset_distance_monitor_service;
    nh_.param<std::string>("reset_distance_monitor_service", reset_distance_monitor_service, "/reset_distance_monitor");
    reset_distance_monitor_service_ = reset_distance_monitor_service;
    ROS_DEBUG("reset_distance_monitor_service: %s", reset_distance_monitor_service_.c_str());

    std::string reset_heading_monitor_service;
    nh_.param<std::string>("reset_heading_monitor_service", reset_heading_monitor_service, "/reset_heading_monitor");
    reset_heading_monitor_service_ = reset_heading_monitor_service;
    ROS_DEBUG("reset_heading_monitor_service: %s", reset_heading_monitor_service_.c_str());

    // low level control
    std::string heading_control_switch_service;
    nh_.param<std::string>("heading_control_switch_service", heading_control_switch_service, "/heading_control_switch");
    heading_control_switch_service_ = heading_control_switch_service ;
    ROS_DEBUG("heading_control_switch_service: %s", heading_control_switch_service.c_str());

    std::string desired_heading_topic;
    nh_.param<std::string>("desired_heading_topic", desired_heading_topic, "/desired_heading");
    desired_heading_topic_ = desired_heading_topic;
    ROS_DEBUG("desired_heading_topic: %s", desired_heading_topic_.c_str());

    std::string desired_velocity_topic;
    nh_.param<std::string>("desired_velocity_topic", desired_velocity_topic, "/desired_velocity");
    desired_velocity_topic_ = desired_velocity_topic;
    ROS_DEBUG("desired_velocity_topic: %s", desired_velocity_topic_.c_str());

    std::string motion_control_switch_service;
    nh_.param<std::string>("motion_control_switch_service", motion_control_switch_service, "/motion_control_switch");
    motion_control_switch_service_ = motion_control_switch_service ;
    ROS_DEBUG("motion_control_switch_service: %s", motion_control_switch_service.c_str());

    std::string motion_control_params_service;
    nh_.param<std::string>("motion_control_params_service", motion_control_params_service, "/motion_control_params");
    motion_control_params_service_ = motion_control_params_service ;
    ROS_DEBUG("motion_control_params_service: %s", motion_control_params_service.c_str());

    std::string motion_control_drive_mode_service;
    nh_.param<std::string>("motion_control_drive_mode_service", motion_control_drive_mode_service, "/motion_control_drive_mode");
    motion_control_drive_mode_service_ = motion_control_drive_mode_service ;
    ROS_DEBUG("motion_control_drive_mode_service: %s", motion_control_drive_mode_service.c_str());

    // corridor navigation params
    double velocity;
    nh_.param<double>("velocity", velocity, 0.3);
    area_navigation_.setNominalVelocity(velocity);
    ROS_DEBUG("velocity: %f", velocity);

    int controller_frequency;
    nh_.param<int>("controller_frequency", controller_frequency, 10);
    controller_frequency_ = controller_frequency;
    ROS_DEBUG("controller_frequency: %d", controller_frequency);
}

void AreaNavigationROS::navigationSignsCallback(const navigation_sign_msgs::NavigationSigns::ConstPtr& msg)
{
    navigation_signs_.clear();
    for (int i = 0; i < msg->navigation_signs.size(); i++)
    {
        navigation_signs_.push_back(navigationSignROSToNavigationSign(msg->navigation_signs[i]));
    }
}

NavigationSign AreaNavigationROS::navigationSignROSToNavigationSign(navigation_sign_msgs::NavigationSign nav_sign_ros)
{ 
    NavigationSign nav_sign;
    nav_sign.direction = nav_sign_ros.direction;
    nav_sign.type = nav_sign_ros.type;

    nav_sign.position.x = nav_sign_ros.pose.position.x;
    nav_sign.position.y = nav_sign_ros.pose.position.y;
    nav_sign.position.z = nav_sign_ros.pose.position.z;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(nav_sign_ros.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    nav_sign.orientation.roll = roll;
    nav_sign.orientation.pitch = pitch;
    nav_sign.orientation.yaw = yaw;

    return nav_sign;
}

void AreaNavigationROS::distanceMonitorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    monitored_distance_ = msg->data;
}

void AreaNavigationROS::headingMonitorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    monitored_heading_ = msg->data;
}

void AreaNavigationROS::resetMonitors()
{
    resetHeadingMonitor();
    resetDistanceMonitor();
}

void AreaNavigationROS::resetHeadingMonitor()
{
    robot_heading_monitor::Reset heading_reset_srv;
    heading_reset_srv.request.reset = true;
    if (heading_monitor_reset_service_client_.call(heading_reset_srv))
    {
        ROS_DEBUG("Heading monitor successfully reset");
        monitored_heading_ = 0;
    }
    else
    {
        ROS_ERROR("Failed to reset heading monitor");
    }
}

void AreaNavigationROS::resetDistanceMonitor()
{ 
    robot_distance_monitor::Reset distance_reset_srv;
    distance_reset_srv.request.reset = true;
    if (distance_monitor_reset_service_client_.call(distance_reset_srv))
    {
        ROS_DEBUG("Distance monitor successfully reset");
        monitored_distance_ = 0;
    }
    else
    {
        ROS_ERROR("Failed to reset distance monitor");
    }
}

void AreaNavigationROS::enableHeadingController()
{
    heading_control::Switch heading_control_switch_service_msg;
    heading_control_switch_service_msg.request.enable = true;
    if (heading_control_switch_service_client_.call(heading_control_switch_service_msg))
    {
        ROS_DEBUG("Heading controller successfully enabled");
    }
    else
    {
        ROS_ERROR("Failed to enable heading controller");
    }
}

void AreaNavigationROS::disableHeadingController()
{
    heading_control::Switch heading_control_switch_service_msg;
    heading_control_switch_service_msg.request.enable = false;
    if (heading_control_switch_service_client_.call(heading_control_switch_service_msg))
    {
        ROS_DEBUG("Heading controller successfully disabled");
    }
    else
    {
        ROS_ERROR("Failed to disable heading controller");
    }
}

void AreaNavigationROS::enableMotionController()
{
    motion_control::Switch motion_control_switch_service_msg;
    motion_control_switch_service_msg.request.enable = true;
    if (motion_control_switch_service_client_.call(motion_control_switch_service_msg))
    {
        ROS_DEBUG("Motion controller successfully enabled");
    }
    else
    {
        ROS_ERROR("Failed to enable motion controller");
    }
}

void AreaNavigationROS::disableMotionController()
{
    motion_control::Switch motion_control_switch_service_msg;
    motion_control_switch_service_msg.request.enable = false;
    if (motion_control_switch_service_client_.call(motion_control_switch_service_msg))
    {
        ROS_DEBUG("Motion controller successfully disabled");
    }
    else
    {
        ROS_ERROR("Failed to disable motion controller");
    }
}

void AreaNavigationROS::setMotionControllerParams(double inflation_radius)
{
    motion_control::Params motion_control_params_service_msg;
    motion_control_params_service_msg.request.inflation_radius = inflation_radius;
    if (motion_control_params_service_client_.call(motion_control_params_service_msg))
    {
        ROS_DEBUG("Motion controller params successfully updated");
    }
    else
    {
        ROS_ERROR("Failed to update motion controller params");
    }
}

void AreaNavigationROS::setMotionControllerDriveMode(int drive_mode)
{
    motion_control::DriveMode motion_control_drive_mode_service_msg;
    motion_control_drive_mode_service_msg.request.drive_mode = drive_mode;
    if (motion_control_drive_mode_service_client_.call(motion_control_drive_mode_service_msg))
    {
        ROS_DEBUG("Motion controller drive mode successfully updated");
    }
    else
    {
        ROS_ERROR("Failed to update motion controller drive mode");
    }
}

void AreaNavigationROS::reset()
{
    area_navigation_.reset();
    resetMonitors();
}