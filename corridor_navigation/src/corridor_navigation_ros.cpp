#include "corridor_navigation/corridor_navigation_ros.h"

CorridorNavigationROS::CorridorNavigationROS(ros::NodeHandle& nh): nh_(nh), corridor_navigation_server_(nh,"/corridor_navigation_server",
  boost::bind(&CorridorNavigationROS::CorridorNavigationExecute, this, _1),false), monitored_heading_(0), monitored_distance_(0)
{
    loadParameters(); 
    
    // subscribers
    gateway_detection_subscriber_ = nh_.subscribe(gateway_detection_topic_, 1, &CorridorNavigationROS::gatewayDetectionCallback, this);
    distance_monitor_subscriber_ = nh_.subscribe(distance_monitor_topic_, 1, &CorridorNavigationROS::distanceMonitorCallback, this);
    heading_monitor_subscriber_ = nh_.subscribe(heading_monitor_topic_, 1, &CorridorNavigationROS::headingMonitorCallback, this);

    // publishers
    desired_heading_publisher_ = nh_.advertise<std_msgs::Float32>(desired_heading_topic_, 1);
    desired_velocity_publisher_ = nh_.advertise<std_msgs::Float32>(desired_velocity_topic_, 1);

    // service clients
    heading_control_switch_service_client_ = nh_.serviceClient<heading_control::Switch>(heading_control_switch_service_);
    heading_monitor_reset_service_client_ = nh_.serviceClient<robot_heading_monitor::Reset>(reset_heading_monitor_service_);
    distance_monitor_reset_service_client_ = nh_.serviceClient<robot_distance_monitor::Reset>(reset_distance_monitor_service_);
}

CorridorNavigationROS::~CorridorNavigationROS()
{
}

void CorridorNavigationROS::run()
{
    corridor_navigation_server_.start();
    ROS_INFO("Corridor navigation action server started");
}

void CorridorNavigationROS::CorridorNavigationExecute(const corridor_navigation_msgs::CorridorNavigationGoalConstPtr& goal)
{
    reset();
    corridor_navigation_.setGoal(goal->goal_type, goal->direction, goal->distance);
    ros::Rate r(controller_frequency_);

    std_msgs::Float32 desired_direction_msg;
    std_msgs::Float32 desired_velocity_msg;

    enableHeadingController();

    while(nh_.ok())
    {
        if(corridor_navigation_server_.isPreemptRequested())
        {
            reset();
            disableHeadingController();
            //notify the ActionServer that we've successfully preempted
            ROS_DEBUG_NAMED("corridor_navigation", "Corridor navigation preempting the current goal");
            corridor_navigation_server_.setPreempted();

            return;
        }
        else
        {
            corridor_navigation_msgs::CorridorNavigationFeedback feedback;
            feedback.distance_travelled = monitored_distance_;
            feedback.current_direction = monitored_heading_;

            double desired_direction, desired_velocity;

            if (corridor_navigation_.determineDirection(desired_direction, monitored_heading_, detected_gateways_.hallway.left_angle, 
            detected_gateways_.hallway.left_range, detected_gateways_.hallway.right_angle, detected_gateways_.hallway.right_angle))
            {   
                desired_velocity = corridor_navigation_.computeVelocity(monitored_distance_, monitored_heading_);
            }
            else
            {
                feedback.recovery_mode = 1;
            }

            desired_direction_msg.data = desired_direction;
            desired_heading_publisher_.publish(desired_direction_msg);
            
            feedback.desired_direction = desired_direction;
            corridor_navigation_server_.publishFeedback(feedback);   

            if (corridor_navigation_.isGoalReached(detected_gateways_, monitored_distance_, monitored_heading_))
            {
                reset();
                disableHeadingController();
                corridor_navigation_server_.setSucceeded(corridor_navigation_msgs::CorridorNavigationResult(), "Goal reached");
                desired_velocity = 0;
                return;
            }
            desired_velocity_msg.data = desired_velocity;
            desired_velocity_publisher_.publish(desired_velocity_msg);
        }

        r.sleep();
    }
}

void CorridorNavigationROS::loadParameters()
{
    // detection
    std::string gateway_detection_topic;
    nh_.param<std::string>("gateway_detection_topic", gateway_detection_topic, "/gateways_detected");
    gateway_detection_topic_ = gateway_detection_topic;
    ROS_DEBUG("gateway_detection_topic: %s", gateway_detection_topic_.c_str());

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
    ROS_DEBUG("heading_control_switch_service service: %s", heading_control_switch_service.c_str());

    std::string desired_heading_topic;
    nh_.param<std::string>("desired_heading_topic", desired_heading_topic, "/desired_heading");
    desired_heading_topic_ = desired_heading_topic;
    ROS_DEBUG("desired_heading_topic: %s", desired_heading_topic_.c_str());

    std::string desired_velocity_topic;
    nh_.param<std::string>("desired_velocity_topic", desired_velocity_topic, "/desired_velocity");
    desired_velocity_topic_ = desired_velocity_topic;
    ROS_DEBUG("desired_velocity_topic: %s", desired_velocity_topic_.c_str());

    // corridor navigation params
    double recovery_direction_threshold;
    nh_.param<double>("recovery_direction_threshold", recovery_direction_threshold, 0.5);
    corridor_navigation_.setRecoveryDirectionThreshold(recovery_direction_threshold);
    ROS_DEBUG("recovery_direction_threshold: %f", recovery_direction_threshold);

    double correction_direction_threshold;
    nh_.param<double>("correction_direction_threshold", correction_direction_threshold, 0.06);
    corridor_navigation_.setCorrectionDirectionThreshold(correction_direction_threshold);
    ROS_DEBUG("correction_direction_threshold: %f", correction_direction_threshold);

    double velocity;
    nh_.param<double>("velocity", velocity, 0.3);
    corridor_navigation_.setNominalVelocity(velocity);
    ROS_DEBUG("velocity: %f", velocity);

    int controller_frequency;
    nh_.param<int>("controller_frequency", controller_frequency, 10);
    controller_frequency_ = controller_frequency;
    ROS_DEBUG("controller_frequency: %d", controller_frequency);
}

void CorridorNavigationROS::gatewayDetectionCallback(const gateway_msgs::Gateways::ConstPtr& msg)
{
    detected_gateways_.hallway.left_angle = msg->hallway.left_angle;
    detected_gateways_.hallway.right_angle = msg->hallway.right_angle;
    detected_gateways_.hallway.left_range = msg->hallway.left_range;
    detected_gateways_.hallway.right_range = msg->hallway.right_range;

    detected_gateways_.t_junction.left_turn_angle = msg->t_junction.left_turn_angle;
    detected_gateways_.t_junction.left_turn_range = msg->t_junction.left_turn_range;
    detected_gateways_.t_junction.right_turn_angle = msg->t_junction.right_turn_angle;
    detected_gateways_.t_junction.right_turn_range = msg->t_junction.right_turn_range;
    detected_gateways_.t_junction.front_angle = msg->t_junction.front_angle;
    detected_gateways_.t_junction.front_range = msg->t_junction.front_range;

    detected_gateways_.x_junction.left_turn_angle = msg->x_junction.left_turn_angle;
    detected_gateways_.x_junction.left_turn_range = msg->x_junction.left_turn_range;
    detected_gateways_.x_junction.right_turn_angle = msg->x_junction.right_turn_angle;
    detected_gateways_.x_junction.right_turn_range = msg->x_junction.right_turn_range;
    detected_gateways_.x_junction.front_angle = msg->x_junction.front_angle;
    detected_gateways_.x_junction.front_range = msg->x_junction.front_range;
}

void CorridorNavigationROS::distanceMonitorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    monitored_distance_ = msg->data;
}

void CorridorNavigationROS::headingMonitorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    monitored_heading_ = msg->data;
}

void CorridorNavigationROS::resetMonitors()
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

void CorridorNavigationROS::enableHeadingController()
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

void CorridorNavigationROS::disableHeadingController()
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

void CorridorNavigationROS::reset()
{
    corridor_navigation_.reset();
    resetMonitors();
}