#include "door_passing/door_passing_ros.h"

DoorPassingROS::DoorPassingROS(ros::NodeHandle& nh): nh_(nh), door_passing_server_(nh,"/door_passing_server",
  boost::bind(&DoorPassingROS::doorPassingExecute, this, _1),false), monitored_heading_(0), monitored_distance_(0)
{
    loadParameters(); 
    
    // subscribers
    gateway_detection_subscriber_ = nh_.subscribe(gateway_detection_topic_, 1, &DoorPassingROS::gatewayDetectionCallback, this);
    distance_monitor_subscriber_ = nh_.subscribe(distance_monitor_topic_, 1, &DoorPassingROS::distanceMonitorCallback, this);
    heading_monitor_subscriber_ = nh_.subscribe(heading_monitor_topic_, 1, &DoorPassingROS::headingMonitorCallback, this);

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

DoorPassingROS::~DoorPassingROS()
{
}

void DoorPassingROS::run()
{
    door_passing_server_.start();
    ROS_INFO("Door passing action server started");
}

void DoorPassingROS::doorPassingExecute(const door_passing::DoorPassingGoalConstPtr& goal)
{
    reset();
    ros::Rate r(controller_frequency_);

    // messages for publishing desired direction & velocity to controller
    std_msgs::Float32 desired_direction_msg;
    std_msgs::Float32 desired_velocity_msg;

    // action server feedback message
    door_passing::DoorPassingFeedback feedback;

    // enable heading & motion controller
    enableHeadingController();
    enableMotionController();

    // set inflation radius to very low value as we are operating only in rotaion and heading control mode (no obstacle avoidance)
    setMotionControllerParams(0.1);
    // we start in rotation drive mode
    setMotionControllerDriveMode(2);

    // set goal received to action server
    // this goal is then updated when robot is infronr of the door
    door_passing_.setGoal(goal->door, goal->distance_inside, detected_gateways_);

    while(nh_.ok())
    {
        // if goal is cancelled by the client
        if(door_passing_server_.isPreemptRequested())
        {
            reset();

            // disable bothe heading & motion controllers
            disableHeadingController();
            disableMotionController();
            
            // notify the ActionServer that we've successfully preempted
            ROS_DEBUG_NAMED("door_passing", "Preempting the current goal");
            door_passing_server_.setPreempted();
            return;
        }
        else
        {
            /**
            Door passing state machine
            --------------------------
            State -1: orient perpendicular to the door direction
            State 0: now move formward apporximately infront of the door
            State 1: now orient facing towards the door
            State 2: start moving in the direction of door until robot almost reach the door
            State 3: enter the door and travel desired distance inside the room/area
            **/

            if(door_passing_.getState() == -1)
            {
                // if initial orientation is updated reset the heading monitor
                if(door_passing_.computeInitialOrientation(goal->door, detected_gateways_))
                    resetHeadingMonitor();

                desired_direction_msg.data = door_passing_.getInitialOrientation();
                desired_velocity_msg.data = velocity_;

                // check if state is changed & update the goal
                if(door_passing_.isStateChanged(monitored_distance_, monitored_heading_, detected_gateways_))
                {
                    if(door_passing_.setGoal(goal->door, goal->distance_inside, detected_gateways_))
                    {
                        // change from rotation to heading control drive mode
                        setMotionControllerDriveMode(1);
                        resetHeadingMonitor();
                        resetDistanceMonitor();
                    }
                    else
                    {
                        // we abort the action if door is no longer detected!
                        disableHeadingController();
                        disableMotionController();
                        door_passing_.reset();
                        ROS_DEBUG_NAMED("door_passing", "Aborting the current goal");
                        door_passing_server_.setAborted();
                        return;
                    }
                }
            }
            else if (door_passing_.getState() == 0)
            {
                
                desired_direction_msg.data = 0;
                desired_velocity_msg.data = velocity_;

                if(door_passing_.isStateChanged(monitored_distance_, monitored_heading_, detected_gateways_))
                {
                    // we switch the drive mode to rotation once robot reach infront of door
                    setMotionControllerDriveMode(2);
                    resetHeadingMonitor();
                    resetDistanceMonitor();
                }
            }
            else if (door_passing_.getState() == 1)
            {
                
                desired_direction_msg.data = door_passing_.getTurnAngle();
                desired_velocity_msg.data = velocity_;

                if(door_passing_.isStateChanged(monitored_distance_, monitored_heading_, detected_gateways_))
                {
                    // we switch the drive mode to heading control once robot is facing towards the door
                    setMotionControllerDriveMode(1);
                    resetHeadingMonitor();
                    resetDistanceMonitor();
                }
            }
            else if (door_passing_.getState() == 2)
            {   
                // keep updating robot orientation w.r.t the door
                if (door_passing_.computePassingOrientation(detected_gateways_))
                {
                    resetHeadingMonitor();
                }

                desired_direction_msg.data = door_passing_.getPassingOrientation();
                desired_velocity_msg.data = velocity_;

                if(door_passing_.isStateChanged(monitored_distance_, monitored_heading_, detected_gateways_))
                {
                    resetHeadingMonitor();
                    resetDistanceMonitor();

                    // we stay in heading control drive mode to enter the door
                    setMotionControllerDriveMode(1);
                }
            }
            else if (door_passing_.getState() == 3)
            {
                if (door_passing_.computeInsideOrientation(detected_gateways_))
                {
                    // we only reset heading monitor (and not distance monitor!!)
                    resetHeadingMonitor();
                }

                desired_direction_msg.data = door_passing_.getInsideOrientation();
                desired_velocity_msg.data = velocity_;

                if(door_passing_.isStateChanged(monitored_distance_, monitored_heading_, detected_gateways_))
                {
                    // mission successful, now set drive mode to default i.e 0 (obstacle avoidance)
                    setMotionControllerDriveMode(0);
                    
                    disableHeadingController();
                    disableMotionController();
                    
                    reset();

                    door_passing_server_.setSucceeded(door_passing::DoorPassingResult(), "Goal reached");
                    return;
                }
            }
            desired_heading_publisher_.publish(desired_direction_msg);
            desired_velocity_publisher_.publish(desired_velocity_msg);
        }

        feedback.state = door_passing_.getState();
        door_passing_server_.publishFeedback(feedback);  
        r.sleep();
    } 
}

void DoorPassingROS::loadParameters()
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

    // door passing params
    int controller_frequency;
    nh_.param<int>("controller_frequency", controller_frequency, 10);
    controller_frequency_ = controller_frequency;
    ROS_DEBUG("controller_frequency: %d", controller_frequency);

    double velocity;
    nh_.param<double>("velocity", velocity, 0.1);
    velocity_ = velocity;
    ROS_DEBUG("velocity: %f", velocity_);

    double laser_robot_center_offset_x;
    nh_.param<double>("laser_robot_center_offset_x", laser_robot_center_offset_x, 0.3);
    door_passing_.setParams(laser_robot_center_offset_x);
    ROS_DEBUG("laser_robot_center_offset_x: %f", laser_robot_center_offset_x);
}

void DoorPassingROS::gatewayDetectionCallback(const gateway_msgs::Gateways::ConstPtr& msg)
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
    detected_gateways_.x_junction.front_range_x = msg->x_junction.front_range_x;
    detected_gateways_.x_junction.front_range_y = msg->x_junction.front_range_y;

    detected_gateways_.left_door.angle = msg->left_door.angle; 
    detected_gateways_.left_door.range_x = msg->left_door.range_x; 
    detected_gateways_.left_door.range_y = msg->left_door.range_y; 

    detected_gateways_.right_door.angle = msg->right_door.angle; 
    detected_gateways_.right_door.range_x = msg->right_door.range_x;
    detected_gateways_.right_door.range_y = msg->right_door.range_y;

    detected_gateways_.front_door.angle = msg->front_door.angle; 
    detected_gateways_.front_door.range_x = msg->front_door.range_x; 
    detected_gateways_.front_door.range_y = msg->front_door.range_y; 
}

void DoorPassingROS::distanceMonitorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    monitored_distance_ = msg->data;
}

void DoorPassingROS::headingMonitorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    monitored_heading_ = msg->data;
}

void DoorPassingROS::resetMonitors()
{
    resetHeadingMonitor();
    resetDistanceMonitor();
}

void DoorPassingROS::resetHeadingMonitor()
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

void DoorPassingROS::resetDistanceMonitor()
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

void DoorPassingROS::enableHeadingController()
{
    heading_control::Switch heading_control_switch_service_msg;
    heading_control_switch_service_msg.request.enable = true;
    if (heading_control_switch_service_client_.call(heading_control_switch_service_msg))
        ROS_DEBUG("Heading controller successfully enabled");
    else
        ROS_ERROR("Failed to enable heading controller");
}

void DoorPassingROS::disableHeadingController()
{
    heading_control::Switch heading_control_switch_service_msg;
    heading_control_switch_service_msg.request.enable = false;
    if (heading_control_switch_service_client_.call(heading_control_switch_service_msg))
        ROS_DEBUG("Heading controller successfully disabled");
    else
        ROS_ERROR("Failed to disable heading controller");
}

void DoorPassingROS::enableMotionController()
{
    motion_control::Switch motion_control_switch_service_msg;
    motion_control_switch_service_msg.request.enable = true;
    if (motion_control_switch_service_client_.call(motion_control_switch_service_msg))
        ROS_DEBUG("Motion controller successfully enabled");
    else
        ROS_ERROR("Failed to enable motion controller");
}

void DoorPassingROS::disableMotionController()
{
    motion_control::Switch motion_control_switch_service_msg;
    motion_control_switch_service_msg.request.enable = false;
    if (motion_control_switch_service_client_.call(motion_control_switch_service_msg))
        ROS_DEBUG("Motion controller successfully disabled");
    else
        ROS_ERROR("Failed to disable motion controller");
}

void DoorPassingROS::setMotionControllerParams(double inflation_radius)
{
    motion_control::Params motion_control_params_service_msg;
    motion_control_params_service_msg.request.inflation_radius = inflation_radius;
    if (motion_control_params_service_client_.call(motion_control_params_service_msg))
        ROS_DEBUG("Motion controller params successfully updated");
    else
        ROS_ERROR("Failed to update motion controller params");
}

void DoorPassingROS::setMotionControllerDriveMode(int drive_mode)
{
    motion_control::DriveMode motion_control_drive_mode_service_msg;
    motion_control_drive_mode_service_msg.request.drive_mode = drive_mode;
    if (motion_control_drive_mode_service_client_.call(motion_control_drive_mode_service_msg))
        ROS_DEBUG("Motion controller drive mode successfully updated");
    else
        ROS_ERROR("Failed to update motion controller drive mode");
}

void DoorPassingROS::reset()
{
    door_passing_.reset();
    resetMonitors();
}