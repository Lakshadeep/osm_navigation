#include "area_navigation/area_navigation_ros.h"

AreaNavigationROS::AreaNavigationROS(ros::NodeHandle& nh): nh_(nh), area_navigation_server_(nh,"/area_navigation_server",
  boost::bind(&AreaNavigationROS::AreaNavigationExecute, this, _1),false), monitored_heading_(0), monitored_distance_(0),
  recovery_enabled_(false)
{
    loadParameters(); 
    
    // subscribers
    gateway_detection_subscriber_ = nh_.subscribe(gateway_detection_topic_, 1, &AreaNavigationROS::gatewayDetectionCallback, this);
    semantic_feature_detection_subscriber_ = nh_.subscribe(semantic_feature_detection_topic_, 1, 
                                             &AreaNavigationROS::semanticFeatureDetectionCallback, this);
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
    area_navigation_.setGoal(goal->goal_type, goal->direction, goal->distance, goal->available_features, 
                             goal->available_features_directions, goal->available_features_distances);
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
            feedback.distance_travelled = monitored_distance_;
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
                desired_direction_msg.data = area_navigation_.getInitialOrientation();
                desired_velocity_msg.data = area_navigation_.getNominalVelocity();

                if(area_navigation_.isStateChanged(monitored_distance_, monitored_heading_, detected_semantic_features_))
                {
                    resetHeadingMonitor();
                    resetDistanceMonitor();
                    setMotionControllerDriveMode(1);
                }
            }
            else if (area_navigation_.getState() == 0)
            {
                
                if (area_navigation_.determineDirection(desired_orientation, monitored_heading_, 
                                             detected_gateways_, detected_semantic_features_))
                    desired_direction_msg.data = desired_orientation;
                else
                    desired_direction_msg.data = 0;

                desired_velocity_msg.data = area_navigation_.getNominalVelocity();

                if(area_navigation_.isStateChanged(monitored_distance_, monitored_heading_, detected_semantic_features_))
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

void AreaNavigationROS::loadParameters()
{
    // detection
    std::string semantic_feature_detection_topic;
    nh_.param<std::string>("semantic_feature_detection_topic", semantic_feature_detection_topic, "/semantic_features_detected");
    semantic_feature_detection_topic_ = semantic_feature_detection_topic;
    ROS_DEBUG("semantic_feature_detection_topic: %s", semantic_feature_detection_topic_.c_str());

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
    double recovery_direction_threshold;
    nh_.param<double>("recovery_direction_threshold", recovery_direction_threshold, 1.0);
    area_navigation_.setRecoveryDirectionThreshold(recovery_direction_threshold);
    ROS_DEBUG("recovery_direction_threshold: %f", recovery_direction_threshold);

    double correction_direction_threshold;
    nh_.param<double>("correction_direction_threshold", correction_direction_threshold, 0.06);
    area_navigation_.setCorrectionDirectionThreshold(correction_direction_threshold);
    ROS_DEBUG("correction_direction_threshold: %f", correction_direction_threshold);

    double velocity;
    nh_.param<double>("velocity", velocity, 0.3);
    area_navigation_.setNominalVelocity(velocity);
    ROS_DEBUG("velocity: %f", velocity);

    int controller_frequency;
    nh_.param<int>("controller_frequency", controller_frequency, 10);
    controller_frequency_ = controller_frequency;
    ROS_DEBUG("controller_frequency: %d", controller_frequency);
}

void AreaNavigationROS::gatewayDetectionCallback(const gateway_msgs::Gateways::ConstPtr& msg)
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
}

void AreaNavigationROS::semanticFeatureDetectionCallback(const osm_map_msgs::SemanticMap::ConstPtr& msg)
{
    detected_semantic_features_.wall_sides.clear();
    detected_semantic_features_.door_sides.clear();
    detected_semantic_features_.pillars.clear();
    detected_semantic_features_.features.clear();
    
    for (int i = 0; i < msg->wall_sides.size(); i++)
    {
        WallSide ws;
        ws.radius = msg->wall_sides[i].radius;
        ws.angle = msg->wall_sides[i].angle;
        ws.corner1.x = msg->wall_sides[i].corners[0].x;
        ws.corner1.y = msg->wall_sides[i].corners[0].y;
        ws.corner2.x = msg->wall_sides[i].corners[1].x;
        ws.corner2.y = msg->wall_sides[i].corners[1].y;
        detected_semantic_features_.wall_sides.push_back(ws);
    }

    for (int i = 0; i < msg->door_sides.size(); i++)
    {
        DoorSide ds;
        ds.radius = msg->door_sides[i].radius;
        ds.angle = msg->door_sides[i].angle;
        ds.corner1.x = msg->door_sides[i].corners[0].x;
        ds.corner1.y = msg->door_sides[i].corners[0].y;
        ds.corner2.x = msg->door_sides[i].corners[1].x;
        ds.corner2.y = msg->door_sides[i].corners[1].y;
        detected_semantic_features_.door_sides.push_back(ds);
    }

    for (int i = 0; i < msg->pillars.size(); i++)
    {
        Pillar p;
        p.radius = msg->pillars[i].radius;
        p.angle = msg->pillars[i].angle;
        detected_semantic_features_.pillars.push_back(p);
    }

    for (int i = 0; i < msg->features.size(); i++)
    {
        Feature f;
        f.type = msg->features[i].type;
        f.height = msg->features[i].height;
        f.breast = msg->features[i].breast;
        f.width = msg->features[i].width;
        f.position.x = msg->features[i].position.x;
        f.position.y = msg->features[i].position.y;
        detected_semantic_features_.features.push_back(f);
    }

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