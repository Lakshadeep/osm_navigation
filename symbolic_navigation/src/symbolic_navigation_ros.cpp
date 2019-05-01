#include "symbolic_navigation/symbolic_navigation_ros.h"

SymbolicNavigationROS::SymbolicNavigationROS(ros::NodeHandle& nh): nh_(nh), symbolic_navigation_server_(nh,"/symbolic_navigation_server",
  boost::bind(&SymbolicNavigationROS::SymbolicNavigationExecute, this, _1),false), 
  osm_topological_planner_client_("/osm_topological_planner", true), corridor_navigation_client_("/corridor_navigation_server", true),
  junction_maneuvering_client_("/junction_maneuvering_server", true), door_passing_client_("/door_passing_server", true),
  area_navigation_client_("/area_navigation_server", true) 
{
    loadParameters(); 
    symbolic_navigation_server_.start();
    ROS_INFO("Symbolic navigation action server started");
}

SymbolicNavigationROS::~SymbolicNavigationROS()
{
}

void SymbolicNavigationROS::run()
{
}

void SymbolicNavigationROS::SymbolicNavigationExecute(const symbolic_navigation::SymbolicNavigationGoalConstPtr& goal)
{
    osm_planner_msgs::OSMTopologicalPlannerGoal planner_req;
    planner_req.start_floor = goal->start_floor;
    planner_req.destination_floor = goal->destination_floor;
    planner_req.start_area = goal->start_area;
    planner_req.destination_area = goal->destination_area;
    planner_req.start_local_area = goal->start_local_area;
    planner_req.destination_local_area = goal->destination_local_area;
    planner_req.start_position = goal->start_position;
    planner_req.destination_task = goal->destination_task;
    if(callTopologicalPlanner(planner_req))
    {
        for(int i = 0; i < osm_topological_planner_result_.topological_actions.size(); i++)
        {
            std::string navigation_skill_type = osm_topological_planner_result_.topological_actions[i].navigation_skill_type;
            ROS_ERROR("Navigation skill type: %s", navigation_skill_type.c_str());
            if (navigation_skill_type == "hallway_navigation")
            {
                if(!executeCorridorNavigation(osm_topological_planner_result_.topological_actions[i]))
                    ROS_ERROR("Corridor navigation failed");
            }
            else if (navigation_skill_type == "junction_navigation")
            {
                if(!executeJunctionManeuvering(osm_topological_planner_result_.topological_actions[i]))
                    ROS_ERROR("Junction maneuvering failed");
            }
            else if (navigation_skill_type == "area_navigation")
            {
                if(!executeAreaNavigation(osm_topological_planner_result_.topological_actions[i]))
                    ROS_ERROR("Area navigation failed");
            }
            else if (navigation_skill_type == "door_navigation")
            {
                if(!executeDoorPassing(osm_topological_planner_result_.topological_actions[i]))
                    ROS_ERROR("Door passing failed");
            }
        }
    }
    else
    {
        ROS_ERROR("There was problem receiving the topological navigation plan");
    }
}

void SymbolicNavigationROS::loadParameters()
{
    // std::string navigation_signs_topic;
    // nh_.param<std::string>("navigation_signs_topic", navigation_signs_topic, "/navigation_signs_detected");
    // navigation_signs_topic_ = navigation_signs_topic;
    // ROS_DEBUG("navigation_signs_topic: %s", navigation_signs_topic_.c_str());
}

bool SymbolicNavigationROS::callTopologicalPlanner(osm_planner_msgs::OSMTopologicalPlannerGoal req)
{
    osm_topological_planner_client_.sendGoal(req, boost::bind(&SymbolicNavigationROS::topologicalPlannerResultCb, this, _1, _2));
    bool finished_before_timeout = osm_topological_planner_client_.waitForResult(ros::Duration(60.0));
    if (osm_topological_planner_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return true;
    }
    return false;
}

void SymbolicNavigationROS::topologicalPlannerResultCb(const actionlib::SimpleClientGoalState& state, const osm_planner_msgs::OSMTopologicalPlannerResultConstPtr& result)
{
    osm_topological_planner_result_ = *result;
}

void SymbolicNavigationROS::corridorNavigationResultCb(const actionlib::SimpleClientGoalState& state, const corridor_navigation::CorridorNavigationResultConstPtr& result)
{
    corridor_navigation_result_ = *result;
}

void SymbolicNavigationROS::junctionManeuveringResultCb(const actionlib::SimpleClientGoalState& state, const junction_maneuvering::JunctionManeuveringResultConstPtr& result)
{
    junction_maneuvering_result_ = *result;
}

void SymbolicNavigationROS::doorPassingResultCb(const actionlib::SimpleClientGoalState& state, const door_passing::DoorPassingResultConstPtr& result)
{
    door_passing_result_ = *result;
}

void SymbolicNavigationROS::areaNavigationResultCb(const actionlib::SimpleClientGoalState& state, const area_navigation::AreaNavigationResultConstPtr& result)
{
    area_navigation_result_ = *result;
}

bool SymbolicNavigationROS::executeJunctionManeuvering(osm_planner_msgs::TopologicalAction topoglogical_action)
{
    junction_maneuvering::JunctionManeuveringGoal junction_maneuvering_request;
    junction_maneuvering_request.turn_direction = topoglogical_action.goal_direction;
    if(topoglogical_action.goal_type == "T-junction")
        junction_maneuvering_request.junction = 0;
    else if(topoglogical_action.goal_type == "X-junction")
        junction_maneuvering_request.junction = 1;
    
    junction_maneuvering_request.distance = topoglogical_action.goal_distance;

    junction_maneuvering_client_.sendGoal(junction_maneuvering_request, boost::bind(&SymbolicNavigationROS::junctionManeuveringResultCb, this, _1, _2));
    bool finished_before_timeout = junction_maneuvering_client_.waitForResult(ros::Duration(600.0));
    if (junction_maneuvering_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return true;
    }
    return false;
} 

bool SymbolicNavigationROS::executeCorridorNavigation(osm_planner_msgs::TopologicalAction topoglogical_action)
{
    corridor_navigation::CorridorNavigationGoal corridor_navigation_request;
    // corridor_navigation_request.direction = topoglogical_action.goal_direction; # resolve this with turn
    corridor_navigation_request.direction = topoglogical_action.goal_direction;
    corridor_navigation_request.distance = topoglogical_action.goal_distance;

    if(topoglogical_action.goal_type == "T-junction")
        corridor_navigation_request.goal_type = 0;
    else if(topoglogical_action.goal_type == "X-junction")
        corridor_navigation_request.goal_type = 1;
    else if(topoglogical_action.goal_type == "left_door")
        corridor_navigation_request.goal_type = 2;
    else if(topoglogical_action.goal_type == "right_door")
        corridor_navigation_request.goal_type = 3; 

    corridor_navigation_client_.sendGoal(corridor_navigation_request, boost::bind(&SymbolicNavigationROS::corridorNavigationResultCb, this, _1, _2));
    bool finished_before_timeout = corridor_navigation_client_.waitForResult(ros::Duration(600.0));
    if (corridor_navigation_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return true;
    }
    return false;
} 

bool SymbolicNavigationROS::executeDoorPassing(osm_planner_msgs::TopologicalAction topoglogical_action)
{
    door_passing::DoorPassingGoal door_passing_request;
    door_passing_request.door = topoglogical_action.goal_direction;
    door_passing_request.distance_inside = topoglogical_action.goal_distance;

    door_passing_client_.sendGoal(door_passing_request, boost::bind(&SymbolicNavigationROS::doorPassingResultCb, this, _1, _2));
    bool finished_before_timeout = door_passing_client_.waitForResult(ros::Duration(600.0));
    if (door_passing_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return true;
    }
    return false;
}

bool SymbolicNavigationROS::executeAreaNavigation(osm_planner_msgs::TopologicalAction topoglogical_action)
{
    area_navigation::AreaNavigationGoal area_navigation_request;
    area_navigation_request.goal_type = 0; // Hardcoded to exit right now

    area_navigation_client_.sendGoal(area_navigation_request, boost::bind(&SymbolicNavigationROS::areaNavigationResultCb, this, _1, _2));
    bool finished_before_timeout = area_navigation_client_.waitForResult(ros::Duration(600.0));
    if (area_navigation_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return true;
    }
    return false;
} 