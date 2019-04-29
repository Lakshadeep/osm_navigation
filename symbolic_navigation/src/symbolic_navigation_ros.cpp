#include "symbolic_navigation/symbolic_navigation_ros.h"

SymbolicNavigationROS::SymbolicNavigationROS(ros::NodeHandle& nh): nh_(nh), symbolic_navigation_server_(nh,"/symbolic_navigation_server",
  boost::bind(&SymbolicNavigationROS::SymbolicNavigationExecute, this, _1),false), 
  osm_topological_planner_client_("/osm_topological_planner", true), corridor_navigation_client_("/corridor_navigation_server", true) 
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
            ROS_DEBUG("Navigation skill type: %s", navigation_skill_type.c_str());
            if (navigation_skill_type == "hallway_navigation")
            {
                executeCorridorNavigation(osm_topological_planner_result_.topological_actions[i]);
            }
            else if (navigation_skill_type == "junction_navigation")
            {
                executeJunctionManeuvering(osm_topological_planner_result_.topological_actions[i]);
            }
            else if (navigation_skill_type == "area_navigation")
            {
                executeAreaNavigation(osm_topological_planner_result_.topological_actions[i]);
            }
            else if (navigation_skill_type == "door_navigation")
            {
                executeDoorPassing(osm_topological_planner_result_.topological_actions[i]);
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

bool SymbolicNavigationROS::executeJunctionManeuvering(osm_planner_msgs::TopologicalAction topoglogical_action)
{
    return false;
} 

bool SymbolicNavigationROS::executeCorridorNavigation(osm_planner_msgs::TopologicalAction topoglogical_action)
{
    corridor_navigation::CorridorNavigationGoal corridor_navigation_request;
    corridor_navigation_request.direction = topoglogical_action.goal_direction;
    corridor_navigation_request.distance = topoglogical_action.goal_distance;

    if(topoglogical_action.goal_type == "junction")
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
    return false;
} 

bool SymbolicNavigationROS::executeAreaNavigation(osm_planner_msgs::TopologicalAction topoglogical_action)
{
    return false;
} 
