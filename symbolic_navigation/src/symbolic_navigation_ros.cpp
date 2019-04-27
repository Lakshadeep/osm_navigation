#include "symbolic_navigation/symbolic_navigation_ros.h"

SymbolicNavigationROS::SymbolicNavigationROS(ros::NodeHandle& nh): nh_(nh), symbolic_navigation_server_(nh,"/symbolic_navigation_server",
  boost::bind(&SymbolicNavigationROS::SymbolicNavigationExecute, this, _1),false), 
  osm_topological_planner_client_("/osm_topological_planner", true)
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
        ROS_INFO("Topological plan successfully received");
    }
    else
    {
        ROS_ERROR("There was problem receiving the plan");
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
    // ropod_ros_msgs::GetTopologyNodeGoal req;
    // req.id = id;
    // req.type = entity_type;
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
