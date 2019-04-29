#ifndef SYMBOLIC_NAVIGATION_ROS_H
#define SYMBOLIC_NAVIGATION_ROS_H

#include <string>
#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <symbolic_navigation/symbolic_navigation.h>
//#include <symbolic_navigation/structs.h>

// ROS messages
#include <symbolic_navigation/SymbolicNavigationAction.h>
#include <corridor_navigation/CorridorNavigationAction.h>
#include <junction_maneuvering/JunctionManeuveringAction.h>
#include <osm_planner_msgs/OSMTopologicalPlannerAction.h>


class SymbolicNavigationROS
{

public:
    // Constructor / destructor
    SymbolicNavigationROS(ros::NodeHandle&);
    ~SymbolicNavigationROS();
    void loadParameters();
    void run();

private:
    // ROS
    ros::NodeHandle nh_;

    actionlib::SimpleActionServer<symbolic_navigation::SymbolicNavigationAction> symbolic_navigation_server_;
    actionlib::SimpleActionClient<osm_planner_msgs::OSMTopologicalPlannerAction> osm_topological_planner_client_;
    actionlib::SimpleActionClient<corridor_navigation::CorridorNavigationAction> corridor_navigation_client_;
    actionlib::SimpleActionClient<junction_maneuvering::JunctionManeuveringAction> junction_maneuvering_client_;
    osm_planner_msgs::OSMTopologicalPlannerResult osm_topological_planner_result_;
    corridor_navigation::CorridorNavigationResult corridor_navigation_result_;
    junction_maneuvering::JunctionManeuveringResult junction_maneuvering_result_;

    int controller_frequency_;

    // symbolic navigation
    SymbolicNavigation symbolic_navigation_;


    // action server callbacks
    void SymbolicNavigationExecute(const symbolic_navigation::SymbolicNavigationGoalConstPtr& goal);
    bool callTopologicalPlanner(osm_planner_msgs::OSMTopologicalPlannerGoal req);
    void topologicalPlannerResultCb(const actionlib::SimpleClientGoalState& state, const osm_planner_msgs::OSMTopologicalPlannerResultConstPtr& result);
    void corridorNavigationResultCb(const actionlib::SimpleClientGoalState& state, const corridor_navigation::CorridorNavigationResultConstPtr& result);
    void junctionManeuveringResultCb(const actionlib::SimpleClientGoalState& state, const junction_maneuvering::JunctionManeuveringResultConstPtr& result);

    bool executeJunctionManeuvering(osm_planner_msgs::TopologicalAction topoglogical_action);
    bool executeCorridorNavigation(osm_planner_msgs::TopologicalAction topoglogical_action);
    bool executeDoorPassing(osm_planner_msgs::TopologicalAction topoglogical_action);
    bool executeAreaNavigation(osm_planner_msgs::TopologicalAction topoglogical_action);

};

#endif
