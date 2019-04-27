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
    osm_planner_msgs::OSMTopologicalPlannerResult osm_topological_planner_result_;

    int controller_frequency_;

    // symbolic navigation
    SymbolicNavigation symbolic_navigation_;


    // action server callbacks
    void SymbolicNavigationExecute(const symbolic_navigation::SymbolicNavigationGoalConstPtr& goal);
    bool callTopologicalPlanner(osm_planner_msgs::OSMTopologicalPlannerGoal req);
    void topologicalPlannerResultCb(const actionlib::SimpleClientGoalState& state, const osm_planner_msgs::OSMTopologicalPlannerResultConstPtr& result);

};

#endif