from osm_planner_msgs.msg import *
import rospy
import math
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class OSMTopologicalPlannerCallback(object):

    def __init__(self, osm_bridge, trajectory_planner_client):
        self.osm_bridge = osm_bridge
        self.osm_trajectory_planner_client = trajectory_planner_client
        self.result = OSMTopologicalPlannerResult()

    def get_safe_response(self, req):
        try:
            res = self._get_response(req)
            return res
        except Exception as e:
            rospy.logerr(str(e))
            return None

    def _get_response(self, req):
        osm_traj_plan_goal = OSMTrajectoryPlannerGoal()
        osm_traj_plan_goal.start_floor = req.start_floor
        osm_traj_plan_goal.destination_floor = req.destination_floor
        osm_traj_plan_goal.start_area = req.start_area
        osm_traj_plan_goal.destination_area = req.destination_area
        osm_traj_plan_goal.start_local_area = req.start_local_area
        osm_traj_plan_goal.destination_local_area = req.destination_local_area
        osm_traj_plan_goal.start_position = req.start_position
        osm_traj_plan_goal.destination_task = req.destination_task

        self.osm_trajectory_planner_client.send_goal(osm_traj_plan_goal, done_cb=self._osm_trajectory_planner_cb)
        self.osm_trajectory_planner_client.wait_for_result()
        return self.result


    def _osm_trajectory_planner_cb(self, status, result):
        self.result.topological_actions = self._generate_topological_action_plan(result.areas)


    def _generate_topological_action_plan(self, areas):
        topological_actions = []

        last_area_type = ''
        for area in areas:
            if last_area_type == area.type and area.type == "corridor":
                topological_actions[len(topological_actions) - 1].area_ids.append(area.id)
                if len(area.waypoints) > 0:
                    exit_wp = area.waypoints[len(area.waypoints) - 1]
                    if exit_wp.type == 'door' and area.type:
                        topological_actions[len(topological_actions) - 1].exit_feature_id = area.waypoints[len(area.waypoints) - 1].id
                        topological_actions[len(topological_actions) - 1].exit_type  = exit_wp.type
            else: 
                if last_area_type == 'corridor' and area.type == 'junction':
                    topological_actions[len(topological_actions) - 1].exit_feature_id = area.id
                    topological_actions[len(topological_actions) - 1].exit_type  = 'junction_entry'
                elif last_area_type == 'junction' and area.type == 'corridor':
                    topological_actions[len(topological_actions) - 1].exit_feature_id = area.id
                    topological_actions[len(topological_actions) - 1].exit_type  = 'junction_exit'

                ta = TopologicalAction()
                ta.area_ids.append(area.id)
                ta.type = area.type
                if len(area.waypoints) > 0 and  area.type != 'junction':
                    exit_wp = area.waypoints[len(area.waypoints) - 1]
                    if exit_wp.type == 'door':
                        ta.exit_feature_id = area.waypoints[len(area.waypoints) - 1].id
                        ta.exit_type  = 'door_passing'
                        ta.nav_type = 'feature_following'
                elif area.type == 'junction':
                    ta.exit_feature_id = area.id
                    ta.exit_type  = 'junction_exit'
                    ta.nav_type = 'junction_maneuvering'
                    orientation = area.waypoints[0].waypoint_pose.orientation
                    ta.heading = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]*180/3.1457
                topological_actions.append(ta)
            last_area_type = area.type

        return topological_actions

