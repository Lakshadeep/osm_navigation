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

    def _angle_to_direction(self, angle):
        if angle > (-math.pi / 4) and angle < (math.pi / 4):
            return 1
        elif angle > (-3 * math.pi / 4) and angle < (-math.pi / 4):
            return 0
        elif angle > (math.pi / 4) and angle < (3 * math.pi / 4):
            return 2
        else:
            return 4

    def _compute_angle(self, prev_wp, curr_wp):
        p_euler = euler_from_quaternion([prev_wp.waypoint_pose.orientation.x, prev_wp.waypoint_pose.orientation.y,
                                         prev_wp.waypoint_pose.orientation.z, prev_wp.waypoint_pose.orientation.w])
        c_euler = euler_from_quaternion([curr_wp.waypoint_pose.orientation.x, curr_wp.waypoint_pose.orientation.y,
                                         curr_wp.waypoint_pose.orientation.z, curr_wp.waypoint_pose.orientation.w])
        angle = self._wrap_to_pi(c_euler[2] - p_euler[2])
        print(angle * 180 / 3.1457)
        return angle

    def _wrap_to_pi(self, angle):
        angle = math.fmod(angle, 2 * math.pi)
        if (angle >= math.pi):
            angle -= 2 * math.pi
        elif (angle <= -math.pi):
            angle += 2 * math.pi
        return angle

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

        self.osm_trajectory_planner_client.send_goal(
            osm_traj_plan_goal, done_cb=self._osm_trajectory_planner_cb)
        self.osm_trajectory_planner_client.wait_for_result()
        return self.result

    def _osm_trajectory_planner_cb(self, status, result):
        self.result.topological_actions = self._generate_topological_action_plan(
            result.areas)

    def _generate_topological_action_plan(self, areas):
        topological_actions = []

        last_area_type = ''
        last_waypoint = Waypoint(waypoint_pose=Pose(
            position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1)))

        for area in areas:

            goal_direction = 0
            combine = False

            if last_area_type == area.type and area.type == "corridor":
                topological_actions[
                    len(topological_actions) - 1].area_ids.append(area.id)
                if len(area.waypoints) > 0:
                    exit_wp = area.waypoints[len(area.waypoints) - 1]
                    if exit_wp.type == 'door':
                        topological_actions[len(
                            topological_actions) - 1].goal_id = area.waypoints[len(area.waypoints) - 1].id
                        topological_actions[
                            len(topological_actions) - 1].goal_type = 'door'
                        topological_actions[
                            len(topological_actions) - 1].navigation_skill_type = 'hallway_navigation'
                        angle = self._compute_angle(last_waypoint, exit_wp)
                        topological_actions[
                            len(topological_actions) - 1].goal_direction = self._angle_to_direction(angle)
                        last_waypoint = exit_wp

                combine = True
            elif last_area_type == 'corridor' and area.type == 'junction':
                topological_actions[
                    len(topological_actions) - 1].goal_id = area.id
                topological_actions[
                    len(topological_actions) - 1].goal_type = 'junction'
                topological_actions[
                    len(topological_actions) - 1].navigation_skill_type = 'hallway_navigation'
            elif last_area_type == 'junction' and area.type == 'corridor':
                topological_actions[
                    len(topological_actions) - 1].goal_id = area.id
                topological_actions[
                    len(topological_actions) - 1].goal_type = 'corridor'
                topological_actions[
                    len(topological_actions) - 1].navigation_skill_type = 'junction_maneuvering'

                exit_wp = area.waypoints[len(area.waypoints) - 1]
                angle = self._compute_angle(last_waypoint, exit_wp)
                topological_actions[
                    len(topological_actions) - 1].goal_direction = self._angle_to_direction(angle)
                last_waypoint = exit_wp

            if last_area_type == 'room' and area.type == 'corridor':
                if len(area.waypoints) > 0:
                    exit_wp = area.waypoints[0]
                    angle = self._compute_angle(last_waypoint, exit_wp)
                    topological_actions[
                        len(topological_actions) - 1].goal_direction = self._angle_to_direction(angle)
                    last_waypoint = area.waypoints[len(area.waypoints) - 1]

            # TODO : Corridor -> room

            if not combine:
                ta = TopologicalAction()
                ta.area_ids.append(area.id)
                ta.type = area.type
                if len(area.waypoints) > 0 and area.type == 'room':
                    exit_wp = area.waypoints[len(area.waypoints) - 1]
                    if exit_wp.type == 'door':
                        ta.goal_id = area.waypoints[
                            len(area.waypoints) - 1].id
                        ta.goal_type = 'door'
                        ta.navigation_skill_type = 'area_navigation'

                    last_waypoint = exit_wp
                elif area.type == 'corridor':
                    ta.goal_id = area.id
                    ta.goal_type = ''
                    ta.navigation_skill_type = 'hallway_navigation'
                    ta.goal_direction = goal_direction
                    last_waypoint = area.waypoints[len(area.waypoints) - 1]
                elif area.type == 'junction':
                    ta.goal_id = area.id
                    ta.goal_type = ''
                    ta.navigation_skill_type = 'junction_maneuvering'
                    ta.goal_direction = goal_direction
                topological_actions.append(ta)

            last_area_type = area.type
            combine = False

        return topological_actions
