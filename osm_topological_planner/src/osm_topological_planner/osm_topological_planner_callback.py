from osm_planner_msgs.msg import *
import rospy
import math
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from osm_topological_planner.compute_orientation import ComputeOrientation


class OSMTopologicalPlannerCallback(object):

    def __init__(self, osm_bridge, path_planner):
        self.osm_bridge = osm_bridge
        self.path_planner = path_planner
        self.compute_orientation = ComputeOrientation()

    def get_safe_response(self, req):
        try:
            res = self._get_response(req)
            return res
        except Exception as e:
            rospy.logerr(str(e))
            return None
        return res

    def _compute_distance(self, pt1, pt2):
        return math.sqrt(math.pow(pt1.x - pt2.x, 2) + math.pow(pt1.y - pt2.y, 2))

    def _angle_to_direction(self, orientation, last_orientation):
        angle = self._wrap_to_pi(orientation - last_orientation)
        if angle > (-math.pi / 4) and angle < (math.pi / 4):
            return 1
        elif angle > (-3 * math.pi / 4) and angle < (-math.pi / 4):
            return 0
        elif angle > (math.pi / 4) and angle < (3 * math.pi / 4):
            return 2
        else:
            return 4

    def _wrap_to_pi(self, angle):
        angle = math.fmod(angle, 2 * math.pi)
        if (angle >= math.pi):
            angle -= 2 * math.pi
        elif (angle <= -math.pi):
            angle += 2 * math.pi
        return angle

    def _get_response(self, req):
        result = OSMTopologicalPlannerResult()

        start_floor = req.start_floor
        destination_floor = req.destination_floor
        start_area = req.start_area
        destination_area = req.destination_area
        start_local_area = req.start_local_area
        destination_local_area = req.destination_local_area
        start_position = [req.start_position.x, req.start_position.y]
        destination_task = req.destination_task

        path = []

        if start_local_area and destination_local_area:
            path = self.path_planner.get_path_plan(start_floor=start_floor, destination_floor=destination_floor, start_area=start_area,
                                                   destination_area=destination_area, start_local_area=start_local_area, destination_local_area=destination_local_area)
        elif start_local_area and destination_task:
            path = self.path_planner.get_path_plan(start_floor=start_floor, destination_floor=destination_floor, start_area=start_area,
                                                   destination_area=destination_area, start_local_area=start_local_area, destination_task=destination_task)
        elif start_position and destination_task:
            path = self.path_planner.get_path_plan(start_floor=start_floor, destination_floor=destination_floor, start_area=start_area,
                                                   destination_area=destination_area, robot_position=start_position, destination_task=destination_task)
        elif start_position and destination_local_area:
            path = self.path_planner.get_path_plan(start_floor=start_floor, destination_floor=destination_floor, start_area=start_area,
                                                   destination_area=destination_area, robot_position=start_position, destination_local_area=destination_local_area)
        else:
            rospy.logerr("Path planner need more arguments to plan the path")

        result.topological_actions = self._generate_topological_action_plan(
            path)
        return result

    def _generate_topological_action_plan(self, areas):
        topological_actions = []
        last_orientation = 0
        last_pt = None
        last_area_type = ''
        combine = False

        for i, area in enumerate(areas):
            print(area)
            if (i == 0):
                if area.type == "room":
                    ta = TopologicalAction()
                    ta.area_ids.append(area.id)
                    ta.type = area.type
                    ta.goal_id = area.exit_door.id
                    ta.goal_type = 'door'
                    ta.goal_direction = 1
                    ta.navigation_skill_type = "area_navigation"
                    topological_actions.append(ta)

                    ta = TopologicalAction()
                    ta.area_ids.append(area.exit_door.id)
                    ta.type = 'door'
                    ta.goal_id = areas[i + 1].id
                    ta.goal_type = areas[i + 1].type
                    ta.navigation_skill_type = "door_navigation"
                    ta.goal_direction = 1
                    topological_actions.append(ta)
                    last_orientation = self.compute_orientation.get_door_orientation(
                        area.exit_door, areas[i + 1])
                    last_area_type = 'door'
                    ta.goal_distance = self._compute_distance(area.exit_door.topology, areas[
                                                              i + 1].local_areas[0].topology)
                    last_pt = areas[i + 1].local_areas[0].topology
                else:
                    rospy.logerr(
                        "Robot must start in a room to localize itself")
                    return topological_actions
            else:
                if last_area_type == area.type and area.type == "corridor":
                    topological_actions[
                        len(topological_actions) - 1].area_ids.append(area.id)
                    combine = True
                elif last_area_type == 'corridor' and area.type == 'junction':
                    topological_actions[
                        len(topological_actions) - 1].goal_id = area.id
                    topological_actions[
                        len(topological_actions) - 1].goal_type = 'junction'
                elif last_area_type == 'junction' and area.type == 'corridor':
                    topological_actions[
                        len(topological_actions) - 1].goal_id = area.id
                    topological_actions[
                        len(topological_actions) - 1].goal_type = 'corridor'
                    topological_actions[
                        len(topological_actions) - 1].navigation_skill_type = 'junction_navigation'
                elif last_area_type == area.type and area.type == "area":
                    topological_actions[
                        len(topological_actions) - 1].area_ids.append(area.id)
                    if area.exit_door:
                        topological_actions[
                            len(topological_actions) - 1].goal_id = area.exit_door.id
                        topological_actions[
                            len(topological_actions) - 1].goal_type = 'door'
                        topological_actions[
                            len(topological_actions) - 1].navigation_skill_type = 'area_navigation'
                        topological_actions[
                            len(topological_actions) - 1].goal_direction = 1
                    combine = True

                if not combine:
                    ta = TopologicalAction()
                    ta.area_ids.append(area.id)
                    ta.type = area.type
                    if area.type == 'room' or area.type == 'area':
                        if area.exit_door:
                            ta.goal_id = area.exit_door.id
                            ta.goal_type = 'door'
                        else:
                            orientation = self.compute_orientation.get_corridor_orientation(
                                areas[i - 1], area, areas[i + 1])
                            ta.goal_direction=self._angle_to_direction(
                                orientation, last_orientation)
                            last_orientation=orientation

                        ta.navigation_skill_type='area_navigation'
                        last_pt = areas[i + 1].local_areas[0].topology
                    elif area.type == 'corridor':
                        ta.goal_id=area.id
                        ta.goal_type=''
                        ta.navigation_skill_type='hallway_navigation'
                        if last_area_type == 'door':
                            orientation=self.compute_orientation.get_corridor_orientation(
                                last_orientation, area, areas[i + 1])
                            ta.goal_direction=self._angle_to_direction(
                                orientation, last_orientation)
                            last_orientation=orientation
                        else:
                            orientation=self.compute_orientation.get_corridor_orientation(
                                areas[i - 1], area, areas[i + 1])
                            ta.goal_direction=self._angle_to_direction(
                                orientation, last_orientation)
                            last_orientation=orientation

                    elif area.type == 'junction':
                        ta.goal_id=area.id
                        ta.goal_type=''
                        ta.navigation_skill_type='junction_maneuvering'
                        orientation=self.compute_orientation.get_junction_orientation(area, areas[
                                                                                        i + 1])
                        ta.goal_direction=self._angle_to_direction(
                            orientation, last_orientation)
                        last_orientation=orientation
                        last_pt = area.topology
                    topological_actions.append(ta)

                last_area_type=area.type
                combine=False
            print(last_area_type)

        return topological_actions
