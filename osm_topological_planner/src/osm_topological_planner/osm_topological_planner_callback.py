import rospy
import math
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from osm_topological_planner.compute_orientation import ComputeOrientation
from osm_planner_msgs.msg import *


class OSMTopologicalPlannerCallback(object):

    def __init__(self, osm_bridge, path_planner):
        self.osm_bridge = osm_bridge
        self.path_planner = path_planner
        self.compute_orientation = ComputeOrientation()

    def get_safe_response(self, req):
        try:
            res = self._get_response(req)
        except Exception as e:
            rospy.logerr(str(e))
            return None
        return res

    def _compute_distance(self, pt1, pt2):
        return math.sqrt(math.pow(pt1.x - pt2.x, 2) + math.pow(pt1.y - pt2.y, 2))

    # converts angle to direction
    # 0 - right
    # 2 -left
    # 1 - right
    # 3 - reverse
    def _angle_to_direction(self, orientation, last_orientation):
        angle = self._wrap_to_pi(orientation - last_orientation)
        if angle > (-math.pi / 4) and angle < (math.pi / 4):
            return 1
        elif angle > (-3 * math.pi / 4) and angle < (-math.pi / 4):
            return 0
        elif angle > (math.pi / 4) and angle < (3 * math.pi / 4):
            return 2
        else:
            return 3

    def _wrap_to_pi(self, angle):
        angle = math.fmod(angle, 2 * math.pi)
        if (angle >= math.pi):
            angle -= 2 * math.pi
        elif (angle <= -math.pi):
            angle += 2 * math.pi
        return angle

    # uses A* based path planner provided by OBL and generates symbolic plan
    # out out it
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
            path = self.path_planner.get_path_plan(start_floor=start_floor,
                                                   destination_floor=destination_floor,
                                                   start_area=start_area,
                                                   destination_area=destination_area,
                                                   start_local_area=start_local_area,
                                                   destination_local_area=destination_local_area)
        elif start_local_area and destination_task:
            path = self.path_planner.get_path_plan(start_floor=start_floor,
                                                   destination_floor=destination_floor,
                                                   start_area=start_area,
                                                   destination_area=destination_area,
                                                   start_local_area=start_local_area,
                                                   destination_task=destination_task)
        elif start_position and destination_task:
            path = self.path_planner.get_path_plan(start_floor=start_floor,
                                                   destination_floor=destination_floor,
                                                   start_area=start_area,
                                                   destination_area=destination_area,
                                                   robot_position=start_position,
                                                   destination_task=destination_task)
        elif start_position and destination_local_area:
            path = self.path_planner.get_path_plan(start_floor=start_floor,
                                                   destination_floor=destination_floor,
                                                   start_area=start_area,
                                                   destination_area=destination_area,
                                                   robot_position=start_position,
                                                   destination_local_area=destination_local_area)
        else:
            rospy.logerr("Path planner need more arguments to plan the path")

        result.topological_actions = self._generate_topological_action_plan(
            path)
        return result

    # generates symbolic topological navigation plan from list of areas
    # ASSUMPTIONS
    # - robot always starts in an area with sufficient features for localization
    # - rooms/ areas are always either start or destination (basically plan should be split in room/area)
    def _generate_topological_action_plan(self, areas):
        topological_actions = []
        last_orientation = 0.0
        last_pt = None
        last_area_type = ''
        combine = False

        for i, area in enumerate(areas):
            # for debugging
            # print(area)

            # for starting room
            if (i == 0):
                if area.type == "room":
                    # area navigation
                    ta = TopologicalAction()
                    ta.area_ids.append(area.id)
                    ta.type = area.type
                    ta.goal_id = area.exit_door.id
                    ta.goal_type = 'door'
                    ta.goal_direction = 1
                    ta.navigation_skill_type = "area_navigation"
                    topological_actions.append(ta)

                    # door passing
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
                # for corridor and room starting area we assume that we know that
                # robot is already facing correct direction
                # NOTE: This is a hack! Ideally starting area should be always a room,
                # following is a fix for testing
                elif area.type == "corridor":
                    ta = TopologicalAction()
                    ta.area_ids.append(area.id)
                    ta.type = area.type
                    ta.goal_id = -1
                    ta.goal_type = ''
                    ta.goal_direction = 1
                    ta.navigation_skill_type = "hallway_navigation"
                    topological_actions.append(ta)
                    last_area_type = area.type
                    last_pt = area.local_areas[
                        len(area.local_areas) - 1].topology
                    last_orientation = self.compute_orientation.get_corridor_orientation(
                        last_orientation, area, areas[i + 1])
                elif area.type == "area":
                    ta = TopologicalAction()
                    ta.area_ids.append(area.id)
                    ta.type = area.type
                    if area.exit_door:
                        ta.goal_id = area.exit_door.id
                        ta.goal_type = 'door'
                    else:
                        ta.goal_id = -1
                        ta.goal_type = ''
                    ta.goal_direction = 1
                    ta.navigation_skill_type = "area_navigation"
                    topological_actions.append(ta)
                    last_area_type = area.type
                else:
                    rospy.logerr(
                        "Robot must start in a room or corridor")
                    return topological_actions
            else:
                # combining corridors
                if last_area_type == area.type and area.type == "corridor":
                    topological_actions[
                        len(topological_actions) - 1].area_ids.append(area.id)
                    combine = True
                    pt = areas[i].local_areas[
                        len(areas[i].local_areas) - 1].topology
                    topological_actions[len(topological_actions) - 1].goal_distance = topological_actions[
                        len(topological_actions) - 1].goal_distance + self._compute_distance(pt, last_pt)
                    last_pt = pt
                # corridor -> junction
                elif last_area_type == 'corridor' and area.type == 'junction':
                    topological_actions[
                        len(topological_actions) - 1].goal_id = area.id
                    topological_actions[
                        len(topological_actions) - 1].goal_type = area.junction_type + '-junction'
                    pt = areas[i].local_areas[0].topology
                    topological_actions[len(topological_actions) - 1].goal_distance = topological_actions[
                        len(topological_actions) - 1].goal_distance + self._compute_distance(pt, last_pt)
                    last_pt = pt
                # junction -> corridor
                elif last_area_type == 'junction' and area.type == 'corridor':
                    topological_actions[
                        len(topological_actions) - 1].goal_id = area.id
                    topological_actions[
                        len(topological_actions) - 1].goal_type = 'corridor'
                    topological_actions[
                        len(topological_actions) - 1].navigation_skill_type = 'junction_navigation'
                    pt = area.local_areas[0].topology
                    topological_actions[
                        len(topological_actions) - 1].goal_distance = self._compute_distance(pt, last_pt)
                    last_pt = pt
                # combining areas
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
                elif last_area_type == 'corridor' and area.type == 'room':
                    # door passing
                    ta = TopologicalAction()
                    ta.area_ids.append(areas[i - 1].exit_door.id)
                    ta.type = 'door'
                    ta.goal_id = area.id
                    ta.goal_type = area.type
                    ta.navigation_skill_type = "door_navigation"
                    door_orientation = self.compute_orientation.get_door_orientation(
                        areas[i - 1].exit_door, area)
                    ta.goal_direction = self._angle_to_direction(
                        door_orientation, last_orientation)
                    last_orientation = door_orientation
                    ta.goal_distance = 1  # hardcoded to 1 m for time being

                    if ta.goal_direction == 2:
                        topological_actions[len(topological_actions) - 1].goal_type = 'left_door'
                    elif ta.goal_direction == 0:
                        topological_actions[len(topological_actions) - 1].goal_type = 'right_door'

                    topological_actions.append(ta)
                    last_pt = areas[i - 1].exit_door.topology

                if not combine:
                    ta = TopologicalAction()
                    ta.area_ids.append(area.id)
                    ta.type = area.type
                    # Assumption: we never have intermediate rooms to pass
                    # through in 1 plan except exit
                    if area.type == 'room' or area.type == 'area':
                        if area.exit_door:
                            ta.goal_id = area.exit_door.id
                            ta.goal_type = 'door'
                        else:
                            if i is not len(areas) - 1:
                                orientation = self.compute_orientation.get_corridor_orientation(
                                    areas[i - 1], area, areas[i + 1])
                                ta.goal_direction = self._angle_to_direction(
                                    orientation, last_orientation)
                                last_orientation = orientation
                                last_pt = areas[i + 1].local_areas[0].topology
                        ta.navigation_skill_type = 'area_navigation'
                    if area.type == 'corridor':
                        ta.goal_id = area.id
                        ta.goal_type = ''
                        ta.navigation_skill_type = 'hallway_navigation'
                        if last_area_type == 'door':
                            orientation = self.compute_orientation.get_corridor_orientation(
                                last_orientation, area, areas[i + 1])
                            ta.goal_direction = self._angle_to_direction(
                                orientation, last_orientation)
                            last_orientation = orientation
                        else:
                            orientation = self.compute_orientation.get_corridor_orientation(
                                areas[i - 1], area, areas[i + 1])
                            ta.goal_direction = self._angle_to_direction(
                                orientation, last_orientation)
                            last_orientation = orientation
                        start_pt = area.local_areas[0].topology
                        end_pt = areas[i + 1].local_areas[0].topology
                        ta.goal_distance = self._compute_distance(
                            start_pt, end_pt)
                        last_pt = end_pt
                    elif area.type == 'junction':
                        ta.goal_id = area.id
                        ta.goal_type = ''
                        ta.navigation_skill_type = 'junction_maneuvering'
                        orientation = self.compute_orientation.get_junction_orientation(area, areas[
                            i + 1])
                        ta.goal_direction = self._angle_to_direction(
                            orientation, last_orientation)
                        last_orientation = orientation
                        last_pt = area.topology
                    topological_actions.append(ta)

                last_area_type = area.type
                combine = False
            # print(last_area_type)

        return topological_actions
