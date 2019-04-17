from osm_planner_msgs.msg import *
import math
from geometry_msgs.msg import Pose, Point, Quaternion
# from OBL.structs.wm.point import Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class ComputeOrientation(object):

    def __init__(self):
        pass

    def get_door_orientation(self, door, next_area):
        door_points = self._get_door_points(door.geometry.points)
        door_center = self._compute_center(door.geometry.points)
        next_area_center = self._compute_center(next_area.geometry.points)
        door_angle = math.atan2(
            door_points[1].y - door_points[0].y, door_points[1].x - door_points[0].x)
        door_angle1 = self._wrap_to_pi(door_angle + math.pi / 2)
        door_angle2 = self._wrap_to_pi(door_angle - math.pi / 2)
        door_area_angle = math.atan2(
            next_area_center.y - door_center.y, next_area_center.x - door_center.x)
        if(math.fabs(self._wrap_to_pi(door_area_angle - door_angle1)) < math.fabs(self._wrap_to_pi(door_area_angle - door_angle2))):
            return door_angle1
        else:
            return door_angle2

    def _wrap_to_pi(self, angle):
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        return angle

    def _get_door_points(self, door_geometry_points):
        sides = []
        for i, pt in enumerate(door_geometry_points):
            if i is not 0:
                pt1 = door_geometry_points[i - 1]
                pt2 = door_geometry_points[i]
                sides.append([pt1, pt2, self._compute_distance(pt1, pt2)])
        list.sort(sides, key=lambda l: l[2], reverse=True)
        pt1 = Point(x=(sides[0][0].x + sides[1][1].x) / 2.0,
                    y=(sides[0][0].y + sides[0][0].y) / 2.0)
        pt2 = Point(x=(sides[1][0].x + sides[0][1].x) / 2.0,
                    y=(sides[1][0].y + sides[0][1].y) / 2.0)
        return [pt1, pt2]

    def get_junction_orientation(self, junction, next_area):
        junction_center = self._compute_center(junction.geometry.points)
        next_area_center = self._compute_center(next_area.geometry.points)
        return math.atan2(next_area_center.y - junction_center.y, next_area_center.x - junction_center.x)

    def get_corridor_orientation(self, prev_area, curr_area, next_area):
        entry_pts = self._get_nearest_points(prev_area, curr_area)
        exit_pts = self._get_nearest_points(next_area, curr_area)
        entry_pt = self._compute_center(entry_pts)
        exit_pt = self._compute_center(exit_pts)
        return math.atan2(exit_pt.y - entry_pt.y, exit_pt.x - entry_pt.x)

    def _get_nearest_points(self, other_area, current_area):
        other_area_mid_point = self._compute_center(other_area.geometry.points)
        distances = []
        current_area_points = current_area.geometry.points
        for i, pt in enumerate(current_area_points):
            if i is not len(current_area_points) - 1:
                pt = current_area_points[i]
                distances.append(
                    [i, self._compute_distance(pt, other_area_mid_point)])
        list.sort(distances, key=lambda l: l[1], reverse=False)
        return [current_area_points[distances[0][0]], current_area_points[distances[1][0]]]

    def _compute_distance(self, pt1, pt2):
        return math.sqrt(math.pow(pt1.x - pt2.x, 2) + math.pow(pt1.y - pt2.y, 2))

    def _compute_center(self, area_geometry_pts):
        x_sum = 0
        y_sum = 0
        no_of_pts = 0
        for pt in area_geometry_pts:
            x_sum = x_sum + pt.x
            y_sum = y_sum + pt.y
            no_of_pts = no_of_pts + 1
        return Point(x=x_sum / no_of_pts, y=y_sum / no_of_pts)
