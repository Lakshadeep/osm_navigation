from osm_planner_msgs.msg import *
import math
from geometry_msgs.msg import Pose, Point, Quaternion
# from OBL.structs.wm.point import Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class ComputeOrientation(object):

    def __init__(self):
        pass

    def get_corridor_orientation(self, prev_area, curr_area, next_area):
        entry_pts = self._get_nearest_points(prev_area, curr_area)
        exit_pts = self._get_nearest_points(next_area, curr_area)
        entry_pt =  self._compute_center(entry_pts)            
        exit_pt =  self._compute_center(exit_pts)
        return math.atan2(exit_pt.y - entry_pt.y, exit_pt.x - entry_pt.x)           

    def _get_nearest_points(self, other_area, current_area):
        other_area_mid_point = self._compute_center(other_area.geometry.points)
        distances = []
        current_area_points = current_area.geometry.points
        for i, pt in enumerate(current_area_points):
            if i is not len(current_area_points)-1:
                pt = current_area_points[i]
                distances.append([i, self._compute_distance(pt, other_area_mid_point)])
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
