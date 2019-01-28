from osm_map_msgs.msg import *
from osm_map_msgs.srv import *
import rospy
import geometry_msgs.msg
from osm_map_server.obl_to_ros_adapter import OBLToROSAdapter 
from OBL.local_area_finder import LocalAreaFinder
from OBL.structs.wm.shape import Shape
import math
from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon

class Get2DMap(object):

    def __init__(self, osm_bridge, osm_adapter, semantic_features_finder):
        self.osm_bridge = osm_bridge
        self.osm_adapter = osm_adapter
        self.semantic_features_finder = semantic_features_finder
        self.local_area_finder = LocalAreaFinder(self.osm_bridge)
        self.cached_floor_number = None
        self.is_wall_cache_available = False
        self.wall_shapes_cached = []
        self.is_pillar_cache_available = False
        self.pillar_shapes_cached = []
        self.is_door_cache_available = False
        self.door_shapes_cached = []
    

    def get_geometric_map(self, req):
        area_ref = req.area_id or req.area_ref
        floor_number = req.level
        res = GetGeometricMapResponse()
        geom_map = GeometricMap()
        geom_map.header.frame_id = "/map"
        geom_map.header.stamp = rospy.get_rostime()
        try:
            walls = self.extract_walls(area_ref, floor_number)
            for wall in walls or []:
                geom_map.walls.append(OBLToROSAdapter.convert_obl_shape_obj_to_ros_polygon_msg(wall))
            pillars = self.extract_pillars(area_ref, floor_number)
            for pillar in pillars or []:
                geom_map.pillars.append(OBLToROSAdapter.convert_obl_shape_obj_to_ros_polygon_msg(pillar))
            doors = self.extract_doors(area_ref, floor_number)
            for door in doors or []:
                geom_map.doors.append(OBLToROSAdapter.convert_obl_shape_obj_to_ros_polygon_msg(door))
            res.map = geom_map
            return res
        except Exception as e:
            rospy.logerr(str(e))
            return None

    def get_semantic_map(self, req):
        area_ref = req.area_id or req.area_ref
        floor_number = req.level
        res = GetSemanticMapResponse()
        semantic_map = SemanticMap()
        semantic_map.header.frame_id = "/map"
        semantic_map.header.stamp = rospy.get_rostime()
        try:
            wall_sides = self.extract_wall_sides(area_ref, floor_number)
            for wall_side in wall_sides or []:
                semantic_map.wall_sides.append(OBLToROSAdapter.convert_obl_side_obj_to_ros_side_msg(wall_side))
            door_sides = self.extract_door_sides(area_ref, floor_number)
            for door_side in door_sides or []:
                semantic_map.door_sides.append(OBLToROSAdapter.convert_obl_side_obj_to_ros_side_msg(door_side))
            pillars = self.extract_pillars(area_ref, floor_number)
            for pillar in pillars or []:
                semantic_map.pillars.append(OBLToROSAdapter.convert_obl_pillar_obj_to_ros_pillar_msg(pillar))
            res.map = semantic_map
            return res
        except Exception as e:
            rospy.logerr(str(e))
            return None

    def extract_walls(self, area_ref, floor_number):
        __, wall_shapes,__ = self.osm_adapter.search_by_tag(data_type='way', key_val_dict={'level': str(floor_number),'indoor':'wall'})
        area = self.osm_bridge.get_area(area_ref) 
        area_geometry = area.geometry
        area_geometry_cached = area_geometry.points

        if self.is_wall_cache_available:
            if self.cached_floor_number != floor_number:
                self.wall_shapes_cached = []
                for wall_shape in wall_shapes:
                    shp = Shape(wall_shape.id)
                    self.wall_shapes_cached.append(shp.points)
                self.is_wall_cache_available = True
                self.cached_floor_number = floor_number
        else:
            self.wall_shapes_cached = []
            for wall_shape in wall_shapes:
                shp = Shape(wall_shape.id)
                self.wall_shapes_cached.append(shp.points)
            self.is_wall_cache_available = True
            self.cached_floor_number = floor_number

        walls = []
        for i,wall in enumerate(self.wall_shapes_cached):
            is_added = False
            for area_node in area_geometry_cached:
                if self._is_point_inside_polygon(area_node, wall):
                    walls.append(wall)
                    is_added = True
                    break
            if is_added == False:
                for pt in wall:
                    if self._is_point_inside_polygon(pt, area_geometry_cached):
                        walls.append(wall)
                        break
        return walls

    def extract_pillars(self, area_ref, floor_number):
        __, pillar_shapes,__ = self.osm_adapter.search_by_tag(data_type='way', key_val_dict={'level': str(floor_number),'indoor':'pillar'})
        area = self.osm_bridge.get_area(area_ref) 
        area_geometry = area.geometry
        area_geometry_cached = area_geometry.points

        if self.is_pillar_cache_available:
            if self.cached_floor_number != floor_number:
                self.pillar_shapes_cached = []
                for pillar_shape in pillar_shapes:
                    shp = Shape(pillar_shape.id)
                    self.pillar_shapes_cached.append(shp.points)
                self.is_pillar_cache_available = True
                self.cached_floor_number = floor_number
        else:
            self.pillar_shapes_cached = []
            for pillar_shape in pillar_shapes:
                shp = Shape(pillar_shape.id)
                self.pillar_shapes_cached.append(shp.points)
            self.is_pillar_cache_available = True
            self.cached_floor_number = floor_number

        pillars = []
        for i,pillar in enumerate(self.pillar_shapes_cached):
            is_added = False
            for area_node in area_geometry_cached:
                if self._is_point_inside_polygon(area_node, pillar):
                    pillars.append(pillar)
                    is_added = True
                    break
            if is_added == False:
                for pt in pillar:
                    if self._is_point_inside_polygon(pt, area_geometry_cached):
                        pillars.append(pillar)
                        break
        return pillars

    def extract_doors(self, area_ref, floor_number):
        __, door_shapes,__ = self.osm_adapter.search_by_tag(data_type='way', key_val_dict={'level': str(floor_number),'indoor':'door'})
        area = self.osm_bridge.get_area(area_ref) 
        area_geometry = area.geometry
        area_geometry_cached = area_geometry.points

        if self.is_door_cache_available:
            if self.cached_floor_number != floor_number:
                self.door_shapes_cached = []
                for door_shape in door_shapes:
                    shp = Shape(door_shape.id)
                    self.door_shapes_cached.append(shp.points)
                self.is_door_cache_available = True
                self.cached_floor_number = floor_number
        else:
            self.door_shapes_cached = []
            for door_shape in door_shapes:
                shp = Shape(door_shape.id)
                self.door_shapes_cached.append(shp.points)
            self.is_door_cache_available = True
            self.cached_floor_number = floor_number

        doors = []
        for i,door in enumerate(self.door_shapes_cached):
            is_added = False
            for area_node in area_geometry_cached:
                if self._is_point_inside_polygon(area_node, door):
                    doors.append(door)
                    is_added = True
                    break
            if is_added == False:
                for pt in door:
                    if self._is_point_inside_polygon(pt, area_geometry_cached):
                        doors.append(door)
                        break
        return doors

    def extract_wall_sides(self, area_ref, floor_number):
        walls = self.extract_walls(area_ref, floor_number)
        area = self.osm_bridge.get_area(area_ref) 
        area_geometry = area.geometry
        area_geometry_cached = area_geometry.points
        sides = []
        for wall_pts in walls:
            for i, pt in enumerate(wall_pts):
                if i == 1:
                    pass
                sides.append([wall_pts[i-1],wall_pts[i]])
        visible_sides = self._get_visible_sides(sides, area_geometry_cached)
        return visible_sides

    def extract_door_sides(self, area_ref, floor_number):
        doors = self.extract_doors(area_ref, floor_number)
        area = self.osm_bridge.get_area(area_ref) 
        area_geometry = area.geometry
        area_geometry_cached = area_geometry.points
        sides = []
        for door_pts in doors:
            for i, pt in enumerate(door_pts):
                if i == 1:
                    pass
                sides.append([door_pts[i-1],door_pts[i]])
        visible_sides = self._get_visible_sides(sides, area_geometry_cached)
        return visible_sides


    def _get_visible_sides(self, sides, area_geometry_points):
        visible_sides = []
        for side in sides:
            if self._check_if_side_is_visible(side, area_geometry_points):
                visible_sides.append(side)
        return visible_sides

    def _check_if_side_is_visible(self, side, area_geometry_points):
        is_corner1_inside = self.local_area_finder.is_inside_polygon(side[0].x,side[0].y, area_geometry_points)
        is_corner2_inside = self.local_area_finder.is_inside_polygon(side[1].x,side[1].y, area_geometry_points)
        if is_corner1_inside and is_corner2_inside:
            # if both corners are inside the area then side is definitely visible from given area from some angle
            return True
        elif (is_corner1_inside and ~is_corner2_inside) or (~is_corner1_inside and is_corner2_inside):
            # if 1 side corner is inside the geometry
            side_line = self._line(side[0], side[1]) 
            area_lines = self._get_area_lines(area_geometry_points)
            # check if side intersects with area geometry
            for area_line in area_lines:
                res = self._check_intersection(side_line, area_line)
                # if yes decide based on how much of side length lies inside the area geometry
                if res:
                    dist1 = math.sqrt((side[0].x - res.x)**2 + (side[0].y-res.y)**2)
                    dist2 = math.sqrt((res.x - side[1].x)**2 + (res.y-side[1].y)**2)
                    if (is_corner1_inside and dist1 > 0.3) or (is_corner2_inside and dist2 > 0.3):
                        if self._calculate_distance(side[0], side[1]) > 0.3:
                            return True
            return False
        elif (~is_corner1_inside and ~is_corner2_inside):
            # if both side corners are outside the area geometry then it should intersect 2 area geometry lines to reach the 
            side_line = self._line(side[0], side[1]) 
            area_lines = self._get_area_lines(area_geometry_points)
            intersection_count = 0
            for area_line in area_lines:
                res = self._check_intersection(side_line, area_line)
                intersection_count = intersection_count + 1 if res else intersection_count
            if intersection_count >= 2:
                return True
        return False

    def _calculate_distance(self, pt1, pt2):
        return math.sqrt((pt1.x - pt2.x)**2 + (pt1.y - pt2.y)**2)

    # Source: https://stackoverflow.com/questions/20677795/
    def _line(self, p1, p2):
        A = (p1.y - p2.y)
        B = (p2.x - p1.x)
        C = (p1.x*p2.y - p2.x*p1.y)
        return A, B, -C, p1, p2

    def _check_intersection(self, L1, L2):
        line_a = LineString([(L1[3].x, L1[3].y), (L1[4].x, L1[4].y)])
        line_b = LineString([(L2[3].x, L2[3].y), (L2[4].x, L2[4].y)])
        intersection_pt = line_a.intersection(line_b)

        if intersection_pt:
            return Point(intersection_pt.x, intersection_pt.y)
        else:
            return False 


    def _get_area_lines(self, area_geometry_points):
        total_points = len(area_geometry_points)
        lines = []
        for i, pt in enumerate(area_geometry_points):
            if i == 0:
                pass
            else:
                lines.append(self._line(area_geometry_points[i-1], pt))
        return lines

    def _is_point_inside_polygon(self, pt, shape_points):
        point = Point(pt.x, pt.y)
        points = []
        for shape_point in shape_points:
            points.append((shape_point.x, shape_point.y))
        polygon =  Polygon(points)
        return polygon.contains(point)


