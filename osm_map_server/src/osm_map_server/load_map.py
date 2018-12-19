from osm_map_msgs.msg import *
import rospy
import geometry_msgs.msg

class LoadMap(object):

    def __init__(self, osm_bridge):
        self.osm_bridge = osm_bridge

    def load_geometric_map(self, req):
        ref = req.area_id or req.area_ref
        res = LoadGeometricMapResult()
        geom_map = GeometricMap()

        try:
            area_obj = self.osm_bridge.get_area(ref)
            for wall in area_obj.walls:
                geom_map.walls.append(self._convert_obl_shape_to_ros_polygon(wall.geometry))
            for doors in area_obj.doors:
                geom_map.doors.append(self._convert_obl_shape_to_ros_polygon(doors.geometry))
            for pillar in area_obj.pillars:
                geom_map.pillars.append(self._convert_obl_shape_to_ros_polygon(pillar.geometry))
            res.map = geom_map
            return res
        except Exception as e:
            rospy.logerr(str(e))
            return None

    def _convert_obl_shape_to_ros_polygon(self, obl_shape_obj):
        polygon = geometry_msgs.msg.Polygon()
        for point in obl_shape_obj.points:
            pt = geometry_msgs.msg.Point(x=point.x, y=point.y, z=0)
            polygon.points.append(pt)
        return polygon
