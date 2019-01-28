from osm_map_msgs.msg import *
from osm_map_msgs.srv import *
import rospy
import geometry_msgs.msg
from osm_map_server.obl_to_ros_adapter import OBLToROSAdapter 

class Get3DMap(object):

    def __init__(self, osm_bridge, semantic_features_finder):
        self.osm_bridge = osm_bridge
        self.semantic_features_finder = semantic_features_finder

    def get_geometric_map(self, req):
        area_refs = []
        if len(req.area_ids) > 0 and len(req.area_refs) > 0:
            rospy.logerr("Please either pass refs or ids. Both are not accepted!")
        elif len(req.area_ids) > 0:
            area_refs = req.area_ids
        elif len(req.area_refs) > 0:
            area_refs = req.area_refs
        res = GetGeometricMapResponse()
        geom_map = GeometricMap()
        geom_map.header.frame_id = "/map"
        geom_map.header.stamp = rospy.get_rostime()
        try:
            for area_ref in area_refs:
                area_obj = self.osm_bridge.get_area(area_ref)
                for wall in area_obj.walls or []:
                    geom_map.walls.append(OBLToROSAdapter.convert_obl_shape_obj_to_ros_polygon_msg(wall.geometry))
                for doors in area_obj.doors or []:
                    geom_map.doors.append(OBLToROSAdapter.convert_obl_shape_obj_to_ros_polygon_msg(doors.geometry))
                for pillar in area_obj.pillars or []:
                    geom_map.pillars.append(OBLToROSAdapter.convert_obl_shape_obj_to_ros_polygon_msg(pillar.geometry))
            res.map = geom_map
            return res
        except Exception as e:
            rospy.logerr(str(e))
            return None

    def get_semantic_map(self, req):
        area_refs = []
        if len(req.area_ids) > 0 and len(req.area_refs) > 0:
            rospy.logerr("Please either pass refs or ids. Both are not accepted!")
        elif len(req.area_ids) > 0:
            area_refs = req.area_ids
        elif len(req.area_refs) > 0:
            area_refs = req.area_refs
        res = GetSemanticMapResponse()
        semantic_map = SemanticMap()
        semantic_map.header.frame_id = "/map"
        semantic_map.header.stamp = rospy.get_rostime()
        try:
            for area_ref in area_refs:
                semantic_features = self.semantic_features_finder.get_features(area_ref)
                for wall_side in semantic_features.wall_sides or []:
                    semantic_map.wall_sides.append(OBLToROSAdapter.convert_obl_side_obj_to_ros_side_msg(wall_side))
                for door_side in semantic_features.door_sides:
                    semantic_map.door_sides.append(OBLToROSAdapter.convert_obl_side_obj_to_ros_side_msg(door_side))
                for pillar in semantic_features.pillars or []:
                    semantic_map.pillars.append(OBLToROSAdapter.convert_obl_pillar_obj_to_ros_pillar_msg(pillar))
                for feature in semantic_features.features or []:
                    semantic_map.features.append(OBLToROSAdapter.convert_obl_feature_obj_to_ros_feature_msg(feature))
            res.map = semantic_map
            return res
        except Exception as e:
            rospy.logerr(str(e))
            return None

