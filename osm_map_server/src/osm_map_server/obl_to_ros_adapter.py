from osm_map_msgs.msg import *
import geometry_msgs.msg

class OBLToROSAdapter(object):

    @staticmethod
    def convert_obl_pillar_obj_to_ros_pillar_msg(pillar_obj) :
        pillar = Pillar()
        if isinstance(pillar_obj, (list,)):
            points = pillar_obj
        else:
            points = pillar_obj.geometry
            pillar.id = pillar_obj.id
        pillar.point = OBLToROSAdapter.convert_obl_shape_obj_to_ros_point_msg(points) 
        return pillar

    @staticmethod
    def convert_obl_side_obj_to_ros_side_msg(side_obj) :
        side = Side()
        if isinstance(side_obj, (list,)):
            side.corners = [] 
            for corner in side_obj:
                side.corners.append(OBLToROSAdapter.convert_obl_point_obj_to_ros_point_msg(corner))
        else: 
            side.texture = str(side_obj.texture)
            side.paint = str(side_obj.paint)
            side.corners = [] 
            for corner in side_obj.corners:
                side.corners.append(OBLToROSAdapter.convert_obl_point_obj_to_ros_point_msg(corner))
            side.features = [] 
            for feature in side_obj.features or []:
                side.features.append(OBLToROSAdapter.convert_obl_feature_obj_to_ros_feature_msg(feature))
        return side

    @staticmethod
    def convert_obl_feature_obj_to_ros_feature_msg(feature_obj) :
        feature = osm_map_msgs.msg.Feature()
        feature.id = feature_obj.id
        #TOOD feature.type  
        feature.height = float(feature_obj.height)
        feature.width = float(feature_obj.width)
        feature.breast = float(feature_obj.breast)
        feature.position = OBLToROSAdapter.convert_obl_point_obj_to_ros_point_msg(feature_obj.point)
        return feature

    @staticmethod
    def convert_obl_point_obj_to_ros_point_msg(point_obj) :
        point = geometry_msgs.msg.Point()
        point.x = point_obj.x
        point.y = point_obj.y
        point.z = 0
        return point

    @staticmethod
    def convert_obl_shape_obj_to_ros_polygon_msg(shape_obj) :
        polygon = geometry_msgs.msg.Polygon()
        if isinstance(shape_obj, (list,)):
            points = shape_obj
        else:
            points = shape_obj.points
        
        for pt_obj in points:
            polygon.points.append(OBLToROSAdapter.convert_obl_point_obj_to_ros_point_msg(pt_obj))
        return polygon

    @staticmethod
    def convert_obl_shape_obj_to_ros_point_msg(shape_obj) :
        point = geometry_msgs.msg.Point()
        x_mid = 0.0
        y_mid = 0.0

        if isinstance(shape_obj, (list,)):
            points = shape_obj
        else:
            points = shape_obj.points

        for pt_obj in points:
            x_mid = x_mid + pt_obj.x
            y_mid = y_mid + pt_obj.y
        point.x = x_mid /len(points)
        point.y = y_mid /len(points)
        return point