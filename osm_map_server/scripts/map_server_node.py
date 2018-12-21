#!/usr/bin/env python

PACKAGE = 'map_server'
NODE = 'map_server_node'

from OBL import OSMBridge, SemanticFeaturesFinder
import rospy
from actionlib import SimpleActionServer 
from osm_map_msgs.msg import *
from osm_map_msgs.srv import *

from osm_map_server.get_map import GetMap

class MapServerNode(object):

    def __init__(self):
        server_ip = rospy.get_param('~overpass_server_ip')
        server_port = rospy.get_param('~overpass_server_port')
        ref_lat = rospy.get_param('~ref_latitude')
        ref_lon = rospy.get_param('~ref_longitude')
        global_origin = [ref_lat, ref_lon]

        rospy.loginfo("Server " + server_ip + ":" + str(server_port))
        rospy.loginfo("Global origin: " + str(global_origin))
        rospy.loginfo("Starting servers...")

        self.osm_bridge = OSMBridge(
                server_ip=server_ip,
                server_port=server_port,
                global_origin=global_origin,
                coordinate_system="cartesian",
                debug=False)

        self.semantic_features_finder = SemanticFeaturesFinder(self.osm_bridge)
        self.get_map = GetMap(self.osm_bridge, self.semantic_features_finder)

        self.get_geometric_map_server = rospy.Service('/get_geometric_map', GetGeometricMap, self._get_geometric_map)
        rospy.loginfo("Get geometric map server started. Listening for queries...")

        self.get_semantic_map_server = rospy.Service('/get_semantic_map', GetSemanticMap, self._get_semantic_map)
        rospy.loginfo("Get semantic map server started. Listening for queries...")
        
        self.geometric_map_pub = rospy.Publisher('/geometric_map', GeometricMap, queue_size=10, latch=True)
        self.semantic_map_pub = rospy.Publisher('/semantic_map', SemanticMap, queue_size=10, latch=True)

    def _get_geometric_map(self, req):
        res = self.get_map.get_geometric_map(req)
        self.geometric_map_pub.publish(res.map)
        return res

    def _get_semantic_map(self, req):
        res = self.get_map.get_semantic_map(req)
        self.semantic_map_pub.publish(res.map)
        return res




if __name__ == "__main__":
    rospy.init_node(NODE)
    map_server_node = MapServerNode()
    rospy.spin()
