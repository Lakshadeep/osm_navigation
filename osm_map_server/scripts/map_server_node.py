#!/usr/bin/env python

PACKAGE = 'map_server'
NODE = 'map_server_node'

from OBL import OSMBridge, SemanticFeaturesFinder
import rospy
from actionlib import SimpleActionServer 
from osm_map_msgs.msg import *

from osm_map_server.load_map import LoadMap

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
        self.load_map = LoadMap(self.osm_bridge, self.semantic_features_finder)

        self.load_geometric_map_server = SimpleActionServer('/load_geometric_map', LoadGeometricMapAction, self._load_geometric_map, False)
        self.load_geometric_map_server.start()
        rospy.loginfo("Load geometric map server started. Listening for queries...")

        self.load_semantic_map_server = SimpleActionServer('/load_semantic_map', LoadSemanticMapAction, self._load_semantic_map, False)
        self.load_semantic_map_server.start()
        rospy.loginfo("Load semantic map server started. Listening for queries...")
        
        self.geometric_map_pub = rospy.Publisher('/geometric_map', GeometricMap, queue_size=10, latch=True)
        self.semantic_map_pub = rospy.Publisher('/semantic_map', SemanticMap, queue_size=10, latch=True)

    def _load_geometric_map(self, req):
        res = self.load_map.load_geometric_map(req)
        if res is not None:
            self.load_geometric_map_server.set_succeeded(res)
            self.geometric_map_pub.publish(res.map)
        else:
            self.load_geometric_map_server.set_aborted(res)

    def _load_semantic_map(self, req):
        res = self.load_map.load_semantic_map(req)
        if res is not None:
            self.load_semantic_map_server.set_succeeded(res)
            self.semantic_map_pub.publish(res.map)
        else:
            self.load_semantic_map_server.set_aborted(res)



if __name__ == "__main__":
    rospy.init_node(NODE)
    map_server_node = MapServerNode()
    rospy.spin()
