#!/usr/bin/env python

PACKAGE = 'osm_low_level_planner'
NODE = 'osm_low_level_planner_node'

from OBL import OSMBridge, PathPlanner
import rospy
from actionlib import SimpleActionServer 
from osm_low_level_planner.msg import *
from osm_low_level_planner.osm_low_level_planner_callback import OSMLowLevelPlannerCallback


class OSMLowLevelPlannerNode(object):

    def __init__(self):
        server_ip = rospy.get_param('~overpass_server_ip')
        server_port = rospy.get_param('~overpass_server_port')
        ref_lat = rospy.get_param('~ref_latitude')
        ref_lon = rospy.get_param('~ref_longitude')
        building = rospy.get_param('~building')
        global_origin = [ref_lat, ref_lon]

        rospy.loginfo("Server " + server_ip + ":" + str(server_port))
        rospy.loginfo("Global origin: " + str(global_origin))
        rospy.loginfo("Starting servers...")

        self.osm_low_level_planner_server = SimpleActionServer('/osm_low_level_planner', OSMLowLevelPlannerAction, self._osm_low_level_planner, False)
        self.osm_low_level_planner_server.start()

        osm_bridge = OSMBridge(
                server_ip=server_ip,
                server_port=server_port,
                global_origin=global_origin,
                coordinate_system="cartesian",
                debug=False)
        path_planner = PathPlanner(osm_bridge)
        path_planner.set_building(building)
        self.osm_low_level_planner_callback = OSMLowLevelPlannerCallback(osm_bridge, path_planner)

        rospy.loginfo("OSM low level server started. Listening for requests...")


    def _osm_low_level_planner(self, req):
        res = self.osm_low_level_planner_callback.get_safe_response(req)
        if res is not None:
            self.osm_low_level_planner_server.set_succeeded(res)
        else:
            self.osm_low_level_planner_server.set_aborted(res) 


if __name__ == "__main__":
    rospy.init_node(NODE)
    osm_low_level_planner_node = OSMLowLevelPlannerNode()
    rospy.spin()
