#!/usr/bin/env python

PACKAGE = 'osm_map_server'
NODE = 'load_geometric_map_client'

import rospy
from osm_map_msgs.msg import *
from actionlib import SimpleActionClient

class LoadGeometricMapClient(object):

    def __init__(self):
        SERVER = "/load_geometric_map"
        self.client = SimpleActionClient(SERVER, LoadGeometricMapAction)
        connected = self.client.wait_for_server()
        rospy.loginfo("Connected to load geometric map server")

        req = LoadGeometricMapGoal(area_ref='BRSU_C_L0_RoomC022')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        
        rospy.signal_shutdown("Load geometric map test complete")

    def done_cb(self, status, result):

        try:
            assert(len(result.map.walls) == 5)
            assert(len(result.map.doors) == 2)
            assert(len(result.map.pillars) == 1)
            rospy.loginfo("Test passed")
        except Exception as e:
            rospy.logerr("Test failed")


if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = LoadGeometricMapClient()
    rospy.spin()
