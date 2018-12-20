#!/usr/bin/env python

PACKAGE = 'osm_map_server'
NODE = 'load_semantic_map_client'

import rospy
from osm_map_msgs.msg import *
from actionlib import SimpleActionClient

class LoadSemanticMapClient(object):

    def __init__(self):
        SERVER = "/load_semantic_map"
        self.client = SimpleActionClient(SERVER, LoadSemanticMapAction)
        connected = self.client.wait_for_server()
        rospy.loginfo("Connected to load semantic map server")

        req = LoadSemanticMapGoal(area_ref='BRSU_C_L0_RoomC022')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        rospy.signal_shutdown("Load semantic map test complete")

    def done_cb(self, status, result):

        try:
            assert(len(result.map.wall_sides) == 5)
            assert(len(result.map.door_sides) == 2)
            assert(len(result.map.pillars) == 1)
            rospy.loginfo("Test passed")
        except Exception as e:
            rospy.logerr("Test failed")


if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = LoadSemanticMapClient()
    rospy.spin()
