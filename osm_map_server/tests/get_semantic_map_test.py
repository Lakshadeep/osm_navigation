#!/usr/bin/env python

PACKAGE = 'osm_map_server'
NODE = 'load_semantic_map_client'

import rospy
from osm_map_msgs.msg import *
from osm_map_msgs.srv import *
from actionlib import SimpleActionClient


class GetSemanticMapClient(object):

    def __init__(self):
        SERVER = "/get_semantic_map"
        rospy.wait_for_service(SERVER)
        rospy.loginfo("Semantic map server service available")

        self.client = rospy.ServiceProxy(SERVER, GetSemanticMap)

        res = self.client(
            area_refs=['BRSU_C_L0_RoomC022'], level=0, is_elevator=False)
        # res = self.client(area_refs=['BRSU_A_E1'], is_elevator=True)

        try:
            assert(len(res.map.wall_sides) == 5)
            assert(len(res.map.door_sides) == 2)
            assert(len(res.map.pillars) == 1)
            rospy.loginfo("Test passed")
        except Exception as e:
            rospy.logerr("Test failed")

        rospy.signal_shutdown("Load semantic map test complete")


if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = GetSemanticMapClient()
    rospy.spin()
