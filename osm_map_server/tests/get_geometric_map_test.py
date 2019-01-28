#!/usr/bin/env python

PACKAGE = 'osm_map_server'
NODE = 'load_geometric_map_client'

import rospy
from osm_map_msgs.msg import *
from osm_map_msgs.srv import *

class GetGeometricMapClient(object):

    def __init__(self):
        SERVER = "/get_geometric_map"
        rospy.wait_for_service(SERVER)
        rospy.loginfo("Geometric map server service available")

        self.client = rospy.ServiceProxy(SERVER, GetGeometricMap)
        
        res = self.client(area_refs=['BRSU_C_L0_RoomC022', 'BRSU_C_L0_C9'], level=0)
        # res = self.client(area_refs=['BRSU_A_E1'], is_elevator=True)

        try:
            assert(len(res.map.walls) == 5)
            assert(len(res.map.doors) == 2)
            assert(len(res.map.pillars) == 1)
            rospy.loginfo("Test passed")
        except Exception as e:
            rospy.logerr("Test failed")

        rospy.signal_shutdown("Get geometric map test complete")

  


if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = GetGeometricMapClient()
    rospy.spin()
