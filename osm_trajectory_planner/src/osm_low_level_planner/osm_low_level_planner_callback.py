from osm_low_level_planner.msg import *
import rospy
import math
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class OSMLowLevelPlannerCallback(object):

    """callback for wm query server"""

    def __init__(self, osm_bridge, path_planner):
        self.osm_bridge = osm_bridge
        self.path_planner = path_planner

    def get_safe_response(self, req):
        try:
            res = self._get_response(req)
            return res
        except Exception as e:
            rospy.logerr(str(e))
            return None

    def _get_response(self, req):
        '''Access the fields of goal of PathPlanner.action message and call
        OSM Bridge Path planner with those parameters and return the response to the sender.
        '''
        start_floor = req.start_floor
        destination_floor = req.destination_floor
        start_area = req.start_area
        destination_area = req.destination_area
        start_local_area = req.start_local_area
        destination_local_area = req.destination_local_area
        start_position = [req.start_position.x, req.start_position.y]
        destination_task = req.destination_task

        path = []

        if start_local_area and destination_local_area:
            path = self.path_planner.get_path_plan(start_floor=start_floor, destination_floor=destination_floor,start_area=start_area,destination_area=destination_area, start_local_area=start_local_area,destination_local_area=destination_local_area)
        elif start_local_area and destination_task:
            path = self.path_planner.get_path_plan(start_floor=start_floor, destination_floor=destination_floor,start_area=start_area,destination_area=destination_area, start_local_area=start_local_area,destination_task=destination_task)
        elif start_position and destination_task:
            path = self.path_planner.get_path_plan(start_floor=start_floor, destination_floor=destination_floor,start_area=start_area,destination_area=destination_area, robot_position=start_position,destination_task=destination_task)
        elif start_position and destination_local_area:
            path = self.path_planner.get_path_plan(start_floor=start_floor, destination_floor=destination_floor,start_area=start_area,destination_area=destination_area, robot_position=start_position,destination_local_area=destination_local_area)
        else:
            rospy.logerr("Path planner need more arguments to plan the path")
  
        
        extracted_path = []
        for area in path:
            for nav_area in area.navigation_areas:
                extracted_path.append({'id':nav_area.id,'ref':str(nav_area.ref),'type':str(nav_area.type)})
            if area.exit_door is not None:
                extracted_path.append({'id':area.exit_door.id,'ref':str(area.exit_door.ref),'type':str(area.exit_door.type)})

        return self._plan_route(extracted_path)

    def _plan_route(self, path):
        is_first_pt = True

        last_pt = [0,0]
        last_orientation = 0

        res = OSMLowLevelPlannerResult()

        for pt in path:
            temp = None
            if pt['type'] == 'local_area':
                temp = self.osm_bridge.get_local_area(pt['id'])
            elif pt['type'] == 'door':
                temp = self.osm_bridge.get_local_area(pt['id'])
            
            if temp is not None:
                topology_node = temp.topology
                if is_first_pt:
                    is_first_pt = False
                else:
                    last_orientation = math.atan2(topology_node.y - last_pt[1], topology_node.x - last_pt[0])
                    p = Pose()
                    p.position = Point(x=last_pt[0],y=last_pt[1],z=0)
                    p.orientation = Quaternion(*quaternion_from_euler(0,0,last_orientation))      
                    res.waypoints.append(Waypoint(id=str(temp.id),ref=temp.ref,floor_number=temp.level,waypoint_pose=p))
                    # print(last_pt[0], last_pt[1], last_orientation*180/3.1457)

                last_pt = [topology_node.x, topology_node.y]
            p = Pose()
            p.position = Point(x=last_pt[0],y=last_pt[1],z=0)
            p.orientation = Quaternion(*quaternion_from_euler(0,0,last_orientation))    
            res.waypoints.append(Waypoint(id=str(temp.id),ref=temp.ref,floor_number=temp.level,waypoint_pose=p))
        return res

