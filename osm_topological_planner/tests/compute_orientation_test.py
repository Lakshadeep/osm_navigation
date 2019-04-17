import unittest
from OBL import OSMBridge
from osm_topological_planner.compute_orientation import ComputeOrientation


class TestComputeOrientation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        self.osm_bridge = OSMBridge(server_ip='172.16.1.101', server_port=8000, global_origin=[
                                    50.7800401, 7.18226], coordinate_system='cartesian')
        self.compute_orientation = ComputeOrientation()

    def tearDown(self):
        pass

    def test_compute_center(self):
        corridor = self.osm_bridge.get_corridor('BRSU_C_L0_C9')
        pt = self.compute_orientation._compute_center(corridor.geometry.points)
        print(pt)

    def test_get_nearest_points(self):
        prev_area = self.osm_bridge.get_corridor('BRSU_C_L0_C5')
        curr_area = self.osm_bridge.get_corridor('BRSU_C_L0_C6')
        pts = self.compute_orientation._get_nearest_points(
            prev_area, curr_area)
        print(pts)

    def test_compute_corridor_orientation(self):
        prev_area = self.osm_bridge.get_corridor('BRSU_C_L0_C5')
        curr_area = self.osm_bridge.get_corridor('BRSU_C_L0_C6')
        next_area = self.osm_bridge.get_corridor('BRSU_C_L0_C7')
        angle = self.compute_orientation.get_corridor_orientation(
            prev_area, curr_area, next_area)
        print(angle)

    def test_get_door_points(self):
        door = self.osm_bridge.get_door('BRSU_C_L0_RoomC022_Door1')
        pts = self.compute_orientation._get_door_points(door.geometry.points)
        print(pts)

    def test_get_door_orientation1(self):
        door = self.osm_bridge.get_door('BRSU_C_L0_RoomC022_Door1')
        corridor = self.osm_bridge.get_corridor('BRSU_C_L0_C9')
        angle = self.compute_orientation.get_door_orientation(door, corridor)
        print(angle)

    def test_get_door_orientation2(self):
        door = self.osm_bridge.get_door('BRSU_C_L0_RoomC022_Door1')
        room = self.osm_bridge.get_room('BRSU_C_L0_RoomC022')
        angle = self.compute_orientation.get_door_orientation(door, room)
        print(angle)


if __name__ == '__main__':
    unittest.main()
