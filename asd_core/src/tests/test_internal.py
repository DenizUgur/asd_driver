import rospy
import unittest
import numpy as np

from internal.sector_service import SectorService
from grid_map_msgs.srv import GetGridMap


class TestSectorService(unittest.TestCase):
    # * Tests elevation_mapping connection
    def test_em_connection(self):
        try:
            self.get_submap = rospy.ServiceProxy(
                "/elevation_mapping/get_submap", GetGridMap
            )
            payload = self.get_submap("odom", 0, 0, 7, 7, ["elevation"],).map
            self.assertIsNotNone(payload)
        except Exception:
            self.fail("elevation_mapping connection problem")


if __name__ == "__main__":
    rospy.init_node("unittest")
    unittest.main()
