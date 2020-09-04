import unittest
import numpy as np

from sdpath.lcp import LCP


class TestLCP(unittest.TestCase):
    # * Tests, R connection
    def test_calculation(self):
        self.lcp = LCP()
        self.lcp.process_dem(np.zeros((50, 50)))
        self.lcp.set_points((0, 0), (30, 30))
        path = self.lcp.calculate()
        self.assertIsInstance(path, np.ndarray)


if __name__ == "__main__":
    unittest.main()
