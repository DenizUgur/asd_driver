"""Run all tests inside of test_*.py modules located in the same directory."""


import sys
import unittest


if __name__ == "__main__":
    test_suite = unittest.defaultTestLoader.discover("./tests", "test_*.py")
    test_runner = unittest.TextTestRunner(resultclass=unittest.TextTestResult)
    result = test_runner.run(test_suite)
    sys.exit(not result.wasSuccessful())
