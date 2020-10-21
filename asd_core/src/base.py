import rospy
import sys
import traceback
import time
import signal
import argparse
import yaml
from timeit import default_timer as dt
import numpy as np
import concurrent.futures

# Import internals
from internal.sector_service import SectorService
from internal.driver import Driver, DriverStatus
from internal.prompt_handler import PromptHandler

# Import types
from nav_msgs.msg import Odometry


def route(payload):
    SectorService_Instance.update_pose(payload.pose)
    Driver_Instance.update_pose(payload.pose)


prev_cancel = 0


def signal_handler(sig, frame):
    global prev_cancel
    if abs(prev_cancel - dt()) < 1:
        rospy.logfatal("User halted the program!")
        exit(1)
    else:
        rospy.logerr("User cancelled the task!")
        Driver_Instance.halt()
    prev_cancel = dt()


def main():
    global SectorService_Instance, Driver_Instance

    # System Initialization
    rospy.init_node("asd_core")
    signal.signal(signal.SIGINT, signal_handler)
    config = yaml.load(open("./config/config.yaml"), Loader=yaml.FullLoader)

    # Initialize Modules
    SectorService_Instance = SectorService()
    Driver_Instance = Driver()
    PH = PromptHandler(Driver_Instance)

    # Subscribe to relevant topics and services
    rospy.Subscriber(config["odometry_topic"], Odometry, route)

    # Wait for elevation map to be available
    rospy.loginfo("Waiting for topics to become online...")
    rospy.wait_for_service("/elevation_mapping/get_submap", timeout=60)
    rospy.wait_for_message(config["odometry_topic"], Odometry, timeout=10)
    rospy.loginfo("OK!")

    try:
        # ROS Loop
        map_count = 0
        error_count = 0
        loop = 0
        last_time = 60
        rospy.loginfo("Running the control loop")
        while not rospy.core.is_shutdown():
            loop += 1
            """
            This is the main loop. Control sequence will live here. 
            Every step is clearly documented with their purpose.
            """
            if (
                Driver_Instance.is_target_reached()
                or Driver_Instance.flag == DriverStatus.HALT
            ):
                # x, y = PH.query()
                Driver_Instance.flag = DriverStatus.NORMAL
            else:
                rospy.loginfo(
                    "{:.1f}% completed".format(Driver_Instance.percent_complete())
                )

            SectorService_Instance.update_terrain()
            SectorService_Instance.start_alpha_thread(Driver_Instance.get_current_loc())

            # TODO: Find out which direction we want to go and Find optimum direction via LCP

            rospy.rostime.wallsleep(0.5)
    except (Exception, rospy.ROSException, KeyboardInterrupt):
        exc_type, exc_value, exc_traceback = sys.exc_info()
        rospy.logfatal("Program crashed or halted")
        traceback.print_exception(
            exc_type, exc_value, exc_traceback, limit=2, file=sys.stdout
        )
        rospy.core.signal_shutdown("exited")
        exit(1)


if __name__ == "__main__":
    main()
