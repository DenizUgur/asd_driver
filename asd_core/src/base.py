from platform import dist

from numpy.lib.function_base import angle
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
import math

# Import internals
from internal.sector_service import SectorService
from internal.driver import Driver, DriverStatus
from internal.prompt_handler import PromptHandler
from helpers.algorithms import custom_search_select

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


def main(args):
    global SectorService_Instance, Driver_Instance

    # System Initialization
    rospy.init_node("asd_core")
    signal.signal(signal.SIGINT, signal_handler)
    config = yaml.load(open("./config/config.yaml"), Loader=yaml.FullLoader)

    # Initialize Modules
    SectorService_Instance = SectorService()
    Driver_Instance = Driver(gainL=args.GAIN_L, gainA=args.GAIN_A)
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
        loop = 0
        x, y = 0, 0
        rospy.loginfo("Running the control loop")
        while not rospy.core.is_shutdown():
            loop += 1
            """
            This is the main loop. Control sequence will live here. 
            Every step is clearly documented with their purpose.
            """
            if Driver_Instance.flag == DriverStatus.HALT:
                x, y = PH.query()
                Driver_Instance.flag = DriverStatus.NORMAL

            start = dt()
            SectorService_Instance.update_terrain()
            px, py, _ = Driver_Instance.get_current_loc()
            costmap = -SectorService_Instance.start_alpha_thread((px, py))

            XMI, _, YMI, _ = SectorService_Instance.get_extent()
            h, w = costmap.shape
            distance_map = np.fromfunction(
                lambda iy, ix: (
                    ((XMI + ix * 0.05) - x) ** 2 + ((YMI + iy * 0.05) - y) ** 2
                )
                ** 0.5,
                (h, w),
                dtype=float,
            )
            distance_map = SectorService_Instance.scale(distance_map, out_range=(0, 1))

            angles = np.linspace(0, 2 * math.pi, retstep=0.1)[0]
            R = 1.0
            perimiter = np.vstack((R * np.cos(angles) + px, R * np.sin(angles) + py)).T
            perimiterR = np.vstack(
                (R * 20 * np.cos(angles) + (h // 2), R * 20 * np.sin(angles) + (w // 2))
            ).T.astype(int)

            #* Plot
            if False:
                from matplotlib import pyplot as plt

                fig, ((ax1, ax2, ax3)) = plt.subplots(
                    nrows=1, ncols=3, sharex=True, sharey=True, figsize=(16, 5)
                )

                fusedmap = args.GAIN_C * costmap + args.GAIN_D * distance_map

                ax1.contourf(distance_map, levels=100)
                ax2.contourf(costmap, levels=100)
                ax3.contourf(fusedmap, levels=100)
                ax3.scatter(perimiterR[:, 0], perimiterR[:, 1], c="black")

                values = []
                for p, pr in zip(perimiter, perimiterR):
                    val = fusedmap[pr[1], pr[0]]
                    if np.isnan(val):
                        continue
                    values.append([pr, val])

                target = sorted(values, key=lambda k: k[1])[0][0]
                ax3.scatter(target[0], target[1], c="red")

                fig.tight_layout()
                plt.show()
                exit(1)

            if costmap is None:
                rospy.logerr("Map error :: moving slowly forward")
                Driver_Instance.move(dl=2.0, da=0)
                continue

            values = []
            fusedmap = args.GAIN_C * costmap + args.GAIN_D * distance_map
            for p, pr in zip(perimiter, perimiterR):
                val = fusedmap[pr[1], pr[0]]
                if np.isnan(val):
                    continue
                values.append([p, val])

            target = sorted(values, key=lambda k: k[1])[0][0]

            if target is None:
                rospy.logerr("No viable target found moving slowly forward")
                Driver_Instance.move(dl=2.0, da=0)
            else:
                rospy.logwarn(
                    "\nx={:.2f} :: y={:.2f} :: in {:.3f} sec\n".format(
                        *target, dt() - start
                    )
                )
                Driver_Instance.go_to_position(*target)

            if math.sqrt((px - x) ** 2 + (py - y) ** 2) < 1:
                Driver_Instance.halt()

            rospy.rostime.wallsleep(0.1)
    except (Exception, rospy.ROSException, KeyboardInterrupt):
        exc_type, exc_value, exc_traceback = sys.exc_info()
        rospy.logfatal("Program crashed or halted")
        traceback.print_exception(
            exc_type, exc_value, exc_traceback, limit=2, file=sys.stdout
        )
        rospy.core.signal_shutdown("exited")
        exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ASD Driver")
    parser.add_argument("--GAIN_A", type=float)
    parser.add_argument("--GAIN_L", type=float)
    parser.add_argument("--GAIN_C", type=float)
    parser.add_argument("--GAIN_D", type=float)
    main(parser.parse_args())
