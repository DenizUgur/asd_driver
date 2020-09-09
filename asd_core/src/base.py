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
from internal.lrs import LRS
from internal.prompt_handler import PromptHandler
from sdpath.lcp import LCP

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
        SectorService_Instance.shutdown()
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
    LCP_Instance = LCP()
    Driver_Instance = Driver()
    LRS_Instance = LRS(Driver_Instance, clean=args.fresh)
    PH = PromptHandler(Driver_Instance)

    # Subscribe to relevant topics and services
    rospy.Subscriber(config["odometry_topic"], Odometry, route)

    # Wait for elevation map to be available
    rospy.loginfo("Waiting for topics to become online...")
    rospy.wait_for_service("/elevation_mapping/get_submap", timeout=60)
    rospy.wait_for_message(config["odometry_topic"], Odometry, timeout=10)
    rospy.loginfo("OK!")

    if not args.dryrun and args.fresh:
        Driver_Instance.halt()
        rospy.loginfo("Searching for the first landmark")

        # Kick start rover
        rospy.loginfo("Kick-starting the rover by 3 to 4.5 meters on +x axis (relative)")
        dx = 3
        while Driver_Instance.go_to_position(dx, 0) != DriverStatus.ACCEPTED:
            if dx > 4.5:
                break
            dx += 0.1

        rospy.loginfo("Waiting for maximum of 5 seconds to initialize Transformation Matrix")
        start = dt()
        while not LRS_Instance.has_landmarks() or LRS_Instance.M_TRANS is None:
            if dt() - start > 5:
                break

        while not LRS_Instance.has_landmarks() or LRS_Instance.M_TRANS is None:
            rospy.loginfo_once("Rotating around base_link axis to look for landmarks")
            angular_speed = config["ar_tags"]["search_vel_max"]
            for _, m in LRS_Instance.dictionary.items():
                if m.id != -1 and m.visible_flag and m.accuracy() != 1:
                    angular_speed = config["ar_tags"]["search_vel_min"]
                    break

            direction = -1 if args.direction == "right" else 1
            Driver_Instance.move(dl=0, da=angular_speed * direction)

            rospy.rostime.wallsleep(0.05)
        rospy.loginfo("Search completed")
        Driver_Instance.halt()

    if args.dryrun:
        time.sleep(2)
        rospy.loginfo("All modules initialized without a problem. Exitting...")
        SectorService_Instance.shutdown()
        exit(0)

    try:
        # ROS Loop
        map_count = 0
        error_count = 0
        loop = 0
        last_time = 10
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
                x, y = PH.query()
                Driver_Instance.flag = DriverStatus.RESUME
            else:
                rospy.loginfo(
                    "{:.1f}% completed".format(Driver_Instance.percent_complete())
                )

            """
            First step is to generate a local area.

            '/elevation_map' topic will serve us 7 m^2 of space around rover.
            With the help of odometry info, we will calculate missing information on terrain.
            Ray-tracing will be used for appliciable areas.
            """
            start = dt()
            try:
                SectorService_Instance.update_terrain()
            except Exception as why:
                rospy.logerr("Problem in updating the map")
                rospy.logerr(repr(why))
                continue

            try:
                with concurrent.futures.ThreadPoolExecutor() as executor:
                    future = executor.submit(SectorService_Instance.start_alpha_thread)
                    local_map = future.result(timeout=last_time + 2)
                    map_count += 1
                    last_time = dt() - start
            except concurrent.futures.TimeoutError:
                rospy.logerr("Problem in Alpha Thread")
                error_count += 1
                last_time = 10
                continue

            if not local_map is None and np.max(local_map) < 1000:
                if last_time > 2:
                    rospy.logwarn("Took {:.2f} seconds to ray trace".format(last_time))
                else:
                    rospy.loginfo("Took {:.2f} seconds to ray trace".format(last_time))
                rospy.loginfo("{:.1f}% successful".format(100 * map_count / (loop)))
            else:
                rospy.logerr("Error occured on local map retrieval")
                continue

            start = dt()
            try:
                LCP_Instance.process_dem(
                    np.flip(local_map, axis=0),
                    extent=SectorService_Instance.get_extent(),
                )
                LCP_Instance.set_points(
                    Driver_Instance.get_current_loc(), Driver_Instance.convert(x, y),
                )
                PATH = LCP_Instance.calculate()
            except Exception as why:
                rospy.logerr("An error occured on LCP!")
                rospy.logerr(repr(why))
                continue

            if dt() - start > 2:
                rospy.logwarn(
                    "Took {:.2f} seconds to plot the route".format(dt() - start)
                )
            else:
                rospy.loginfo(
                    "Took {:.2f} seconds to plot the route".format(dt() - start)
                )

            Driver_Instance.follow_path(PATH)

            rospy.loginfo("")
            rospy.rostime.wallsleep(0.5)
    except (Exception, rospy.ROSException, KeyboardInterrupt):
        exc_type, exc_value, exc_traceback = sys.exc_info()
        SectorService_Instance.shutdown()
        rospy.logfatal("Program crashed or halted")
        traceback.print_exception(
            exc_type, exc_value, exc_traceback, limit=2, file=sys.stdout
        )
        rospy.core.signal_shutdown("exited")
        exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ASD Driver")
    parser.add_argument(
        "--fresh", help="Do not use previous backup files", action="store_true"
    )
    parser.add_argument(
        "--dryrun", help="Do not start the control loop", action="store_true"
    )
    parser.add_argument(
        "--direction",
        help="Choose starting search direction",
        choices=["left", "right"],
        default="left",
    )
    main(parser.parse_args())
