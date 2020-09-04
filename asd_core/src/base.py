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
from sdpath.lcp import LCP

# Import types
from geometry_msgs.msg import PoseWithCovarianceStamped


def route(payload):
    SectorService_Instance.update_pose(payload.pose)
    Driver_Instance.update_pose(payload.pose)


def signal_handler(sig, frame):
    rospy.logerr("User cancelled the task!")
    Driver_Instance.halt()


def main(args):
    global SectorService_Instance, Driver_Instance

    # System Initialization
    rospy.init_node("asd_core")
    signal.signal(signal.SIGINT, signal_handler)
    config = yaml.load(open("./config.yaml"), Loader=yaml.FullLoader)

    # Initialize Modules
    SectorService_Instance = SectorService()
    Driver_Instance = Driver()
    LCP_Instance = LCP()
    LRS_Instance = LRS(Driver_Instance, clean=args.fresh)

    if not args.dryrun:
        Driver_Instance.halt()
        rospy.loginfo("Searching for the first landmark")
        while not LRS_Instance.has_landmarks() or LRS_Instance.M_TRANS is None:
            angular_speed = 0.7
            for _, m in LRS_Instance.dictionary.items():
                if m.id != -1 and m.visible_flag and m.accuracy() != 1:
                    angular_speed = 0.1
                    break

            direction = -1 if args.direction == "right" else 1
            Driver_Instance.move(dl=0, da=angular_speed * direction)
            rospy.rostime.wallsleep(0.2)
        rospy.loginfo("Search completed")
        Driver_Instance.halt()

    # Subscribe to relevant topics and services
    rospy.Subscriber(config["pose_with_covariance_topic"], PoseWithCovarianceStamped, route)

    # Wait for elevation map to be available
    rospy.loginfo("Waiting for topics to become online...")
    rospy.wait_for_service("/elevation_mapping/get_submap", timeout=60)
    rospy.wait_for_message(
        config["pose_with_covariance_topic"], PoseWithCovarianceStamped, timeout=10
    )
    rospy.loginfo("OK!")

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
                # TODO: Complete redesign (Confirmation, custom stdout etc.)
                rospy.loginfo(
                    "Current location: x={:.2f} y={:.2f}".format(
                        *Driver_Instance.get_current_loc(True)
                    )
                )
                while 1:
                    try:
                        print("Actions\n\tNW: New Waypoint\tE: Exit")
                        prompt = str(input("Write action: "))
                        if prompt == "E":
                            prompt = str(
                                input(
                                    "Are you sure? This action is irreversible! (write 'exit' to exit) "
                                )
                            )
                            if prompt == "exit":
                                raise Exception("User decided to exit.")

                        x, y = float(input("X=")), float(input("Y="))
                        Driver_Instance.flag = DriverStatus.RESUME
                        break
                    except ValueError as why:
                        rospy.logerr(repr(why))
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
                rospy.logerr("Error occured on local ap retrieval")
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
    parser.add_argument("--fresh", help="Do not use previous backup files", action="store_true")
    parser.add_argument("--dryrun", help="Do not start the control loop", action="store_true")
    parser.add_argument(
        "--direction",
        help="Choose starting search direction",
        choices=["left", "right"],
        default="left"
    )
    main(parser.parse_args())
