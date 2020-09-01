import rospy
import time
import yaml
from matplotlib import pyplot as plt
from timeit import default_timer as dt
import numpy as np
import concurrent.futures

# Import internals
from internal.sector_service import SectorService
from internal.driver import Driver
from internal.lrs import LRS
from sdpath.lcp import LCP

# Import types
from geometry_msgs.msg import PoseWithCovarianceStamped


def route(payload):
    SectorService_Instance.update_pose(payload.pose)
    Driver_Instance.update_pose(payload.pose)


def main():
    global SectorService_Instance, Driver_Instance

    rospy.init_node("asd_core")
    config = yaml.load(open("./config.yaml"), Loader=yaml.FullLoader)

    # Initialize Modules
    SectorService_Instance = SectorService()
    Driver_Instance = Driver()
    LCP_Instance = LCP()
    LRS(Driver_Instance)

    while 1:
        # TODO: Find first landmark
        break

    # Subscribe to relevant topics and services
    rospy.Subscriber(config["pose_with_covariance"], PoseWithCovarianceStamped, route)

    # Wait for elevation map to be available
    rospy.loginfo("Waiting for topics to become online...")
    try:
        rospy.wait_for_service("/elevation_mapping/get_submap", timeout=60)
        rospy.wait_for_message(
            config["pose_with_covariance"], PoseWithCovarianceStamped, timeout=10
        )
        rospy.loginfo("OK!")
    except rospy.ROSException as why:
        rospy.logerr(repr(why))
        exit(5)

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
            if Driver_Instance.is_target_reached():
                rospy.loginfo(
                    "Current location: x={:.2f} y={:.2f} theta={:.0f}".format(
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

                        x, y, r = (
                            float(input("X=")),
                            float(input("Y=")),
                            float(input("R=")),
                        )
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
                rospy.logerr("Error occured!")
                continue

            start = dt()
            try:
                LCP_Instance.process_dem(
                    np.flip(local_map, axis=0),
                    extent=SectorService_Instance.get_extent(),
                )
                LCP_Instance.set_points(
                    Driver_Instance.get_current_loc(),
                    (x + Driver_Instance.cx, y + Driver_Instance.cy),
                )
                PATH, COST = LCP_Instance.calculate()
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

            Driver_Instance.follow_path(PATH, r)

            rospy.loginfo("")
            rospy.rostime.wallsleep(0.5)
    except (Exception, KeyboardInterrupt) as why:
        SectorService_Instance.shutdown()
        rospy.logfatal("Program crashed or halted")
        rospy.logfatal(repr(why))
        rospy.core.signal_shutdown("exited")


if __name__ == "__main__":
    main()
