import rospy
import time
from matplotlib import pyplot as plt
from timeit import default_timer as dt
import numpy as np
import concurrent.futures
import curses

# Import internals
from internal.information_server import InformationServer
from internal.sector_service import SectorService

# Import types
from geometry_msgs.msg import PoseWithCovarianceStamped


def main(scr, *args):
    global InformationServer_Instance, SectorService_Instance

    rospy.init_node("asd_core")

    # Screen initialization
    curses.noecho()
    curses.cbreak()
    curses.curs_set(0)
    scr.keypad(True)
    scr.erase()

    # Initialize Information Server
    InformationServer_Instance = InformationServer()

    while not InformationServer_Instance.is_online():
        print("Waiting for information server to become available...")
        time.sleep(1)

    # Initialize Sector Server
    SectorService_Instance = SectorService()

    while not SectorService_Instance.is_online():
        print("Waiting for sector service to become available...")
        time.sleep(1)

    # Subscribe to relevant topics and services
    rospy.Subscriber(
        "/base_footprint_pose",
        PoseWithCovarianceStamped,
        lambda payload: SectorService_Instance.update_pose(payload.pose),
    )

    # Wait for elevation map to be available
    scr.addstr(0, 0, "Waiting for topics to be available...")
    try:
        rospy.wait_for_service("/elevation_mapping/get_submap", timeout=60)
        rospy.wait_for_message(
            "/base_footprint_pose", PoseWithCovarianceStamped, timeout=10
        )
    except rospy.ROSException as why:
        print(why)
        exit(5)

    scr.addstr(0, 38, "OK", curses.A_BOLD | curses.COLOR_GREEN)
    scr.addstr(1, 0, "Starting control loop in 2 seconds", curses.A_BOLD)
    scr.refresh()
    time.sleep(2)

    try:
        # ROS Loop
        while not rospy.core.is_shutdown():
            scr.erase()
            scr.addstr(5, 5, "Running the control loop", curses.A_BOLD)
            """
            This is the main loop. Control sequence will live here. 
            Every step is clearly documented with their purpose.
            """

            # TODO: Actually, first step should be generating initial map. Rotating rover 360 degrees

            """
            First step is to generate a local area.

            '/elevation_map' topic will serve us 10 m^2 of space around rover.
            With the help of odometry info, we will calculate missing information on terrain.
            Ray-tracing will be used for appliciable areas. Other areas will be either average of its surroundings
            or marked as unknown.
            """
            # * In this step we are checking if the elevation map at this time has any new infromation that hasn't been on our current route.
            # * Or maybe ray-traced areas have valid information now.
            start = dt()
            if SectorService_Instance.is_update_required():
                # InformationServer_Instance.update(
                #     {"sector_service": {"status": "updating"}}
                # )

                # TODO: We can implement spawning threads with different settings to increase crash recovery
                # * For example: If alpha thread dies consecutively for 3 times we can try to fall back and/or try with smaller size
                try:
                    with concurrent.futures.ThreadPoolExecutor() as executor:
                        future = executor.submit(
                            SectorService_Instance.start_alpha_thread
                        )
                        status, errors, result, original = future.result(timeout=7)
                except concurrent.futures.TimeoutError:
                    print("Error occured in alpha thread!")

                # InformationServer_Instance.update(
                #     {"sector_service": {"status": "updated", "sector_info": result}}
                # )
                end = dt()

            #! Remove this after testing
            if not result is None and np.max(result) < 1000:
                fig, (ax1, ax2) = plt.subplots(
                    ncols=2, sharex=True, sharey=True, figsize=(14, 6)
                )

                if int(np.nanmax(original) / 50) == 0:
                    dtm_o = ax1.contourf(original, cmap="terrain")
                else:
                    dtm_o = ax1.contourf(
                        original,
                        cmap="terrain",
                        levels=range(
                            int(np.nanmin(original)),
                            int(np.nanmax(original)),
                            int(np.nanmax(original) / 50),
                        ),
                    )

                if int(np.nanmax(result) / 50) == 0:
                    dtm = ax2.contourf(result, cmap="terrain")
                else:
                    dtm = ax2.contourf(
                        result,
                        cmap="terrain",
                        levels=range(
                            int(np.nanmin(result)),
                            int(np.nanmax(result)),
                            int(np.nanmax(result) / 50),
                        ),
                    )

                fig.suptitle("Took {:.2f} seconds".format(end - start), fontsize=14)
                plt.colorbar(dtm_o, ax=ax1)
                plt.colorbar(dtm, ax=ax2)
                fig.tight_layout()
                plt.show()
                scr.addstr(
                    0, 1, "Took {:.2f} seconds STATUS={}".format(end - start, status)
                )
                # if len(errors) > 0:
                #     print(errors)
            else:
                scr.addstr(1, 1, "Error occured!")

            """
            # TODO: Now that we have a guess of our surroundings, we can calculate a route. But that's for later.
            """

            scr.refresh()
            rospy.rostime.wallsleep(1.0)
    except KeyboardInterrupt:
        rospy.core.signal_shutdown("keyboard interrupt")


if __name__ == "__main__":
    curses.wrapper(main)
