import rospy
import time
import ray

# Import internals
from internal.information_server import InformationServer
from internal.sector_service import SectorService

# Import types
from geometry_msgs.msg import PoseWithCovarianceStamped

# ROS Routing
def route_pose(payload):
    SectorService_Instance.update_pose(payload.pose)


if __name__ == "__main__":
    global InformationServer_Instance, SectorService_Instance

    rospy.init_node("asd_core")

    # Initialize Ray (Thread Handler)
    ray.init()

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
    rospy.Subscriber("/base_footprint_pose", PoseWithCovarianceStamped, route_pose)

    # Wait for elevation map to be available
    print("Waiting for topics to be available...")
    try:
        rospy.wait_for_service("/elevation_mapping/get_submap", timeout=60)
        rospy.wait_for_message(
            "/base_footprint_pose", PoseWithCovarianceStamped, timeout=10
        )
    except rospy.ROSException as why:
        print(why)
        exit(5)

    print("ok")

    try:
        # ROS Loop
        while not rospy.core.is_shutdown():
            """
            This is the main loop. Control sequence will live here. 
            Every step is clearly documented with their purpose.
            """

            """
            First step is to generate a local area.

            '/elevation_map' topic will serve us 10 m^2 of space around rover.
            With the help of odometry info, we will calculate missing information on terrain.
            Ray-tracing will be used for appliciable areas. Other areas will be either average of its surroundings
            or marked as unknown.
            """
            # * In this step we are checking if the elevation map at this time has any new infromation that hasn't been on our current route.
            # * Or maybe ray-traced areas have valid information now.
            if SectorService_Instance.is_update_required():
                InformationServer_Instance.update(
                    {"sector_service": {"status": "updating"}}
                )

                result = SectorService_Instance.start_alpha_thread()

                InformationServer_Instance.update(
                    {"sector_service": {"status": "updated", "sector_info": result}}
                )

            rospy.rostime.wallsleep(1.0)
    except KeyboardInterrupt:
        rospy.core.signal_shutdown("keyboard interrupt")
