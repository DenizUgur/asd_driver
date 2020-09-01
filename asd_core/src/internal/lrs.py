import rospy
import math
import yaml
import numpy as np
from threading import Thread

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseWithCovarianceStamped


class Marker:
    LIMIT = 50

    def __init__(self, m_id, known_location):
        self.id = m_id
        self.logged = False
        self.known_x, self.known_y = known_location
        self.observed_positions = []

    def observed(self, x, y):
        """
        Append to observed locations
        """
        if len(self.observed_positions) == self.LIMIT:
            self.observed_positions.pop(0)
        self.observed_positions.append([x, y])

    def kloc(self):
        """
        Return known location
        """
        return self.known_x, self.known_y

    def loc(self):
        """
        Return observed location
        """
        if len(self.observed_positions) == 0:
            return (0, None, None)
        med = np.median(self.observed_positions, axis=0)
        return self.accuracy(), med[0], med[1]

    def accuracy(self):
        return len(self.observed_positions) / self.LIMIT


class LRS:
    def __init__(self, driver):
        self.config = yaml.load(open("./config.yaml"), Loader=yaml.FullLoader)
        self.driver_instance = driver
        self.dictionary = dict()
        self.pose = None

        self.initial_loc = (
            self.config["inital_position"]["x"],
            self.config["inital_position"]["y"],
        )
        self.origin_mcp_to_rcp = None
        self.init_state = True
        self.visible_landmarks = 0

        landmarks = yaml.load(open("./landmarks.yaml"), Loader=yaml.FullLoader)
        for val in landmarks["landmarks"]:
            m_id, x, y = val.values()
            self.dictionary[m_id] = Marker(m_id, (x, y))

        self.worker = Thread(name="LRS_worker", target=self.daemon_worker)
        self.worker.setDaemon(True)
        self.worker.start()

    def daemon_worker(self):
        try:
            rospy.Subscriber(
                "/ar_pose_marker", AlvarMarkers, lambda p: self.commit(p.markers),
            )
            rospy.Subscriber(
                self.config["pose_with_covariance"],
                PoseWithCovarianceStamped,
                self.update_pose,
            )

            rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)
            rospy.wait_for_message(
                self.config["pose_with_covariance"], PoseWithCovarianceStamped
            )

            # Save starting point and rcp pose for that
            self.origin_mcp_to_rcp = (
                self.initial_loc[0],
                self.initial_loc[1],
                self.pose[0],
                self.pose[1],
            )

            while not rospy.core.is_shutdown():
                rcp_angle_estimates = []
                try:
                    if self.has_landmarks():
                        px, py, pr = self.pose
                        for m_id, m in self.dictionary.items():
                            a, x, y = m.loc()
                            if a == 1 and not m.logged:
                                rospy.loginfo(
                                    "Locked on Marker {} -> ({}, {})".format(m_id, x, y)
                                )
                                m.logged = True

                            # * Calculate Rotation Matrix
                            # TODO: Do it for every marker and get median of it
                            # Marsyard CP angle
                            # mx, my = m.kloc()
                            # kx, ky = self.initial_loc
                            # mcp_angle = math.atan2(my - ky, mx - kx)
                            # RTAB CP angle

                    rospy.rostime.wallsleep(0.01)
                except Exception as why:
                    rospy.logerr("Error in LRS... Recovering...")
                    rospy.logerr(repr(why))
        except Exception as why:
            rospy.logfatal("Landmark Recognition System halted!")

    def update_pose(self, payload):
        x = payload.pose.pose.position.x
        y = payload.pose.pose.position.y
        r = math.acos(payload.pose.pose.orientation.w) * 2
        self.pose = (x, y, r)

    def commit(self, markers):
        self.visible_landmarks = len(markers)
        for marker in markers:
            m_id = marker.id
            x = marker.pose.pose.position.x
            y = marker.pose.pose.position.y
            if m_id in self.dictionary:
                self.dictionary[m_id].observed(x, y)

    def has_landmarks(self):
        for _, m in self.dictionary.items():
            if m.accuracy() == 1:
                return True
        return False


if __name__ == "__main__":
    LRS(None)
    rospy.init_node("test")
    rospy.spin()
