import rospy
import pickle
import os
import math
import yaml
import numpy as np
from threading import Thread

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseWithCovarianceStamped


class Marker:
    LIMIT = 50

    def __init__(self, m_id=None, known_location=(None, None)):
        self.id = m_id
        self.logged = False
        self.visible_flag = False
        self.known_x, self.known_y = known_location
        self.observed_positions = []
        self.distance_to_robot = 999

    def update_distance(self, pose):
        _, x, y = self.loc()
        self.distance_to_robot = math.sqrt((y - pose[1]) ** 2 + (x - pose[0]) ** 2)

    def observed(self, x, y):
        """
        Append to observed locations
        """
        if len(self.observed_positions) == self.LIMIT:
            self.observed_positions.pop(0)
        self.observed_positions.append([x, y])

    def kloc(self, convert=False):
        """
        Return known location
        """
        if convert:
            return np.array([self.known_x, self.known_y, 1])
        return self.known_x, self.known_y

    def loc(self, convert=False):
        """
        Return observed location
        """
        if len(self.observed_positions) == 0:
            return (0, None, None)
        med = np.median(self.observed_positions, axis=0)
        if convert:
            return np.array([med[0], med[1], 1])
        return self.accuracy(), med[0], med[1]

    def visible(self, flag):
        self.visible_flag = flag if self.id != -1 else True

    def accuracy(self):
        if self.id == -1:
            return 1.0
        return len(self.observed_positions) / self.LIMIT


class LRS:
    LIMIT = 50

    def __init__(self, driver, clean=False):
        self.config = yaml.load(open("./config.yaml"), Loader=yaml.FullLoader)
        self.driver_instance = driver
        self.visible_landmarks = 0
        self.pose = None
        self.recovered = False
        self.dictionary = dict()
        self.M_TRANS = None

        if (
            os.path.exists("/tmp/LRS_matrix.npy")
            and os.path.exists("/tmp/LRS_landmarks.bak")
            and not clean
        ):
            self.recovered = True
            self.M_TRANS = np.load("/tmp/LRS_matrix.npy")
            self.driver_instance.M_T = np.median(self.M_TRANS, axis=2)
            bakfile = open("/tmp/LRS_landmarks.bak", "rb")
            self.dictionary = pickle.load(bakfile)
            bakfile.close()
        else:
            self.dictionary[-1] = Marker(
                -1,
                (
                    self.config["initial_position"]["x"],
                    self.config["initial_position"]["y"],
                ),
            )
            landmarks = yaml.load(open("./landmarks.yaml"), Loader=yaml.FullLoader)
            for val in landmarks["landmarks"]:
                m_id, x, y = val.values()
                self.dictionary[m_id] = Marker(m_id, (x, y))

        self.worker = Thread(name="LRS_worker", target=self.daemon_worker)
        self.worker.setDaemon(True)
        self.worker.start()

        self.crs_worker = Thread(name="LRS_CRS", target=self.daemon_crs_worker)
        self.crs_worker.setDaemon(True)
        self.crs_worker.start()

    def daemon_crs_worker(self):
        rospy.loginfo("LRS Crash Recovery System is starting")
        try:
            while not rospy.core.is_shutdown():
                try:
                    np.save("/tmp/LRS_matrix", self.M_TRANS)

                    bakfile = open("/tmp/LRS_landmarks.bak", "wb")
                    pickle.dump(self.dictionary, bakfile)
                    bakfile.close()

                    rospy.rostime.wallsleep(1.0)
                except Exception as why:
                    rospy.logerr("Error in LRS CRS... Recovering...")
                    rospy.logerr(repr(why))
        except Exception as why:
            rospy.logfatal("LRS Crash Recovery System halted!")

    def daemon_worker(self):
        rospy.loginfo("Landmark Recognition System waiting for initialization...")
        try:
            rospy.Subscriber(
                "/ar_pose_marker", AlvarMarkers, lambda p: self.commit(p.markers),
            )
            rospy.Subscriber(
                self.config["pose_with_covariance_topic"],
                PoseWithCovarianceStamped,
                self.update_pose,
            )

            rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)
            rospy.wait_for_message(
                self.config["pose_with_covariance_topic"], PoseWithCovarianceStamped
            )

            rospy.loginfo("Landmark Recognition System has started the worker thread")

            M_tr = lambda a, dx, dy: np.array(
                [
                    [math.cos(a), -math.sin(a), 0],
                    [math.sin(a), math.cos(a), 0],
                    [dx, dy, 1],
                ]
            )

            # Save starting point and rcp pose for that
            if not self.recovered:
                self.dictionary[-1].observed(
                    self.pose[0], self.pose[1],
                )
            else:
                rospy.loginfo("Recovered from backup file.")

            while not rospy.core.is_shutdown():
                try:
                    if self.has_landmarks():
                        px, py = self.pose
                        for m_id, m in self.dictionary.items():
                            a, x, y = m.loc()
                            if a > 0:
                                m.update_distance((px, py))
                            if m_id != -1 and a == 1 and not m.logged:
                                rospy.loginfo(
                                    "Locked on Marker {} -> ({}, {})".format(m_id, x, y)
                                )
                                m.logged = True

                        # * Calculate Transformation Matrix
                        if self.visible_landmarks > 0:
                            landmarks = [v for k, v in self.dictionary.items()]
                            landmarks = sorted(
                                landmarks,
                                key=lambda m: (
                                    m.visible_flag,
                                    m.accuracy(),
                                    -m.distance_to_robot,
                                ),
                                reverse=True,
                            )

                            p1_m, p2_m = landmarks[:2]
                            if (
                                len(p1_m.observed_positions) == 0
                                or len(p2_m.observed_positions) == 0
                            ):
                                continue

                            p1_fr0, p2_fr0 = p1_m.loc(True), p2_m.loc(True)
                            p1_fr1, p2_fr1 = p1_m.kloc(True), p2_m.kloc(True)

                            # theta_to_be_added
                            ttba = -self.theta_df(
                                math.atan2(
                                    p2_fr1[1] - p1_fr1[1], p2_fr1[0] - p1_fr1[0]
                                ),
                                math.atan2(
                                    p2_fr0[1] - p1_fr0[1], p2_fr0[0] - p1_fr0[0]
                                ),
                            )
                            pivot_trans = np.dot(p1_fr0, M_tr(ttba, 0, 0))
                            M_trans = M_tr(
                                ttba,
                                p1_fr1[0] - pivot_trans[0],
                                p1_fr1[1] - pivot_trans[1],
                            )

                            if self.M_TRANS is None:
                                self.M_TRANS = M_trans
                                self.driver_instance.M_T = self.M_TRANS
                            else:
                                try:
                                    _, _, d = self.M_TRANS.shape
                                    if d == self.LIMIT:
                                        rospy.loginfo_once(
                                            "Transformation Matrix is established!"
                                        )
                                        m = np.abs(M_trans - np.median(self.M_TRANS, axis=2))
                                        if 1 > np.sum(m) > -1:
                                            self.M_TRANS = np.dstack(
                                                (self.M_TRANS[:, :, 1:], M_trans)
                                            )
                                    else:
                                        self.M_TRANS = np.dstack(
                                            (self.M_TRANS, M_trans)
                                        )
                                except ValueError:
                                    self.M_TRANS = np.dstack((self.M_TRANS, M_trans))

                                self.driver_instance.M_T = np.median(
                                    self.M_TRANS, axis=2
                                )

                    rospy.rostime.wallsleep(0.1)
                except Exception as why:
                    rospy.logerr("Error in LRS... Recovering...")
                    rospy.logerr(repr(why))
        except Exception as why:
            rospy.logfatal("Landmark Recognition System halted!")

    def theta_df(self, unit1, unit2):
        unit1 = math.degrees(unit1)
        unit2 = math.degrees(unit2)
        phi = abs(unit2 - unit1) % 360
        sign = 1
        # used to calculate sign
        if not (
            (unit1 - unit2 >= 0 and unit1 - unit2 <= 180)
            or (unit1 - unit2 <= -180 and unit1 - unit2 >= -360)
        ):
            sign = -1
        if phi > 180:
            result = 360 - phi
        else:
            result = phi

        return math.radians(result * sign)

    def update_pose(self, payload):
        x = payload.pose.pose.position.x
        y = payload.pose.pose.position.y
        self.pose = x, y

    def commit(self, markers):
        visible_ids = [marker.id for marker in markers]
        self.visible_landmarks = len(visible_ids)

        for m_id, m in self.dictionary.items():
            if not m_id in visible_ids:
                m.visible(False)

        for marker in markers:
            m_id = marker.id
            x = marker.pose.pose.position.x
            y = marker.pose.pose.position.y
            if m_id in self.dictionary:
                self.dictionary[m_id].observed(x, y)
                self.dictionary[m_id].visible(True)

    def has_landmarks(self):
        for _, m in self.dictionary.items():
            if m.accuracy() == 1 and m.id != -1:
                return True
        return False
