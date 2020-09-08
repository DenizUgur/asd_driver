import rospy
import pickle
import os
import math
import yaml
import sympy as sp
import numpy as np
from scipy.optimize import minimize
from threading import Thread

from ar_track_alvar_msgs.msg import AlvarMarkers
from nav_msgs.msg import Odometry
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
    LIMIT = 20

    def __init__(self, driver, clean=False):
        self.config = yaml.load(open("./config/config.yaml"), Loader=yaml.FullLoader)
        self.driver_instance = driver
        self.visible_landmarks = 0
        self.pose = None
        self.recovered = False
        self.dictionary = dict()
        self.M_TRANS = None

        if (
            os.path.exists("/tmp/LRS_matrix.bak")
            and os.path.exists("/tmp/LRS_landmarks.bak")
            and not clean
        ):
            self.recovered = True
            bakfile = open("/tmp/LRS_matrix.bak", "rb")
            self.M_TRANS = pickle.load(bakfile)
            self.update_MT()
            bakfile.close()

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
            landmarks = yaml.load(
                open("./config/landmarks.yaml"), Loader=yaml.FullLoader
            )
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
                    bakfile = open("/tmp/LRS_matrix.bak", "wb")
                    pickle.dump(self.M_TRANS, bakfile)
                    bakfile.close()

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
                self.config["odometry_topic"], Odometry, self.update_pose,
            )

            rospy.wait_for_message(self.config["odometry_topic"], Odometry)

            # Save starting point and rcp pose for that
            if not self.recovered:
                self.dictionary[-1].observed(
                    self.pose[0], self.pose[1],
                )
            else:
                rospy.loginfo("Recovered from backup file.")

            rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)

            while not rospy.core.is_shutdown():
                try:
                    if self.has_landmarks():
                        px, py = self.pose
                        for m_id, m in self.dictionary.items():
                            a, _, _ = m.loc()
                            x, y = m.kloc()
                            if a > 0:
                                m.update_distance((px, py))
                            if m_id != -1 and a == 1 and not m.logged:
                                rospy.loginfo(
                                    "Locked on Marker {} -> ({}, {})".format(m_id, x, y)
                                )
                                m.logged = True

                        # * Calculate Transformation Matrix
                        if self.visible_landmarks > 0:
                            # Sort landmarks
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

                            # Remove the ones where there is no observed position
                            landmarks = [
                                l for l in landmarks if len(l.observed_positions) > 0
                            ]
                            if len(landmarks) < 2:
                                continue

                            # Get best two landmarks
                            p1_m, p2_m = landmarks[:2]

                            p1_fr0, p1_fr1 = p1_m.loc(True), p1_m.kloc(True)
                            p2_fr0, p2_fr1 = p2_m.loc(True), p2_m.kloc(True)

                            # If we have only 2 landmarks then create 2 new points between these two landmarks
                            if len(landmarks) == 2:
                                md1_fr0 = np.array(
                                    [
                                        p1_fr0[0] + (1 / 3) * (p2_fr0[0] - p1_fr0[0]),
                                        p1_fr0[1] + (1 / 3) * (p2_fr0[1] - p1_fr0[1]),
                                        1,
                                    ]
                                )
                                md1_fr1 = np.array(
                                    [
                                        p1_fr1[0] + (1 / 3) * (p2_fr1[0] - p1_fr1[0]),
                                        p1_fr1[1] + (1 / 3) * (p2_fr1[1] - p1_fr1[1]),
                                        1,
                                    ]
                                )

                                md2_fr0 = np.array(
                                    [
                                        p1_fr0[0] + (2 / 3) * (p2_fr0[0] - p1_fr0[0]),
                                        p1_fr0[1] + (2 / 3) * (p2_fr0[1] - p1_fr0[1]),
                                        1,
                                    ]
                                )
                                md2_fr1 = np.array(
                                    [
                                        p1_fr1[0] + (2 / 3) * (p2_fr1[0] - p1_fr1[0]),
                                        p1_fr1[1] + (2 / 3) * (p2_fr1[1] - p1_fr1[1]),
                                        1,
                                    ]
                                )
                            elif len(landmarks) == 3:
                                # If there is 3 landmarks then create a midpoint between two best landmarks
                                md1_m = landmarks[2]
                                md1_fr0, md1_fr1 = md1_m.loc(True), md1_m.kloc(True)

                                md2_fr0 = np.array(
                                    [
                                        p1_fr0[0] + (1 / 2) * (p2_fr0[0] - p1_fr0[0]),
                                        p1_fr0[1] + (1 / 2) * (p2_fr0[1] - p1_fr0[1]),
                                        1,
                                    ]
                                )
                                md2_fr1 = np.array(
                                    [
                                        p1_fr1[0] + (1 / 2) * (p2_fr1[0] - p1_fr1[0]),
                                        p1_fr1[1] + (1 / 2) * (p2_fr1[1] - p1_fr1[1]),
                                        1,
                                    ]
                                )
                            else:
                                md1_m = landmarks[2]
                                md1_fr0, md1_fr1 = md1_m.loc(True), md1_m.kloc(True)
                                md2_m = landmarks[3]
                                md2_fr0, md2_fr1 = md2_m.loc(True), md2_m.kloc(True)

                            sp_x, sp_y, sp_t, sp_s = sp.symbols("sp_x sp_y sp_t sp_s")
                            sp_M_trans = sp.Matrix(
                                (
                                    (sp_s * sp.cos(sp_t), -sp.sin(sp_t), 0),
                                    (sp.sin(sp_t), sp_s * sp.cos(sp_t), 0),
                                    (sp_x, sp_y, 1),
                                )
                            )

                            sp_eq0 = sp.Matrix(1, 3, p1_fr0) * sp_M_trans
                            sp_eq1 = sp.Matrix(1, 3, p2_fr0) * sp_M_trans
                            sp_eq2 = sp.Matrix(1, 3, md1_fr0) * sp_M_trans
                            sp_eq3 = sp.Matrix(1, 3, md2_fr0) * sp_M_trans

                            sp_A = sp.Matrix((sp_eq0, sp_eq1, sp_eq2, sp_eq3))
                            b_np = np.array((p1_fr1, p2_fr1, md1_fr1, md2_fr1))

                            delta = lambda result: np.sum(
                                np.abs(np.array(result) - b_np)
                            )
                            f = lambda args: delta(
                                sp_A.evalf(
                                    subs={
                                        sp_x: args[0],
                                        sp_y: args[1],
                                        sp_t: args[2],
                                        sp_s: args[3],
                                    }
                                )
                            )
                            result = minimize(
                                f, [0, 0, 0, 1], method="L-BFGS-B", tol=1e-5
                            )

                            if result.success:
                                self.update_MT(
                                    np.array(
                                        [
                                            [
                                                result.x[3] * math.cos(result.x[2]),
                                                -math.sin(result.x[2]),
                                                0,
                                            ],
                                            [
                                                math.sin(result.x[2]),
                                                result.x[3] * math.cos(result.x[2]),
                                                0,
                                            ],
                                            [result.x[0], result.x[1], 1],
                                        ]
                                    ),
                                    f(result.x),
                                )

                    rospy.rostime.wallsleep(1.5)
                except Exception as why:
                    rospy.logerr("Error in LRS... Recovering...")
                    rospy.logerr(repr(why))
        except Exception as why:
            rospy.logfatal("Landmark Recognition System halted!")

    def update_MT(self, MT=None, error=None):
        if not MT is None:
            if self.M_TRANS is None:
                self.M_TRANS = [(error, MT)]
            elif len(self.M_TRANS) < self.LIMIT:
                self.M_TRANS.append((error, MT))
            else:
                self.M_TRANS.sort(key=lambda x: x[0], reverse=True)
                if error < self.M_TRANS[0][0]:
                    self.M_TRANS.pop(0)
                    self.M_TRANS.append((error, MT))

        # errors = [e for e, _ in self.M_TRANS]
        # best = list(filter(lambda v: v[0] <= np.percentile(errors, 25), self.M_TRANS))
        best = sorted(self.M_TRANS, key=lambda x: x[0])
        self.driver_instance.M_T = best[0][1]
        self.driver_instance.M_T_err = best[0][0]

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
