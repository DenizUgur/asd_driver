import rospy
import math
import yaml
import numpy as np
import actionlib
from threading import Thread
from enum import Enum

from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Vector3, Twist
from std_msgs.msg import Header


class Status(Enum):
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    PREEMPTING = 6
    RECALLING = 7
    RECALLED = 8
    LOST = 9


class DriverStatus(Enum):
    HALT = 0
    RESUME = 1


class Driver:
    def __init__(self):
        self.config = yaml.load(open("./config.yaml"), Loader=yaml.FullLoader)
        self.path_pub = rospy.Publisher("/asd_core/path", Path, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher(self.config["cmd_vel"], Twist, queue_size=10)
        self.status_sub = rospy.Subscriber(
            "/move_base/status", GoalStatusArray, self.update_status
        )

        self.worker = Thread(name="driver_worker", target=self.daemon_worker)
        self.worker.setDaemon(True)
        self.worker.start()

        self.target, self.source, self.latch, self.status = None, None, True, 0
        self.flag = DriverStatus.RESUME
        self.M_T = None
        assert self.config["xy_tolerance"] / 2 >= 0.25  # defined in move_base params

    def daemon_worker(self):
        try:
            pose = {"pose": None}
            extract = lambda val: (
                val.pose.position.x,
                val.pose.position.y,
                val.pose.orientation,
            )
            path = []

            client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            rospy.loginfo("Waiting for actionlib server to become online.")
            client.wait_for_server()
            client.cancel_all_goals()

            rospy.Subscriber(
                "/asd_core/path",
                Path,
                lambda payload: path.__init__([extract(x) for x in payload.poses]),
            )
            rospy.Subscriber(
                self.config["pose_with_covariance_topic"],
                PoseWithCovarianceStamped,
                lambda payload: pose.update({"pose": payload}),
            )

            rospy.wait_for_message(
                self.config["pose_with_covariance_topic"], PoseWithCovarianceStamped
            )
            rospy.wait_for_message("/asd_core/path", Path)

            rospy.loginfo("Driver Daemon online")

            will_rotate_to_first = True
            last_dist = 999
            while not rospy.core.is_shutdown():
                try:
                    if self.flag == DriverStatus.HALT:
                        will_rotate_to_first = True
                        last_dist = 999
                        client.cancel_all_goals()

                    if len(path) > 0 and self.flag == DriverStatus.RESUME:
                        x, y, _ = extract(pose.get("pose").pose)
                        tx, ty, _ = path[-1]
                        ta = math.atan2(ty - y, tx - x)
                        path_copy = path.copy()

                        while len(path_copy) > 0:
                            nx, ny, rot = path_copy.pop(0)
                            a = math.atan2(ny - y, nx - x)
                            dist = math.sqrt((y - ny) ** 2 + (x - nx) ** 2)
                            dist_t = math.sqrt((y - ty) ** 2 + (x - tx) ** 2)

                            waiting = True
                            if dist_t <= self.config["xy_tolerance"]:
                                if dist_t < last_dist:
                                    last_dist = dist_t
                                else:
                                    break

                            if (
                                ta - math.pi / 2 < a < ta + math.pi / 2
                                and dist >= self.config["xy_tolerance"]
                            ):
                                waiting = False
                                break

                        if not waiting:
                            goal = MoveBaseGoal()
                            goal.target_pose.header.stamp = rospy.Time.now()
                            goal.target_pose.header.frame_id = "map"

                            # set position
                            goal.target_pose.pose.position.x = nx
                            goal.target_pose.pose.position.y = ny
                            goal.target_pose.pose.position.z = 0.0

                            # set orientation
                            goal.target_pose.pose.orientation = rot

                            if will_rotate_to_first:
                                goal.target_pose.pose.position.x = x
                                goal.target_pose.pose.position.y = y
                                client.send_goal(goal)

                                client.wait_for_result()
                                will_rotate_to_first = False
                                path = []
                            else:
                                client.send_goal(goal)
                        else:
                            client.cancel_all_goals()
                            will_rotate_to_first = True
                            last_dist = 999

                except Exception as why:
                    rospy.logerr("Error in Driver... Recovering.")
                    rospy.logerr(repr(why))

                rospy.rostime.wallsleep(0.2)
        except Exception:
            rospy.logfatal("Driver Daemon crashed or halted!")

    def update_pose(self, payload):
        self.pose = payload.pose

    def update_status(self, payload):
        if len(payload.status_list) > 0:
            self.status = payload.status_list.pop().status
        else:
            self.status = Status.PENDING

    def move(self, dl=0, da=0):
        msg = Twist()
        linear = Vector3()
        angular = Vector3()

        linear.x = dl
        linear.y = 0
        linear.z = 0
        angular.x = 0
        angular.y = 0
        angular.z = da

        msg.linear = linear
        msg.angular = angular

        self.cmd_vel_pub.publish(msg)

    def halt(self):
        self.latch = True
        self.flag = DriverStatus.HALT
        self.move()

    def convert(self, cx, cy):
        t = np.dot(np.array([cx, cy, 1]), np.linalg.inv(self.M_T))
        return t[0], t[1]

    def get_current_loc(self, relative=False):
        if relative:
            t = np.dot(
                np.array([self.pose.position.x, self.pose.position.y, 1]), self.M_T
            )
            return t[0], t[1]
        return self.pose.position.x, self.pose.position.y

    def is_target_reached(self):
        reached = False
        if self.target is None:
            reached = True
        else:
            x, y = self.get_current_loc()
            tx, ty = self.target
            reached = (
                self.config["xy_tolerance"] >= math.sqrt((x - tx) ** 2 + (y - ty) ** 2)
                and self.status != Status.ACTIVE
            )

        if self.flag == DriverStatus.HALT:
            return True

        self.latch = reached
        return reached

    def percent_complete(self):
        if self.source is None or self.target is None:
            return 0
        else:
            sx, sy = self.source
            tx, ty = self.target
            cx, cy = self.get_current_loc()
            return 100 * (
                1
                - (
                    math.sqrt((cx - tx) ** 2 + (cy - ty) ** 2)
                    / math.sqrt((sx - tx) ** 2 + (sy - ty) ** 2)
                )
            )

    def follow_path(self, path):
        msg = Path()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        if self.latch:
            self.source = path[0]
            self.target = path[-1]

        for i, (x, y) in enumerate(path):
            if i + 1 < len(path):
                nx, ny = path[i + 1]
                r = math.atan2(ny - y, nx - x)
            else:
                nx, ny = path[i - 1]
                r = math.atan2(y - ny, x - nx)

            pose = PoseStamped()
            pose.header = msg.header

            # set position
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.2

            # set orientation
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(r / 2)
            pose.pose.orientation.w = math.cos(r / 2)

            # add to path
            msg.poses.append(pose)

        self.path_pub.publish(msg)

