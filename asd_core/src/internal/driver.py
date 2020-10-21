import rospy
import math
import yaml
import numpy as np
from threading import Thread
from enum import Enum

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Vector3, Twist
from std_msgs.msg import Header


class DriverStatus(Enum):
    HALT = 0
    NORMAL = 1
    REJECTED = 2
    ACCEPTED = 3
    PREEMPTING = 4
    CANCELLED = 5


class Driver:
    def __init__(self):
        self.config = yaml.load(open("./config/config.yaml"), Loader=yaml.FullLoader)
        self.cmd_vel_pub = rospy.Publisher(self.config["cmd_vel"], Twist, queue_size=10)

        self.worker = Thread(name="driver_worker", target=self.daemon_worker)
        self.worker.setDaemon(True)
        self.worker.start()

        self.target, self.source, self.latch = (None, None), (None, None), True
        self.flag = DriverStatus.NORMAL

    def daemon_worker(self):
        try:
            pose = {"pose": None}
            extract = lambda val: (
                val.pose.position.x,
                val.pose.position.y,
                val.pose.orientation,
            )

            rospy.Subscriber(
                self.config["odometry_topic"],
                Odometry,
                lambda payload: pose.update({"pose": payload.pose}),
            )

            rospy.wait_for_message(self.config["odometry_topic"], Odometry)

            rospy.loginfo("Driver Daemon online")

            while not rospy.core.is_shutdown():
                try:
                    pass
                except Exception as why:
                    rospy.logerr("Error in Driver... Recovering.")
                    rospy.logerr(repr(why))

                rospy.rostime.wallsleep(0.1)
        except Exception:
            rospy.logfatal("Driver Daemon crashed or halted!")

    def go_to_position(self, x, y, wait=True):
        self.target_goal = (x, y, 0)
        self.flag = DriverStatus.PREEMPTING
        if wait:
            while self.flag == DriverStatus.PREEMPTING:
                pass

    def update_pose(self, payload):
        self.pose = payload.pose

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

    def get_current_loc(self):
        return self.pose.position.x, self.pose.position.y

    def is_target_reached(self):
        reached = False
        if self.target == (None, None):
            reached = True
        else:
            x, y = self.get_current_loc()
            tx, ty = self.target
            reached = self.config["xy_tolerance"] >= math.sqrt(
                (x - tx) ** 2 + (y - ty) ** 2
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
