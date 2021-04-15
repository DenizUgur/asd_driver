#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

def callback(msg):
    try:
        leo_p = msg.pose[msg.name.index("leo")]
        leo_t = msg.twist[msg.name.index("leo")]

        new_msg = Odometry()
        new_msg.header.stamp = rospy.Time.now()
        new_msg.header.frame_id = "odom"

        new_msg.pose.pose = leo_p
        new_msg.twist.twist = leo_t

        pub.publish(new_msg)
    except ValueError:
        return

rospy.init_node("odom_relay")

sub = rospy.Subscriber("/gazebo/model_states", ModelStates, callback, tcp_nodelay=True)
pub = rospy.Publisher("/zed2/odom", Odometry, queue_size=1)

rospy.spin()
