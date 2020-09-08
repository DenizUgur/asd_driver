#!/bin/bash -em

source /catkin_ws/devel/setup.bash

echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc
echo "ROS_IP=${ROS_IP}" >> /etc/environment
echo "ROS_MASTER_URI=${ROS_MASTER_URI}" >> /etc/environment

if [ -z "$EXEC_UNIT_TESTS" ]
then
    python /root/.local/lib/python2.7/site-packages/freedomrobotics/agent.py &
    fg %1
else
    echo "Starting execution of unit tests..."
    roscd asd_core/src
    python3 root_test.py
fi
