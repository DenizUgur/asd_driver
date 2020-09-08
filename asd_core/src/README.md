# Autonomous Traversal Program

## ROS side

You need to launch our main launch file. To test this launch file, ZED needs to be connected before-hand.

- cd into root directory `roscd asd_core`

- edit `launch/proxy.launch` with correct topics. You only need to modify following arguments:

  - depth_image_topic
  - depth_camera_info_topic
  - rgb_image_topic
  - rgb_camera_info_topic
  - odom_topic

- Now you can launch it.

  ```bash
  roslaunch asd_core proxy.launch
  ```

## Python side

- Go into root directory of the program

  ```bash
  roscd asd_core/src
  ```

- Start the unit test. There is only one test. It should only take 5-10 seconds. If it lasts more, then the system is not fast enough. Please report this.

  ```bash
  python3 -m unittest tests/test_sdpath.py
  ```

- Test the main program with `--dryrun` flag. This should exit the program gracefully. You can ignore the fatal error about Driver Daemon.

  ```bash
  python3 -Wignore base.py --fresh --dryrun
  ```
