# turtlebot3_datasets

This package provides helper scripts to download and use datasets for the [Introduction to Robotics class](https://guilhermelawless.github.io/introduction-robotics/).

The datasets were captured on a [Turtlebot 3 Waffle Pi](http://www.robotis.us/turtlebot-3-waffle-pi/).

This guide assumes that you have followed the previous [tutorials](https://guilhermelawless.github.io/introduction-robotics/).

## Dataset information

A map and rosbag are provided, along with some helper scripts. The bag includes:

```bash
rosbag info slam_easy.bag

topics:      /imu                              14805 msgs    : sensor_msgs/Imu
             /odom                              3252 msgs    : nav_msgs/Odometry
             /raspicam_node/camera_info         1814 msgs    : sensor_msgs/CameraInfo
             /raspicam_node/image/compressed    1811 msgs    : sensor_msgs/CompressedImage
             /scan                               626 msgs    : sensor_msgs/LaserScan
             /tf                               21329 msgs    : tf/tfMessage
             /tf_static                            1 msg     : tf2_msgs/TFMessage
```

Ground-truth data is provided in the `/tf` topic, as a transform `mocap -> mocap_laser_link`. Ground-truth is sampled at 60Hz and the child frame is the center of the laser of Turtlebot 3 Waffle Pi. The initial transform can be used to connect `mocap` to `map` or other fixed frames, with the provided `get_groundtruth_tf.py` script.

The map was obtained using [turtlebot3_slam](http://wiki.ros.org/turtlebot3_slam) gmapping using the default parameters.

## Notices

First, some things to know:

- Use simulation time when reading data from rosbags. If not, then things like `rospy.Time.now()` will not output the time at which the dataset was recorded. To use simulation time:
    `rosparam set use_sim_time true` after `roscore`. If you were running `rviz` already, it needs to be restarted. Otherwise, TFs might not be picked up.

- Always play back the rosbag with the `--clock` option (related to simulation time). `--pause` and `--r RATE` can also help.


## Steps

1. `git clone https://github.com/guilhermelawless/turtlebot3_datasets.git` into your ROS workspace

2. Build with catkin:
    `cd $ROS_WORKSPACE/../src && catkin_make && source ~/.bashrc`

3. Download the dataset (the map is already in the `data` directory, this downloads the rosbag):
    `roscd turtlebot3_datasets/scripts && bash download_dataset.sh`

4. Extract the initial tf (provide the rosbag):
    `rosrun turtlebot3_datasets get_groundtruth_tf.py`

5. Use the output from (4) to run a static transform publisher between `mocap` and your map/odom frame. Consider adding this to the launch file in step (6).

6. Launch the description launch file:
    `roslaunch turtlebot3_datasets turtlebot3_description.launch`

7. Launch map server and/or other algorithms...

8. Play the bag.
