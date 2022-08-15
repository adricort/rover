# How to use rosbags in this package #

Once the slam.launch is running, one can record a rosbag while inside this directory.

command: `rosbag record rosout tf rtabmap/mapData rtabmap/grid_map`

After finishing (Ctrl+C), it will be stored, and one can replay it with:

terminal 1: `roscore`
terminal 2: `rviz`

(here, change the config file manually for rviz visualization, e.g. slam.rviz)

terminal 3: `rosbag play <rosbag_name>`