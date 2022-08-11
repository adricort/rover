# How to use this package #

## Pre-requisites ##

### Real-Time Appearance-Based Mapping Package ###

command: `sudo apt-get install ros-melodic-rtabmap-ros`

The Rtabmap package is a RGB-D SLAM approach. This is used to generate a 3D pointcloud of the occupancy and/or to create a 2D occupancy grid map for navigation.

More info about this package: http://wiki.ros.org/rtabmap_ros

### Instalation ###

command: `git clone git@github.com:AdrianSiGmA/rovy.git**`

### Launching ###

1. `roslaunch slam slam.launch`
2. `rosrun qr_loc_reader qr_tf_broadcaster_fromimage.py`

The previous command will setup the image topic into Rviz, and will detect and localize (with TF) the QR codes within the 3D map.

3. `roslaunch slam move_base.launch`

The previous command will show the costmap by layering with different colors the 2D map. It will also create the global and local paths once a 2D Nav Goal is manually set on the map.