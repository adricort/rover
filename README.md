# How to use this package #

## Pre-requisites ##

### Real-Time Appearance-Based Mapping Package ###

command: `sudo apt-get install ros-melodic-rtabmap-ros`

The Rtabmap package is a RGB-D SLAM approach. This is used to generate a 3D pointcloud of the occupancy and/or to create a 2D occupancy grid map for navigation.

More info about this package: http://wiki.ros.org/rtabmap_ros

### Instalation ###

command: `git clone git@github.com:AdrianSiGmA/rovy.git`

This repository contains 4 main packages:
1. navigation: 
2. qr_loc_reader: 
3. realsense2_camera: 
4. slam: contains the navigation node
	i. rs_camera.launch: settings for processing the data from the camera
	ii. rtabmap.launch: settings for Rviz and rtabmapviz, cfg files, depth and stereo, odometry?, RGB-D, pointcloud, SLAM, and other many parameters
	iii. 

And, of course, the documentation, maps, images, and other useful files inside "rovy_docs".

### Launching ###

1. `roslaunch slam slam.launch`

The previous command will launch the rs_camera.launch that is from the realsense2_camera package from Intel Realsense, for the D435i depth camera. In addition, an madgwick- imu filter is run as a node. The already explained rtabmap.launch from the rtabmap_ros package is executed (generates poincloud and occupancy with a SLAM approach), it is important to mention that, within this rtabmap launch file, Rviz is settup and run. Furthermore, some other parameters are given, such as robot descriptions, imu settings, and a template for robot localization. 

2. `rosrun qr_loc_reader qr_tf_broadcaster_fromimage.py`

The previous command will setup the image topic into Rviz, and will detect and localize (with TF) the QR codes within the 3D map.

3. `roslaunch slam move_base.launch`

The previous command will show the costmap by layering with different colors the 2D map. It will also create the global and local paths once a 2D Nav Goal is manually set on the map.

4. `rosrun slam navigation_node.py`