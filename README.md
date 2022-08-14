# How to use the packages #

## Pre-requisites ##

### 1. Real-Time Appearance-Based Mapping package ###

command: `sudo apt-get install ros-melodic-rtabmap-ros`

The Rtabmap package http://wiki.ros.org/rtabmap_ros is a RGB-D SLAM approach. It is used to generate a 3D pointcloud of the occupancy and/or to create a 2D occupancy grid map for navigation.

### 2. The Navigation stack ###

command: `git clone https://github.com/ros-planning/navigation.git`

The navigation stack http://wiki.ros.org/navigation is a 2D navigation stack that takes in information from odometry, sensor streams, and a goal pose and outputs safe velocity commands that are sent to a mobile base (we will be using the move_base node). In other words, creates the costmap layers and generates the path planner for autonomous navigation. It is important to be cloned in your main workspace and not in your opt/ros/noetic/share directory (sudo apt-get inst...), or at least, this is how it worked for me.

### 3. IntelRealSense package ### 

The installation is contained in the following section.

> :warning: **Please, check if you already have realsense2 package installed**: it might colapse with some features of this repo (e.g. Rviz). It is essential to remove it and to get it from this repo and not from the original source https://github.com/IntelRealSense/realsense-ros since it has been modified to work for this rover project.

### 4. Installation of the full Rover stack ###

command: `git clone git@github.com:AdrianSiGmA/rovy.git`

This repository contains 3 main packages:
1. **slam**: contains the developed autonavigation node
     1. slam.launch: calls the rs_camera.launch, rtabmap.launch, and the ukf_template.launch (robot_localization package). It also contains a setup for the Madgwick imu filter, and some description parameters (realsense2_description package).
     2. move_base.launch: sets some topics, and runs the move_base node for the cost map and path planner
2. **realsense-ros** (camera and description): contains all the parameters and nodes to receive data from the d435i
     1. realsense2_camera: contains the rs_camera.launch, that are settings for processing the data from the camera
     2. realsense2_description: contains the urdf and other files for drawing the module in Rviz
3. **qr_loc_reader**: this is a qr-code localizator and reader. This means that it will make some transformations based on the readings it finds from the camera, and will draw them in the 3D space, plus it can display what the qr code states, and the main aim is to be capable to store the obtained data in a text file. 

Some extra considerations:

- The rtabmap.launch that is called from the slam.launch has some settings for Rviz and Rtabmapviz, cfg files, depth and stereo, odometry?, RGB-D, pointcloud, SLAM, and many other parameters. 

> :warning: It is important that, before going to the next section, one modifies this launch file by changing the rviz config to: `<arg name="rviz_cfg"                default="$(find slam)/rviz/slam.rviz" />`

The documentation, maps, images, and other useful files inside "rovy_docs".

## Usage ##

1. `roslaunch slam slam.launch`

The previous command will launch the rs_camera.launch that is from the realsense2_camera package from Intel Realsense, for the D435i depth camera. In addition, an madgwick- imu filter is run as a node. The already explained rtabmap.launch from the rtabmap_ros package is executed (generates poincloud and occupancy with a SLAM approach), it is important to mention that, within this rtabmap launch file, Rviz is settup and run. Furthermore, in this slam launch file, some other parameters are given, such as robot descriptions, imu settings, and a template for robot localization. 

2. `rosrun qr_loc_reader qr_tf_broadcaster_fromimage.py`

The previous command will setup the image topic into Rviz, and will detect and localize (with TF) the QR codes within the 3D map.

3. `roslaunch slam move_base.launch`

The previous command will show the costmap by layering with different colors the 2D map. It will also create the global and local paths once a 2D Nav Goal is manually set on the map.

4. `rosrun slam navigation_node.py`