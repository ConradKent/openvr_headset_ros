# openvr_headset_ros

## What is this?

openvr_headset_ros is a package for ros based on vrui_mdf by Zhenyu Shi(https://github.com/zhenyushi/vrui_mdf). The goal of this project is to provide an open source way to view a ros/gazebo simulation through a vr headset. This package will use openvr to get tracking info and send images to the headset for display. The package currently works with the original Vive headset and controllers or the Vive Pro headset and controllers, and with lighthouse version 1.0 or 2.0. Other SteamVR capable headsets should be relatively simple to add.

## Some Gifs

A demo of controlling a Turtlebot3, With perspective from inside the headset and outside. This demo uses the waypoint, throwto, and velocity controls.
![Uh oh, Something's gone wrong with this link.](https://github.com/ConradKent/conrad_media/blob/main/openvr_headset_ros/TurtlebotDemo1.gif)

Sometimes they just don't navigate the way you want...
![Uh oh, Something's gone wrong with this link.](https://github.com/ConradKent/conrad_media/blob/main/openvr_headset_ros/QuadrotorHittingWall.gif)

## Current Progress:

This software currently works on Ubuntu 20.04.3 LTS, with an AMD Ryzen 7 2700x CPU and AMD rx 5700xt GPU. The graphics driver we're using is "amdgpu", amdgpu-pro has not been tested. To check your graphics driver run "$lsmod | grep amd" and look for amdgpu or amdgpu-pro. We're running ROS noetic (roscpp version 1.15.11) and SteamVR Build ID: 7205650 (updated Aug 19, 2021).

openvr_headset_ros will display a stereoscopic view of a gazebo scene to an HTC vive. It will also track the vive's position and orientation to let you move around inside this scene. There are two examples with a model of the turtlebot robot and a quadrotor. In these examples, the robot models can be moved with the controllers.

Things to be added are:

-Support for other steamvr headsets

-A statictest.cpp file which will operate independently of ROS to demonstrate/test a basic opengl to openvr display pipeline

## Software:


### OpenCV:

https://docs.opencv.org/4.5.2/d0/d3d/tutorial_general_install.html

Dev package can be installed using:

		$sudo apt install libopencv-dev


### OpenGL/GLEW/SDL2:

OpenGL: https://www.opengl.org//
GLEW: http://glew.sourceforge.net/
SDL: https://www.libsdl.org/

Some more work needs to be done on paring down what exact things from OpenGL/GLEW/SDL need to be installed and used. For now, it's a safe bet to just install GLEW and SDL2, which can be done using

		$sudo apt install libglew-dev
	
as well as

		$sudo apt install libsdl2-dev

GLEW is the "GL Extension Wrangler". It grabs things related to OpenGL and makes it easier to load them into a program. SDL is a library that sets up OpenGL contexts. There are a few redundancies with SDL and GLUT and GLEW in this package as far as I'm aware. I need to do a bit more digging to streamline the OpenGL bits of the package. In the future, you will most likely just need to install GLEW.

### Steam

Steam is a popular platform for hosting games, software, and tools. It is the only portion of this package that is closed source, unfortunately, but SteamVR is necessary for interfacing with VR headsets.

Install steam using:
		$sudo apt install steam
		
Log in to steam/make and account and install SteamVR: https://store.steampowered.com/app/250820/SteamVR/

### OpenVR

https://github.com/ValveSoftware/openvr
Must have SDL2 and GLEW dev packages already installed.
Install by using:

		$git clone https://github.com/ValveSoftware/openvr
		$cd openvr
		$mkdir build && cd build
		$cmake ..
		$make
		$sudo make install
		
### ROS/Gazebo

Install ROS Noetic and Gazebo using this tutorial: http://wiki.ros.org/noetic/Installation/Ubuntu
Set up a Catkin Workspace (~/catkin_ws) using this tutorial: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

## Install Process:

Once OpenCV, OpenGL, SteamVR, OpenVR, and ROS are installed:

### Install Turtlebot 3:

		$cd ~/catkin_ws/src
		$git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
		$cd ~/catkin_ws && catkin_make
		
### Install Hector Quadrotor:

Hector is an old package and requires some work to install. First, make sure Qt5 is installed (sudo apt-get install qt5-default).

You'll need to make a new catkin_ws workspace in the same way as the original. If you don't have a separate workspace already, follow this guide: http://wiki.ros.org/catkin/Tutorials/create_a_workspace and call the new ws "catkin_ws_2".

Next, install geographic_info into catkin_ws_2.

		$cd ~/catkin_ws_2/src
		$git clone https://github.com/ros-geographic-info/geographic_info
		$cd ~/catkin_ws_2 && catkin_make
		
Next we'll clone hector_quadrotor into catkin_ws.

First, source your ros setup.bash (for me, the command is "$source /opt/ros/noetic/setup.bash"). Then:

		$sudo apt install ros-noetic-joystick-drivers ros-noetic-teleop-twist-keyboard
		$cd ~/catkin_ws/src
		$git clone https://github.com/AS4SR/hector_quadrotor
		
You can try running catkin_make at this point, but I had to make the following changes to obselete code to get it to run:

-Remove "signals" from line 10 of of ~/catkin_ws/src/hector_slam/hector_mapping/CMakeLists.txt (signals is deprecated and what it provided has been rolled into elsewhere)

-In ~/catkin_ws/src/hector_slam/hector_geotiff/CMakeLists.txt
	-delete line 27
	-in line 88 replace "${QT_LIBRARIES}" with Qt5::Widgets (the name here changed during a version switch)

-On ~/catkin_ws/src/hector_slam/hector_geotiff/include/hector_geotiff/geotiff_writer.h
	-Changed line 40 from "QtGui" to "QtWidgets" (change in Qt's naming system)
	-Had to make this same change to line 34 of ~/catkin_ws/src/hector_slam/hector_geotiff/src/geotiff_writer/geotiff_writer.cpp
	-As well as to line 37 of "geotiff_saver.cpp" and line 43 of "geotiff_node.cpp" in hector_slam/hector_geotiff

After these changes you should be able to run

		$cd ~/catkin_ws && catkin_make
		
If you can't... Well you might have to figure that out on your own. This is running with Qt version 5.12.8 and should definitely be run with Qt5.

### Install openvr_headset_ros

		$cd ~/catkin_ws/src
		$git clone https://github.com/ConradKent/openvr_headset_ros
		$cd ..
		$catkin_make

### Running openvr_headset_ros example code
		
To run any example code, open a new terminal and source ROS and your catkin workspace according to: http://wiki.ros.org/catkin/Tutorials/create_a_workspace. Make sure SteamVR is running and that your VR headset and controllers are connected. Then run:

When launching, make sure that SteamVR is open and that the headset and controllers are active. I've also found it best to have your finger over the sensor inside the headset that wakes it up (The sensor that aims at your forehead to check if you have the headset on). 

To launch an empty scene with headset/controllers:

		$roslaunch openvr_headset_ros VR_empty_test.launch
		
To run the turtlebot 3 example:

		$roslaunch openvr_headset_ros VR_turtlebot.launch

To run the hector_quadrotor example:

		$roslaunch openvr_headset_ros VR_quadrotor.launch

After launching the quadrotor, wait a few seconds for everything to set up. Then, open a new terminal, source your catkin_ws, and type

		$rosservice call /enable_motors "enable: true"
		
The quadrotor needs to fully load in to enable the motors, I'll add something better to the code eventually but this works for now.

The turtlebot and quadrotor examples have three control methods. Throwto, Velocity, and Waypoint. To switch between each control method, hit the grip button (on Vive/Vive Pro controllers. Not sure if grip is mapped the same for other controllers). To aim the Throwto point or Waypoint, hold down the trigger and move the flat cylinder around. Throwto will teleport the robot there, Waypoint will have the robot navigate to there. To use Velocity, hold the trigger down and move the controller to directly control the robot's motors. This will be changed to using the joystick of the controller soon. 

## Code Based On (updates to comments in code to show where different parts are from coming soon):

### vrui_mdf
This package was adapted from vrui_mdf to use OpenVR in place of Vrui.
Much of this code was copied from or inspired by the code from vrui_mdf.
See the repository commit history, and/or: https://github.com/zhenyushi/vrui_mdf

### OpenVR_ros
The tracking code is heavily inspired by/adopted from OpenVR_ros.

