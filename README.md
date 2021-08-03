# New README (in the works, will keep the previous readme info below for now)

##What is this?

openvr_headset_ros is a package for ros based on vrui_mdf by Zenyu Shi(https://github.com/zhenyushi/vrui_mdf). The goal of this project is to provide an open source way to view a ros/gazebo simulation through a vr headset. This package will use openvr to get tracking info and send images to the headset for display.


##Current Progress:

Currently, if you use "roslaunch openvr_headset_ros start.launch", "rosrun openvr_headset_ros opentracking", and "rosrun openvr_headset_ros imagesub" with a headset hooked up to steamvr, a headset model will spawn in Gazebo and your headset will focus on the imagesub node (leave the SteamVR home app) and display some pixel snow. Current priority is getting a video pipeline from gazebo to the headset. The current pipeline (or at least attempted pipeline) is gazebo->a gazebo stereo camera model called "vr_view"->ros publisher in opentracking.cpp->ros subscriber/cv_bridge in imagesub.cpp->OpenGL tex2d in imagesub.cpp->OpenVR Compositor.


##Software:


###OpenCV:

https://docs.opencv.org/4.5.2/d0/d3d/tutorial_general_install.html

Dev package can be installed using:

		$sudo apt install libopencv-dev


###OpenGL/GLEW/SDL2:

OpenGL: https://www.opengl.org//
GLEW: http://glew.sourceforge.net/
SDL: https://www.libsdl.org/

Some more work needs to be done on paring down what exact things from OpenGL/GLEW/SDL need to be installed and used. For now, it's a safe bet to just install GLEW and SDL2, which can be done using

		$sudo apt install libglew-dev
	
as well as

		$sudo apt install libsdl2-dev

GLEW is the "GL Extension Wrangler". It grabs things related to OpenGL and makes it easier to load them into a program. SDL is a library that sets up OpenGL contexts. There are a few redundancies with SDL and GLUT and GLEW in this package as far as I'm aware. I need to do a bit more digging to streamline the OpenGL bits of the package. In the future, you will most likely just need to install GLEW.

###Steam

Steam is a popular platform for hosting games, software, and tools. It is the only portion of this package that is closed source, unfortunately, but SteamVR is necessary for interfacing with VR headsets.

Install steam using:
		$sudo apt install steam
		
Log in to steam/make and account and install SteamVR: https://store.steampowered.com/app/250820/SteamVR/

###OpenVR

https://github.com/ValveSoftware/openvr
Must have SDL2 and GLEW dev packages already installed.
Install by using:

		$git clone https://github.com/ValveSoftware/openvr
		$cd openvr
		$mkdir build && cd build
		$cmake ..
		$make
		$sudo make install
		
###ROS/Gazebo

Install ROS Noetic and Gazebo using this tutorial: http://wiki.ros.org/noetic/Installation/Ubuntu
Set up a Catkin Workspace (~/catkin_ws) using this tutorial: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

##Install Process:

Once OpenCV, OpenGL, SteamVR, OpenVR, and ROS are installed:

		$cd ~/catkin_ws/src
		$git clone https://github.com/ConradKent/openvr_headset_ros
		$cd ..
		$catkin_make
		
To run the example code, open a new terminal and source ROS and your catkin workspace according to: http://wiki.ros.org/catkin/Tutorials/create_a_workspace. Then run:

		$roslaunch openvr_headset_ros VR_empty_test.launch

##Code Based On:

###vrui_mdf
This package was adapted from vrui_mdf to use OpenVR in place of Vrui.
Much of this code was copied from or inspired by the code from vrui_mdf.
See the repository commit history, and/or: https://github.com/zhenyushi/vrui_mdf

###OpenVR_ros
The tracking code is heavily inspired by/adopted from OpenVR_ros

