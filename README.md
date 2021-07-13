# New README (in the works, will keep the previous readme info below for now)

What is this?
openvr_headset_ros is a package for ros based on vrui_mdf by Zenyu Shi(https://github.com/zhenyushi/vrui_mdf). The goal of this project is to provide an open source way to view a ros/gazebo simulation through a vr headset. This package will use openvr to get tracking info and send images to the headset for display.

Current Progress:
Currently, if you use "roslaunch openvr_headset_ros start.launch", "rosrun openvr_headset_ros opentracking", and "rosrun openvr_headset_ros imagesub" with a headset hooked up to steamvr, a headset model will spawn in Gazebo and your headset will focus on the imagesub node (leave the SteamVR home app) and display some pixel snow. Current priority is getting a video pipeline from gazebo to the headset. The current pipeline (or at least attempted pipeline) is gazebo->a gazebo stereo camera model called "vr_view"->ros publisher in opentracking.cpp->ros subscriber/cv_bridge in imagesub.cpp->OpenGL tex2d in imagesub.cpp->OpenVR Compositor.

Software:
OpenCV
https://docs.opencv.org/4.5.2/d0/d3d/tutorial_general_install.html
Dev package can be installed using:

	$sudo apt install libopencv-dev

OpenGL
https://www.khronos.org/opengl/wiki/Getting_Started#Downloading_OpenGL
OpenGL is more or less guaranteed to come with your operating system (according to the makers).


SDL 
https://wiki.libsdl.org/Installation
Dev package can be installed using:

	$sudo apt install libsdl2-dev

Install Process:

To do (it doesn't currently work so no worries!)

# openvr_headset_ros <-- the new name for this repository!!

OpenVR APIs used to interface with ROS + Gazebo, for use with HTC Vive, Vive Pro, and other SteamVR-compatible headsets

(Originally called vrui_mdf as "'Vrui modified' for use with HTC Vive and ROS + Gazebo")

Much of this code was copied from or inspired by the code from vrui_mdf  
See the repository commit history, and/or: https://github.com/zhenyushi/vrui_mdf

This fork of vrui_mdf has been renamed from the original repository, as we are moving away from using Vrui to an OpenVR/OpenCV/OpenGL implementation that is using the SteamVR interface API. This version will use a subscriber to pull images from a stereo camera as before using the cv bridge feature in ROS. These OpenCV images will then be turned into OpenGL textures, and these textures will be submitted to each eye of the HMD via OpenVR. OpenVR will also handle tracking which will be used to position the stereo camera in the Gazebo simulation.

We are moving to no longer require Vrui as a dependency. We will, however, still be dependent on versions of OpenVR, SteamVR, ROS, and Gazebo for this code to function properly. (We may be moving away from OpenCV to OpenGL as well / eventually.)

We are attempting to show the changes in a way that makes sense and does not break diff status, by having moved the original code into an "archive" subdirectory for the interim.

For code updates, as we work through the older vrui_mdf files, we plan to first stage a commit that moves the old code back up to the original top-level locations, then work on / modify the code there.

As an example, for single-files, starting at the top-level directory:  
$ git mv archive/README.md README.md  
$ git commit -m "moving README.md up a level prior to file update"  
$ gedit README.md  
-- make changes to README.md --  
$ git add README.md  
$ git commit -m "update of README.md"  
$ git push  

(Note that using the 'git rm' here command is equivalent to 'git rm archive/README.md' then 'git add README.md'.)

As an example that includes making new directories, etc., starting at the top-level directory:  
$ mkdir src  
$ git add src  
$ git mv archive/src/imagesub.cpp src  
$ git commit -m "moving src/imagesub.cpp back to original location prior to file update"  
$ gedit src/imagesub.cpp  
-- make changes to src/imagesub.cpp --  
$ git add src/imagesub.cpp  
$ git commit -m "update of src/imagesub.cpp"  
$ git push  

# Old README.md information continues below!

The below will be updated soon! :)

## Caution

Version of Vrui we're using is Vrui-4.6-005. (up to 04/08/2019)
(will be updated)

In case that if Vrui have a new version released, it'll make it an issue to build and we're not changing it in time, 
please change it to the right version for line 154 of vrui_mdf/CMakeLists.txt, line 14 of vrui_mdf/scripts/cod_edi.py.

We're cleaning up the code, so feel free to post an issue when there's any.

## For More Details
### Figure for implementation

![figure_1](https://user-images.githubusercontent.com/24307076/42660737-0c8e63b0-85fa-11e8-95ff-bbf9ec9a4d53.png)

### Demo Video 

https://www.youtube.com/watch?v=fZ7kt_WORCY

### Publication
https://arc.aiaa.org/doi/pdf/10.2514/6.2018-5229

## Software:
### Target Version:
OS:        Ubuntu 16.04<br />
ROS:       Kinetic<br />
Vrui:      4.6-005 (12/06/2018)<br />
SteamVR:   1.1.3b (11/27/2018)<br />

### SteamVR:
SteamCMD is needed to download the right version of SteamVR, to install SteamCMD:<br />
(Source site: https://github.com/OSVR/OSVR-Vive/issues/20)

	$ sudo apt-get install steamcmd
	
To use steamcmd:

	$ steamcmd
	$ login <YOUR-STEAM-NAME>  
	$ download_depot 250820 250823 2021799642281401918
	(download_depot  {appid}  {depotid}  {target manifestid})

For steamvr, appid is 250820, depotid is 25083, manifestid of the target version(1.1.3b, 11/27/2018) is 2021799642281401918.<br />

Look for the right version of steamvr needed in this site:<br />
http://idav.ucdavis.edu/~okreylos/ResDev/Vrui/LinkDownload.html

Manifestid can be find in this site based on the releasing date:<br />
https://steamdb.info/depot/250823/manifests/

Usually, the depot will be downloaded to:

	/.steam/steamcmd/linux32\steamapps\content\app_250820\depot_250823

Replace the package with the current one which is at:<br />

	/.local/share/Steam/steamapps/common/SteamVR

Then run the script of Vrui to install<br />


### Vrui:
Releasing time line of vrui: <br />
http://idav.ucdavis.edu/~okreylos/

The newest version 4.6-005 (up to 04/08/2019) :<br />
http://idav.ucdavis.edu/~okreylos/ResDev/Vrui/LinkDownload.html<br />
(this package is based on Ubuntu 16.04, "Build-Ubuntu.sh" should work)

Follow the instruction on the website above to install, may need to use "sudo" to bash the script<br />

### Gcc:
To switch between different versions(may not be necessary for this repository):

	$ sudo update-alternatives --config gcc
	
### Turtlebot_Gazebo:

Installation tutorial: <br />
http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation<br />
(this package is based on Kinetic, not indigo, need to change the counterpart when installing)

### Hector_quadrotor:

	$ source /opt/ros/kinetic/setup.bash
	$ sudo apt install ros-kinetic-joystick-drivers ros-kinetic-teleop-twist-keyboard
	$ mkdir ~/catkin_ws/src
	$ rosinstall ~/catkin_ws/src /opt/ros/kinetic https://raw.githubusercontent.com/AS4SR/hector_quadrotor/kinetic-devel/tutorials.rosinstall
	$ cd ~/catkin_ws
	$ catkin_make
	$ source devel/setup.bash
for more information, go to: https://github.com/AS4SR/hector_quadrotor

### Other packages:

Xbox controller in Ubuntu:<br />
http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

ROS installation tutorial: <br />
http://wiki.ros.org/kinetic/Installation/Ubuntu<br />
(including methods to search and install indivitual packages)<br />
Use last command in section 1.4 to seach for the correct name of the packages, and install them if needed.

## Displays setting:
	
Vive (usually named as HVR 5" in setting) needs to be set as **secondary display** on the right side of the main moniter<br />
Resolution: 1920*1200 (**16:10**)<br />
Rotation: Normal<br />
Launcher placement: Main monitor<br />
Scale all window contents to match: Main monitor<br />

The first number in the 59th line from the code of "src/imagesub" needs to be the width of the main monitor plus one, so change the resolution of the main monitor or change the number in the code.<br />

	
## Before implementation:

build the tracking node:<br />
(assume the package path is "~/catkin/src/vrui_mdf")


	$cd vrui_mdf/scripts

	$chmod +x cod_edi.py

	$cd ~/catkin_ws
	(Assume the work space is ~/catkin_ws, and the codes start reading from there)

	$rosrun vrui_mdf cod_edi.py

Then you can compile the package with:

	$cd ~/catkin_ws
	
	$catkin_make

model file path in following codes need to be changed according to the local path:<br />
tracking.cpp, controllers.cpp (all the other controllers), standingpoint.cpp, joy_hector.cpp<br />
(I'll fix this ASAP)

## Implementation:
	
	$roslaunch vrui_mdf VR_quadrotor.launch
(using **Vive controllers** to control **quadrotor**)<br />
(system button to switch between methods, right triger to assign the point)<br />

	$roslaunch vrui_mdf VR_quadrotor_outdoor.launch
(using **Vive controllers** to control **quadrotor**, with environment)<br />
(system button to switch between methods, right triger to assign the point)<br />

	$roslaunch vrui_mdf VR_quadrotor_xbox.launch
(using **xbox controller** to control **quadrotor**)<br />
(up&down button to switch between methods, right triger to assign the point)<br />


	$roslaunch vrui_mdf VR_turtlebot.launch
(using **Vive controllers** to control **turtlebot**)<br />
(system button to switch between methods, right triger to assign the point)<br />

	$roslaunch vrui_mdf VR_turtlebot_xbox.launch
(using **xbox controllers** to control **turtlebot**)<br />
(system button to switch between methods, right triger to assign the point)<br />
