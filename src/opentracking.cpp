/* This is empty for now. The old tracking.cpp node was built using
 * a python code called "cod_edi.py". The new opentracking node will
 * not do this and will be included in the src folder as a cpp file.
 * This code will be written from scratch but use a similar structure
 * to the tracking node built by cod_edi.py */
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <stdexcept>

//#include <Misc/FunctionCalls.h>
//#include <Misc/ConfigurationFile.h>
//#include <Realtime/Time.h>
//#include <Geometry/AffineCombiner.h>

#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <tf/transform_broadcaster.h>

#include "tf/transform_datatypes.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <math.h>
#include<fstream>

int main(int argc,char* argv[])
{
    //CHECK that this is correct.
    /* Set up ros node and define publisher for gazebo */
    ros::init(argc, argv, "camera_state");
    ros::NodeHandle n;
    ros::Rate r(90);
    ros::service::waitForService("/gazebo/spawn_urdf_model", -1);
    /* gazebo model state publisher and topic */
    ros::Publisher gazebo_pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    gazebo_msgs::ModelState camera;
    camera.model_name = "vr_view";
    camera.reference_frame="world";

    //try
    //{
        //CHECK this and make sure we're spawning correctly. Gazebo recommends spawning via roslaunch so we might switch to that.
        gazebo_msgs::SpawnModel sm;
        ros::ServiceClient spawn_model;
        std::ifstream ifs;
        ifs.open("/open_headset_ros/models/vr_view/model.sdf"); //CHECK to generalize.
        std::stringstream stringstream1;
        stringstream1 << ifs.rdbuf();
        sm.request.model_name = "vr_view";
        sm.request.model_xml = stringstream1.str();
        sm.request.robot_namespace = ros::this_node::getNamespace();
        sm.request.reference_frame = "world";
        spawn_model.call(sm);

        system("rosrun gazebo_ros spawn_model -file $(find vrui_mdf)/models/vr_view/model.sdf -sdf -model vr_view -y 0 -x 0 -z 1");


        //Publish ros camera loop (this will eventually cover tracking as well, camera position will be updated on this loop)
        while(ros::ok())
        {
            gazebo_pub.publish(camera);
        }
    //}
}
