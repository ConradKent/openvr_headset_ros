﻿/* This is empty for now. The old tracking.cpp node was built using
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
#include "openvr_ros/utility.h"
#include "openvr_ros/TrackedDevicePose.h"


#include "tf/transform_datatypes.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <math.h>
#include<fstream>



//TODO figure out a less dumb way of doing this
//--globals--
        static float POSx;
        static float POSy;
        static float POSz;

        static float ORIx;
        static float ORIy;
        static float ORIz;
        static float ORIw;


// Initialization of the VR System
inline vr::IVRSystem* initialize()
{
    vr::EVRInitError eError = vr::VRInitError_None;
    vr::IVRSystem* vr_pointer = NULL;
    vr_pointer = vr::VR_Init(&eError, vr::VRApplication_Background);
    if (eError != vr::VRInitError_None)
    {
        vr_pointer = NULL;
        printf("Unable to init VR runtime: %s \n",
        vr::VR_GetVRInitErrorAsEnglishDescription(eError));
        exit(EXIT_FAILURE);
    }
    return vr_pointer;
}

inline void shutdown(vr::IVRSystem* vr_pointer)
{
        if (vr_pointer != NULL)
        {
                vr::VR_Shutdown();
                vr_pointer = NULL;
        }
}


inline void PollEvents(vr::IVRSystem* vr_pointer)
{
    vr::VREvent_t event;
    static const char* classNames[] = {"Invalid Device", "HMD", "Controller", "Tracker", "Tracking Reference", "Display Redirect"};
    static const char* roleNames[]  = {"invalid", "left", "right", "opt out", "treadmill"};

    if(vr_pointer->PollNextEvent(&event, sizeof(event)))
    {
        vr::TrackedDeviceIndex_t id = event.trackedDeviceIndex;
        uint32_t button = event.data.controller.button;
        vr::ETrackedDeviceClass deviceClass = vr_pointer->GetTrackedDeviceClass(id);
        vr::ETrackedControllerRole deviceRole = vr_pointer->GetControllerRoleForTrackedDeviceIndex(id);

        switch(event.eventType)
        {
            case vr::VREvent_TrackedDeviceActivated:
                ROS_WARN("EVENT (OpenVR) %s (id=%d, role=%s) attached.", classNames[(int)deviceClass], id, roleNames[(int)deviceRole]);
                break;
            case vr::VREvent_TrackedDeviceDeactivated:
                ROS_WARN("EVENT (OpenVR) %s (id=%d, role=%s) detached.", classNames[(int)deviceClass], id, roleNames[(int)deviceRole]);
                break;
            case vr::VREvent_TrackedDeviceUpdated:
                ROS_WARN("EVENT (OpenVR) %s (id=%d, role=%s) updated.", classNames[(int)deviceClass], id, roleNames[(int)deviceRole]);
                break;
            case vr::VREvent_EnterStandbyMode:
                ROS_WARN("EVENT (OpenVR) Entered Standby mode.");
                break;
            case vr::VREvent_LeaveStandbyMode:
                ROS_WARN("EVENT (OpenVR) Leave Standby mode.");
                break;

            default:
                // printf("EVENT--(OpenVR) Event: %d\n", event.eventType);
                break;
        }
        }
}

//-----------------------------------------------------------------------------
// Purpose: Calculates quaternion (qw,qx,qy,qz) representing the rotation
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
// from: https://www.codeproject.com/Articles/1171122/How-to-Get-Raw-Positional-Data-from-HTC-Vive
//-----------------------------------------------------------------------------

vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix) {
    vr::HmdQuaternion_t q;

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
    return q;
}
//-----------------------------------------------------------------------------
// Purpose: Extracts position (x,y,z).
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
// from: https://www.codeproject.com/Articles/1171122/How-to-Get-Raw-Positional-Data-from-HTC-Vive
//-----------------------------------------------------------------------------

vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix) {
    vr::HmdVector3_t vector;

    vector.v[0] = matrix.m[0][3];
    vector.v[1] = matrix.m[1][3];
    vector.v[2] = matrix.m[2][3];

    return vector;
}

inline void PublishTrackedDevicePose(vr::IVRSystem* vr_pointer, const vr::TrackedDevicePose_t& trackedDevicePose)
{
    // Do not waste time, just record this moment's timestamp.
   /*
    openvr_ros::TrackedDevicePose msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "";
    */

    vr::ETrackedDeviceClass trackedDeviceClass   = vr::TrackedDeviceClass_HMD; //Tracked vr_pointer->GetTrackedDeviceClass(id);

    //vr::ETrackedControllerRole trackedDeviceRole = vr_pointer->GetControllerRoleForTrackedDeviceIndex(id);

    /*
    msg.device_header.ID = (uint16_t) id;
    msg.device_header.Class = (uint8_t) trackedDeviceClass;
    msg.device_header.Role  = (uint8_t) trackedDeviceRole;
    msg.device_header.TrackingResult = (uint8_t) trackedDevicePose.eTrackingResult;
    msg.device_header.PoseIsValid = (bool) trackedDevicePose.bPoseIsValid;
    msg.device_header.DeviceIsConnected = (bool) trackedDevicePose.bDeviceIsConnected;
    */

    vr::HmdVector3_t position;
    position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
    POSx = position.v[0];
    POSy = position.v[1];
    POSz = position.v[2];

    vr::HmdQuaternion_t quaternion;
    quaternion = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
    ORIx = quaternion.x;
    ORIy = quaternion.y;
    ORIz = quaternion.z;
    ORIw = quaternion.w;

    /*
    vr::HmdVector3_t velocity;
    velocity = trackedDevicePose.vVelocity;
    msg.velocity.linear.x = velocity.v[0];
    msg.velocity.linear.y = velocity.v[1];
    msg.velocity.linear.z = velocity.v[2];

    vr::HmdVector3_t angular;
    angular = trackedDevicePose.vAngularVelocity;
    msg.velocity.angular.x = angular.v[0];
    msg.velocity.angular.y = angular.v[1];
    msg.velocity.angular.z = angular.v[2];

    publisher.publish(msg);
    */
}

inline void PollPoses(vr::IVRSystem* vr_pointer)
{
    /*
    for (vr::TrackedDeviceIndex_t id = 0; id < vr::k_unMaxTrackedDeviceCount; id++)
    {
        if (!vr_pointer->IsTrackedDeviceConnected(id))
            continue;
    */
//TODO change this back to vr_pointer->GetTrackedDeviceClass(id) instead of TrackedDeviceClass_HMD when we track multiple devices.
        vr::ETrackedDeviceClass trackedDeviceClass = vr::TrackedDeviceClass_HMD;

        vr::TrackedDevicePose_t trackedDevicePose;
        /*
        vr::VRControllerState_t controllerState;
//TODO eventually we will iterate through trackedDeviceClass so we can track each device (HMD, controllers, and generic trackers if we have any) for now I'm focusing on HMD.
        switch (trackedDeviceClass)
        {
            case vr::ETrackedDeviceClass::TrackedDeviceClass_HMD:
            */
                // printf("Headset\n");
                vr_pointer->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
                PublishTrackedDevicePose(vr_pointer, trackedDevicePose);
                /*break;

            case vr::ETrackedDeviceClass::TrackedDeviceClass_Controller:
                // printf("Controller: %d\n", id);
                vr_pointer->GetControllerStateWithPose(vr::TrackingUniverseStanding, id, &controllerState, sizeof(controllerState), &trackedDevicePose);
                PublishTrackedDevicePose(vr_pointer, publisher, id, trackedDevicePose);
                break;
            case vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker:
                // printf("Tracker: %d\n", id);
                vr_pointer->GetControllerStateWithPose(vr::TrackingUniverseStanding, id, &controllerState, sizeof(controllerState), &trackedDevicePose);
                PublishTrackedDevicePose(vr_pointer, publisher, id, trackedDevicePose);
                break;
            default:
                break;

        }
    }
*/
}

//TODO change this to injest from this script instead of external msg (I was testing with just taking in a message from openvr_ros but I think that will be overly complicated, eventually I'll just use all the openvr_ros stuff in a header file I think)
void callback_positioner(const openvr_ros::TrackedDevicePose::ConstPtr& msg)
{
    //struct Position pos;
    //struct Orientation ori;
    std::cout <<"CallbackTick"<< std::endl;
    POSx = msg->pose.position.x;
    POSy = msg->pose.position.y;
    POSz = msg->pose.position.z;

    ORIx = msg->pose.orientation.x;
    ORIy = msg->pose.orientation.y;
    ORIz = msg->pose.orientation.z;
    ORIw = msg->pose.orientation.w;
}

int main(int argc,char* argv[])
{
    vr::IVRSystem* vr_pointer = NULL;
    vr_pointer = initialize();


    /* Set up ros node and define publisher for gazebo */
    ros::init(argc, argv, "camera_state");
    ros::NodeHandle n;
    ros::Rate r(180);
    ros::service::waitForService("/gazebo/spawn_urdf_model", -1);
    /* gazebo model state publisher and topic */
    ros::Publisher gazebo_pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    gazebo_msgs::ModelState camera;
    camera.model_name = "vr_view";
    camera.reference_frame="world";

    ros::Subscriber sub=n.subscribe("tracked_device_pose", 1, callback_positioner); //TODO make sure I understand what the "1" argument is doing correctly. Should be fine as long as I spin ROS right?
    //try
    //{
        //CHECK this and make sure we're spawning correctly. Gazebo recommends spawning via roslaunch so we might switch to that.
        gazebo_msgs::SpawnModel sm;
        ros::ServiceClient spawn_model;
        std::ifstream ifs;
        ifs.open("/home/conrad/catkin_ws/src/openvr_headset_ros/models/vr_view/model.sdf"); //TODO generalize.
        std::stringstream stringstream1;
        stringstream1 << ifs.rdbuf();
        sm.request.model_name = "vr_view";
        sm.request.model_xml = stringstream1.str();
        sm.request.robot_namespace = ros::this_node::getNamespace();
        sm.request.reference_frame = "world";
        spawn_model.call(sm);

        system("rosrun gazebo_ros spawn_model -file /home/conrad/catkin_ws/src/openvr_headset_ros/models/vr_view/model.sdf -sdf -model vr_view -y 0 -x 0 -z 1");


        //Publish ros camera loop (this will eventually cover tracking as well, camera position will be updated on this loop)
        while(ros::ok())
        {
            PollEvents(vr_pointer);
            PollPoses(vr_pointer/*, tracked_device_pose_publisher*/);

            camera.pose.position.x = POSx;
            camera.pose.position.y = POSy;
            camera.pose.position.z = POSz;

            camera.pose.orientation.x = ORIx;
            camera.pose.orientation.y = ORIy;
            camera.pose.orientation.z = ORIz;
            camera.pose.orientation.w = ORIw;

            gazebo_pub.publish(camera);

            ros::spinOnce();
            r.sleep();

        }
    //}
}
