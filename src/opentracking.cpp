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

#include <openvr_headset_ros/Vive.h>

/*TODO LIST
 * Controllers: Trackpad
 * Take more stuff out of main and add it to functions for better organization
 * Make a header file so this file isn't a jillion lines long
 * replace the globals with something less silly, could probably just add them to the local pose functions instead of sending the data around.
 *
 */

//--globals--
        static float POSx;
        static float POSy;
        static float POSz;

        static float ORIx;
        static float ORIy;
        static float ORIz;
        static float ORIw;

        static float POSlx;
        static float POSly;
        static float POSlz;

        static float ORIlx;
        static float ORIly;
        static float ORIlz;
        static float ORIlw;

        static float POSrx;
        static float POSry;
        static float POSrz;

        static float ORIrx;
        static float ORIry;
        static float ORIrz;
        static float ORIrw;

        int triggerpress_right;
        int triggerpress_left;
        int grippress_right;
        int grippress_left;

        vr::TrackedDeviceIndex_t controller_Left_id;
        vr::TrackedDeviceIndex_t controller_Right_id;


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

inline void ButtonPub(vr::VREvent_t event, vr::ETrackedControllerRole deviceRole)
{


	switch(deviceRole) //test for which controller (left or right) was pressed
	{	
		case vr::TrackedControllerRole_Invalid: //The controller isn't visible to base stations, or something of that sort
			break;
		case vr::TrackedControllerRole_RightHand: // case Right Controller
            switch( event.data.controller.button )
            {
                    case vr::k_EButton_Grip:  //case grip
                    switch(event.eventType)
                    {
                            case vr::VREvent_ButtonPress:   // If the grip was pressed...
                            grippress_right=1;
                            ROS_WARN("Right Grip Press");
                            break;

                            case vr::VREvent_ButtonUnpress: // If the grip was released...
                            grippress_right=0;
                            break;
                    }
                    break; //break grip

                    case vr::k_EButton_SteamVR_Trigger: //case trigger
                    switch(event.eventType)
                    {
                            case vr::VREvent_ButtonPress:  //If the trigger was pressed...
                            triggerpress_right=1;
                            ROS_WARN("Right Trigger Press");
                            break;

                            case vr::VREvent_ButtonUnpress://If the trigger was released...
                            triggerpress_right=0;
                            break;
                    }
                    break; //break trigger

            } 
        break; //break Right Controller
        
        
        
        
        
        		case vr::TrackedControllerRole_LeftHand: // case Left Controller
            switch( event.data.controller.button )
            {
                    case vr::k_EButton_Grip:  //case grip
                    switch(event.eventType)
                    {
                            case vr::VREvent_ButtonPress:   // If the grip was pressed...
                            grippress_left=1;
                            ROS_WARN("Left Grip Press");
                            break;

                            case vr::VREvent_ButtonUnpress: // If the grip was released...
                            grippress_left=0;
                            break;
                    }
                    break; //break grip

                    case vr::k_EButton_SteamVR_Trigger: //case trigger
                    switch(event.eventType)
                    {
                            case vr::VREvent_ButtonPress:  //If the trigger was pressed...
                            triggerpress_left=1;
                            ROS_WARN("Left Trigger Press");
                            break;

                            case vr::VREvent_ButtonUnpress://If the trigger was released...
                            triggerpress_left=0;
                            break;
                    }
                    break; //break trigger

            } //switch(event.data...
        break; //break Left Controller
	} //switch(deviceRole)
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
                if (event.eventType >= 200 && event.eventType <= 203) //Button events range from 200-203
                    ButtonPub(event, deviceRole);


                // printf("EVENT--(OpenVR) Event: %d\n", event.eventType);
                //break;
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

inline void PublishTrackedDevicePose(vr::IVRSystem* vr_pointer,
                                     const ros::Publisher &publisher,
                                     vr::TrackedDeviceIndex_t id,
                                     const vr::TrackedDevicePose_t &trackedDevicePose)
{


    vr::ETrackedDeviceClass trackedDeviceClass   = vr_pointer->GetTrackedDeviceClass(id);

    vr::ETrackedControllerRole trackedDeviceRole = vr_pointer->GetControllerRoleForTrackedDeviceIndex(id);


    /* NEW SETUP <-NO ITS NOT, this is a weird setup because the left and right identifiers are arbitrary. Instead of controller_X_id I'm just using the vr::TrackedControllerRole now because it's defined by openvr and will be consistent. Hopefully.
     * if (id=0) set the HMD globals equal to the position of this tracked device's ID
     * if (id=controller_Right_id) set the right controller globals equal to the position of this tracked device's ID
     * if (id=controller_Left_id) set the left controller globals equal to the position of this tracked device's ID
     */
    if (id==0)
    {
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
    }
    //TODO, revisit this NEW CHANGE, TRYING TO STANDARDIZE IDing, controller_Left_id and controller_Right_id might be unnecessary with this (Dec 1 2021)
    else if (trackedDeviceRole == vr::TrackedControllerRole_RightHand)
    {
        vr::HmdVector3_t position;
        position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
        POSrx = position.v[0];
        POSry = position.v[1];
        POSrz = position.v[2];

        vr::HmdQuaternion_t quaternion;
        quaternion = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
        ORIrx = quaternion.x;
        ORIry = quaternion.y;
        ORIrz = quaternion.z;
        ORIrw = quaternion.w;
    }
    else if (trackedDeviceRole == vr::TrackedControllerRole_LeftHand)
    {
        vr::HmdVector3_t position;
        position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
        POSlx = position.v[0];
        POSly = position.v[1];
        POSlz = position.v[2];

        vr::HmdQuaternion_t quaternion;
        quaternion = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
        ORIlx = quaternion.x;
        ORIly = quaternion.y;
        ORIlz = quaternion.z;
        ORIlw = quaternion.w;
    }
    }


inline void PollPoses(vr::IVRSystem* vr_pointer, const ros::Publisher &publisher)
{

    for (vr::TrackedDeviceIndex_t id = 0; id < vr::k_unMaxTrackedDeviceCount; id++)
    {
        if (!vr_pointer->IsTrackedDeviceConnected(id))
            continue;

        //Old code for not using cases vr::ETrackedDeviceClass trackedDeviceClass = vr::TrackedDeviceClass_HMD;

        vr::ETrackedDeviceClass trackedDeviceClass = vr_pointer->GetTrackedDeviceClass(id);

        vr::TrackedDevicePose_t trackedDevicePose;

        vr::VRControllerState_t controllerState;

        switch (trackedDeviceClass)
        {
            case vr::ETrackedDeviceClass::TrackedDeviceClass_HMD:

                // printf("Headset\n");
                vr_pointer->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
                PublishTrackedDevicePose(vr_pointer, publisher, id, trackedDevicePose);
                break;

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

}




int main(int argc,char* argv[])
{
    vr::IVRSystem* vr_pointer = NULL;
    vr_pointer = initialize();


    /* Set up ros node and define publisher for gazebo */
    ros::init(argc, argv, "camera_state");
    ros::NodeHandle n;
    ros::NodeHandle nh;

    //TODO try to do this without setting up a new publisher maybe? seems ineffecient since everything's happening in this program.
    ros::Publisher tracked_device_pose_publisher = nh.advertise<openvr_ros::TrackedDevicePose>("tracked_device_pose", 300);


    ros::Rate r(180);
    ros::service::waitForService("/gazebo/spawn_urdf_model", -1);
    /* gazebo model state publisher and topic */
    ros::Publisher gazebo_pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    gazebo_msgs::ModelState camera,controller_left,controller_right;

    openvr_headset_ros::Vive vive;
    vive.user_name = "vive";
    ros::Publisher vive_state = n.advertise<openvr_headset_ros::Vive>("openvr_headset_ros/vive", 10);

    //Get the recommended resolution sizes
    uint32_t pnWidth;
    uint32_t pnHeight;
    vr_pointer->GetRecommendedRenderTargetSize(&pnWidth, &pnHeight );

    std::cout << "Width: " << pnWidth << std::endl;
    std::cout << "Height: " << pnHeight << std::endl;
    //TODO clean this up, this is just proof of concept with code that I know works for now. I'll generalize the path to the sdf and get the vr_view tags as variables later.
    if (pnHeight==2056 && pnWidth==1852) {
            //Use Vive models
            camera.model_name = "vr_view_vive";
            camera.reference_frame="world";

            //ros::Subscriber sub=n.subscribe("tracked_device_pose", 1, callback_positioner); //TODO make sure I understand what the "1" argument is doing correctly. Should be fine as long as I spin ROS right?

            //{
                //CHECK this and make sure we're spawning correctly. Gazebo recommends spawning via roslaunch so we might switch to that.
                gazebo_msgs::SpawnModel sm;
                ros::ServiceClient spawn_model;
                std::ifstream ifs;
                ifs.open("/home/conrad/catkin_ws/src/openvr_headset_ros/models/vr_view_vive/model.sdf"); //TODO generalize.
                std::stringstream stringstream;
                stringstream << ifs.rdbuf();
                sm.request.model_name = "vr_view_vive";
                sm.request.model_xml = stringstream.str();
                sm.request.robot_namespace = ros::this_node::getNamespace();
                sm.request.reference_frame = "world";
                spawn_model.call(sm);

                system("rosrun gazebo_ros spawn_model -file /home/conrad/catkin_ws/src/openvr_headset_ros/models/vr_view_vive/model.sdf -sdf -model vr_view_vive -y 0 -x 0 -z 1");
}   else if (pnHeight==2628 && pnWidth==2368) {
            //Use Vive Pro models
            camera.model_name = "vr_view_vive_pro";
            camera.reference_frame="world";

            //ros::Subscriber sub=n.subscribe("tracked_device_pose", 1, callback_positioner); //TODO make sure I understand what the "1" argument is doing correctly. Should be fine as long as I spin ROS right?

            //{
                //CHECK this and make sure we're spawning correctly. Gazebo recommends spawning via roslaunch so we might switch to that.
                gazebo_msgs::SpawnModel sm;
                ros::ServiceClient spawn_model;
                std::ifstream ifs;
                ifs.open("/home/conrad/catkin_ws/src/openvr_headset_ros/models/vr_view_vive_pro/model.sdf"); //TODO generalize.
                std::stringstream stringstream;
                stringstream << ifs.rdbuf();
                sm.request.model_name = "vr_view_vive_pro";
                sm.request.model_xml = stringstream.str();
                sm.request.robot_namespace = ros::this_node::getNamespace();
                sm.request.reference_frame = "world";
                spawn_model.call(sm);

                system("rosrun gazebo_ros spawn_model -file /home/conrad/catkin_ws/src/openvr_headset_ros/models/vr_view_vive_pro/model.sdf -sdf -model vr_view_vive_pro -y 0 -x 0 -z 1");
}   else {std::cout << "no camera model found for this headset" << std::endl;}



    //----------------------------------------------------------
    //Get number of controllers and set an identifier for the left and right (Left and right aren't actually the controllers set as left and right, this is just to keep track of them)
    //----------------------------------------------------------

    int controller_count = 0;
    for (vr::TrackedDeviceIndex_t id = 0; id < vr::k_unMaxTrackedDeviceCount; id++)
    {
        if (controller_count < 3){
        if (!vr_pointer->IsTrackedDeviceConnected(id))
            continue;

        vr::ETrackedDeviceClass trackedDeviceClass = vr_pointer->GetTrackedDeviceClass(id);

        vr::TrackedDevicePose_t trackedDevicePose;

        vr::VRControllerState_t controllerState;

        switch (trackedDeviceClass)
        {
            case vr::ETrackedDeviceClass::TrackedDeviceClass_HMD:
                std::cout << "HMD found at" << id << " (should be 0) " << std::endl;
                break;

            case vr::ETrackedDeviceClass::TrackedDeviceClass_Controller:
                if (controller_count==0){
                    controller_Right_id=id;
                    controller_count=controller_count+1;
                }
                    else {
                    controller_Left_id=id;
                    controller_count=controller_count+1;
                }
                break;
            case vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker:
            std::cout << "trackers not supported" << std::endl;
                break;
            default:
                break;

        }
        }

        else {
            std::cout << "ERROR: openvr_headset_ros only supports two controllers at a time" << std::endl;
            shutdown(vr_pointer);
        }
        }

    //---------Spawn Controllers------------

    controller_left.model_name = "Vive_Controller_left";
    controller_left.reference_frame="world";

    gazebo_msgs::SpawnModel sm;
    ros::ServiceClient spawn_model;
    std::ifstream ifs1,ifs2;
    ifs1.open("/home/conrad/catkin_ws/src/openvr_headset_ros/models/Gazebo_Vive_Wand/left/model.sdf"); //TODO generalize.
    std::stringstream stringstream1;
    stringstream1 << ifs1.rdbuf();
    sm.request.model_name = "Gazebo_Vive_Wand";
    sm.request.model_xml = stringstream1.str();
    sm.request.robot_namespace = ros::this_node::getNamespace();
    sm.request.reference_frame = "world";
    spawn_model.call(sm);

        system("rosrun gazebo_ros spawn_model -file /home/conrad/catkin_ws/src/openvr_headset_ros/models/Gazebo_Vive_Wand/left/model.sdf -sdf -model Vive_Controller_left -y 0 -x 0 -z 1");


    controller_right.model_name = "Vive_Controller_right";
    controller_right.reference_frame="world";



    ifs2.open("/home/conrad/catkin_ws/src/openvr_headset_ros/models/Gazebo_Vive_Wand/right/model.sdf"); //TODO generalize.
    std::stringstream stringstream2;
    stringstream2 << ifs1.rdbuf();
    sm.request.model_name = "Gazebo_Vive_Wand";
    sm.request.model_xml = stringstream2.str();
    sm.request.robot_namespace = ros::this_node::getNamespace();
    sm.request.reference_frame = "world";
    spawn_model.call(sm);

    system("rosrun gazebo_ros spawn_model -file /home/conrad/catkin_ws/src/openvr_headset_ros/models/Gazebo_Vive_Wand/right/model.sdf -sdf -model Vive_Controller_right -y 0 -x 0 -z 1");


        while(ros::ok())
        {
            PollEvents(vr_pointer);
            PollPoses(vr_pointer, tracked_device_pose_publisher);



            camera.pose.position.x = -1*POSz;
            camera.pose.position.y = -1*POSx;
            camera.pose.position.z = POSy;

            camera.pose.orientation.x = -1*ORIz;
            camera.pose.orientation.y = -1*ORIx;
            camera.pose.orientation.z = ORIy;
            camera.pose.orientation.w = ORIw;


            /* left controller */
            std::cout << POSlz << std::endl;
            controller_left.pose.position.x = -1*POSlz;
            controller_left.pose.position.y = -1*POSlx;
            controller_left.pose.position.z = POSly;

            controller_left.pose.orientation.x = -1*ORIlz;
            controller_left.pose.orientation.y = -1*ORIlx;
            controller_left.pose.orientation.z = ORIly;
            controller_left.pose.orientation.w = ORIlw;
            
            vive.ctrl_left.buttons.trigger=triggerpress_left;	//from "ButtonPub" function
            vive.ctrl_left.buttons.system=grippress_left;
            
/*
            controller_left.buttons.system = state.getButtonState(2);
            controller_left.buttons.grip = state.getButtonState(3);
            controller_left.buttons.menu = state.getButtonState(4);
            controller_left.buttons.trigger = state.getButtonState(5);
            controller_left.buttons.trackpad = state.getButtonState(6);
*/

            /* right controller */
            controller_right.pose.position.x = -1*POSrz;
            controller_right.pose.position.y = -1*POSrx;
            controller_right.pose.position.z = POSry;

            controller_right.pose.orientation.x = -1*ORIrz;
            controller_right.pose.orientation.y = -1*ORIrx;
            controller_right.pose.orientation.z = ORIry;
            controller_right.pose.orientation.w = ORIrw;

            vive.ctrl_right.buttons.trigger=triggerpress_right;	//from "ButtonPub" function
            vive.ctrl_right.buttons.system=grippress_right;

            vive.ctrl_right.pose=controller_right.pose;
            vive.ctrl_left.pose=controller_left.pose;
            /*
            vive.ctrl_right.pose.position.x = -1*pos_right[2] - bias_2 + getmodelstate.response.pose.position.x;
            vive.ctrl_right.pose.position.y = -1*pos_right[0] - bias_0 + getmodelstate.response.pose.position.y;
            vive.ctrl_right.pose.position.z = pos_right[1] + bias_1 + getmodelstate.response.pose.position.z;

            vive.ctrl_right.pose.orientation.x = -1*quaternion_right[2];
            vive.ctrl_right.pose.orientation.y = -1*quaternion_right[0];
            vive.ctrl_right.pose.orientation.z = quaternion_right[1];
            vive.ctrl_right.pose.orientation.w = quaternion_right[3];
            */

               vive_state.publish(vive);

            gazebo_pub.publish(camera);
            gazebo_pub.publish(controller_left);
            gazebo_pub.publish(controller_right);
            ros::spinOnce();
            r.sleep();

        }
    //}
}
