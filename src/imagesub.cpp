/*

This new version of imagesub subscribes the images
from ROS to OpenCV, then it takes the BGR OpenCV
image data and adds it to an OpenGL texture2d in
RGBA8, then it submits those texture2d's to each
eye of a VR headset using OpenVR.

*/
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
//#include <opencv2/highgui/highgui.hpp> CHECK OpenCV versioning/how to get this futureproofed. We're on opencv version 4 now.
#include <cv_bridge/cv_bridge.h>
//#include <time.h> CHECK this
//#include <tf/transform_broadcaster.h> CHECK this
//#include <math.h> CHECK this
#include <openvr/openvr.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
//Check for more needed includes

// define callback function in a class so that data running inside the class can be used globally
class Listener_image
{
public:
	  cv::Mat image;

	  void callback(const sensor_msgs::ImageConstPtr& msg)
	    {
            //copy image data to the image under the same class, which will be assign as a pointer. Use rba8 format as this is what OpenVR requires for rendering.
            cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(image);
            }

};

/* Don't need Vive msgs for now. May remove later if we stop using a separate node for tracking.
class Vive_Listener
{
public:
          vrui_mdf::Vive vive;

          void callback(const vrui_mdf::Vive& msg)
            {
                vive = msg;
            }
};
*/

//from https://github.com/matinas/openvrsimplexamples/blob/master/openvrsimplexamples/src/main.cpp
vr::IVRSystem* vr_system;
vr::TrackedDevicePose_t tracked_device_pose[vr::k_unMaxTrackedDeviceCount];

std::string driver_name, driver_serial;
std::string tracked_device_type[vr::k_unMaxTrackedDeviceCount];
int init_OpenVR();

//-----------------------------------------------------------------------------
// Purpose: helper to get a string from a tracked device property and turn it
//			into a std::string. FROM https://github.com/matinas/openvrsimplexamples/blob/master/openvrsimplexamples/src/utils.cpp
//-----------------------------------------------------------------------------
std::string GetTrackedDeviceString(vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL)
{
        uint32_t requiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
        if( requiredBufferLen == 0 )
                return "";

        char *pchBuffer = new char[requiredBufferLen];
        requiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice,prop,pchBuffer,requiredBufferLen,peError);
        std::string sResult = pchBuffer;
        delete[] pchBuffer;

        return sResult;
}

//-----------------------------------------------------------------------------
// Purpose: helper to get a string from a tracked device type class. FROM https://github.com/matinas/openvrsimplexamples/blob/master/openvrsimplexamples/src/utils.cpp
//-----------------------------------------------------------------------------
std::string GetTrackedDeviceClassString(vr::ETrackedDeviceClass td_class) {

        std::string str_td_class = "Unknown class";

        switch (td_class)
        {
        case vr::TrackedDeviceClass_Invalid:			// = 0, the ID was not valid.
                str_td_class = "invalid";
                break;
        case vr::TrackedDeviceClass_HMD:				// = 1, Head-Mounted Displays
                str_td_class = "hmd";
                break;
        case vr::TrackedDeviceClass_Controller:			// = 2, Tracked controllers
                str_td_class = "controller";
                break;
        case vr::TrackedDeviceClass_GenericTracker:		// = 3, Generic trackers, similar to controllers
                str_td_class = "generic tracker";
                break;
        case vr::TrackedDeviceClass_TrackingReference:	// = 4, Camera and base stations that serve as tracking reference points
                str_td_class = "base station";
                break;
        case vr::TrackedDeviceClass_DisplayRedirect:	// = 5, Accessories that aren't necessarily tracked themselves, but may redirect video output from other tracked devices
                str_td_class = "display redirect";
                break;
        }

        return str_td_class;
}


int main(int argc, char **argv)
{
    // setup ros node
    ros::init(argc, argv, "image_sub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    //initialize OpenVR
    if (init_OpenVR() != 0) return -1;

    //define and generate textures. We're clamping the textures and using linear interpolation for now. OpenGL information on textures is from https://open.gl/textures
    GLuint tex_left;
    glGenTextures(1,&tex_left);

    GLuint tex_right;
    glGenTextures(1,&tex_right);
    uint32_t pnWidth;
    uint32_t pnHeight;
    vr::TrackedDevicePose_t m_rTrackedDevicePose[ vr::k_unMaxTrackedDeviceCount ];

    vr_system->GetRecommendedRenderTargetSize(&pnWidth, &pnHeight );
    ros::Rate r(90);

    //define class for callback class and subscriber
    Listener_image listener_left,listener_right;
    image_transport::Subscriber sub_left = it.subscribe("/camera/rgb/left_eye", 1, &Listener_image::callback, &listener_left);
    image_transport::Subscriber sub_right = it.subscribe("/camera/rgb/right_eye", 1, &Listener_image::callback, &listener_right);
  
//I'm not sure if I need the images as mats to read the image data out of? I'll try reading the bgr data out of the listeners directly for now. Otherwise, we'll read from image_left.data and image_right.data
    cv::Mat image_left(pnHeight,pnWidth, CV_8UC3,cv::Scalar(0,255,255));
    cv::Mat image_right(pnHeight,pnWidth, CV_8UC3,cv::Scalar(0,255,255));

    listener_left.image = image_left(cv::Range(0,1200),cv::Range(0,960));
    listener_right.image = image_right(cv::Range(0,1200),cv::Range(0,960));


/* Not listening for vive data for now.
    Vive_Listener vive_data;
    ros::Subscriber sub_vive = nh.subscribe("vrui/vive", 1, &Vive_Listener::callback, &vive_data);
*/

    while(ros::ok())
    {
        ros::spinOnce();
        // ros::spin() works too, but extra code can run outside the callback function between each spinning if spinOnce() is used

            if(listener_left.image.cols!=0 && listener_right.image.cols!=0)
            {
                //format of waitgetposes used in hellovr_opengl
                vr::VRCompositor()->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0);

                glBindTexture(GL_TEXTURE_2D, tex_left);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);    //hellovr_opengl runs glTexParameteri once per cycle as far as I can tell, which I wanted to take note of because that seems weird.
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA,image_left.cols,image_left.rows,0,GL_BGR,GL_UNSIGNED_BYTE,image_left.data);
                vr::Texture_t leftEyeTexture = {(void*)(uintptr_t)tex_left, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
                vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture );

                glBindTexture(GL_TEXTURE_2D, tex_right);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA,image_right.cols,image_right.rows,0,GL_BGR,GL_UNSIGNED_BYTE,image_right.data);
                vr::Texture_t rightEyeTexture = {(void*)(uintptr_t)tex_right, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
                vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );

            }
            r.sleep();
    }

    return 0;
}

//from https://github.com/matinas/openvrsimplexamples/blob/master/openvrsimplexamples/src/main.cpp
int init_OpenVR()
{
        // Check whether there is an HMD plugged-in and the SteamVR runtime is installed
        if (vr::VR_IsHmdPresent())
        {
                std::cout << "An HMD was successfully found in the system" << std::endl;

                if (vr::VR_IsRuntimeInstalled()) {
                        std::cout << "Runtime correctly installed" << std::endl;
                }
                else
                {
                        std::cout << "Runtime was not found, quitting app" << std::endl;
                        return -1;
                }
        }
        else
        {
                std::cout << "No HMD was found in the system, quitting app" << std::endl;
                return -1;
        }

        // Load the SteamVR Runtime
        vr::HmdError err;
        vr_system = vr::VR_Init(&err,vr::EVRApplicationType::VRApplication_Scene);
        vr_system == NULL ? std::cout << "Error while initializing SteamVR runtime. Error code is " << vr::VR_GetVRInitErrorAsSymbol(err) << std::endl : std::cout << "SteamVR runtime successfully initialized" << std::endl;

        // Obtain some basic information given by the runtime
        int base_stations_count = 0;
        for (uint32_t td=vr::k_unTrackedDeviceIndex_Hmd; td<vr::k_unMaxTrackedDeviceCount; td++) {

                if (vr_system->IsTrackedDeviceConnected(td))
                {
                        vr::ETrackedDeviceClass tracked_device_class = vr_system->GetTrackedDeviceClass(td);

                        std::string td_type = GetTrackedDeviceClassString(tracked_device_class);
                        tracked_device_type[td] = td_type;

                        std::cout << "Tracking device " << td << " is connected " << std::endl;
                        std::cout << "  Device type: " << td_type << ". Name: " << GetTrackedDeviceString(vr_system,td,vr::Prop_TrackingSystemName_String) << std::endl;

                        if (tracked_device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference) base_stations_count++;

                        if (td == vr::k_unTrackedDeviceIndex_Hmd)
                        {
                                // Fill variables used for obtaining the device name and serial ID (used later for naming the SDL window)
                                driver_name = GetTrackedDeviceString(vr_system,vr::k_unTrackedDeviceIndex_Hmd,vr::Prop_TrackingSystemName_String);
                                driver_serial = GetTrackedDeviceString(vr_system,vr::k_unTrackedDeviceIndex_Hmd,vr::Prop_SerialNumber_String);
                        }
                }
                else
                        std::cout << "Tracking device " << td << " not connected" << std::endl;
        }

        // Check whether both base stations are found, not mandatory but just in case...
        if (base_stations_count < 2)
        {
                std::cout << "There was a problem indentifying the base stations, please check they are powered on" << std::endl;

                return -1;
        }

        return 0;
}
