/*

This new version of imagesub subscribes the images
from ROS to OpenCV, then it takes the BGR OpenCV
image data and adds it to an OpenGL texture2d in
RGBA8, then it submits those texture2d's to each
eye of a VR headset using OpenVR.

*/
#include <stdlib.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
//#include <opencv2/highgui/highgui.hpp> CHECK OpenCV versioning/how to get this futureproofed. We're on opencv version 4 now.
#include <cv_bridge/cv_bridge.h>
//#include <time.h> CHECK this
//#include <tf/transform_broadcaster.h> CHECK this
//#include <math.h> CHECK this
#include <openvr.h>
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

//from https://github.com/matinas/openvrsimplexamples/blob/master/openvrsimplexamples/src/main.cpp
vr::IVRSystem* vr_system;
vr::TrackedDevicePose_t tracked_device_pose[vr::k_unMaxTrackedDeviceCount];

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

int main(int argc, char **argv)
{
    // setup ros node
    ros::init(argc, argv, "image_sub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    //define and generate textures. We're clamping the textures and using linear interpolation for now. OpenGL information on textures is from https://open.gl/textures
    GLuint tex_left;
    glGenTextures(1,&tex_left);

    GLuint tex_right;
    glGenTextures(1,&tex_right);
    int pnWidth;
    int pnHeight;
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
                cout << "An HMD was successfully found in the system" << endl;

                if (vr::VR_IsRuntimeInstalled()) {
                        const char* runtime_path = vr::VR_RuntimePath();
                        cout << "Runtime correctly installed at '" << runtime_path << "'" << endl;
                }
                else
                {
                        cout << "Runtime was not found, quitting app" << endl;
                        return -1;
                }
        }
        else
        {
                cout << "No HMD was found in the system, quitting app" << endl;
                return -1;
        }

        // Load the SteamVR Runtime
        vr::HmdError err;
        vr_context = vr::VR_Init(&err,vr::EVRApplicationType::VRApplication_Scene);
        vr_context == NULL ? cout << "Error while initializing SteamVR runtime. Error code is " << vr::VR_GetVRInitErrorAsSymbol(err) << endl : cout << "SteamVR runtime successfully initialized" << endl;

        // Obtain some basic information given by the runtime
        int base_stations_count = 0;
        for (uint32_t td=vr::k_unTrackedDeviceIndex_Hmd; td<vr::k_unMaxTrackedDeviceCount; td++) {

                if (vr_context->IsTrackedDeviceConnected(td))
                {
                        vr::ETrackedDeviceClass tracked_device_class = vr_context->GetTrackedDeviceClass(td);

                        string td_type = GetTrackedDeviceClassString(tracked_device_class);
                        tracked_device_type[td] = td_type;

                        cout << "Tracking device " << td << " is connected " << endl;
                        cout << "  Device type: " << td_type << ". Name: " << GetTrackedDeviceString(vr_context,td,vr::Prop_TrackingSystemName_String) << endl;

                        if (tracked_device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference) base_stations_count++;

                        if (td == vr::k_unTrackedDeviceIndex_Hmd)
                        {
                                // Fill variables used for obtaining the device name and serial ID (used later for naming the SDL window)
                                driver_name = GetTrackedDeviceString(vr_context,vr::k_unTrackedDeviceIndex_Hmd,vr::Prop_TrackingSystemName_String);
                                driver_serial = GetTrackedDeviceString(vr_context,vr::k_unTrackedDeviceIndex_Hmd,vr::Prop_SerialNumber_String);
                        }
                }
                else
                        cout << "Tracking device " << td << " not connected" << endl;
        }

        // Check whether both base stations are found, not mandatory but just in case...
        if (base_stations_count < 2)
        {
                cout << "There was a problem indentifying the base stations, please check they are powered on" << endl;

                return -1;
        }

        return 0;
}
