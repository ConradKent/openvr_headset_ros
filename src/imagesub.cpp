/*

This new version of imagesub subscribes the images
from ROS to OpenCV, then it takes the BGR OpenCV
image data and adds it to an OpenGL texture2d in
RGBA8, then it submits those texture2d's to each
eye of a VR headset using OpenVR.

*/
#include <SDL2/SDL.h>
#include <iostream>
#include <GL/glew.h>
#include <SDL2/SDL_opengl.h>
#include <GL/glut.h>


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <openvr/openvr.h>

#include <string>
#include <ros/package.h> // for ros::package::getPath() to not have to hardcode the paths for this

// define callback function in a class so that data running inside the class can be used globally
class Listener_image
{
public:
	  cv::Mat image;

	  void callback(const sensor_msgs::ImageConstPtr& msg)
	    {
            //copy image data to the image under the same class, which will be assign as a pointer. Use rba8 format as this is what OpenVR requires for rendering.
            cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(image);
            //flip(image,image,0); //flips the image upside down (the opencv and opengl formats read the pixel rows in a different order) MOVED TO MAIN LOOP DUE TO MESSAGE ADDITION
            }

};
// I'm going to just split this into distinct channels for now, probably a more elegant way of doing this
class Listener_message_ch0
{
public:
		
		std::string display_message_ch0;
		void chatterCallback(const std_msgs::String::ConstPtr& msg)
		{
			display_message_ch0 = msg->data.c_str();
		}
};

class Listener_message_ch1
{
public:
		
		std::string display_message_ch1;
		void chatterCallback(const std_msgs::String::ConstPtr& msg)
		{
			display_message_ch1 = msg->data.c_str();
		}
};


//from https://github.com/matinas/openvrsimplexamples/blob/master/openvrsimplexamples/src/main.cpp


    vr::IVRSystem* vr_system;
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
    std::string pkglocalpath = ros::package::getPath("openvr_headset_ros"); // to not have to hardcode the paths for this
    // should give back "/home/USERNAME/catkin_ws/src/openvr_headset_ros"

    //initialize glut and glew (opengl stuff)
    glutInit(&argc, argv);
    glutCreateWindow("GLEW Test");
    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
      /* Problem: glewInit failed, something is seriously wrong. */
      fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
    }
    fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));

    // setup ros node
    ros::init(argc, argv, "image_sub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Rate r(90);

    //initialize OpenVR
    if (init_OpenVR() != 0) return -1;

    //define the framebuffers
    GLuint FramebufferLeft = 0;
    GLuint FramebufferRight = 0;

    //define the textures
    GLuint tex_left;
    GLuint tex_right;


    //Get the recommended sizes to make cv mats
    uint32_t pnWidth;
    uint32_t pnHeight;
    vr_system->GetRecommendedRenderTargetSize(&pnWidth, &pnHeight );
    std::cout << "Width: " << pnWidth << std::endl;
    std::cout << "Height: " << pnHeight << std::endl;


    //define class for callback class and subscriber
    Listener_image listener_left,listener_right;
    image_transport::Subscriber sub_left = it.subscribe("/camera/rgb/left_eye", 1, &Listener_image::callback, &listener_left);
    image_transport::Subscriber sub_right = it.subscribe("/camera/rgb/right_eye", 1, &Listener_image::callback, &listener_right);
  
	//message class and subscriber
	Listener_message_ch0 listener_message0; //these do not need to be separate classes but they are because I'm lazy
	Listener_message_ch1 listener_message1;
	std::string display_message0 = "";
	std::string display_message1 = "---";
	std::string display_message2 = "---";
	std::string display_message3 = "---";
	ros::Subscriber message_sub0 = nh.subscribe("display_message_ch0", 10, &Listener_message_ch0::chatterCallback, &listener_message0);
	ros::Subscriber message_sub1 = nh.subscribe("display_message_ch1", 10, &Listener_message_ch1::chatterCallback, &listener_message1);
  
	//message log info
	cv::Point message_coords0;
	cv::Point message_coords1;
	cv::Point message_coords2;
	cv::Point message_coords3;
	cv::Point message_coordsheader;
	double spacing = 30;
	std::string display_header = "Message Log";
	
	message_coords0.x = .55*pnWidth;
	message_coords0.y = .66*pnHeight;
	
	message_coords1.x = message_coords0.x;
	message_coords1.y = message_coords0.y-spacing;
	
	message_coords2.x = message_coords0.x;
	message_coords2.y = message_coords1.y-spacing;
	
	message_coords3.x = message_coords0.x;
	message_coords3.y = message_coords2.y-spacing;
	
	message_coordsheader.x = message_coords0.x;
	message_coordsheader.y = message_coords3.y-spacing;
	
	double textsize =0.8;
	int thickness = 2;
  
    //putting the image info from our listeners onto the opencv mats
    cv::Mat image_left(pnHeight,pnWidth, CV_8UC3,cv::Scalar(0,255,255));
    cv::Mat image_right(pnHeight,pnWidth, CV_8UC3,cv::Scalar(0,255,255));

    listener_left.image = image_left(cv::Range(0,pnHeight),cv::Range(0,pnWidth));
    listener_right.image = image_right(cv::Range(0,pnHeight),cv::Range(0,pnWidth));


    //Left eye texture and framebuffer binding

    glGenFramebuffers(1, &FramebufferLeft);
    std::cout << "glGenFramebuffers" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferLeft);
    std::cout << "glBindFramebuffer" << std::endl;

    glGenTextures(1,&tex_left);
    std::cout << "glGenTextures" << std::endl;

    glBindTexture(GL_TEXTURE_2D, tex_left);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);    //hellovr_opengl runs glTexParameteri once per cycle as far as I can tell, which I wanted to take note of because that seems weird.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA,image_left.cols,image_left.rows,0,GL_BGR,GL_UNSIGNED_BYTE,image_left.data);

    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, tex_left, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferLeft);
    glViewport(0,0,pnHeight,pnWidth); // Render on the whole framebuffer, complete from the lower left corner to the upper right


  //  layout(location = 0) out vec3 color;

    //This is a bit of code to check that the framebuffer is okay. Implement later.
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        {
        std::cout <<"Framebuffer not complete"<<std::endl;
        }

    //Right eye texture and framebuffer binding

    glGenFramebuffers(1, &FramebufferRight);
    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferRight);

    glGenTextures(1,&tex_right);

    glBindTexture(GL_TEXTURE_2D, tex_right);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA,image_right.cols,image_right.rows,0,GL_BGR,GL_UNSIGNED_BYTE,image_right.data);


    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, tex_right, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferRight);
    glViewport(0,0,pnWidth,pnHeight); // Render on the whole framebuffer, complete from the lower left corner to the upper right

    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        {
        std::cout <<"Framebuffer not complete"<<std::endl;
        }


cv::VideoWriter video(pkglocalpath + "/out.avi",cv::VideoWriter::fourcc('M','J','P','G'),30, cv::Size(pnWidth,pnHeight),true); // "/home/USERNAME/catkin_ws/src/openvr_headset_ros/out.avi"

    while(ros::ok())
    {
        ros::spinOnce();
        // ros::spin() works too, but extra code can run outside the callback function between each spinning if spinOnce() is used
				
				
				if (listener_message0.display_message_ch0 != display_message0 && listener_message1.display_message_ch1 != display_message0)
				{
					if (listener_message0.display_message_ch0 != display_message0)
						{
							display_message3=display_message2;
							display_message2=display_message1;
							display_message1=display_message0;
							display_message0=listener_message0.display_message_ch0;
						}
					if (listener_message1.display_message_ch1 != display_message0)
						{
							display_message3=display_message2;
							display_message2=display_message1;
							display_message1=display_message0;
							display_message0=listener_message1.display_message_ch1;
						}
				}
				
				//Put in the text
				cv::putText(image_right, display_message0, message_coords0, 0, textsize, cv::Scalar(0,0,255), thickness, 8);
				cv::putText(image_right, display_message1, message_coords1, 0, textsize, cv::Scalar(0,0,255), thickness, 8);
				cv::putText(image_right, display_message2, message_coords2, 0, textsize, cv::Scalar(0,0,255), thickness, 8);
				cv::putText(image_right, display_message3, message_coords3, 0, textsize, cv::Scalar(0,0,255), thickness, 8);
				cv::putText(image_right, display_header, message_coordsheader, 0, textsize, cv::Scalar(0,0,255), thickness, 8);
				
				
				
				//flip the images upside down (the opencv and opengl formats read the pixel rows in a different order)
				flip(image_left,image_left,0);
				flip(image_right,image_right,0);

				//Update vr, put images into OpenGL mats, send OpenGL mats to headset
                vr::TrackedDevicePose_t pose[vr::k_unMaxTrackedDeviceCount];
                vr::VRCompositor()->WaitGetPoses(pose, vr::k_unMaxTrackedDeviceCount, NULL, 0);


                glBindTexture(GL_TEXTURE_2D, tex_left);
                glTexSubImage2D(GL_TEXTURE_2D, 0,0,0,image_left.cols,image_left.rows,GL_BGR,GL_UNSIGNED_BYTE,image_left.data);
                vr::Texture_t leftEyeTexture = {(void*)(uintptr_t)tex_left, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
                vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture );

                glBindTexture(GL_TEXTURE_2D, tex_right);
                glTexSubImage2D(GL_TEXTURE_2D, 0,0,0,image_right.cols,image_right.rows,GL_BGR,GL_UNSIGNED_BYTE,image_right.data);
                vr::Texture_t rightEyeTexture = {(void*)(uintptr_t)tex_right, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
                vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );
//glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA,image_right.cols,image_right.rows,0,GL_BGR,GL_UNSIGNED_BYTE,image_right.data);
                glFinish();

                video.write(image_right);

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


        vr::TrackedDevicePose_t tracked_device_pose[vr::k_unMaxTrackedDeviceCount];

        return 0;


}
