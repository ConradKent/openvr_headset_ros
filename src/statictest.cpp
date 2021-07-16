// This node will be a simple test of loading a static image using OpenCV, then applying that image to an OpenGL texture, then sending that texture to OpenVR to be displayed on a headset.

//----------------------------------------------------------------
// Includes
//----------------------------------------------------------------

#include <iostream>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <openvr/openvr.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>

//----------------------------------------------------------------
// Declarations
//----------------------------------------------------------------

class CMainApplication
{
public:
        CMainApplication( int argc, char *argv[] );

        void MainInit();
        void MatToTex();
        void RenderFrame();
        void init_OpenVR();
private:


        //The framebuffer
        GLuint FramebufferLeft = 0;
        GLuint FramebufferRight = 0;

        //The textures
        GLuint tex_left;
        GLuint tex_right;

};
std::string driver_name, driver_serial;
std::string tracked_device_type[vr::k_unMaxTrackedDeviceCount];

CMainApplication::CMainApplication( int argc, char *argv[] ){};//Should add initializations here I think? I'll figure that out later.


//----------------------------------------------------------------
// Main
//----------------------------------------------------------------

int main(int argc,char* argv[])
{

    CMainApplication *pMainApplication = new CMainApplication( argc, argv );

    pMainApplication->init_OpenVR();

    pMainApplication->MatToTex();

/*
    // define and modify OpenCV window to make sure OpenCV data is ingested correctly
    cv::namedWindow("view",cv::WINDOW_NORMAL); // cv::WINDOW_NORMAL means the window size can be changed
    cv::moveWindow("view",0,0);      // move the window to the VIVE monitor, which is the second moniter on the right side of main monitor, and main monitor has a width of 1920
    cv::setWindowProperty("view",0,1);  // setWindowProperty(window name, type of window property(full screen = 0), value of window property(full screen = 1))
    cv::startWindowThread();
    cv::Mat image_test = cv::imread("/home/conrad/catkin_ws/src/openvr_headset_ros/src/statictest.png");
*/

    //Ros Init
    ros::init(argc, argv, "statictest");
    ros::NodeHandle nh;
    ros::Rate r(90);

    while(ros::ok())
    {
        ros::spinOnce();
        // ros::spin() works too, but extra code can run outside the callback function between each spinning if spinOnce() is used

        /*cv::imshow("view", image_test);*/
        pMainApplication->RenderFrame();

        r.sleep();
    }
}


//----------------------------------------------------------------
// Helpers
//----------------------------------------------------------------

// Purpose: helper to get a string from a tracked device property and turn it into a std::string. FROM https://github.com/matinas/openvrsimplexamples/blob/master/openvrsimplexamples/src/utils.cpp
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

// Purpose: helper to get a string from a tracked device type class. FROM https://github.com/matinas/openvrsimplexamples/blob/master/openvrsimplexamples/src/utils.cpp
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

//----------------------------------------------------------------
// Initialization
//----------------------------------------------------------------

//Initialize OpenVR and get pnWidth/pnHeight
void CMainApplication::init_OpenVR()
{
    vr::IVRSystem* vr_system;
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
                }
        }
        else
        {
                std::cout << "No HMD was found in the system, quitting app" << std::endl;
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

        }
        uint32_t pnWidth;
        uint32_t pnHeight;
        vr_system->GetRecommendedRenderTargetSize(&pnWidth, &pnHeight );
        vr::TrackedDevicePose_t m_rTrackedDevicePose[ vr::k_unMaxTrackedDeviceCount ];

}

//----------------------------------------------------------------
// Convert OpenCV mat to OpenGL texture
//----------------------------------------------------------------

void CMainApplication::MatToTex()
{


    //LoadPNG
    cv::Mat image_left = cv::imread("/home/conrad/catkin_ws/src/openvr_headset_ros/src/statictest.png");
    cv::Mat image_right = cv::imread("/home/conrad/catkin_ws/src/openvr_headset_ros/src/statictest.png");
    if(image_left.empty())
        {
        std::cout << "image empty"<< std::endl;
        }

    //Left eye texture and framebuffer binding

    glGenFramebuffers(1, &FramebufferLeft);
    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferLeft);

    glGenTextures(1,&tex_left);

    glBindTexture(GL_TEXTURE_2D, tex_left);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);    //hellovr_opengl runs glTexParameteri once per cycle as far as I can tell, which I wanted to take note of because that seems weird.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA,image_left.cols,image_left.rows,0,GL_BGR,GL_UNSIGNED_BYTE,image_left.data);

    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, tex_left, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferLeft);
    glViewport(0,0,1852,2056); // Render on the whole framebuffer, complete from the lower left corner to the upper right

    layout(location = 0) out vec3 color;
    /*
    //This is a bit of code to check that the framebuffer is okay. Implement later.
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    return false;
    */
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
    glViewport(0,0,1852,2056); // Render on the whole framebuffer, complete from the lower left corner to the upper right

    layout(location = 0) out vec3 color;

}

//----------------------------------------------------------------
// Send frame to headset
//----------------------------------------------------------------

void CMainApplication::RenderFrame()
{



    vr::Texture_t leftEyeTexture = {(void*)(uintptr_t)FramebufferLeft, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
    vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture );

    vr::Texture_t rightEyeTexture = {(void*)(uintptr_t)FramebufferRight, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
    vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );

    glFinish();
}
