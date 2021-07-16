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

void CMainApplication::MainInit()
{
    //OpenVR Init
    if (init_OpenVR() != 0) return -1;

    //OpenGL Init



    //


}

//Initialize OpenVR and get pnWidth/pnHeight
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
//Initialize Left/Right Textures,
int init_OpenGL()
{

}
//Might not be necessary, leaving in so I remember to look over what needs to be done pre-usage for OpenCV
int init_OpenCV
{

}

//----------------------------------------------------------------
// Load Image from png
//----------------------------------------------------------------

void CMainApplication::LoadPNG()
{

}

//----------------------------------------------------------------
// Convert OpenCV mat to OpenGL texture
//----------------------------------------------------------------

void CMainApplication::MatToTex()
{

}

//----------------------------------------------------------------
// Send frame to headset
//----------------------------------------------------------------

void CMainApplication::RenderFrame()
{

}

//----------------------------------------------------------------
// Run main loop
//----------------------------------------------------------------

void CMainApplication::RunLoop()
{

}

//----------------------------------------------------------------
// Main
//----------------------------------------------------------------

int main(int argc,char* argv[])
{

}
