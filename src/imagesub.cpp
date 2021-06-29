/*

Vrui_mdf/imagesub - Node to subscribe images from the camera modle in Gazebo
It will subscribe the images for both the left eye and right eye, and stitch 
the images together, having the left eye image on the left side and the right 
eye image on the right side. 

Potentially, this node can also subscribe for the button states from the controllers
and switch display mode or display different iamge on the Vive headset, like the 
first person view of the turtlebot.

*/
#include <stdlib.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <time.h>
#include <vrui_mdf/Vive.h>
#include <tf/transform_broadcaster.h> 
#include <math.h> 

/* Ripped from hellovr_opengl_main.cpp in the openvr distro. Will need to create an opengl texture using an opencv mat with the rbga8 image from one of the eyes of vr_view.
 * The pipeline will go vr_view->image subscriber->opencv mat->opengl texture->vr::Texture_t->vrcompositor submit for each eye. Probably need to utilize glTexImage2D.
void CMainApplication::RenderFrame()
{
        // for now as fast as possible
        if ( m_pHMD )
        {
                RenderControllerAxes();
                RenderStereoTargets();
                RenderCompanionWindow();

                vr::Texture_t leftEyeTexture = {(void*)(uintptr_t)leftEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
                vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture );
                vr::Texture_t rightEyeTexture = {(void*)(uintptr_t)rightEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
                vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );
        }

        if ( m_bVblank && m_bGlFinishHack )
        {
                //$ HACKHACK. From gpuview profiling, it looks like there is a bug where two renders and a present
                // happen right before and after the vsync causing all kinds of jittering issues. This glFinish()
                // appears to clear that up. Temporary fix while I try to get nvidia to investigate this problem.
                // 1/29/2014 mikesart
                glFinish();
        }

        // SwapWindow
        {
                SDL_GL_SwapWindow( m_pCompanionWindow );
        }

        // Clear
        {
                // We want to make sure the glFinish waits for the entire present to complete, not just the submission
                // of the command. So, we do a clear here right here so the glFinish will wait fully for the swap.
                glClearColor( 0, 0, 0, 1 );
                glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        }

        // Flush and wait for swap.
        if ( m_bVblank )
        {
                glFlush();
                glFinish();
        }

        // Spew out the controller and pose count whenever they change.
        if ( m_iTrackedControllerCount != m_iTrackedControllerCount_Last || m_iValidPoseCount != m_iValidPoseCount_Last )
        {
                m_iValidPoseCount_Last = m_iValidPoseCount;
                m_iTrackedControllerCount_Last = m_iTrackedControllerCount;

                dprintf( "PoseCount:%d(%s) Controllers:%d\n", m_iValidPoseCount, m_strPoseClasses.c_str(), m_iTrackedControllerCount );
        }

        UpdateHMDMatrixPose();
}
 */

// define callback function in a class so that data running inside the class can be used globally
class Listener_image
{
public:
	  cv::Mat image;

	  void callback(const sensor_msgs::ImageConstPtr& msg)
	    {
            //copy image data to the image under the same class, which will be assign as a pointer. Use rba8 format as this is what OpenVR requires for rendering.
            cv_bridge::toCvShare(msg, "rgba8")->image.copyTo(image);
            }

};


class Listener_ctrlMethod
{
public:

	std::string control_method;

	void chatterCallback(const std_msgs::String::ConstPtr& msg)
	{
	   control_method = msg->data.c_str();
	}

};

class Vive_Listener
{
public:
	  vrui_mdf::Vive vive;

	  void callback(const vrui_mdf::Vive& msg)
	    {
		vive = msg;
	    }
};

int main(int argc, char **argv)
{
  // setup ros node
  ros::init(argc, argv, "image_sub");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // define and modify window
  //cv::namedWindow("view",cv::WINDOW_NORMAL); // cv::WINDOW_NORMAL means the window size can be changed
  //cv::moveWindow("view",1921,0);      // move the window to the VIVE monitor, which is the second moniter on the right side of main monitor, and main monitor has a width of 1920
  //cv::setWindowProperty("view",0,1);  // setWindowProperty(window name, type of window property(full screen = 0), value of window property(full screen = 1))
  //cv::startWindowThread();
  
  ros::Rate r(100);

  //define class for callback class and subscriber
  Listener_image listener_left,listener_right;
  image_transport::Subscriber sub_left = it.subscribe("/camera/rgb/left_eye", 1, &Listener_image::callback, &listener_left);
  image_transport::Subscriber sub_right = it.subscribe("/camera/rgb/right_eye", 1, &Listener_image::callback, &listener_right);

  Listener_ctrlMethod ctrl_methods;
  ctrl_methods.control_method = "Velocity";
  ros::Subscriber ctrl_sub = nh.subscribe("control_method", 10, &Listener_ctrlMethod::chatterCallback,&ctrl_methods);
  

  // final image, left half shows left eye, right half shows right eye
  cv::Mat image_final_left(1200,1920, CV_8UC3,cv::Scalar(0,255,255));


  // assign the images under listerner classes as pointers pointing at left or right half of final image
  // so that the copyTo() will copy the data directly to the final image
  //listener_left.image = image_final(cv::Range(0,1200),cv::Range(0,960));
  //listener_right.image = image_final(cv::Range(0,1200),cv::Range(960,1920));

  Vive_Listener vive_data;
  ros::Subscriber sub_vive = nh.subscribe("vrui/vive", 1, &Vive_Listener::callback, &vive_data);


  while(ros::ok())
  {
  ros::spinOnce(); 
  // ros::spin() works too, but extra code can run outside the callback function between each spinning if spinOnce() is used

    if(listener_left.image.cols!=0 && listener_right.image.cols!=0) 
    {
    //probably need to use waitgetposes here
    Renderframe()
    //video.write(image_final);

    //cv::imshow("view", image_final);
    //cv::waitKey(1); // necessary for imshow()



    }
  r.sleep();
  }

  return 0;
}
