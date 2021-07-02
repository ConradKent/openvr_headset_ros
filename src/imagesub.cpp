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
#include <SDL_opengl.h>
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

    vr::IVRSystem *vr::VR_Init( vr::HmdError *peError, vr::EVRApplicationType eApplicationType );

    ros::Rate r(90);

    //define class for callback class and subscriber
    Listener_image listener_left,listener_right;
    image_transport::Subscriber sub_left = it.subscribe("/camera/rgb/left_eye", 1, &Listener_image::callback, &listener_left);
    image_transport::Subscriber sub_right = it.subscribe("/camera/rgb/right_eye", 1, &Listener_image::callback, &listener_right);
  
/* I'm not sure if I need the images as mats to read the image data out of? I'll try reading the bgr data out of the listeners directly for now. Otherwise, we'll read from image_left.data and image_right.data
    cv::Mat image_left(1200,960, CV_8UC3,cv::Scalar(0,255,255));
    cv::Mat image_right(1200,960, CV_8UC3,cv::Scalar(0,255,255));

    listener_left.image = image_left(cv::Range(0,1200),cv::Range(0,960));
    listener_right.image = image_right(cv::Range(0,1200),cv::Range(0,960));
*/

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
                vr::VRCompositor()->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );

                glBindTexture(GL_TEXTURE_2D, tex_left)
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);    //hellovr_opengl runs glTexParameteri once per cycle as far as I can tell, which I wanted to take note of because that seems weird.
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                TexImage2D(GL_TEXTURE_2D, 0,GL_RGBA,image.cols,image.rows,0,GL_BGR,GL_UNSIGNED_BYTE,listener_left.image)
                vr::Texture_t leftEyeTexture = {(void*)(uintptr_t)tex_left, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
                vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture );

                glBindTexture(GL_TEXTURE_2D, tex_right)
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                TexImage2D(GL_TEXTURE_2D, 0,GL_RGBA,image.cols,image.rows,0,GL_BGR,GL_UNSIGNED_BYTE,listener_right.image)
                vr::Texture_t rightEyeTexture = {(void*)(uintptr_t)tex_right, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
                vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );

            }
            r.sleep();
    }

    return 0;
}
