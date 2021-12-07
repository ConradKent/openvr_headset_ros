/*



*/
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>
#include<fstream>

#include <ros/ros.h>
#include <openvr_headset_ros/Vive.h>

#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>  // translation between Eular angle and Quaternion, unnecessary for tracking since Quaternion can be received directly
#include "tf/transform_datatypes.h"
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/SetModelState.h>
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/SpawnModel.h"

// define callback function in a class so that data running inside the class can be used globally
class Vive_Listener
{
public:
          openvr_headset_ros::Vive vive;

          void callback(const openvr_headset_ros::Vive& msg)
	    {
		vive = msg;
	    }
};

class Surveycontroller
{
	public:
		std::string current_status;
		std::string idle;
		std::string start;
		ros::Time start_time;
		std::string progress;
		std::string results;
		bool surveying;
		void DoSurvey(int s_trigger);

};

void Surveycontroller::DoSurvey(int s_trigger)
{
		if(surveying == true) {
			ros::Duration survey_time = ros::Time::now() - start_time;
			ros::Duration two(2.0);
			ros::Duration five(5.0);
			ros::Duration eight(8.0);
			if(survey_time < two){
					current_status=start;
			}else if(survey_time < five){
					double survey_time_sec=survey_time.toSec();
					int percentage = (survey_time_sec-2)*(100/3);
					std::stringstream progress_append;
					progress_append << progress << percentage << "%";
					current_status=progress_append.str();
			}else if(survey_time < eight){
					current_status=results;
			}else{
					surveying=false;
		}
		} else if(s_trigger==1) {
			current_status=start;
			start_time=ros::Time::now();
			surveying = true;
		}
}

class Waypointcontroller
{
public:
	ros::ServiceClient client_get;//= n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	ros::Publisher base_control;//= n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	gazebo_msgs::GetModelState get_state;

	geometry_msgs::Twist base_motion;

	float x_tar;
	float y_tar;
	float x_cur;
	float y_cur;
        float dis;

	float kp_ang;
	float kp_lin;

	float psidot;
	float v;

	int trigger;// 0
	bool once;

	ros::ServiceClient delete_model;
        ros::ServiceClient spawn_model;

        gazebo_msgs::SpawnModel sm;
	gazebo_msgs::DeleteModel deletemodel;

	gazebo_msgs::ModelState waypoint;
	
	std::string waypoint_name;//"waypoint"

   	void readmodel(const char* path)
	  {

		std::ifstream ifs;
		ifs.open(path);
		std::stringstream stringstream;
    		stringstream << ifs.rdbuf();
  		sm.request.model_name = waypoint_name;
  		sm.request.model_xml = stringstream.str();
    		sm.request.robot_namespace = ros::this_node::getNamespace();
    		sm.request.reference_frame = "world";

	  }


	void init()
	{

		client_get.call(get_state);
		x_tar = get_state.response.pose.position.x;
                y_tar = get_state.response.pose.position.y;
		
	}

        void waypoint_controller(const openvr_headset_ros::Vive& vive)
	{

		if(trigger==0 & (int)vive.ctrl_right.buttons.trigger == 0)
		{
			std::cout<< "here" <<std::endl;
			client_get.call(get_state);
			double roll, pitch, yaw;
                        tf::Quaternion Qua(get_state.response.pose.orientation.x, get_state.response.pose.orientation.y, get_state.response.pose.orientation.z, get_state.response.pose.orientation.w);
			tf::Matrix3x3 m(Qua); //rotation matrix from Quaternion
			m.getRPY(roll, pitch, yaw); //eular angle form rotation matrix

                        x_cur = get_state.response.pose.position.x;
                        y_cur = get_state.response.pose.position.y;

			float psid = atan2(y_tar-y_cur, x_tar-x_cur);
			float err_ang = psid - yaw;

                        psidot = 0;
                        v = 0;

			dis = pow( (pow((y_tar-y_cur), 2) + pow((x_tar-x_cur), 2)), 1);
			if (dis >= 0.15)
 			{
            			v = kp_lin*dis;
           			psidot = kp_ang*(err_ang);

				if (v>0.5){v = 0.5;}
				if (psidot>0.8){psidot = 0.8;}
			}


			std::cout<< x_tar << " , "<< y_tar<<" ; "<<x_cur<<" , "<< y_cur<<" ; "<< v<<" , "<<psidot<<std::endl;

	

                        if( abs(get_state.response.twist.angular.z)>0.8){psidot = 0;}

			base_motion.linear.x = v;
                        base_motion.angular.z = psidot;

			std::cout<<base_motion<<std::endl;
                        base_control.publish(base_motion);
		}
	}


        void controller(const openvr_headset_ros::Vive& vive)
	  {
		if(trigger==0 & (int)vive.ctrl_right.buttons.trigger == 1)
		  {
			spawn_model.call(sm);
			trigger = 1;

			base_motion.linear.x = 0;
                        base_motion.angular.z = 0;

                        base_control.publish(base_motion);
		  }
		
		if((int)vive.ctrl_right.buttons.trigger == 1)
		  {
			double roll, pitch, yaw;
                        tf::Quaternion Qua(vive.ctrl_right.pose.orientation.x,vive.ctrl_right.pose.orientation.y,vive.ctrl_right.pose.orientation.z,vive.ctrl_right.pose.orientation.w);
			tf::Matrix3x3 m(Qua); //rotation matrix from Quaternion
			m.getRPY(roll, pitch, yaw); //eular angle form rotation matrix

			waypoint.model_name = waypoint_name;
			waypoint.reference_frame="world";

			waypoint.pose.position.x = vive.ctrl_right.pose.position.x + 2*(m[0][0]*vive.ctrl_right.pose.position.z);
			waypoint.pose.position.y = vive.ctrl_right.pose.position.y + 2*(m[1][0]*vive.ctrl_right.pose.position.z);
			waypoint.pose.position.z = 0;

			waypoint.pose.orientation.x = 0;
			waypoint.pose.orientation.y = 0;
			waypoint.pose.orientation.z = 0;
			waypoint.pose.orientation.w = 1;
		  }

		if(trigger==1 & (int)vive.ctrl_right.buttons.trigger == 0)
		  {

			x_tar = waypoint.pose.position.x;
			y_tar = waypoint.pose.position.y;
			
			once = true;
                        trigger=0;

			//deletemodel.request.model_name = waypoint_name;

			//delete_model.call(deletemodel);


		  }
	  }



};


/*
std::string first_controller = "Velocity";
std::string second_controller = "Waypoint";
std::string third_controller = "Throw";
*/

std::string message_surv = "Survey Messages";


int main(int argc, char **argv)
{
  // setup ros node
  ros::init(argc, argv, "vive_controller_surv");
  ros::NodeHandle nh;

  ros::Rate r(180);
  ros::service::waitForService("/gazebo/spawn_urdf_model", -1);
  //define class for callback class and subscriber
  Vive_Listener vive_data;
  ros::Subscriber sub_vive = nh.subscribe("openvr_headset_ros/vive", 1, &Vive_Listener::callback, &vive_data);



    ros::Publisher gazebo_pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    gazebo_msgs::ModelState controller_throw,controller_line;


    /* turtlebot twist command */
    ros::Publisher base_control = nh.advertise<geometry_msgs::Twist>("/turtlesurv/commands/velocity", 1);
    geometry_msgs::Twist base_motion;

    /* get mobile base state*/
    ros::ServiceClient client_get = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState get_state;
    get_state.request.model_name = "turtlebot3_surv_burger";


    /* previous value */
    openvr_headset_ros::Vive vive_previ;

	/* Survey */
	Surveycontroller Survey_controller;
	Survey_controller.idle="Survey Equipment Idling";
	Survey_controller.start="Beginning Survey";
	//Survey_controller.start_time;
	Survey_controller.progress="Survey in progress: ";
	Survey_controller.results="Survey Complete: Cool Rocks found!";
	Survey_controller.surveying=false;
	Survey_controller.current_status=Survey_controller.idle;

    /* Waypoint */
    Waypointcontroller Way_point_controller;
    Way_point_controller.trigger = 0;
    Way_point_controller.waypoint_name = "waypoint_surv";
    Way_point_controller.once = false;
    Way_point_controller.delete_model = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    Way_point_controller.spawn_model = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    Way_point_controller.readmodel("/home/conrad/catkin_ws/src/openvr_headset_ros/models/controller/surv/model.sdf");
    Way_point_controller.client_get = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    Way_point_controller.base_control = nh.advertise<geometry_msgs::Twist>("/turtlesurv/cmd_vel", 1);
    Way_point_controller.get_state.request.model_name = "turtlebot3_surv_burger";
    Way_point_controller.kp_ang = 4;
    Way_point_controller.kp_lin = 0.3;


    /* for display */
    ros::Publisher ToDisplay = nh.advertise<std_msgs::String>("message_survey", 10);
    std_msgs::String msg;
    msg.data = Survey_controller.current_status;
    ToDisplay.publish(msg);






  while(ros::ok())
  {
  ros::spinOnce(); 
  // ros::spin() works too, but extra code can run outside the callback function between each spinning if spinOnce() is used

   msg.data = Survey_controller.current_status;
    ToDisplay.publish(msg);



  switch(vive_data.vive.controller_channel) {

   case 01  : //"Waypoint"

        Way_point_controller.waypoint_controller(vive_data.vive);
        Way_point_controller.controller(vive_data.vive);

	if(Way_point_controller.trigger){gazebo_pub.publish(Way_point_controller.waypoint);}
	if(Way_point_controller.once)
	  {
		Way_point_controller.once = false;
	  }
	break;
   case 02  : //"TakeSurvey"
		Survey_controller.DoSurvey(vive_data.vive.ctrl_right.buttons.trigger);
	break;
  }


if(vive_data.vive.ctrl_left.buttons.system == 1)
{
	std::cout<<"left"<<std::endl;
}





  /* over write previous value  */
  vive_previ = vive_data.vive;

  r.sleep();
  }

  return 0;
}









