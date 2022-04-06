/*



*/
#include <stdlib.h>
#include<iostream>
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

#include <string>
#include <ros/package.h> // for ros::package::getPath() to not have to hardcode the paths for this

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


class Toolcontroller
{
	public:
	int pos;
	int last_grippress;
	float carr_pos_x;
	float carr_pos_y;
	float carr_pos_z;
	float carr_ori_x;
	float carr_ori_y;
	float carr_ori_z;
	float carr_ori_w;
	ros::ServiceClient client_get;
	gazebo_msgs::GetModelState get_state;
	void checkpos(int t_grippress);
};

void Toolcontroller::checkpos(int t_grippress)
{
	client_get.call(get_state);
	carr_pos_x=get_state.response.pose.position.x;
	carr_pos_y=get_state.response.pose.position.y;
	carr_pos_z=get_state.response.pose.position.z;
	
	carr_ori_x=get_state.response.pose.orientation.x;
	carr_ori_y=get_state.response.pose.orientation.y;
	carr_ori_z=get_state.response.pose.orientation.z;
	carr_ori_w=get_state.response.pose.orientation.w;
	
	if(last_grippress==0 && t_grippress==1) {
		if(pos==1) {
			pos=0;
			 ROS_WARN("Tool in Hand");
		}else {
			pos=1;
			ROS_WARN("Tool on Carrier");
			}
		}
		last_grippress=t_grippress;
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




class velocitycontroller
{
public:
	ros::ServiceClient client_get;//= n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	ros::Publisher base_control;//= n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	gazebo_msgs::GetModelState get_state;

	geometry_msgs::Twist base_motion;

	float x_ori;
	float y_ori;

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

/*
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
*/

        void controller(const openvr_headset_ros::Vive& vive)
	  {
		if(trigger==0 & (int)vive.ctrl_right.buttons.trigger == 1)
		  {
			//spawn_model.call(sm);
			trigger = 1;

			x_ori = vive.ctrl_right.pose.position.x;
			y_ori = vive.ctrl_right.pose.position.y;

		  }
		
		if((int)vive.ctrl_right.buttons.trigger == 1)
		  {


			float x_diff = vive.ctrl_right.pose.position.x - x_ori;
			float y_diff = vive.ctrl_right.pose.position.y - y_ori;
			float yaw_tar = atan2(y_diff, x_diff);


    			client_get.call(get_state);
			double roll, pitch, yaw;
                        tf::Quaternion Qua(get_state.response.pose.orientation.x, get_state.response.pose.orientation.y, get_state.response.pose.orientation.z, get_state.response.pose.orientation.w);
			tf::Matrix3x3 m(Qua); //rotation matrix from Quaternion
			m.getRPY(roll, pitch, yaw); //eular angle form rotation matrix

			float yaw_diff = yaw_tar - yaw;
			float scale =  pow( (pow((y_diff), 2) + pow((x_diff), 2)), 1);			

			psidot = kp_ang*(yaw_diff);
			v = kp_lin*scale;

			if (v>0.5){v = 0.5;}
			if (psidot>0.8){psidot = 0.8;}
      			if( abs(get_state.response.twist.angular.z)>0.8){psidot = 0;}

			base_motion.linear.x = v;
                        base_motion.angular.z = psidot;

                        base_control.publish(base_motion);

		  }

		if(trigger==1 & (int)vive.ctrl_right.buttons.trigger == 0)
		  {
			
			once = true;
                        trigger=0;

			//deletemodel.request.model_name = waypoint_name;

			//delete_model.call(deletemodel);


		  }

	  }



};
	
	


//int num_controllers = 3;
//int controller_switch = 1;



std::string first_controller = "Velocity";
std::string second_controller = "Waypoint";
std::string third_controller = "Throw";




int main(int argc, char **argv)
{
  std::string pkglocalpath = ros::package::getPath("openvr_headset_ros"); // to not have to hardcode the paths for this
  // should give back "/home/USERNAME/catkin_ws/src/openvr_headset_ros"

  // setup ros node
  ros::init(argc, argv, "vive_controller_tool");
  ros::NodeHandle nh;

  ros::Rate r(180);
  ros::service::waitForService("/gazebo/spawn_urdf_model", -1);
  //define class for callback class and subscriber
  Vive_Listener vive_data;
  ros::Subscriber sub_vive = nh.subscribe("openvr_headset_ros/vive", 1, &Vive_Listener::callback, &vive_data);



    ros::Publisher gazebo_pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    gazebo_msgs::ModelState controller_throw,controller_line,tool;

	/* Spawn Tool*/

    tool.model_name = "Tool";
    tool.reference_frame="world";

    gazebo_msgs::SpawnModel sm;
    ros::ServiceClient spawn_model;
    std::ifstream ifs;
    ifs.open(pkglocalpath + "/models/tool/model.sdf"); //TODO generalize. // "/home/USERNAME/catkin_ws/src/openvr_headset_ros/models/tool/model.sdf"
    std::stringstream stringstream;
    stringstream << ifs.rdbuf();
    sm.request.model_name = "Tool";
    sm.request.model_xml = stringstream.str();
    sm.request.robot_namespace = ros::this_node::getNamespace();
    sm.request.reference_frame = "world";
    spawn_model.call(sm);

    system(("rosrun gazebo_ros spawn_model -file " + pkglocalpath + "/models/tool/model.sdf -sdf -model Tool -y 0 -x 0 -z 1").c_str()); // "rosrun gazebo_ros spawn_model -file /home/USERNAME/catkin_ws/src/openvr_headset_ros/models/tool/model.sdf -sdf -model Tool -y 0 -x 0 -z 1"

    /* turtlebot twist command */
    ros::Publisher base_control = nh.advertise<geometry_msgs::Twist>("/turtletool/commands/velocity", 1);
    geometry_msgs::Twist base_motion;

    /* get mobile base state*/
    ros::ServiceClient client_get = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState get_state;
    get_state.request.model_name = "turtlebot3_tool_burger";

    /* previous value */
    openvr_headset_ros::Vive vive_previ;

    /* controller 1 */
    velocitycontroller velocity_controller;
    velocity_controller.trigger = 0;
    velocity_controller.client_get = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    velocity_controller.base_control = nh.advertise<geometry_msgs::Twist>("/turtletool/cmd_vel", 1);
    velocity_controller.get_state.request.model_name = "turtlebot3_tool_burger";
    velocity_controller.kp_ang = 4;
    velocity_controller.kp_lin = 20;


    /* controller 2 */
    Waypointcontroller Way_point_controller;
    Way_point_controller.trigger = 0;
    Way_point_controller.waypoint_name = "waypoint_tool";
    Way_point_controller.once = false;
    Way_point_controller.delete_model = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    Way_point_controller.spawn_model = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    Way_point_controller.readmodel((pkglocalpath + "/models/controller/tool/model.sdf").c_str()); // readmodel req.s const char* currently // "/home/USERNAME/catkin_ws/src/openvr_headset_ros/models/controller/tool/model.sdf"
    Way_point_controller.client_get = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    Way_point_controller.base_control = nh.advertise<geometry_msgs::Twist>("/turtletool/cmd_vel", 1);
    Way_point_controller.get_state.request.model_name = "turtlebot3_tool_burger";
    Way_point_controller.kp_ang = 4;
    Way_point_controller.kp_lin = 0.3;

    /* Tool */
	Toolcontroller tool_state;
	tool_state.pos=0;
    tool_state.client_get = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    tool_state.get_state.request.model_name = "turtlebot3_tool_burger";
	tool_state.last_grippress=0;

    /* for display */
    ros::Publisher ToDisplay = nh.advertise<std_msgs::String>("control_method", 10);
    std_msgs::String msg;
    msg.data = first_controller ;
    ToDisplay.publish(msg);






  while(ros::ok())
  {
  ros::spinOnce(); 

tool_state.checkpos(vive_data.vive.ctrl_left.buttons.grip);

  ToDisplay.publish(msg);


  switch(vive_data.vive.controller_channel) {
   case 12  : //"Velocity"

        velocity_controller.controller(vive_data.vive);

	if(velocity_controller.once)
	  {
		velocity_controller.once = false;
	  }

	break;
   case 11  : //"Waypoint"

        Way_point_controller.waypoint_controller(vive_data.vive);
        Way_point_controller.controller(vive_data.vive);

	if(Way_point_controller.trigger){gazebo_pub.publish(Way_point_controller.waypoint);}
	if(Way_point_controller.once)
	  {
		Way_point_controller.once = false;
	  }
	break;
  }


if(vive_data.vive.ctrl_left.buttons.system == 1)
{
	std::cout<<"left"<<std::endl;
}

if(tool_state.pos==0)
{
            tool.pose.position.x=vive_data.vive.ctrl_left.pose.position.x;
            tool.pose.position.y=vive_data.vive.ctrl_left.pose.position.y;
            tool.pose.position.z=vive_data.vive.ctrl_left.pose.position.z;

            tool.pose.orientation.x = vive_data.vive.ctrl_left.pose.orientation.x;
            tool.pose.orientation.y = vive_data.vive.ctrl_left.pose.orientation.y;
            tool.pose.orientation.z = vive_data.vive.ctrl_left.pose.orientation.z;
            tool.pose.orientation.w = vive_data.vive.ctrl_left.pose.orientation.w;
}
else
{
			tool.pose.position.x=tool_state.carr_pos_x;
			tool.pose.position.x=tool_state.carr_pos_y;
			tool.pose.position.x=tool_state.carr_pos_z+1;
			
			tool.pose.orientation.x=tool_state.carr_ori_x;
			tool.pose.orientation.y=tool_state.carr_ori_y;
			tool.pose.orientation.z=tool_state.carr_ori_z;
			tool.pose.orientation.w=tool_state.carr_ori_w;
}

            gazebo_pub.publish(tool);

  /* over write previous value  */
  vive_previ = vive_data.vive;

  r.sleep();
  }

  return 0;
}









