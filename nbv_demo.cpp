#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Int32MultiArray.h"
#include "docc/DOCC.h"
#include "docc/DOCC.cpp"
#include "docc/basicController/PID.cpp"
#include "docc/cameraProjection/PinHole.cpp"
#include "docc/filters/MAFilter.cpp"
#include "docc/trajectory/TrajectoryPlanning.cpp"
#include "geometry_msgs/Point.h"
#define PI 3.14159265




using namespace std;
using namespace cv;


int waypointSent = 0;
double drone_x, drone_y, drone_z;
double orientation[4] = {0, 0, 0, 1};
double arrayx[1] = {1};
double arrayy[1] = {0};
double arrayz[1] = {2};
int counter = 0;
double yaw = 0.0;


MAFilter maf_xt_send;
MAFilter maf_yt_send;


geometry_msgs::Point center;

std::vector<geometry_msgs::Point> waypoints;

ros::Publisher waypoint_pub;

void read_from_file(std::string dir, std::vector<geometry_msgs::Point>& skeleton, double flag)
{
	std::ifstream in(dir.c_str());
	if(in == NULL)
	{
		printf("Cannot open file\n");
		exit(-1);
	}
	std::string str;
	std::string temp;
	std::vector<std::vector<double> > data;
	// read file
	while(std::getline(in, str))
	{
		std::stringstream ss(str);
		std::vector<double> skeleton_point(24 * 3 + 1, 0);
		int i = 0;
		// tokenizer
		while(std::getline(ss, temp, ','))
		{
					skeleton_point[i] = std::atof(temp.c_str());
					i++;
		}
		if(skeleton_point.back() == flag)
		{
			// get rid of the flag
			skeleton_point.pop_back();
			data.push_back(skeleton_point);
		}
	}
	
	
	// transfer data to point
	for(unsigned long i = 0; i < data.size(); i++)
	{
		geometry_msgs::Point point_temp;
		for(unsigned long j = 0; j < data[i].size(); j++)
		{
			if(j % 3 == 0) point_temp.y = data[i][j];
			else if(j % 3 == 1) point_temp.z = data[i][j] + 0.7;
			else if(j % 3 == 2)
			{
				point_temp.x = data[i][j];
				skeleton.push_back(point_temp);
			}
		}
	}
}




//DOCC
DOCC docc;
	float cameraTransformation[6] = {0,0,0,0,0,0};
	float controlIn[4] = {29,2,3,4};	
	float output[3];
	float target[5] = {400,200,9400,7200};
	
void toQuaternion(double pitch, double roll, double yaw)
{
        // Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	orientation[3] = cy * cr * cp + sy * sr * sp;
	orientation[0] = cy * sr * cp - sy * cr * sp;
	orientation[1] = cy * cr * sp + sy * sr * cp;
	orientation[2] = sy * cr * cp - cy * sr * sp;

        //cout<<orientation[0]<<" "<<orientation[1]<<" "<<orientation[2]<<" "<<orientation[3]<<" "<<endl;
}


void plant(float xt_in, float yt_in, float yaw_in){
	  //control camera rotation
	  static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(0, 0, 0) );
		tf::Quaternion q;
		q.setRPY(0,  -yaw_in, 0.0);
		tf::Matrix3x3 m(q);
		//m.getRPY(roll, pitch,  yaw_in);
		//q.setRPY(0.0, 0.0, 0.0);
		transform.setRotation(q);

		std::cout<<"plant: xt_in:"<< xt_in<<", yt_in:"<<yt_in<<", yaw_in:"<<yaw_in<<std::endl;


		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "gimbal", "camera1"));
		
		
		
		//control drone body
    nav_msgs::Path waypoint;
    geometry_msgs::PoseStamped pose;

		pose.pose.position.x = xt_in;
		pose.pose.position.y = yt_in;
		pose.pose.position.z = 1;  //height

    toQuaternion(0, 0, yaw_in);

		pose.pose.orientation.x = orientation[0];
    pose.pose.orientation.y = orientation[1];
    pose.pose.orientation.z = orientation[2];
    pose.pose.orientation.w = orientation[3];

    waypoint.poses.push_back(pose);

	  waypoint_pub.publish(waypoint);



}




void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
    drone_x = msg->pose.pose.position.x;
    drone_y = msg->pose.pose.position.y;
    drone_z = msg->pose.pose.position.z;
    //ROS_INFO("drone_x y z = %f,  %f,  %f\n",drone_x,drone_y,drone_z);

}










void OnScreenObjectCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{

	//update camera x and y
	  cameraTransformation[0] = drone_x;//xt;
    cameraTransformation[1] = drone_y;//yt;
std::cout<<"xt = "<<cameraTransformation[0]<<std::endl;
std::cout<<"yt = "<<cameraTransformation[1]<<std::endl;

	// sensor information from here
	//ROS_INFO("msg.data[0] = %d\n",msg->data[0]);
	//ROS_INFO("msg.data[1] = %d\n",msg->data[1]);
	//ROS_INFO("msg.data[2] = %d\n",msg->data[2]);
	//ROS_INFO("msg.data[3] = %d\n",msg->data[3]);
	//ROS_INFO("msg.data[4] = %d\n",msg->data[4]);
	//ROS_INFO("msg.data[5] = %d\n",msg->data[5]);
	
	
		//controller
	//handle lost of tracking
	if(msg->data[0]==0&&msg->data[1]==0&&msg->data[3]==0&&msg->data[4]==0){
	std::cout<<"visual track lost!"<<std::endl;
 	output[1] = cameraTransformation[0];  //x
	output[2] = cameraTransformation[1];  //y
	output[0] = -0.1;  //yaw

	}
	else{
	controlIn[0] = (msg->data[3]  + msg->data[0])/2.0f;
	controlIn[1] = msg->data[0]  - msg->data[3];
	controlIn[2] = msg->data[5];
	controlIn[3] = msg->data[2];
	

	ROS_INFO("xm = %f, distX = %f, s1 = %f, s2 = %f,  distS = %f\n",controlIn[0],controlIn[1],controlIn[2],controlIn[3],(controlIn[2]-controlIn[3]));
	//ROS_INFO("xm_e = %f, distX_e = %f, distS_e = %f\n",(controlIn[0]-target[0]),(controlIn[1]-target[1]),(controlIn[2]-target[2]));
//	ROS_INFO("cameraTransformation_x y z = %f,  %f,  %f\n",cameraTransformation[0],cameraTransformation[1],cameraTransformation[2]);
	//ROS_INFO("controlIn:  %f,  %f,  %f, %f\n",controlIn[0],controlIn[1],controlIn[2],controlIn[3]);
 docc.setControlInput(controlIn);
 docc.setCameraTransformation(cameraTransformation);

std::cout<<" docc.pid_yaw.error = "<< docc.pid_yaw.error<<std::endl;
std::cout<<" docc.pid_forward_v.error = "<< docc.pid_forward_v.error<<std::endl;
//std::cout<<" docc.pid_obj1_v.error = "<< docc.pid_obj1_v.error<<std::endl;
//std::cout<<" docc.pid_obj2_v.error = "<< docc.pid_obj2_v.error<<std::endl;
std::cout<<" docc.pid_objDistS_v.target = "<< docc.pid_objDistS_v.target<<std::endl;
std::cout<<" docc.pid_objDistS_v.input = "<< docc.pid_objDistS_v.input<<std::endl;
std::cout<<" docc.pid_objDistS_v.error = "<< docc.pid_objDistS_v.error<<std::endl;


//adjust PIDs
if(abs(docc.pid_forward_v.error)<8){
docc.pid_forward_v.setPID(0.001,0.0000,0.001);
}
else{
docc.pid_forward_v.setPID(0.01,0.0000,0.001);
}

if(abs(docc.pid_objDistS_v.error)<500){
docc.pid_objDistS_v.setPID(0.0001,0.0000,0.00);
}
else{
docc.pid_objDistS_v.setPID(0.001,0.00000,0.01);
}

 docc.spin(output);
	
	}
	
	
	
	
	
	
	
	
	/*

	controlIn[0] = (msg->data[3]  + msg->data[0])/2.0f;
	controlIn[1] = msg->data[0]  - msg->data[3];
	controlIn[2] = msg->data[2];
	controlIn[4] = msg->data[5];
	
	//controller
	ROS_INFO("xm = %f, distX = %f, distS = %f\n",controlIn[0],controlIn[1],controlIn[2]);
	ROS_INFO("xm_e = %f, distX_e = %f, distS_e = %f\n",(controlIn[0]-target[0]),(controlIn[1]-target[1]),(controlIn[2]-target[2]));
	ROS_INFO("cameraTransformation_x y z = %f,  %f,  %f\n",cameraTransformation[0],cameraTransformation[1],cameraTransformation[2]);
	//ROS_INFO("controlIn:  %f,  %f,  %f, %f\n",controlIn[0],controlIn[1],controlIn[2],controlIn[3]);
 docc.setControlInput(controlIn);
 docc.setCameraTransformation(cameraTransformation);
 
 docc.spin(output);
 */


//x y test
 //output[1] = 0;
 //output[2] = counter/200.0f;
 
 
 std::cout<<"output[0] = "<<output[0]<<std::endl;
 std::cout<<"output[1] = "<<output[1]<<std::endl;
 std::cout<<"output[2] = "<<output[2]<<std::endl;
  std::cout<<"counter = "<<counter<<std::endl;
 
 //cameraTransformation[0] = output[1];
 //cameraTransformation[2] = output[2];
 //cameraTransformation[4] = output[0];
	
	
	
	
	
	
	
	
	//control goes here
	cameraTransformation[4] +=  output[0];
	plant((output[1]),(output[2]),  cameraTransformation[4]);  //fly

std::cout<<"========================"<<std::endl;
	counter++;
	
	if(counter ==300){
	target[2] = 7200;
	target[3] = 9800;
	docc.setControlTargets(target);
	}
	
		if(counter ==600){
	target[2] = 9200;
	target[3] = 6000;
	docc.setControlTargets(target);
	}
	
	ros::Duration(0.1).sleep(); 
}

int main(int argc, char** argv)
{


std::string skeleton_data1_dir = "test3.txt";
	// string skeleton_data2_dir = "/skeleton_data/test2.txt";
	std::vector<geometry_msgs::Point> skeleton0, skeleton1;
	read_from_file(skeleton_data1_dir, skeleton0, 0.0);
	read_from_file(skeleton_data1_dir, skeleton1, 1.0);


	
	//  p1: (2.882683, -0.137306, 1.562213), p2: (3.478386, -1.914549, 1.562213)
	center.x = (2.882683 + 3.478386) / 2;
	center.y = (-0.137306 + -1.914549) / 2;
	center.z = 1.562213;
	
  ros::init(argc, argv, "nbv_demo");
  ros::NodeHandle n("~");
  waypoint_pub = n.advertise<nav_msgs::Path>("/waypoint_generator/waypoints",10);
  ros::Subscriber uav_sub = n.subscribe("/visual_slam/odom", 10, OdomCallback);
	ros::Subscriber local_vec_sub = n.subscribe<std_msgs::Int32MultiArray>("/array", 10, OnScreenObjectCallback);
	

	docc.init();
	docc.setCameraTransformation(cameraTransformation);
	float cameramodel[6] = 	{0,0,0,0,3.14159f*(110.0f/180),640};
	docc.setCameraModel(cameramodel);

	docc.pid_yaw.setPID(0.0005,0.00000,0.0001);
	//docc.pid_yaw.isFlipped = true;

	docc.pid_forward_v.setPID(0.01,0.0000,0.001);
	//docc.pid_forward_v.setPID(0.00000,0.000000,0);
		
		
	docc.pid_obj1_v.setPID(0.0005,0.00000,0.001);
	//docc.pid_obj1_v.setPID(0.00000,0.000000,0);
	//docc.pid_obj1_v.isFlipped = true;

	docc.pid_obj2_v.setPID(0.0005,0.00000,0.001);
	//docc.pid_obj2_v.setPID(0.0000,0.000000,0);
	//docc.pid_obj2_v.isFlipped = true;

	docc.pid_objDistS_v.setPID(0.001,0.00000,0.001);
	
	
	docc.setControlTargets(target);
	docc.setControlInput(controlIn);


	maf_xt_send.init();
	maf_xt_send.setBufferSize(5);
	maf_yt_send.init();
	maf_yt_send.setBufferSize(5);
	
  while(ros::ok()) {
 
    ros::spinOnce();
    
  }

  return 0;
}



