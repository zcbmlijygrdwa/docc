/*===================================
  This is class for Dual Object Camera Controller (DOCC).
  This controller requires 3 inputs and produces 3 outputs.

  Controller Inputs:
  [1] xm: The x coordinate of the middle point of two objects in a camera frame.
  [2] distX: The x coordinate difference between two objects in a camera frame.
  [3] distS: This difference in size between two objects in a camaera frame. 


  Controller Output:
  [1] yaw: The camera rotation about x axis of the gloable coordinate.
  [2] xt: The camera translation in x derection of the gloable coordinate.
  [3] zt: The camera translation in z derection of the gloable coordinate.



Usage:

float cameraModel[7];
cameraModel[0] = x0;
cameraModel[1] = y0;
cameraModel[2] = fx;
cameraModel[3] = fy;
cameraModel[4] = fov;
cameraModel[5] = resWidth;
cameraModel[6] = resHeight;



float cameraTransformation[6];
cameraTransformation[0] = xt;
cameraTransformation[1] = yt;
cameraTransformation[2] = zt;
cameraTransformation[3] = roll;
cameraTransformation[4] = yaw;
cameraTransformation[5] = pitch;

float controlIn[3] = {29,2,3};

float target[4] = {300,20,10,5};

float output[3];

DOCC tocc;
tocc.init();
tocc.setCameraModel(cameraModel);
tocc.setCameraTransformation(cameraTransformation);
tocc.set3DPoint1(point3D1);
tocc.set3DPoint2(point3D2);
tocc.setControlTargets(target);
tocc.setControlInput(controlIn);


realTimeLoop{

..........
tocc.setControlInput(controlIn);
tocc.set3DPoint1(point3D1);
tocc.set3DPoint2(point3D2);
tocc.setCameraTransformation(cameraTransformation);

tocc.spin(output);
cameraTransformation[0] = output[1];
cameraTransformation[2] = output[2];
cameraTransformation[4] = output[0];
......................


}

tocc.reset(); //reset the tocc

Created on 2017/11/01 By Zhenyu Yang
================================== */



#include<iostream>
#include <cmath>
#include "DOCC.h"

TrajectoryPlanning tp;

void DOCC::init()
{
	PI =  3.141592653f;  //define pi
	fov = (140.0f/180)*PI; //default field of view

	//PID controllers initialization
	pid_yaw.init();
	pid_yaw.setPID(0.01,0,0);
	
	pid_forward_v.init();
	pid_forward_v.setPID(0.01,0,0);

	pid_obj1_v.init();
	pid_obj1_v.setPID(0.01,0,0);

	pid_obj2_v.init();
	pid_obj2_v.setPID(0.01,0,0);
<<<<<<< HEAD
	
	pid_objDistS_v.init();
	pid_objDistS_v.setPID(0.01,0,0);
=======
>>>>>>> b9834d0b10c80870db1c58da0ad910a8f0826da0

	//MA filters initialization
	maf_xm.init();
	maf_distX.init();
	maf_s1.init();
	maf_s2.init();
<<<<<<< HEAD
	maf_xm.setBufferSize(5); 
  maf_distX.setBufferSize(5); 
  maf_s1.setBufferSize(5); 
  maf_s2.setBufferSize(5); 
=======
	maf_xm.setBufferSize(10); 
        maf_distX.setBufferSize(10); 
        maf_s1.setBufferSize(10); 
        maf_s2.setBufferSize(10); 
>>>>>>> b9834d0b10c80870db1c58da0ad910a8f0826da0
 
	maf_xt.init();
	maf_yt.init();
	maf_zt.init();
<<<<<<< HEAD
	maf_xt.setBufferSize(5); 
	maf_yt.setBufferSize(5); 
  maf_zt.setBufferSize(5);
=======
	maf_xt.setBufferSize(10); 
	maf_yt.setBufferSize(10); 
        maf_zt.setBufferSize(10);
>>>>>>> b9834d0b10c80870db1c58da0ad910a8f0826da0

}


void DOCC::spin(float *controlOutput)
{
	//this function runs every frame to calculate the next transformation of the camera.
	//assuming the roll, pitch and yt remains zeros. 
	float yaw_control_output = pid_yaw.spin(xm);
	//std::cout<<"pid_yaw.target = "<<pid_yaw.target<<std::endl;
	//std::cout<<"pid_yaw.input = "<<pid_yaw.input<<std::endl;
	//std::cout<<"pid_yaw.output_offset = "<<pid_yaw.output_offset<<std::endl;
	//std::cout<<"yaw_control_output = "<<yaw_control_output<<std::endl;
	controlOutput[0] = yaw_control_output;

	float forward_v_control_output = pid_forward_v.spin(distX);	
	float obj1_v_control_output = pid_obj1_v.spin(s1);	
	float obj2_v_control_output = pid_obj2_v.spin(s2);	
<<<<<<< HEAD
	float objDistS_v_control_output = pid_objDistS_v.spin(s1-s2);	
		
		
	//std::cout<<"forward_v_control_output = "<<forward_v_control_output<<std::endl;
	//std::cout<<"obj1_v_control_output = "<<obj1_v_control_output<<std::endl;
	//std::cout<<"obj2_v_control_output = "<<obj2_v_control_output<<std::endl;
=======
	
	std::cout<<"forward_v_control_output = "<<forward_v_control_output<<std::endl;
	std::cout<<"obj1_v_control_output = "<<obj1_v_control_output<<std::endl;
	std::cout<<"obj2_v_control_output = "<<obj2_v_control_output<<std::endl;
>>>>>>> b9834d0b10c80870db1c58da0ad910a8f0826da0
	
	//restore the x1 and x2;
	float x1 = xm-distX/2.0f;
	float x2 = xm+distX/2.0f;

	//get the forward angle
	float forward = (((resWidth/2.0f) - xm) / (((resWidth/2.0f)))) *(fov/2.0f) + yaw;
<<<<<<< HEAD
 std::cout<<"forward angle = "<<forward<<std::endl;
=======
 
>>>>>>> b9834d0b10c80870db1c58da0ad910a8f0826da0
	//get the tilt angle from center to objs
	float tilt1 = (((resWidth/2.0f) - x1) / (((resWidth/2.0f)))) *(fov/2.0f) + yaw;  
	float tilt2 = (((resWidth/2.0f) - x2) / (((resWidth/2.0f)))) *(fov/2.0f) + yaw;  

<<<<<<< HEAD
//std::cout<<"fov= "<<fov<<std::endl;
//std::cout<<"x2= "<<x2<<std::endl;
//std::cout<<"resWidth= "<<resWidth<<std::endl;

	float delta_x_forward = cos(forward)*forward_v_control_output;
	float delta_y_forward = sin(forward)*forward_v_control_output;
	
	std::cout<<"delta_x_forward = "<<delta_x_forward<<std::endl;
	std::cout<<"delta_y_forward = "<<delta_y_forward<<std::endl;
	

/*
=======
	float delta_x_forward = cos(forward)*forward_v_control_output;
	float delta_y_forward = sin(forward)*forward_v_control_output;
>>>>>>> b9834d0b10c80870db1c58da0ad910a8f0826da0

	
	float delta_x_tilt1 = cos(tilt1)*obj1_v_control_output;
	float delta_y_tilt1 = sin(tilt1)*obj1_v_control_output;
	
	float delta_x_tilt2 = cos(tilt2)*obj2_v_control_output;
	float delta_y_tilt2 = sin(tilt2)*obj2_v_control_output;

<<<<<<< HEAD
	std::cout<<"delta_x_forward = "<<delta_x_forward<<std::endl;
	std::cout<<"delta_y_forward = "<<delta_y_forward<<std::endl;

	std::cout<<"delta_x_tilt1 = "<<delta_x_tilt1<<std::endl;
	std::cout<<"delta_y_tilt1 = "<<delta_y_tilt1<<std::endl;

	std::cout<<"delta_x_tilt2 = "<<delta_x_tilt2<<std::endl;
	std::cout<<"delta_y_tilt2 = "<<delta_y_tilt2<<std::endl;
	*/
	
	


	
	float delta_x_tilt1 = cos(tilt1)*objDistS_v_control_output;
	float delta_y_tilt1 = sin(tilt1)*objDistS_v_control_output;
	
	float delta_x_tilt2 = -cos(tilt2)*objDistS_v_control_output;
	float delta_y_tilt2 = -sin(tilt2)*objDistS_v_control_output;


	std::cout<<"delta_x_tilt1 = "<<delta_x_tilt1<<std::endl;
	std::cout<<"delta_y_tilt1 = "<<delta_y_tilt1<<std::endl;

	std::cout<<"delta_x_tilt2 = "<<delta_x_tilt2<<std::endl;
	std::cout<<"delta_y_tilt2 = "<<delta_y_tilt2<<std::endl;
	
=======
>>>>>>> b9834d0b10c80870db1c58da0ad910a8f0826da0

	float delta_x = delta_x_forward + delta_x_tilt1 + delta_x_tilt2;
	float delta_y = delta_y_forward + delta_y_tilt1 + delta_y_tilt2;
	
<<<<<<< HEAD
	
	std::cout<<"delta_x = "<<delta_x<<std::endl;
	std::cout<<"delta_y = "<<delta_y<<std::endl;

=======
	std::cout<<"delta_x = "<<delta_x<<std::endl;
	std::cout<<"delta_y = "<<delta_y<<std::endl;
>>>>>>> b9834d0b10c80870db1c58da0ad910a8f0826da0
	controlOutput[1] = xt+delta_x;
	controlOutput[2] = yt+delta_y;
}


<<<<<<< HEAD



=======
>>>>>>> b9834d0b10c80870db1c58da0ad910a8f0826da0
void DOCC::setCameraModel(float model[])
{
	fov = model[4];
	resWidth = model[5];        	
}

void DOCC::setCameraTransformation(float data[])
{
        xt = (data[0]);
        yt =  (data[1]);
        zt =  (data[2]);
        roll =  (data[3]);
        yaw =  (data[4]);
        pitch =  (data[5]);
}



void DOCC::setControlInput(float data[])
{
<<<<<<< HEAD
	xm = (data[0]);
	distX = (data[1]);
	//s1 = (data[2]);
	//s2 = (data[3]);
	
	//xm = maf_xm.put(data[0]);
	//distX = maf_distX.put(data[1]);
	s1 = maf_s1.put(data[2]);
	s2 = maf_s2.put(data[3]);
=======
	//xm = (data[0]);
	//distX = (data[1]);
	//s1 = (data[2]);
	//s2 = (data[2]);
	
	xm = maf_xm.put(0);
	distX = maf_distX.put(data[1]);
	s1 = maf_s1.put(data[2]);
	s2 = maf_s2.put(data[2]);
>>>>>>> b9834d0b10c80870db1c58da0ad910a8f0826da0
}

void DOCC::setControlTargets(float data[])
{
	xm_target = data[0];
	distX_target = data[1];
	s1_target = data[2];
<<<<<<< HEAD
	s2_target = data[3];
=======
	s2_target = data[2];
>>>>>>> b9834d0b10c80870db1c58da0ad910a8f0826da0

	pid_yaw.setTarget(xm_target);
	pid_forward_v.setTarget(distX_target);
	pid_obj1_v.setTarget(s1_target);
	pid_obj2_v.setTarget(s2_target);
<<<<<<< HEAD
	pid_objDistS_v.setTarget(s1_target-s2_target);	
=======
>>>>>>> b9834d0b10c80870db1c58da0ad910a8f0826da0
}

void DOCC::reset()
{
	//PID controllers initialization
	pid_yaw.reset();
	pid_forward_v.reset();
	pid_obj1_v.reset();
	pid_obj2_v.reset();
<<<<<<< HEAD
	pid_objDistS_v.reset();
=======
>>>>>>> b9834d0b10c80870db1c58da0ad910a8f0826da0
}

