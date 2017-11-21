/*===================================
  This is class for computation of trajectory planning.
  
Inputs:
r: the radius of the constrain circle.

x3d: the x component of the first object 3D coordinate
z3d: the z component of the first object 3D coordinate
x3d1: the x component of the second object 3D coordinate
z3d1: the z component of the second object 3D coordinate


Outputs:
xt: the x component of the camera translation
zt: the z component of the camera translation

Parameters:
resWidth: the width of the camera screen, in pixels.
fov: camera field of view in horizontal direction.
d: the target distX



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



float output[3];

TrajectoryPlanning tp;
tp.init();
tp.setCameraModel(cameraModel);
tp.setCameraTransformation(cameraTransformation);
tp.set3DPoint1(point3D1);
tp.set3DPoint2(point3D2);
tp.setDistXTarget(distX_target);


realTimeLoop{

..........
tp.set3DPoint1(point3D1);
tp.set3DPoint2(point3D2);
tp.setCameraTransformation(cameraTransformation);

float wapPoint[2];
tp.getWayPoint(phi,circle,wapPoint);	
......................


}

tp.reset(); //reset the tocc

Created on 2017/10/26 By Zhenyu Yang
================================== */


#include <cmath>
#include <iostream>
#include "TrajectoryPlanning.h"



void TrajectoryPlanning::init()
{
	r = 0;
	xt = 0;
	zt = 0;
	d = 0;
	resWidth = 0;
}

void TrajectoryPlanning::setCameraTransformation(float data[])
{
	xt = data[0];
	zt =  data[2];
}

void TrajectoryPlanning::set3DPoints(float data[])
{
	x3d = (data[0]);
	z3d = (data[2]);

	x3d1 =(data[3]);
	z3d1 =(data[5]);

}


void TrajectoryPlanning::set3DPoint1(float data[])
{
	x3d = data[0];
	z3d = data[2];
}

void TrajectoryPlanning::set3DPoint2(float data[])
{
	x3d1 = data[0];
	z3d1 = data[2];
}

void TrajectoryPlanning::setScreenWidth(float data)
{
	resWidth = data;
}

void TrajectoryPlanning::setFov(float data)
{
	fov = data;
}

void TrajectoryPlanning::setDistXTarget(float distS_target)
{

	d = std::abs(distS_target);
}

void TrajectoryPlanning::getWayPoint(float phi, float circle[], float *wayPoint)
{
	float red_x_3d = x3d;
	float red_y_3d = z3d;
	float blue_x_3d = x3d1;
	float blue_y_3d = z3d1;
	float w = resWidth;
	float alpha = 2*atan((d/w)*tan(fov/2));

	float distance = sqrt((red_x_3d-blue_x_3d)*(red_x_3d-blue_x_3d)+(red_y_3d-blue_y_3d)*(red_y_3d-blue_y_3d));
	r = (distance/2)/sin(alpha);//radius

	
	wayPoint[0] = r*cos(phi)+circle[0];
	wayPoint[1] = r*sin(phi)+circle[1];	

}

void TrajectoryPlanning::reset()
{
	r = 0;
	xt = 0;
	zt = 0;
	d = 0;
	resWidth = 0;
}
