#ifndef TrajectoryPlanning_H
#define TrajectoryPlanning_H

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

class TrajectoryPlanning
{

	public:

		float r;
		float xt;
		float zt;
		float d;
		float resWidth;
		float fov;
		float x3d;
		float z3d;
		float x3d1;
		float z3d1;

	public:

		void init();
		void setCameraTransformation(float data[]);
		void set3DPoints(float data[]);
		void set3DPoint1(float data[]);
		void set3DPoint2(float data[]);
		void setScreenWidth(float data);
		void setFov(float data);
		void setDistXTarget(float distS_target);
		void getWayPoint(float phi, float circle[], float *wayPoint);
		void reset();
};


#endif
