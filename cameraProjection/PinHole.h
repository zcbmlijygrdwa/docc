/*============================
  This class computes projection of two 3D points into 2D space throught poin hole camera projection model. The focal length in X and Y directions needs to be specified. The offsets of 2D projection in X and Y direction (X0 and Y0) are set to zeros by default.

  The class take two 3D points coordinates and the tranforamtion(translation + rotation) of the camera as the input, and returns two 2D points as the output.

Inputs:
xt:	The x component in camera translation
yt:	The y component in camera translation
zt:	The z component in camera translation
roll:	The roll component in camera rotation
yaw:	The yaw component in camera rotation
pitch:	The pitch component in camera rotation

x3d:	The X component of the first point 3D coordinate
y3d:    The Y component of the first point 3D coordinate
z3d:    The Z component of the first point 3D coordinate
x3d2:    The X component of the second point 3D coordinate
y3d2:    The Y component of the second point 3D coordinate
z3d2:    The Z component of the second point 3D coordinate

Outputs:
x2d1:	The X component of the first point 2D projection coordinate
y2d1:	The Y component of the first point 2D projection coordinate
x2d2:	The X component of the second point 2D projection coordinate
y2d2:	The Y component of the second point 2D projection coordinate

Parameters:
fx: Focal length in X direction
fy: Focal length in Y direction
x0: 2D projection offset in X direction
y0: 2D projection offset in Y direction

Usage:

float cameraModel[4];
cameraModel[0] = x0;
cameraModel[1] = y0;
cameraModel[2] = fx;
cameraModel[3] = fy;


float cameraTransformation[6];
cameraTransformation[0] = xt;
cameraTransformation[1] = yt;
cameraTransformation[2] = zt;
cameraTransformation[3] = roll;
cameraTransformation[4] = yaw;
cameraTransformation[5] = pitch;

float point3D1[3];
point3D1[0] = x3d;
point3D1[1] = y3d;
point3D1[2] = z3d;


float point3D2[3];
point3D2[0] = x3d2;
point3D2[1] = y3d2;
point3D2[2] = z3d2;

PinHole ph;
ph.setCameraModel(cameraModel);
ph.setCameraTransformation(cameraTransformation);
ph.set3DPoint1(point3D1);
ph.set3DPoint2(point3D2);

ph.compute();
float phOutput[4];
ph.getResults(phOutput);




ph.reset(); reset the pin hole projector


Created by Zhenyu Yang on 2017/10/23

==============================================*/

#ifndef PinHole_H
#define PinHole_H

#include <math.h>

class PinHole
{

	public:
		//inputs for camera model
		float x0;
		float y0;
		float fx;
		float fy;

		//inputs for camera translationi
		float xt;
		float yt;
		float zt;
		float roll;
		float yaw;
		float pitch;

		//inputs for 2 3D points
		float x3d;
		float y3d;
		float z3d;
		float x3d2;
		float y3d2;
		float z3d2;

		//2D points
		float x2d1;
		float y2d1;
		float x2d2;
		float y2d2;

	public:

		void init();
		//sets
		void setCameraModel(float model[]);
		void setCameraModel_x0(float data);
		void setCameraModel_y0(float data);
		void setCameraModel_fx(float data);
		void setCameraModel_fy(float data);
		void set3DPoint1(float x, float y, float z);
		void set3DPoint2(float x, float y, float z);
		void set3DPoint1(float data[]);
		void set3DPoint2(float data[]);
		void setCameraTransformation(float data[]);
		
		void getResults(float *data);
		void compute();
		void reset();
};


#endif
