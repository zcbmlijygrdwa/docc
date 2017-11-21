/*===========================
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


#include "PinHole.h"
#include <math.h>
		void PinHole::init()
		{


			//default inputs for camera model
			x0 = 0;
			y0 = 0;
			fx = 500;
			fy = fx;
			//default inputs for camera translationi
			xt = 0;
			yt = 0;
			zt = 0;
			roll = 0;
			yaw = 0;
			pitch = 0;

			//default inputs for 2 3D points
			x3d = -55;
			y3d = 0;
			z3d = 10;
			x3d2 = 5;
			y3d2 = 0;
			z3d2 = 10;

		}

		//sets
		void PinHole::setCameraModel(float model[])
		{
			x0 = model[0];
			y0 = model[1];
			fx = model[2];
			fy = model[3];
		}

		void PinHole::setCameraModel_x0(float data)
		{
			x0 = data;
		}

		void PinHole::setCameraModel_y0(float data)
		{
			y0 = data;
		}

		void PinHole::setCameraModel_fx(float data)
		{
			fx = data;
		}

		void PinHole::setCameraModel_fy(float data)
		{
			fy = data;
		}



		void PinHole::set3DPoint1(float x, float y, float z)
		{
			x3d = x;
			y3d = y;
			z3d =z;
		}

		void PinHole::set3DPoint2(float x, float y, float z)
		{
			x3d2 = x;
			y3d2 = y;
			z3d2 = z;
		}


		void PinHole::set3DPoint1(float data[])
		{
			x3d = data[0];
			y3d = data[1];
			z3d = data[2];
		}

		void PinHole::set3DPoint2(float data[])
		{
			x3d2 = data[0];
			y3d2 = data[1];
			z3d2 = data[2];
		}


		void PinHole::setCameraTransformation(float data[])
		{
			xt = data[0];
			yt =  data[1];
			zt =  data[2];
			roll =  data[3];
			yaw =  data[4];
			pitch =  data[5];


		}

		void PinHole::getResults(float *data)
		{
			data[0] = x2d1;
			data[1] = y2d1;
			data[2] = x2d2;
			data[3] = y2d2;

		}



		void PinHole::compute()
		{
			x2d1 = (fx*xt + x0*zt + x3d*(fx*cos(yaw) - x0*sin(yaw)) + z3d*(fx*sin(yaw) + x0*cos(yaw)))/(zt + z3d*cos(yaw) - x3d*sin(yaw));

			y2d1 = (fy*y3d + fy*yt + y0*zt + y0*z3d*cos(yaw) - x3d*y0*sin(yaw))/(zt + z3d*cos(yaw) - x3d*sin(yaw));

			x2d2 = (fx*xt + x0*zt + x3d2*(fx*cos(yaw) - x0*sin(yaw)) + z3d2*(fx*sin(yaw) + x0*cos(yaw)))/(zt + z3d2*cos(yaw) - x3d2*sin(yaw));

			y2d2 = (fy*y3d2 + fy*yt + y0*zt + y0*z3d2*cos(yaw) - x3d2*y0*sin(yaw))/(zt + z3d2*cos(yaw) - x3d2*sin(yaw));
		}

		void PinHole::reset()
		{

			//default inputs for camera model
			x0 = 0;
			y0 = 0;
			fx = 500;
			fy = fx;
			//default inputs for camera translationi
			xt = 0;
			yt = 0;
			zt = 0;
			roll = 0;
			yaw = 0;
			pitch = 0;

			//default inputs for 2 3D points
			x3d = -55;
			y3d = 0;
			z3d = 10;
			x3d2 = 5;
			y3d2 = 0;
			z3d2 = 10;

		}
