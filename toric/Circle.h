
/*============================
  This class computes the centers of circles that have a given radius and pass two given points.

  The class will return the circle center point that is closer to a point representing location of a camera.

Inputs:

pp1: the coordinate of the first point
pp2: the coordinate of the second point
cam: the coordinate of the camera

Outputs:
center: the center of the cirlc

Parameters:
r: the radius of the circle, r = 10 by default



Usage:

Point2D pp1;
Point2D pp2;
Point2D cam;
Circle c;

pp1.x = 10;
pp1.y = 2;

pp2.x = -10,
pp2.y = 2;

cam.x = 3;
cam.y = 4;

c.r = 15;  //specify the radius of the circle.

float center[2];
c.getCenter(pp1,pp2,cam,out);

float center_x = out[0];
float center_y = out[1];


c.reset(); //reset the circle generator

Created by Zhenyu Yang on 2017/11/03

==============================================*/

#ifndef Circle_H
#define Circle_H


#include <cmath>
#include "../struct/Point2D.h"
#include <iostream>


class Circle
{

	public: 
		float r;
		Point2D center1;
		Point2D center2;



	public:

		void init();
		float dist2D(Point2D n, Point2D m);
		void getCenter(Point2D p1, Point2D p2, Point2D camera, float *center);
		void getRawCenters(Point2D p1, Point2D p2);
		void reset();
};

#endif
