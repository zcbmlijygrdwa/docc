
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

#include <cmath>
#include "../struct/Point2D.h"
#include "Circle.h"
#include <iostream>
		void Circle::init()
		{
			r = 10;
		}
		float Circle::dist2D(Point2D n, Point2D m)
		{
			return sqrt( (n.x-m.x)*(n.x-m.x)  + (n.y - m.y)*(n.y  -m.y)  );
		}

		void Circle::getCenter(Point2D p1, Point2D p2, Point2D camera, float *center)
		{

			if(dist2D(p1,p2)>=2*r)
			{
				center[0] = 0;
				center[1] = 0;
				return;
			}
			getRawCenters(p1,p2);
			float distance1 = dist2D(center1,camera);
			float distance2 = dist2D(center2,camera);
			if(distance1<=distance2)
			{
				center[0] = center1.x;
				center[1] = center1.y;
			}
			else{

				center[0] = center2.x;
				center[1] = center2.y;
			}

		}

		void Circle::getRawCenters(Point2D p1, Point2D p2)
		{
			float x_center1 = 0;
			float y_center1 = 0;
			float x_center2 = 0;
			float y_center2 = 0;

			center1.x = 0;
			center1.y = 0;

			center2.x = 0;
			center2.y = 0;

			float x_middle = (p1.x+p2.x)/2.0f;
			float y_middle = (p1.y+p2.y)/2.0f;

			if((p2.x-p1.x)!=0&&(p2.y-p1.y!=0)){
				float k = (p2.y-p1.y)/(p2.x-p1.x);
				float K_middle_to_center = -1/k;

				float d = dist2D(p1,p2); 
				float theta = acos((d/2)/r);
				float d_middle_to_center = r*sin(theta);

				float beta = atan(K_middle_to_center);
				float delta_x = d_middle_to_center*cos(beta);
				float delta_y = d_middle_to_center*sin(beta);
				x_center1 = x_middle+delta_x;
				y_center1 = y_middle+delta_y;

				x_center2 = x_middle-delta_x;
				y_center2 = y_middle-delta_y;

				center1.x = x_middle+delta_x;
				center1.y = y_middle+delta_y;

				center2.x = x_middle-delta_x;
				center2.y = y_middle-delta_y;
			}

			else if((p2.x-p1.x)==0)
			{
				//the x coordinate of two points are the same in this case.
				y_center1 = (p2.y+p1.y)/2.0f;
				y_center2 = y_center1;	

				center1.y = (p2.y+p1.y)/2.0f;
				center2.y = y_center1;	

				float d = dist2D(p1,p2);
				float theta = acos((d/2)/r);
				float d_middle_to_center = r*sin(theta);
				x_center1 = x_middle + d_middle_to_center;
				x_center2 = x_middle - d_middle_to_center;

				center1.x = x_middle + d_middle_to_center;
				center2.x = x_middle - d_middle_to_center;


			}

			else if((p2.y-p1.y)==0)
			{
				//the y coordinate of two points are the same in this case.
				x_center1 = (p2.x+p1.x)/2.0f;
				x_center2 = y_center1;	

				center1.x = (p2.x+p1.x)/2.0f;
				center2.x = y_center1;	

				float d = dist2D(p1,p2);
				float theta = acos((d/2)/r);
				float d_middle_to_center = r*sin(theta);

				center1.y = y_middle + d_middle_to_center;
				center2.y = y_middle - d_middle_to_center;


			}

			else
			{
				std::cout<<"Cicle: Unkown state."<<std::endl;
			}
		}


		void Circle::reset()
		{
			r = 10;
		}
