
// g++ -o test test.cpp DOCC.cpp basicController/PID.cpp filters/MAFilter.cpp cameraProjection/PinHole.cpp toric/Circle.cpp trajectory/TrajectoryPlanning.cpp

#include <iostream>
#include <math.h>
#include "DOCC.h"
#include "cameraProjection/PinHole.h"
#include "toric/Circle.h"
#include "struct/Point2D.h"
#include "filters/MAFilter.h"
#include "trajectory/TrajectoryPlanning.h"

int main()
{


	//inputs for camera model
	float x0 = 0;
	float y0 = 0;
	float fx = 50.0f/1000;
	float fy = fx;
	float cameraModel[7];
	float fov = (60/180.0f)*3.14;
	float resWidth = 500;
	float resHeight  = 800;
	cameraModel[0] = x0;
	cameraModel[1] = y0;
	cameraModel[2] = fx;
	cameraModel[3] = fy;
	cameraModel[4] = fov;
	cameraModel[5] = resWidth;
	cameraModel[6] = resHeight;

	float K_size = 1.0f;
	//inputs for camera translationi
	float xt = 0;
	float yt = 0;
	float zt = 0;
	float roll = 0;
	float yaw = 0;
	float pitch = 0;
	float cameraTransformation[6];
	cameraTransformation[0] = xt;
	cameraTransformation[1] = yt;
	cameraTransformation[2] = zt;
	cameraTransformation[3] = roll;
	cameraTransformation[4] = yaw;
	cameraTransformation[5] = pitch;



	//on-screen 2D points

	float xm = 0;
	float distX = 0;
	float distS = 0;


	float xm_v = 0;
	float distX_v = 0;
	float distS_v = 0;

	float xm_target = 0.2f;
	float distX_target = 0.15f;
	float distS_target = 180.0f;




	std::cout<<"--------- docc test -------"<<std::endl;



	//DOCC test
	DOCC docc;
	docc.init();

	std::cout<<"--------- docc test 1-------"<<std::endl;
	docc.setCameraModel(cameraModel);
	std::cout<<"--------- docc test 2-------"<<std::endl;
	std::cout<<"cameraTransformation[1] = "<<cameraTransformation[1]<<std::endl;
	docc.setCameraTransformation(cameraTransformation);
	std::cout<<"--------- docc test 3-------"<<std::endl;
	float target[5] = {300,20,10,4000,3000};
	docc.setControlTargets(target);
	std::cout<<"--------- docc test 4-------"<<std::endl;
	float controlIn[4] = {59,10,3000,3400};
	docc.setControlInput(controlIn);
	std::cout<<"--------- docc test 5-------"<<std::endl;
	float output[3];


	for(int i = 0; i < 10;i++){

		controlIn[0] =0;// 49+i*0.05f;
		docc.setControlInput(controlIn);
		docc.setCameraTransformation(cameraTransformation);

		docc.spin(output);
		std::cout<<"output[0] yaw = "<<output[0]<<std::endl;
		std::cout<<"output[1] xt = "<<output[1]<<std::endl;
		std::cout<<"output[2] yt = "<<output[2]<<std::endl;
		std::cout<<"========================"<<std::endl;
		cameraTransformation[0] = output[1];
		cameraTransformation[1] = output[2];
		cameraTransformation[4] = output[0];

	}







}
