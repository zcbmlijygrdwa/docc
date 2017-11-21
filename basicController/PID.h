#ifndef PID_H
#define PID_H
/*===================================
  This is a PID controller that can adjust its output based on the error (between the input and the reference value). 

  P, I, and D parameters can be used to tune the controller.

  This controller is a single-input single-output controller.

  A PID controller is a proportional–integral–derivative controller (PID controller or three term controller) is a control loop feedback mechanism widely used in industrial control systems and a variety of other applications requiring continuously modulated control. 

Input:
inputIn: the input data to pid controller


Output:
output_to_plant: the output of the controller

Parameters:
p = the proportional coeffcient
i = the integral coeffcient
d = the derivative coeffcient

Usage:
PID pid;
pid.init();	//initialization
pid.setTarget(10);	//set target value
pid.setPID(0.1,0.01,0.02);	//set P, I, and D parameters

...
loop()
{

pid_output = pid.spin(pid_input);

}


pid.reset(); //reset the controller

Created on 2017/06/01 By Zhenyu Yang
================================== */

class PID{

	public:
		float target;	//target value
		float input;	//current value
		float error;	//current error
		float errorPrev;//previous error
		float error_d;	//error derivative
		float error_i;	//error integral

		float p;	//proportional coeffcient	
		float i;	//integral coefficient
		float d;	//derivative coefficient

		float output_to_plant;	//output of the controller
		float output_offset;
		bool isFlipped;		//whether switch sign of the output

	public:
		void init();
		void setTarget(float targetIn);
		void setPID(float pIn, float iIn, float dIn);
		float spin(float inputIn);
		void setOutputOffset(float input);
		void reset();
};

#endif
