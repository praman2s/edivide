#include <math.h>
#include "RP6RobotBaseLib.h" 
#include "trajectory.h"

// Function Prototypes
void fancyled(uint8_t *runningLight);
void Odometry();
void reInitOdom();
void CalculateSpherical(int16_t spherical[3]);
void Controller(int16_t spherical[3]);
void mainloop();

double source[3] = {0,0,0};
double destination[3] = {50,50,0};
double destinationarray[][3] = {{50,20,0},{80,0,0},{50,-20,0},{80,10,0},{0,-10,0},{0,0,0}};
int16_t spherical[3] = {0,0,0};
static uint8_t motioncomplete;
static double robot_old_x = 0,robot_old_y=0,robot_old_theta=0,old_beta=0;

void fancyled(uint8_t *runningLight)
{
	setLEDs(*runningLight); 
	*runningLight <<= 1; 
	if(*runningLight > 32)
		*runningLight = 1;

}
//ODOMETRY NEEDS TO BE IMPROVED A LOT> IT IS BUGGY >>> :-(
void Odometry()
{
	double robot_x,robot_y,robot_theta,beta;
	double temp;
	//robot_theta= (((getLeftDistance()-getRightDistance()))/(uint32_t)(ROTATION_FACTOR));
	if (getLeftDistance() > getRightDistance())
		robot_theta = (double)((uint32_t)(100)*(getLeftDistance()-getRightDistance())/(uint32_t)(2*ROTATION_FACTOR));
	else
		robot_theta = -(double)((uint32_t)(100)*(getRightDistance()-getLeftDistance())/(uint32_t)(2*ROTATION_FACTOR));	
	robot_theta = (abs(robot_theta) <=359 && abs(robot_theta) > 0) ? robot_theta : 0.0;
	/*temp = cos(robot_theta);
	if (mleft_dir != mright_dir)
		robot_x = (M_PI*25*temp*(getRightDistance()+getLeftDistance()))/625;
	else{
		if (mleft_dir != 0)
			robot_x = -(M_PI*25*temp*(getRightDistance()+getLeftDistance()))/625;
		else
			robot_x = (M_PI*25*temp*(getRightDistance()+getLeftDistance()))/625;
	}
	robot_x = (M_PI*25*temp*(getRightDistance()+getLeftDistance()))/625;
	temp = sin(beta);
	robot_y = (M_PI*25*temp*(getRightDistance()+getLeftDistance()))/625;
	temp = (robot_x-robot_old_x);	
	robot_old_x = robot_x;
	
	
	
	source[0] = source[0]+temp;
	source[1] = robot_y/10;*/
	double temp_theta,temp_x,temp_y;
	temp_theta= robot_theta - robot_old_theta;
	source[2] = source[2]+temp_theta;
	if (abs(source[2]) >180)
		source[2] = (source[2] < 0)? (360+source[2]):(source[2]-360);
	
	robot_x = (M_PI*25*cos((robot_theta*M_PI)/180)*(getRightDistance()+getLeftDistance()))/625;
	robot_y = (M_PI*25*sin((robot_theta*M_PI)/180)*(getRightDistance()+getLeftDistance()))/625;
	temp_x = robot_x - robot_old_x;
	temp_y = robot_y - robot_old_y;
	source[0] = source[0]+temp_x;
	source[1] = source[1]+temp_y;
	robot_old_x = robot_x;
	robot_old_y = robot_y;
	robot_old_theta = robot_theta;	
	
	
		
	

}

void reInitOdom()
{
	//ENCODERS RESET TO ZERO

}

void OpenLoopController(int16_t alpha,uint16_t pho, int16_t beta)
{	
	motioncomplete = 1;	
	uint8_t dir;
	uint16_t angle = 0;
	if (abs(alpha) > 0){
		dir = alpha < 0 ? LEFT : RIGHT;
		angle = abs(alpha);
		rotate(60, dir,angle, true);
	}
	writeIntegerLength(pho, DEC,16);
	move(80, FWD, DIST_MM(pho*10),true);
	beta = -alpha;
	if (abs(beta) > 0){
		dir = beta < 0 ? LEFT : RIGHT;
		angle = abs(beta);
		rotate(60, dir, angle, true);
	}
	  //VERY VERY BAD AS IT IS A OPEN LOOP CONTROLLER
	motioncomplete = 0;
}

void SiegwartController(double alpha,uint16_t pho, int16_t beta)
{
	
	
	if ((destination[0]-source[0])<10){
		motioncomplete = 0; 
		writeString("Goal Reached\n");
	}
	else{
		uint8_t linear_speed = 2*(pho/20);
		uint8_t angular_speed = 24*alpha;
		//uint8_t angular_speed = 0;
		uint8_t v_right = (linear_speed) + (10 * angular_speed);
		uint8_t v_left =  (linear_speed) - (10 * angular_speed);
		//uint8_t v_right = (linear_speed) + (10*angular_speed);
		//uint8_t v_left =  (linear_speed) - (10*angular_speed);
		moveAtSpeedDirection(v_left,v_right);
	}
	
}

void mainloopClosed()
{
	motioncomplete = 1;
	while(motioncomplete == 1){
	
		double delta_x=0.0, delta_y=0.0,pho=0.0;
		Odometry();
		delta_x = destination[0] - source[0];
		delta_y = destination[1] - source[1];
		pho = sqrt(pow(delta_x,2)+pow(delta_y,2));
		double alpha = atan2(delta_y,delta_x);
		/*if (abs(alpha*100) > 0){
			alpha = (alpha*180)/M_PI; 
			
		}*/
		task_RP6System();
		SiegwartController(alpha,pho,0);
		task_RP6System(); 
		task_motionControl();
		mSleep(200);
		

	
	}


}


void mainloopOpen()
{	
	motioncomplete=1;
	while(motioncomplete == 1) {	
		double delta_x=0.0, delta_y=0.0,pho=0.0;
		//Odometry();	   //should comment for open loop controller
		delta_x = destination[0] - source[0];
		delta_y = destination[1] - source[1];
		pho = sqrt(pow(delta_x,2)+pow(delta_y,2));
		double alpha = atan2(delta_y,delta_x);
		
	
		if (abs(alpha*100) > 0){
			alpha = (alpha*180)/M_PI; 
			//alpha = (alpha < 0 ? -(alpha +180):alpha);
		
		}
		/*writeIntegerLength((int16_t)delta_y, DEC,16);
		writeString("\n");
		writeIntegerLength((int16_t)delta_x, DEC,16);
		writeString("\n");
		writeIntegerLength((int16_t)alpha, DEC,16);
		writeString("\n");*/
		task_RP6System();
		/*REPLACE THIS CONTROLLER*/
		OpenLoopController((int16_t)alpha,pho,0); 
		//SiegwartController(alpha,pho,0); 
		task_RP6System();
		//motioncomplete=0;  //debug ... do not forget to remove :-)
		mSleep(200);
	}                 
	
	

}


int16_t main(void)
{
	initRobotBase();
	powerON(); 
	setLEDs(0b111111); 
	mSleep(1000); 
	setLEDs(0b000000);
	mSleep(500);
	uint8_t runningLight = 1;
	for (uint8_t i = 0; i < 6; i++){
		for (uint8_t j = 0; j< 3;j++)
			destination[j] = destinationarray[i][j];
		//mainloopOpen();
		mainloopClosed();
		fancyled(&runningLight);
		/*for (uint8_t j = 0; j< 3;j++)
			source[j] = destination[j];*/
		mSleep(2000);
		
	}
	task_RP6System();
	while(true)
	{
		fancyled(&runningLight);
		if (motioncomplete == 0)
			moveAtSpeed(0,0);
		mSleep(200);
		task_RP6System();
	}
	return 0; 
}
