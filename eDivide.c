#include "RP6RobotBaseLib.h" 
#define right_tick (getRightDistance()/400) 
#define left_tick (getLeftDistance()/400)  

int main(void)
{
	initRobotBase(); // Always call this first! The Processor will not work
	powerON();
	//Declrations
	double linear_speed = 0,angular_speed = 0,v_left,v_right,distance_bw=100;
	double delta_x,delta_y,theta = 0,temp;
        double source_x=0,source_y=0,source_theta=0;
	double goal_x=100,goal_y=0,goal_theta=0,robot_x,robot_y;
	double robot_theta = 0;
        double rho,alpha,beta;
	//controller
	cos(robot_theta);
	setLEDs(0b111111); 
	uint8_t kp = 2; 
	delta_x = goal_x - source_x;
	delta_y = goal_y - source_y;
	while(true)
	{	
		robot_theta = (2*M_PI*25*(right_tick-left_tick))/(625*distance_bw);
		temp = cos(robot_theta);
		robot_x = (2*M_PI*temp*(right_tick+left_tick))/625;
		temp = sin(robot_theta);
		robot_y = (2*M_PI*temp*(right_tick+left_tick))/625;
		delta_x = goal_x - robot_x;
		delta_y = goal_y - robot_y;		
		rho = sqrt(delta_x*delta_x + delta_y*delta_y);
        	alpha = -theta + atan2(delta_y,delta_x);
		beta = -theta-alpha;
		linear_speed = kp*rho;
		angular_speed = 0;
		v_right = (2*linear_speed) + (distance_bw * angular_speed);
		v_left =  (2*linear_speed) + (distance_bw * angular_speed); 
                moveAtSpeed(v_left,v_right);
                /*if (getLeftDistance() >=(40*) && getRightDistance() >=(4000+error)){
			setLEDs(0b000000); 
			stop();
			moveAtSpeed(0,0); 
		}*/
			
               	task_RP6System(); 
	}
	
	return 0; 
}
