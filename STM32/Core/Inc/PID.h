#ifndef __PID_H
#define __PID_H

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

typedef struct PID_Structure
{
	float kp,ki,kd;
	float error,last_error;
	float integral,max_integral;
	float output,max_output; 
}PID;

void PID_Calc(PID * pid, float error_in);

#endif
