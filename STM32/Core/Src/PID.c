#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "PID.h"
#include "Serial.h"

void PID_Calc(PID * pid, float error_in)
{
	pid->last_error = pid->error;
	pid->error = error_in;
	
	pid->output = pid->error * pid->kp;
	
	pid->integral+=pid->ki;
	LIMIT(pid->integral,-pid->max_integral,pid->max_integral);
	pid->output += pid->integral;
	
	pid->output += (pid->error - pid->last_error) * pid->kd;
	
	LIMIT(pid->output,-pid->max_output,pid->max_output);
}
