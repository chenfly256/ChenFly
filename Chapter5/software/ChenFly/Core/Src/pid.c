#include "pid.h"

float update_pid(float err, float p, float i, float d, float* integral, float integral_max, float out_max, float* last_err)
{
	float output;
	*integral = *integral+err;
	if(*integral > integral_max)
		*integral = integral_max;
	if(*integral < -integral_max)
		*integral = -integral_max;
	output = p * err + i * (*integral) + d * (err-*last_err);
	if(output > out_max)
		output = out_max;
	if(output < -out_max)
		output = -out_max;
	*last_err = err;
	return output;
}
