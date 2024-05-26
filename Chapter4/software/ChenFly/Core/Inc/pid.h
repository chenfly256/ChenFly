#ifndef _PID_H_
#define _PID_H_

float update_pid(float err, float p, float i, float d, float* integral, float integral_max, float out_max, float* last_err);

#endif
