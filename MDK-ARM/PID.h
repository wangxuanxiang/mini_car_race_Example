//PID函数头文件

#ifndef __PID_H
#define __PID_H

#include <stdio.h>
#include "assist_function.h"

//PID控制结构体
typedef struct{
    float kp,kp2,ki,kd,gkd;
    float error,last_error;
    float last_gyro;
    float integral,integral_max;
    float output;
}PID_Controller;

float pid_calculate(PID_Controller *controller,float setpoint,float measurement);
float ppdd_calculate(PID_Controller *controller,float setpoint,float measurement,float gyro_z);

#endif