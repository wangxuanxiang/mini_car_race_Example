//PID函数逻辑实现

#include "PID.h"

/**
 * @brief PID控制器计算
 * @param controller PID控制器指针
 * @param setpoint 目标值
 * @param measurement 测量值
 * @return 控制输出
 */
float pid_calculate(PID_Controller *controller,float setpoint,float measurement){
    float error=setpoint-measurement;
    controller->integral+=error;
    float derivative=error-controller->last_error;
	
	  if(controller->integral>200) controller->integral=200;
		if(controller->integral<-200) controller->integral=-200;
	
    controller->output=controller->kp*error+controller->ki*controller->integral+controller->kd*derivative;

    controller->last_error=error;
    controller->error=error;

    return controller->output;
}

/**
 * @brief PPDD控制器计算(转向环)
 * @param controller PID控制器指针
 * @param setpoint 目标值
 * @param measurement 测量值
 * @return 控制输出
 */
float ppdd_calculate(PID_Controller *controller,float setpoint,float measurement,float gyro_z){
    float error=setpoint-measurement;
    float pos_derivative=error-controller->last_error;
    float gyro_derivative=gyro_z;

    float output=error*controller->kp+my_abs(error)*error*controller->kp2+pos_derivative*controller->kd+gyro_derivative*controller->gkd;

    controller->last_error=error;
    controller->last_gyro=gyro_z;
    controller->error=error;

    return output;
}