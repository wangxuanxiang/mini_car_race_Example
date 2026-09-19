#ifndef _DODO_BMI270_H_
#define _DODO_BMI270_H_

#include "bmi270_port.h"

extern int16_t BMI270_gyro_x, BMI270_gyro_y, BMI270_gyro_z;
extern int16_t BMI270_accel_x, BMI270_accel_y, BMI270_accel_z;
extern float BMI270_transition_factor[2];

int8_t dodo_BMI270_init();
void dodo_BMI270_get_data(void);
void dodo_BMI270_get_accel(void);

#define BMI270_acc_transition(acc_value)      ((float)acc_value / BMI270_transition_factor[0])
#define BMI270_gyro_transition(gyro_value)    ((float)gyro_value / BMI270_transition_factor[1])

#endif