#ifndef ASSIST_FUNCTION_H
#define ASSIST_FUNCTION_H

#include <stdio.h>
#include <stdint.h>

float my_abs(float x);

int16_t my_abs_int(int16_t x);

float constrain(float val,float min_val,float max_val);

#endif