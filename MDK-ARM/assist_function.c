#include "assist_function.h"

float my_abs(float x){
    return x>0.0f? x:-x;
}

int16_t my_abs_int(int16_t x){
    return x>0? x:-x;
}

float constrain(float val,float min_val,float max_val){
    if(val>max_val) return max_val;
    if(val<min_val) return min_val;
    return val;
}