/**
 * @file    dodo_BMI270.c
 * @author  pupydodo (github.com/pupydodo)
 * 使用方法：
 * 1.包含头文件 "dodo_BMI270.h"
 * 2.调用 dodo_BMI270_init() 初始化 BMI270，返回0表示成功
 * 3.调用 dodo_BMI270_get_data() 获取最新的传感器数据，传感器数据会更新到六个全局变量
 *   BMI270_gyro_x,BMI270_gyro_y,BMI270_gyro_z和BMI270_accel_x,BMI270_accel_y,BMI270_accel_z中
 */
#include "dodo_BMI270.h"

int16_t BMI270_gyro_x=0, BMI270_gyro_y=0, BMI270_gyro_z=0;
int16_t BMI270_accel_x=0, BMI270_accel_y=0, BMI270_accel_z=0;
float BMI270_transition_factor[2] = {4096, 16.4};
struct bmi2_dev bmi270;
struct bmi2_sens_data accel_data;
struct bmi2_sens_data gyro_data;
extern SPI_HandleTypeDef hspi1;
int8_t BMI270_init_flag = 0;

/**
 * @brief 初始化 BMI270 传感器
 * @param[in] hspi : SPI 句柄指针
 * @return 0 表示成功，其他值表示失败
 */
int8_t dodo_BMI270_init(){
    if(BMI270_init_flag) return 0; // 已初始化，直接返回成功
    int8_t rslt;
    rslt = bmi270_sensor_init_spi(&bmi270, &hspi1);
    if (rslt != BMI2_OK) {
        return rslt;
    }
    rslt = bmi270_set_config(&bmi270);
    if (rslt != BMI2_OK) {
        return 2;
    } else {
        BMI270_init_flag = 1;
        return 0;
    }
}

/**
 * @brief 获取传感器数据
 */
void dodo_BMI270_get_data(void){
    if(!BMI270_init_flag){
        BMI270_gyro_x = 0;
        BMI270_gyro_y = 0;
        BMI270_gyro_z = 0;
        return; // 未初始化，直接返回
    } // 未初始化，直接返回0

    int8_t rslt = bmi270_get_accel_gyro(&bmi270, &accel_data, &gyro_data);
    if (rslt == BMI2_OK) {
        BMI270_gyro_x = gyro_data.gyr.x;
        BMI270_gyro_y = gyro_data.gyr.y;
        BMI270_gyro_z = gyro_data.gyr.z;
        BMI270_accel_x = accel_data.acc.x;
        BMI270_accel_y = accel_data.acc.y;
        BMI270_accel_z = accel_data.acc.z;
    } else {
        BMI270_gyro_x = 0;
        BMI270_gyro_y = 0;
        BMI270_gyro_z = 0;
        BMI270_accel_x = 0;
        BMI270_accel_y = 0;
        BMI270_accel_z = 0;
    }
    return;
}
