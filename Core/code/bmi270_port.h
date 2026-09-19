#ifndef BMI270_PORT_H_
#define BMI270_PORT_H_

#include "stm32f1xx_hal.h"
#include "bmi270.h"

// 定义 CS 引脚
#define BMI270_CS_GPIO_Port     GPIOA
#define BMI270_CS_Pin           GPIO_PIN_4

/**
 * @brief 初始化 BMI270 传感器 (SPI)
 * @param[in,out] dev : bmi2_dev 结构体指针
 * @param[in] hspi : SPI 句柄指针
 * @return API 执行结果
 */
int8_t bmi270_sensor_init_spi(struct bmi2_dev *dev, SPI_HandleTypeDef *hspi);

/**
 * @brief 设置传感器配置
 * @param[in] dev : bmi2_dev 结构体指针
 * @return API 执行结果
 */
int8_t bmi270_set_config(struct bmi2_dev *dev);

/**
 * @brief 从 BMI270 读取加速度和角速度数据
 * @param[in] dev : bmi2_dev 结构体指针
 * @param[out] acc : 加速度数据结构体指针
 * @param[out] gyr : 角速度数据结构体指针
 * @return API 执行结果
 */
int8_t bmi270_get_accel_gyro(struct bmi2_dev *dev, struct bmi2_sens_data *acc, struct bmi2_sens_data *gyr);

void BMI270_SPI_DMA_TransferComplete_Callback(void);

#endif /* BMI270_PORT_H_ */