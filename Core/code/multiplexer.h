#ifndef _MULTIPLEXER_H_
#define _MULTIPLEXER_H_
#include "stm32f1xx_hal.h"
#include "main.h"

#define MULTIPLEXER_READ_PORT  MUX_READ_GPIO_Port
#define MULTIPLEXER_READ_PIN   MUX_READ_Pin
#define MULTIPLEXER_0_PORT     MUX_0_GPIO_Port
#define MULTIPLEXER_0_PIN      MUX_0_Pin
#define MULTIPLEXER_1_PORT     MUX_1_GPIO_Port
#define MULTIPLEXER_1_PIN      MUX_1_Pin
#define MULTIPLEXER_2_PORT     MUX_2_GPIO_Port
#define MULTIPLEXER_2_PIN      MUX_2_Pin
#define MULTIPLEXER_3_PORT     MUX_3_GPIO_Port
#define MULTIPLEXER_3_PIN      MUX_3_Pin
#define MULTIPLEXER_CHANNEL_NUM 12

void MUX_get_value(uint16_t *value);

/** 获取指定通道的值，mux_channel范围：0到(MULTIPLEXER_CHANNEL_NUM-1)，默认范围为0到11
 * @return 0或1
 */
#define MUX_GET_CHANNEL(value,mux_channel) ((value>>(MULTIPLEXER_CHANNEL_NUM-mux_channel-1))&1)

#endif