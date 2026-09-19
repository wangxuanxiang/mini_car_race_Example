#include "multiplexer.h"
#include "bmi270_port.h"

/**
 * @brief 精确微秒延时函数 (基于 DWT)
 * @param period : 延时周期，单位微秒
 * @param intf_ptr : 接口指针 (未使用)
 * @return 无
 */
void multiplexer_delay_us(uint32_t period, void *intf_ptr)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = period * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

int16_t multiplexer_set_channel(int16_t channel){
    if(channel<0||channel>15) return -1; // 通道号错误，返回-1
    if((channel&1)==0){
        HAL_GPIO_WritePin(MULTIPLEXER_0_PORT, MULTIPLEXER_0_PIN, GPIO_PIN_RESET);
    }else{
        HAL_GPIO_WritePin(MULTIPLEXER_0_PORT, MULTIPLEXER_0_PIN, GPIO_PIN_SET);
    }
    if(((channel>>1)&1)==0){
        HAL_GPIO_WritePin(MULTIPLEXER_1_PORT, MULTIPLEXER_1_PIN, GPIO_PIN_RESET);
    }else{
        HAL_GPIO_WritePin(MULTIPLEXER_1_PORT, MULTIPLEXER_1_PIN, GPIO_PIN_SET);
    }
    if(((channel>>2)&1)==0){
        HAL_GPIO_WritePin(MULTIPLEXER_2_PORT, MULTIPLEXER_2_PIN, GPIO_PIN_RESET);
    }else{
        HAL_GPIO_WritePin(MULTIPLEXER_2_PORT, MULTIPLEXER_2_PIN, GPIO_PIN_SET);
    }
    if(((channel>>3)&1)==0){
        HAL_GPIO_WritePin(MULTIPLEXER_3_PORT, MULTIPLEXER_3_PIN, GPIO_PIN_RESET);
    }else{
        HAL_GPIO_WritePin(MULTIPLEXER_3_PORT, MULTIPLEXER_3_PIN, GPIO_PIN_SET);
    }
    return 0;
}

/**
 * @brief 读取多路复用器的值
 * @param[out] value 指向存储读取值的变量的指针
 */
void MUX_get_value(uint16_t *value){
    *value=0;
    for(int16_t i=0;i<MULTIPLEXER_CHANNEL_NUM;i++){
        multiplexer_set_channel(i);
        multiplexer_delay_us(1,NULL);
        if(HAL_GPIO_ReadPin(MULTIPLEXER_READ_PORT,MULTIPLEXER_READ_PIN)==GPIO_PIN_SET){
            (*value)+=(1<<(MULTIPLEXER_CHANNEL_NUM-i-1));
        }
    }
}