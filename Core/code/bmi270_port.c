#include "bmi270_port.h"

static SPI_HandleTypeDef *bmi2_hspi = NULL;

// CS 引脚控制宏
#define BMI270_CS_LOW()     HAL_GPIO_WritePin(BMI270_CS_GPIO_Port, BMI270_CS_Pin, GPIO_PIN_RESET)
#define BMI270_CS_HIGH()    HAL_GPIO_WritePin(BMI270_CS_GPIO_Port, BMI270_CS_Pin, GPIO_PIN_SET)

static volatile uint8_t spi_dma_tx_rx_complete = 0;

/**
 * @brief SPI 读函数 (DMA)
 */
BMI2_INTF_RETURN_TYPE bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if (bmi2_hspi == NULL) {
        return BMI2_E_NULL_PTR;
    }

    reg_addr |= 0x80;
    
    static uint8_t tx_buf[2048];
    static uint8_t rx_buf[2048];
    if ((len + 1) > sizeof(tx_buf)) {
        return BMI2_E_COM_FAIL;
    }

    tx_buf[0] = reg_addr;

    spi_dma_tx_rx_complete = 0;

    BMI270_CS_LOW();

    if (HAL_SPI_TransmitReceive_DMA(bmi2_hspi, tx_buf, rx_buf, len + 1) != HAL_OK) {
        BMI270_CS_HIGH();
        return BMI2_E_COM_FAIL;
    }

    while (spi_dma_tx_rx_complete == 0) {}

    BMI270_CS_HIGH();
    
    for (uint32_t i = 0; i < len; i++) {
        reg_data[i] = rx_buf[i + 1];
    }

    return BMI2_OK;
}

/**
 * @brief SPI 写函数 (DMA)
 */
BMI2_INTF_RETURN_TYPE bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if (bmi2_hspi == NULL) {
        return BMI2_E_NULL_PTR;
    }

    reg_addr &= 0x7F;
    
    static uint8_t tx_buf[4096];
    if ((len + 1) > sizeof(tx_buf)) {
        return BMI2_E_COM_FAIL;
    }
    
    tx_buf[0] = reg_addr;
    for(uint32_t i=0; i<len; i++) {
        tx_buf[i+1] = reg_data[i];
    }
    
    spi_dma_tx_rx_complete = 0;
    
    BMI270_CS_LOW();

    if (HAL_SPI_Transmit_DMA(bmi2_hspi, tx_buf, len + 1) != HAL_OK) {
        BMI270_CS_HIGH();
        return BMI2_E_COM_FAIL;
    }else{
    }

    while (spi_dma_tx_rx_complete == 0) {}

    BMI270_CS_HIGH();

    return BMI2_OK;
}


/**
 * @brief SPI DMA 传输完成的回调函数
 * @note  这个函数是 HAL 库提供的标准回调，当任何 SPI 的 DMA 传输完成时，
 *        并且 SPI 中断被使能，它就会被自动调用。
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // 确保是SPI1完成了传输
    if (hspi->Instance == SPI1)
    {
        // 置位完成标志
        spi_dma_tx_rx_complete = 1;
    }
}

// 如果只调用了 Transmit_DMA，则会触发这个回调
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        spi_dma_tx_rx_complete = 1;
    }
}

/**
 * @brief 精确微秒延时函数 (基于 DWT)
 * @param period : 延时周期，单位微秒
 * @param intf_ptr : 接口指针 (未使用)
 * @return 无
 */
void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = period * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

/**
 * @brief 初始化 DWT (用于精确延时)
 */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 使能 DWT
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 使能周期计数器
    DWT->CYCCNT = 0; // 计数器清零
}

/**
 * @brief 初始化 BMI270 传感器 (SPI)
 */
int8_t bmi270_sensor_init_spi(struct bmi2_dev *dev, SPI_HandleTypeDef *hspi)
{
    
    if (hspi == NULL || dev == NULL) {
        return BMI2_E_NULL_PTR;
    }
    
    dwt_init(); // 初始化 DWT
    
    bmi2_hspi = hspi;
    
    dev->intf = BMI2_SPI_INTF;
    dev->read = bmi2_spi_read;
    dev->write = bmi2_spi_write;
    dev->delay_us = bmi2_delay_us;
    dev->intf_ptr = NULL; 
    dev->read_write_len = 2048; 
    dev->config_file_ptr = NULL; 
    
    // 根据BMI270数据手册，SPI接口在初始化时需要一个dummy read来进入SPI模式
    uint8_t dummy;
    bmi2_spi_read(BMI2_CHIP_ID_ADDR, &dummy, 1, NULL);
    bmi2_delay_us(1000, NULL); // 延时 1ms
    
    // 初始化 BMI270
    int8_t rslt = bmi270_init(dev);
    // printf("rslt:%d\r\n",rslt);
    return rslt;
}

/**
 * @brief 配置加速度计和陀螺仪的参数
 */
int8_t bmi270_set_config(struct bmi2_dev *dev)
{
    int8_t rslt;
    struct bmi2_sens_config config;

    // 1. 配置加速度计
    config.type = BMI2_ACCEL;
    rslt = bmi2_get_sensor_config(&config, 1, dev);
    if (rslt != BMI2_OK) return rslt;

    config.cfg.acc.odr = BMI2_ACC_ODR_1600HZ;       // 输出数据速率
    config.cfg.acc.range = BMI2_ACC_RANGE_8G;     // 量程 G (2G, 4G, 8G, 16G)
    config.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;    // 滤波器
    config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE; // 性能模式

    rslt = bmi2_set_sensor_config(&config, 1, dev);
    if (rslt != BMI2_OK) return rslt;

    // 2. 配置陀螺仪
    config.type = BMI2_GYRO;
    rslt = bmi2_get_sensor_config(&config, 1, dev);
    if (rslt != BMI2_OK) return rslt;

    config.cfg.gyr.odr = BMI2_GYR_ODR_1600HZ;       // 输出数据速率
    config.cfg.gyr.range = BMI2_GYR_RANGE_2000; // 量程 dps (125, 250, 500, 1000, 2000)
    config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;    // 带宽参数
    config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE; // 性能模式
    config.cfg.gyr.noise_perf = BMI2_GYR_NOISE_PERF_MODE_MASK; // 噪声性能

    rslt = bmi2_set_sensor_config(&config, 1, dev);
    if (rslt != BMI2_OK) return rslt;
    
    // 3. 使能加速度计和陀螺仪
    uint8_t sens_list[] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sens_list, 2, dev);
    
    return rslt;
}

/**
 * @brief 获取加速度计和陀螺仪的传感器数据
 * @details 该函数一次性获取所有传感器数据。
 * @param[in] dev : 已初始化的 bmi2_dev 结构体指针
 * @param[out] acc : 用于存储加速度数据的结构体指针
 * @param[out] gyr : 用于存储陀螺仪数据的结构体指针
 * @return API 执行结果，BMI2_OK 表示成功
 */
int8_t bmi270_get_accel_gyro(struct bmi2_dev *dev, struct bmi2_sens_data *acc, struct bmi2_sens_data *gyr)
{
    struct bmi2_sens_data sensor_data;
    int8_t rslt = bmi2_get_sensor_data(&sensor_data, dev);
    if (rslt == BMI2_OK)
    {
        *acc = sensor_data;
        *gyr = sensor_data;
    }
    return rslt;
}