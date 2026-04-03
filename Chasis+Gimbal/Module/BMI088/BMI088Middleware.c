/* BMI088 硬件适配实现：封装片选、SPI 单字节交换以及毫秒/微秒延时。 */
//
// Created by lisil on 2026/3/2.
//

#include "main.h"
#include "BMI088Register.h"
#include "BMI088Middleware.h"
#include "BMI088Driver.h"
#include "cmsis_os2.h"
#include "spi.h"

void BMI088_GPIO_init(void)
{

}

void BMI088_com_init(void)
{


}

void BMI088_delay_ms(uint16_t ms)
{
    osDelay(ms);
}

void BMI088_delay_us(uint16_t us)
{
    Delay_us(us);
}

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
