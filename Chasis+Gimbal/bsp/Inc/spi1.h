/* SPI1 + DMA 辅助接口：主要服务 BMI088 传感器高速读写。 */
//
// Created by lisil on 2026/3/2.
//

#ifndef GIMBAL_SPI1_H
#define GIMBAL_SPI1_H

void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);


#endif //GIMBAL_SPI1_H