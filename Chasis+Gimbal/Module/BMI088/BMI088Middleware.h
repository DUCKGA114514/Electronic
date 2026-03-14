/* BMI088 中间件适配层：把驱动与具体硬件接口（GPIO/SPI/延时）解耦。 */
//
// Created by lisil on 2026/3/2.
//

#ifndef GIMBAL_BMI088MIDDLEWARE_H
#define GIMBAL_BMI088MIDDLEWARE_H

#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(unsigned short int ms);
extern void BMI088_delay_us(unsigned short int us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern unsigned char BMI088_read_write_byte(unsigned char reg);

#elif defined(BMI088_USE_IIC)

#endif

#endif


#endif //GIMBAL_BMI088MIDDLEWARE_H