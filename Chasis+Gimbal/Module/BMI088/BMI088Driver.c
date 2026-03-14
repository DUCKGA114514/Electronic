//
// Created by lisil on 2026/3/2.
//

#include "main.h"
#include "BMI088Register.h"
#include "BMI088Driver.h"
#include "BMI088Middleware.h"
/**
 * 陀螺仪和加速度计传感器数据的宽度为 16 位温度传感器为 11 位）
 * 以二进制补码表示形式给出
 * 每个轴的位被分为 MSB 上部和 LSB 下部
 * 读取传感器数据寄存器应始终从 LSB 部分开始
 * 为了确保传感器数据的完整性
 * 通过读取相应的LSB寄存器来锁定MSB寄存器的内容（阴影过程）
 * 当进行读操作时，加速度计部分的SPI接口在主机发送相应的寄存器地址后并不直接发送请求的信息
 * 而是先发送一个虚拟字节，其内容是不可预测的
 * 仅在该虚拟字节之后，才会发送所需的内容。 （此虚拟字节过程不适用于陀螺仪部分。）
 *
 */


/**
 * 引脚初始化
 * BMI088 的陀螺仪部分根据 PS 引脚给出的选择初始化其 I/O 引脚
 * 加速计部分以 I2C 模式启动。它将保持在 I2C 模式，
 * 直到检测到 CSB1 引脚（加速度计的片选）上的上升沿
 * 此时加速度计部分切换到 SPI 模式并保持在此模式，直到下一次上电复位
 *
 * POR 后，陀螺仪处于正常模式，而加速度计处于挂起模式
 * 要将加速度计切换到正常模式，用户必须执行以下步骤：
 * a.给传感器通电
 * b.等待 1 毫秒
 * c.将“4”写入 ACC_PWR_CTRL 进入正常模式
 * d.等待 50 毫秒
 *
 * BMI088加速度计的电源状态通过寄存器ACC_PWR_CTRL控制
 * 要进入正常模式，必须将值 0x04 写入 ACC_PWR_CTRL。
 *
 * 注意：传感器在复位（POR 或软复位）后处于挂起模式，因此用户需要主动进入正常模式才能获取加速度值
 * 注意：POR 或软复位后，加速度传感器需要长达 1ms 的启动时间。
 * 更改电源模式时，传感器需要长达 5 毫秒的时间才能稳定。在此期间应避免与传感器进行任何通信。
 *
 * 加电后，陀螺仪处于正常模式，以便设备的所有部件保持加电状态并连续执行数据采集。
 * 尽管在全接口时钟速度（SCL 或 SCK）下支持对寄存器的写访问，但必须在两个连续的写周期之间插入等待期
 *
 * 注意：POR 或软复位后，或在不同电源模式之间切换时，陀螺仪传感器需要最多 30ms 的时间才能达到新状态。
 * 在此期间应避免与传感器进行任何通信。
 *
 * 加速度计
 *
 * 来自加速度传感器和陀螺仪模拟前端的传感器信号均经过低通滤波器。
 *
 * 来自加速度传感器和陀螺仪模拟前端的传感器信号均经过低通滤波器。
 *
 * 激活自检会导致加速度数据出现静态偏移
 * 自检期间施加到传感器的任何外部加速度或重力都将在传感器输出中观察到，作为加速度和自检信号的叠加
 * 这意味着自检信号取决于传感器的方向
 * 为了克服这个问题，完整的自检程序应该在静态环境下执行，例如在静态环境下
 * 当零件不受重力以外的任何加速度激发时
 *
 * 推荐的自检程序如下:
 * a.通过向寄存器 ACC_RANGE (0x41) 写入 0x03 设置 ±24g 范围
 * b.通过写入 0xA7 来设置 ODR=1.6kHz、连续采样模式、“正常模式”(norm_avg4)
 * 寄存器 ACC_CONF (0x40) ? 连续滤波器功能：设置 ACC_CONF 中的 bit7 ? “正常 avg4 模式”：ACC_CONF |= 0x02<<4 ? ODR=1.6kHz：ACC_CONF |= 0x0C
 * c.等待 > 2 毫秒
 * d.使能正自检极性（即将0x0D写入寄存器ACC_SELF_TEST（0x6D））
 * e.等待 > 50 毫秒
 * f.读取每个轴的加速度计偏移值（正自检响应）
 * g.使能负自检极性（即向寄存器ACC_SELF_TEST（0x6D）写入0x09）
 * h.等待 > 50 毫秒
 * i.读取每个轴的加速度计偏移值（负自检响应）
 * j.禁用自检（即将 0x00 写入寄存器 ACC_SELF_TEST (0x6D)）
 * k.计算正负自测响应的差异并与预期值进行比较（x轴信号≥1000 mg y轴信号≥1000 mg z轴信号≥500 mg）
 * l.等待 > 50ms 让传感器稳定到正常模式稳态运行
 *
 * 建议在执行自检后对器件进行复位，因为自检响应也会影响中断的生成。
 * 如果无法执行复位，则必须保持以下顺序以防止产生不需要的中断：
 * 禁用中断，更改中断参数，等待至少 50ms，然后启用所需的中断。
 *
 * 陀螺仪
 *
 * a.要触发自测试，必须设置地址 GYRO_SELF_TEST 中的位 #0 (‘bite_trig’)。
 * b.测试完成后，陀螺仪将设置位#1（‘bist_rdy’），然后可以在位＃2（‘bist_fail’）中找到测试结果。
 * c.“0”表示测试已顺利通过。如果发生故障，‘bist_fail’位将被设置为‘1’。
 * d.可以通过读取地址 GYRO_SELF_TEST 中的位 #4 来检查在后台连续运行的进一步测试。
 * e.如果该位设置为“1”，则表明传感器功能正常。
 *
 * 加速度计和陀螺仪部分都提供新的数据就绪中断，只要新的数据样本集完成并在相应的传感器数据寄存器中可用，就会触发该中断。
 *
 * 加速度计
 *
 * 新的数据中断标志可以在寄存器 ACC_INT_STAT_1（位#7）中找到。只要数据寄存器中有新数据可用，就会设置它并自动清除。
 * 中断可以映射到寄存器 INT1_INT2_MAP_DATA 中的中断引脚 INT1 和/或 INT2。
 *
 * 陀螺仪
 * 陀螺仪提供了新的数据中断，每次在数据寄存器中存储新的z轴角速率数据值后都会产生中断。 280-400 μs 后中断自动清除
 * 与加速度计部分相反，对于陀螺仪，必须通过将 0x80 写入寄存器 GYRO_INT_CTRL 来显式启用新数据中断。
 * 中断可以映射到寄存器 INT3_INT4_IO_MAP 中的中断引脚 INT3 和/或 INT4。
 *
 * 软复位
 *
 * 可以随时启动软复位
 * 对于加速度计部分，通过将命令软复位 (0xB6) 写入寄存器 ACC_SOFTRESET
 * 对于陀螺仪部分，通过将命令软复位 (0xB6) 写入寄存器 GYRO_SOFTRESET
 * 软复位对设备执行基本复位，这在很大程度上相当于电源周期。延迟后，所有用户配置设置都将被覆盖为其默认状态（如果适用）。
 *
 * FIFO
 *
 * BMI088 为加速计和陀螺仪传感器信号提供两个集成 FIFO（先进先出）缓冲区
 *
 * FIFO 可以在不同的模式下运行：FIFO（或满时停止）模式和 STREAM 模式。
 *
 * FIFO 或满停模式：在 FIFO 或满停模式下，传感器值随后存储在 FIFO 缓冲区中，直到满为止
 * STREAM 模式：STREAM 模式的工作方式类似于 FIFO 模式，不同之处在于一旦缓冲区已满，FIFO 中最旧的数据将被来自传感器的最新数据覆盖。
 *
 * 通过设置寄存器 0x??49 中的位 #6 来启用加速度计传感器数据的 FIFO
 *
 * 模式选择 当需要 STREAM 模式时，必须清除寄存器 0x??48 中的位 #0（设置为“0”）
 * 对于 FIFO 或满停模式，寄存器 0x??48 中的位 #0 必须设置为“1”。
 *
 * FIFO 数据采样率 FIFO 的输入数据率与传感器配置的ODR 相同
 * 然而，可以通过选择下采样因子 2 且 k=[0, 1, … 7] 来减少它
 * 系数 k 必须写入寄存器 0x??45 的位#4-6
 *
 * 加速器的 FIFO 与外部中断（标签应用）同步 如果 INT1 和/或 INT2 引脚配置为输入引脚
 * （通过在寄存器 INT2_IO_CTRL 中设置 int2_io 和/或在寄存器 INT1_IO_CTRL 中设置 int1_io），
 * 这些引脚上的信号引脚也可以记录在 FIFO 中，并且帧会被相应地“标记”。
 * 因此，需要激活引脚以在寄存器 FIFO_CONFIG_1 中进行 FIFO 记录
 *
 * FIFO中的数据格式
 *
 * FIFO 以帧的形式捕获数据。
 * 第一个字节是标头字节，定义帧的类型。
 * 由此可以得出连续字节的数量及其内容。
 * 标头字节由标头签名（前 6 位）和指示中断引脚 INT1 和 INT2 状态（如果进行了相应配置）的两位组成
 *
 * 加速度传感器数据帧
 * 帧长度：7 字节（1 字节标头 + 6 字节有效负载）
 *
 * FIFO复位
 *
 * 用户可以通过将 0xB0 写入 ACC_SOFTRESET（寄存器 0x??7E）来触发 FIFO 复位。
 *
 * 启用 FIFO 并选择模式
 * 通过在寄存器 0x??3E：FIFO_CONFIG_1 中设置适当的 FIFO 模式来启用陀螺仪传感器数据的 FIFO。
 *
 * 任何陀螺仪中断引脚（INT3 或 INT4）都可以重新配置为输入引脚，但不能同时用作输入引脚。
 * 此外，必须启用标签模式。这样配置的中断引脚将充当输入引脚，而不是中断引脚
 *
 * 为了启用标签模式，必须在寄存器 0x??34 中设置位 5。可以在同一寄存器的位 4 中选择引脚
 *
 * 在此模式下，z 轴的最低有效位用作标记位，因此失去其作为陀螺仪数据位的意义
 * z 轴陀螺仪数据的其余 15 位与标准模式下的含义相同
 *
 * 一旦配置为标签模式的引脚设置为高电平，下一个 FIFO 字将被标记为标签（z 轴 LSB = 1）
 * 当引脚保持高电平时，相应的 FIFO 字将持续被标记。
 * 引脚复位为低电平后，紧邻的下一个 FIFO 字仍然可以被标记，并且只有在该字之后，下一个标记才会被复位（z 轴 LSB=0）。
 *
 * FIFO 存储的数据也可在读出寄存器 0x??02-0x07 处获取
 * 可以通过寄存器 0x??3F (FIFO_DATA) 进行 FIFO 读出
 *
 * 与设备的整个通信是通过读取和写入寄存器来执行的。
 * 寄存器的宽度为8位；它们被映射到8位地址空间。
 * 加速度计和陀螺仪有单独的寄存器图。
 * 通过选择相应的片选引脚（SPI 模式）或 I2C 地址（I2C 模式），可以在数字接口级别选择适当的寄存器映射。
 *
 * 建议屏蔽掉部分包含功能位的寄存器的（逻辑和零）非功能位（用“-”标记）
 * （即先读取寄存器内容，通过按位操作更改位，然后写入修改后的位）字节回寄存器）。
 *
 * 包含加速度传感器输出的寄存器。
 * 传感器输出以 2 的补码格式作为有符号 16 位数字存储在每 2 个寄存器中。
 * 从寄存器中，加速度值可以计算如下：
 * Accel_X_int16 = ACC_X_MSB * 256 + ACC_X_LSB
 * Accel_Y_int16 = ACC_Y_MSB * 256 + ACC_Y_LSB
 * Accel_Z_int16 = ACC_Z_MSB * 256 + ACC_Z_LSB
 *
 * 当读取包含加速度值的 LSB 值的寄存器时，相应的 MSB 寄存器将在内部锁定，直到它被读取
 * 通过这种机制，可以确保LSB和MSB值都属于相同的加速度值，并且在各个寄存器的读出之间不会更新。
 *
 * 单位为 LSB。从 LSB 到加速度（mg）的转换基于范围设置，计算公式如下（A：ACC_RANGE 寄存器的内容）：
 * Accel_X_in_mg = Accel_X_int16 / 32768 * 1000 * 2^(<0x41> + 1) * 1.5
 * Accel_Y_in_mg = Accel_Y_int16 / 32768 * 1000 * 2^(<0x41> + 1) * 1.5
 * Accel_Z_in_mg = Accel_Z_int16 / 32768 * 1000 * 2^(<0x41> + 1) * 1.5
 *
 * 包含温度传感器数据输出的寄存器。数据以 2 的补码格式存储在 11 位值中。分辨率为0.125°C/LSB，因此温度可由下式获得:
 * Temp_uint11 = (TEMP_MSB * 8) + (TEMP_LSB / 32)
 * if Temp_uint11 > 1023:
 * Temp_int11 = Temp_uint11 – 2048
 * else:
 * Temp_int11 = Temp_uint11
 * Temperature = Temp_int11 * 0,125°C/LSB + 23°C
 *
 * 温度传感器数据每 1.28 秒更新一次。
 *
 * 包含角速度传感器输出的寄存器。传感器输出以 2 的补码格式作为有符号 16 位数字存储在每 2 个寄存器中。从寄存器中，陀螺仪值可以计算如下：
 * Rate_X: RATE_X_MSB * 256 + RATE_X_LSB
 * Rate_Y: RATE_Y_MSB * 256 + RATE_Y_LSB
 * Rate_Z: RATE_Z_MSB * 256 + RATE_Z_LSB
 *
 * 当读取包含速率值的 LSB 值的寄存器时，相应的 MSB 寄存器将在内部锁定，直到它被读取。
 * 通过这种机制，可以确保LSB和MSB值都属于相同的速率范围值，并且在各个寄存器的读出之间不会更新。
 *
 * 请注意，在 SPI 协议的情况下，BMI088 的加速计部分的初始化过程需要一些额外的步骤
 * 另请注意，由于加速度计和陀螺仪部分共享封装引脚，因此不建议为这两个部分配置不同的接口。
 *
 *
为了实现写入 BMI088 的数据的正确内部同步，必须遵循至少 2 μs（正常模式）或 1000 μs（挂起模式）的等待时间。
 *
 * 陀螺仪部分和加速度计部分的 SPI 接口行为略有不同：
 * 初始化阶段：陀螺仪部分的接口由PS引脚的电平选择。
 * 与此相反，加速计部分始终以 I2C 模式启动（无论 PS 引脚的电平如何），
 * 需要通过在 CSB1 引脚（加速计的片选）上发送上升沿主动更改为 SPI 模式，
 * 加速度计部分切换到 SPI 模式并保持在该模式，直到下一次上电复位。
 *
 * 要在初始化阶段将传感器更改为 SPI 模式，用户可以执行虚拟 SPI 读取操作，例如寄存器ACC_CHIP_ID（获取的值将无效）。
 *
 * 在进行读操作时，加速度计部分的 SPI 接口在主机发送相应的寄存器地址后并不直接发送请求的信息，
 * 而是先发送一个虚拟字节，其内容是不可预测的。
 * 仅在该虚拟字节之后，才会发送所需的内容。 （此虚拟字节过程不适用于陀螺仪部分。）
 *
 * 在加速度计部分的读取操作的情况下，所请求的数据不会立即发送，而是首先发送虚拟字节，
 * 并??且在该虚拟字节之后传输实际请求的寄存器内容。
 * 这意味着，单字节读取操作需要以突发模式读取 2 个字节，其中第一个接收到的字节可以被丢弃，而第二个字节包含所需的数据。
 * 这同样适用于突发读取操作。
 * 例如，要在 SPI 模式下读取加速度计值，用户必须读取从地址 0x12（ACC 数据）开始的 7 个字节。
 * 用户必须从这些字节中丢弃第一个字节，并在字节 #2 – #7 中找到加速度信息（对应于地址 0x12 – 0x17 的内容）。
 *
 */
float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

#if defined(BMI088_USE_SPI)

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
//#define BMI088_accel_write_muli_reg( reg,  data, len) { BMI088_ACCEL_NS_L(); BMI088_write_muli_reg(reg, data, len); BMI088_ACCEL_NS_H(); }
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }

#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
//#define BMI088_gyro_write_muli_reg( reg,  data, len) { BMI088_GYRO_NS_L(); BMI088_write_muli_reg( ( reg ), ( data ), ( len ) ); BMI088_GYRO_NS_H(); }
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)


#endif

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;
    // GPIO and SPI  Init .
    BMI088_GPIO_init();
    BMI088_com_init();

    // self test pass and init
    if (bmi088_accel_self_test() != BMI088_NO_ERROR)
    {
        error |= BMI088_SELF_TEST_ACCEL_ERROR;
    }
    else
    {
        error |= bmi088_accel_init();
    }

    if (bmi088_gyro_self_test() != BMI088_NO_ERROR)
    {
        error |= BMI088_SELF_TEST_GYRO_ERROR;
    }
    else
    {
        error |= bmi088_gyro_init();
    }

    return error;
}

unsigned char bmi088_accel_init(void)
{
    volatile uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

unsigned char bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

unsigned char bmi088_accel_self_test(void)
{

    int16_t self_test_accel[2][3];

    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    volatile uint8_t res = 0;

    uint8_t write_reg_num = 0;

    static const uint8_t write_BMI088_ACCEL_self_test_Reg_Data_Error[6][3] =
        {
            {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
            {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
            {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_ERROR},
            {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR}

        };

    //check commiunication is normal
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // reset  bmi088 accel sensor and wait for > 50ms
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // set the accel register
    for (write_reg_num = 0; write_reg_num < 4; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1])
        {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    }

    // self test include postive and negative
    for (write_reg_num = 0; write_reg_num < 2; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1])
        {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

        // read response accel
        BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

        self_test_accel[write_reg_num][0] = (int16_t)((buf[1]) << 8) | buf[0];
        self_test_accel[write_reg_num][1] = (int16_t)((buf[3]) << 8) | buf[2];
        self_test_accel[write_reg_num][2] = (int16_t)((buf[5]) << 8) | buf[4];
    }

    //set self test off
    BMI088_accel_write_single_reg(BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_OFF);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_SELF_TEST, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != (BMI088_ACC_SELF_TEST_OFF))
    {
        return BMI088_ACC_SELF_TEST_ERROR;
    }

    //reset the accel sensor
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    if ((self_test_accel[0][0] - self_test_accel[1][0] < 1365) || (self_test_accel[0][1] - self_test_accel[1][1] < 1365) || (self_test_accel[0][2] - self_test_accel[1][2] < 680))
    {
        return BMI088_SELF_TEST_ACCEL_ERROR;
    }

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    return BMI088_NO_ERROR;
}
unsigned char bmi088_gyro_self_test(void)
{
    uint8_t res = 0;
    uint8_t retry = 0;
    //check commiunication is normal
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_gyro_write_single_reg(BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    do
    {

        BMI088_gyro_read_single_reg(BMI088_GYRO_SELF_TEST, res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        retry++;
    } while (!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

    if (retry == 10)
    {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    if (res & BMI088_GYRO_BIST_FAIL)
    {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    return BMI088_NO_ERROR;
}

void BMI088_read_gyro_who_am_i(void)
{
    uint8_t buf;
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, buf);
}


// void BMI088_read_accel_who_am_i(void)
// {
//     volatile uint8_t buf;
//     BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, buf);
//     buf = 0;
//
// }





void BMI088_temperature_read_over(uint8_t *rx_buf, float *temperate)
{
    int16_t bmi088_raw_temp;
    bmi088_raw_temp = (int16_t)((rx_buf[0] << 3) | (rx_buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }
    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

}

void BMI088_accel_read_over(uint8_t *rx_buf, float accel[3], float *time)
{
    int16_t bmi088_raw_temp;
    uint32_t sensor_time;
    bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    sensor_time = (uint32_t)((rx_buf[8] << 16) | (rx_buf[7] << 8) | rx_buf[6]);
    *time = sensor_time * 39.0625f;

}

void BMI088_gyro_read_over(uint8_t *rx_buf, float gyro[3])
{
    int16_t bmi088_raw_temp;
    bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
    gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
    gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
    gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
}

void BMI088_read(float gyro[3], float accel[3], float *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

uint32_t get_BMI088_sensor_time(void)
{
    uint32_t sensor_time = 0;
    uint8_t buf[3];
    BMI088_accel_read_muli_reg(BMI088_SENSORTIME_DATA_L, buf, 3);

    sensor_time = (uint32_t)((buf[2] << 16) | (buf[1] << 8) | (buf[0]));

    return sensor_time;
}

float get_BMI088_temperate(void)
{
    uint8_t buf[2];
    float temperate;
    int16_t temperate_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    temperate_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (temperate_raw_temp > 1023)
    {
        temperate_raw_temp -= 2048;
    }

    temperate = temperate_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    return temperate;
}

void get_BMI088_gyro(int16_t gyro[3])
{
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    int16_t gyro_raw_temp;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_X_L, buf, 6);

    gyro_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    gyro[0] = gyro_raw_temp ;
    gyro_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    gyro[1] = gyro_raw_temp ;
    gyro_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    gyro[2] = gyro_raw_temp ;
}

void get_BMI088_accel(float accel[3])
{
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    int16_t accel_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    accel_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = accel_raw_temp * BMI088_ACCEL_SEN;
    accel_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = accel_raw_temp * BMI088_ACCEL_SEN;
    accel_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = accel_raw_temp * BMI088_ACCEL_SEN;
}

#if defined(BMI088_USE_SPI)

static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//{
//    BMI088_read_write_byte( reg );
//    while( len != 0 )
//    {

//        BMI088_read_write_byte( *buf );
//        buf ++;
//        len --;
//    }

//}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}

#elif defined(BMI088_USE_IIC)

#endif
