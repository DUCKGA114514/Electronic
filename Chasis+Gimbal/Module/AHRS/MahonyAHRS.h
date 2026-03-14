/* Mahony AHRS 姿态融合算法接口与使用说明。 */
//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

/*==================================================使用说明===================================================*/

/*
 *Step 1：初始化
 *q[0]=1.0f; q[1]=q[2]=q[3]=0.0f;  // 单位四元数
  integralFBx = integralFBy = integralFBz = 0.0f;
  sampleFreq = 100.0f;             // 例如 100Hz
  twoKp = 2.0f * 0.5f;             // 先从 Kp=0.5 试
  twoKi = 2.0f * 0.0f;             // 先关闭积分，稳定后再开
  常见做法：先 Ki=0 跑通，再慢慢加一点 Ki 抑制漂移。

Step 2：固定频率调用（最重要）

必须以接近恒定的周期调用，例如 100Hz：
// 每 10ms 调一次    ---------------->这里就可以用到我们伟大的RTOS任务调度系统了
MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);
sampleFreq 必须与你的调用频率一致
如果你是用 dt 变化的系统，这份代码是“固定频率版”；要么让调用周期固定，要么把里面的 1.0f/sampleFreq 改成实时 dt

Step 3：单位与坐标系要一致
陀螺仪：强烈建议用 rad/s
如果传感器给的是 deg/s：
rad = deg * (π/180)
加速度计：单位无所谓（会归一化），但方向必须和陀螺轴一致
轴向正负必须统一：很多“姿态乱跳/越修越偏”都是轴方向或单位错

Step 4：从四元数得到你想要的角度（可选）
你可以再把 q 转欧拉角（roll/pitch/yaw）。例如常见 ZYX（yaw-pitch-roll）：
roll = atan2(2(w x + y z), 1 - 2(x² + y²))
pitch = asin(2(w y - z x))
yaw = atan2(2(w z + x y), 1 - 2(y² + z²))
注意：没磁力计时 yaw 会漂，这是物理限制，不是你代码错。


4) 参数怎么调（经验值）
Kp（比例）：越大收敛越快，但震荡/抖动更明显
常用起步：Kp = 0.3 ~ 1.0
Ki（积分）：用于消偏置，太大容易“慢性发散”或积分饱和
常用起步：Ki = 0.0 → 稳定后 0.001 ~ 0.05 级别慢慢加
若设备会经常有线性加速度（比如运动很剧烈），加速度不再代表纯重力，会导致纠正错误
可加“加速度幅值门限”（接近 1g 才做纠正），你这版只判断是否为 0

 */
#define rad_to_angle 57.29578f
void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);

//q[4]：当前姿态四元数，函数会原地更新它 分别对应 w,x,y,z   传入数组，它会自己更新q里面w,x,y,z的值
//常见约定：q[0]=w, q[1]=x, q[2]=y, q[3]=z
//gx, gy, gz：陀螺仪角速度（建议单位 rad/s）
//ax, ay, az：加速度计（单位可以是 m/s² 或 g 都行，因为会归一化；但方向要对）

/*=================================================使用方法=====================================================*/
/*
 利用RTOS和SPI读取到BMI088的角速度计和加速度计的数据
 利用MahonyAHRSupdateIMU并传入q[4]也就是四元数的值(w,x,y,z)
 q里面w,x,y,z要保证   w^2 + x^2 + y^2 + z^2 = 1
 初始我们给w=1.0,其它为0 就行，程序会慢慢更新的

 之后就进行四元数解算，我们将传入的四元数进行解算：
 roll = atan2(2(w x + y z), 1 - 2(x² + y²))
 pitch = asin(2(w y - z x))
 yaw = atan2(2(w z + x y), 1 - 2(y² + z²))

 最后得到这三个的弧度
 最后再乘57.29578f就是角度了









*/
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
