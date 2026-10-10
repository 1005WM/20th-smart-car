/*
 * IMU.h
 *
 *  Created on: 2025年10月18日
 *      Author: 冰凌凌凌
 */

#ifndef CODE_IMU_H_
#define CODE_IMU_H_

//全局变量

extern float gyro_Offset_flag;//标志位
extern float GL_IMU_Flag;//陀螺仪数据开启标志位
extern float Z_360;//Z轴规划
extern float T_M;//Z轴规划(备用1)
extern float T_N;//Z轴规划(备用2)

typedef struct{
        float Xdata;//零飘参数X
        float Ydata;//零飘参数Y
        float Zdata;//零飘参数Z
}gyro_param_t;


typedef struct{
        float acc_x;//x轴加速度
        float acc_y;//y轴加速度
        float acc_z;//z轴加速度

        float gyro_x;//x轴角速度
        float gyro_y;//y轴角速度
        float gyro_z;//z轴角速度

}IMU_param_t;


float IMU_gyro_Offset_Init(void);//零飘初始化
void  IMU_GetValues();//将采集到的数值转化为实际物理值，并对陀螺仪进行去零飘处理
void IMU_YAW_integral(void); //积分出角度值
void IMU_Handle_0();//规划为0-正无穷和0-负无穷  T_N(备用1)
void IMU_Handle_180();//规划为0-180和0-(-180)   Z_360
void IMU_Handle_360();//规划为0-360和0-(-360)//弧度转化为角度 限制IMU的数值(备用2)
void IMU_init(void);
void IMU_show(void);


#endif /* CODE_IMU_H_ */
