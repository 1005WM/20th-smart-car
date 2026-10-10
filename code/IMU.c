
#include "zf_common_headfile.h"

gyro_param_t Gyro_Offset;//零飘参数
IMU_param_t IMU_Data;//加速度 角速度


float gyro_Offset_flag = 0;//标志位
float GL_IMU_Flag = 0;//陀螺仪数据开启标志位

float Z_360 =0;
float T_M=0;
float T_N=0;

void IMU_init()
{
    imu660rb_init();  
    IMU_gyro_Offset_Init();//零飘初始化
}


float IMU_gyro_Offset_Init()//零飘初始化
{
    Gyro_Offset.Xdata = 0;
    Gyro_Offset.Ydata = 0;
    Gyro_Offset.Zdata = 0;

    for(uint16_t i=0; i<100;i++)
    {
        Gyro_Offset.Xdata += imu660rb_gyro_x;//三轴数据
        Gyro_Offset.Ydata += imu660rb_gyro_y;
        Gyro_Offset.Zdata += imu660rb_gyro_z;
        system_delay_ms(5);//最大1Khz 采集时间
    }

    Gyro_Offset.Xdata /=100;
    Gyro_Offset.Ydata /=100;
    Gyro_Offset.Zdata /=100;

    return gyro_Offset_flag= 1;

}
void  IMU_GetValues()/
{
    //灵敏度根据型号选择
    //IMU660--16.4
    //IMU963--14.3
    IMU_Data.gyro_x = ((float)imu660rb_gyro_x - Gyro_Offset.Xdata) * PI /180 /14.3f;
    IMU_Data.gyro_y = ((float)imu660rb_gyro_y - Gyro_Offset.Ydata) * PI /180 /14.3f;
    IMU_Data.gyro_z = ((float)imu660rb_gyro_z - Gyro_Offset.Zdata) * PI /180 /14.3f;



}

//正负自己可调,默认顺时针旋转
void IMU_Handle_0()//规划为0-正无穷和0-负无穷  T_N
{
    T_N-=RAD_TO_ANGLE(IMU_Data.gyro_z*0.005);
}
//180°
void IMU_Handle_180()
{
    Z_360-=RAD_TO_ANGLE(IMU_Data.gyro_z*0.005);

    if((Z_360>0&&Z_360<=180)  ||   (Z_360<0&&Z_360>=(-180)))//顺时针旋转
    {
        Z_360= Z_360;
    }
    else if(Z_360>180 && Z_360<=360)
    {
        Z_360-=360;
    }
    else if(Z_360<(-180) && Z_360>=(-360))
    {
        Z_360+=360;
    }

}
//360度
void IMU_Handle_360()
{
    T_M-=RAD_TO_ANGLE(IMU_Data.gyro_z*0.005);

    if(T_M>360||T_M<-360)
    {
        T_M=0;
    }
}



void IMU_YAW_integral() //积分出角度值
{
    IMU_GetValues();
    if(IMU_Data.gyro_z<0.015&&IMU_Data.gyro_z>-0.015)//滤波
    {
        Z_360-=0;
        T_M-=0;
        T_N-=0;
    }
    else
    {
        IMU_Handle_180();//规划为0-180和0-(-180)   Z_360
        IMU_Handle_360();//规划为0-360和0-(-360)   T_M
        IMU_Handle_0();//规划为0-正无穷和0-负无穷  T_N
    }

}

