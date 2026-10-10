/*
 * IMU.h
 *
 *  Created on: 2025��10��18��
 *      Author: ��������
 */

#ifndef CODE_IMU_H_
#define CODE_IMU_H_

//ȫ�ֱ���

extern float gyro_Offset_flag;//��־λ
extern float GL_IMU_Flag;//���������ݿ�����־λ
extern float Z_360;//Z��滮
extern float T_M;//Z��滮(����1)
extern float T_N;//Z��滮(����2)

typedef struct{
        float Xdata;//��Ʈ����X
        float Ydata;//��Ʈ����Y
        float Zdata;//��Ʈ����Z
}gyro_param_t;


typedef struct{
        float acc_x;//x����ٶ�
        float acc_y;//y����ٶ�
        float acc_z;//z����ٶ�

        float gyro_x;//x����ٶ�
        float gyro_y;//y����ٶ�
        float gyro_z;//z����ٶ�

}IMU_param_t;


float IMU_gyro_Offset_Init(void);//��Ʈ��ʼ��
void  IMU_GetValues();//���ɼ�������ֵת��Ϊʵ������ֵ�����������ǽ���ȥ��Ʈ����
void IMU_YAW_integral(void); //���ֳ��Ƕ�ֵ
void IMU_Handle_0();//�滮Ϊ0-�������0-������  T_N(����1)
void IMU_Handle_180();//�滮Ϊ0-180��0-(-180)   Z_360
void IMU_Handle_360();//�滮Ϊ0-360��0-(-360)//����ת��Ϊ�Ƕ� ����IMU����ֵ(����2)
void IMU_init(void);
void IMU_show(void);


#endif /* CODE_IMU_H_ */
