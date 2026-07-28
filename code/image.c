/*
 * image.c
 *
 *  Created on: 2026年6月5日
 *      Author: hp
 */
#include "image.h"
#include <math.h>

float left_jidian;
float right_jidian;
uint8 left_line_save[MT9V03X_1_H]={0};
uint8 right_line_save[MT9V03X_1_H]={0};
uint8 mid_line[MT9V03X_1_H]={0};
uint8 imageindex[MT9V03X_1_H][MT9V03X_1_W];//二值化图像
int threhold;
//mt9v03x_image_1
/*********************************
大津法寻找阈值
函数名uint8 Otsu(uint8 mt9v03x_image_1[MT9V03X_1_H][MT9V03X_1_W]);
**********************************/

float P[256]={0};
float PK[256]={0};
float MK[256]={0};
uint16 huidu[256]={0};
#define Ostu_threhold_min 100
uint8 Otsu(uint8 mt9v03x_image_1[MT9V03X_1_H][MT9V03X_1_W])
{
    uint8 image_temp=0;
    uint8 threhold=0;
    float totalpixel=MT9V03X_1_H*MT9V03X_1_W;
    //将前一次的数据清零
    for(int i=0;i<256;i++)
    {
        P[i]=0;
        PK[i]=0;
        MK[i]=0;
        huidu[i]=0;
    }
    //统计灰度直方图y
    for(int i=0;i<MT9V03X_1_H;i++)
    {
        for(int j=0;j<MT9V03X_1_W;j++)
        {
            image_temp=mt9v03x_image_1[i][j];
            huidu[image_temp]++;
        }
    }
    //计算类间方差
    float sumPK=0;
    float sumMK=0;//临时存储
    for(int i=0;i<256;i++)
    {
        P[i]=(float)huidu[i]/totalpixel;
        PK[i]=sumPK+P[i];
        sumPK=PK[i];
        MK[i]=sumMK+i*P[i];
        sumMK=MK[i];
    }
    //最大类间方差下的阈值
    float wondvalue=0;
    float value=0;
    for(int i=5;i<245;i++)
    {
        wondvalue=((MK[255]*PK[i]-MK[i])*(MK[255]*PK[i]-MK[i]))/(PK[i]*(1-PK[i]));
        if(wondvalue>value)
        {
            value=wondvalue;
            threhold=i;
        }
    }
    if(threhold<Ostu_threhold_min)
    {
        threhold=Ostu_threhold_min;
    }
    return threhold;
}
/*********************************
进行二值化处理将图像变为只有0和255的黑白图像
函数名BinaryZation(uint8 value)；
**********************************/
void BinaryZation(uint8 value)
{
    uint8 temp;
    for(uint8 i=0;i<MT9V03X_1_H;i++)
    {
        for(uint8 j=0;j<MT9V03X_1_W;j++)
        {
            imageindex[i][j] = (mt9v03x_image_1[i][j] < value) ? 0 : 255;
        }
    }
}
/*********************************
 * 边界跟随搜边线(类似迷宫)
 *
 * 1. 从底部中心向左右找黑白跳变 → 左右基点
 * 2. 逐行向上，在上一行边界 ±SEARCH_RANGE 窗口内搜索
 * 3. 中线 = (左边界 + 右边界) / 2
 *********************************/
#define SEARCH_RANGE   10
#define start_search_line 120
#define end_search_line 5
#define expand_row 10

void follow_bianjie(uint8 imageindex[MT9V03X_1_H][MT9V03X_1_W])
{
    int last_left  = 0;
    int last_right = 0;
    int left_lost_cnt=0;
    int right_lost_cnt=0;
    int base_row   = start_search_line - 1;   // 119，搜线起始行

    // ===== 第1步：基行从左到右扫最长白段 =====
    int best_left = 0, best_right = 0, best_len = 0;
    int seg_start = 0, in_seg = 0;

    for (int j = 1; j < MT9V03X_1_W - 1; j++)
    {
        if (imageindex[base_row][j] == 255)
        {
            if (!in_seg) { seg_start = j; in_seg = 1; }
        }
        else
        {
            if (in_seg)
            {
                int seg_len = j - seg_start;
                if (seg_len > best_len)
                {
                    best_len   = seg_len;
                    best_left  = seg_start;
                    best_right = j - 1;
                }
                in_seg = 0;
            }
        }
    }
    if (in_seg)
    {
        int seg_len = MT9V03X_1_W - 1 - seg_start;
        if (seg_len > best_len)
        {
            best_left  = seg_start;
            best_right = MT9V03X_1_W - 2;
        }
    }

    if (best_len > 3)
    {
        last_left  = best_left;
        last_right = best_right;
    }
    else
    {
        last_left  = 1;
        last_right = MT9V03X_1_W - 2;
    }

    left_line_save[base_row]  = (uint8)last_left;
    right_line_save[base_row] = (uint8)last_right;
    mid_line[base_row]        = (uint8)((last_left + last_right) / 2);

    left_jidian  = (float)last_left;
    right_jidian = (float)last_right;

    // 网上搜
    for (int i = MT9V03X_1_H - 1 - 1; i >= 1; i--)
    {
        int left_line  = 0;
        int right_line = 0;

        // 左边界：在上一行左边界 ±SEARCH_RANGE 窗口内找 黑→白
        int L = last_left - SEARCH_RANGE;  if (L < 1) L = 1;
        int R = last_left + SEARCH_RANGE;  if (R > MT9V03X_1_W - 2) R = MT9V03X_1_W - 2;
        for (int j = L; j <= R; j++)
            if (imageindex[i][j-1] == 0 && imageindex[i][j] == 255)
                { left_line = j; break; }

        // 右边界：在上一行右边界 ±SEARCH_RANGE 窗口内找 白→黑
        L = last_right - SEARCH_RANGE;  if (L < 1) L = 1;
        R = last_right + SEARCH_RANGE;  if (R > MT9V03X_1_W - 2) R = MT9V03X_1_W - 2;
        for (int j = R; j >= L; j--)
            if (imageindex[i][j] == 255 && imageindex[i][j+1] == 0)
                { right_line = j; break; }

        // 找到了就更新，找不到就保持上一行位置


            if (left_line  != 0) last_left  = left_line;
            else                left_line   = last_left;

            if (right_line != 0) last_right = right_line;
            else                 right_line = last_right;
        /*if (left_line != 0)
        {
            last_left = left_line;
            left_lost_cnt = 0;
        }
        else
        {
            left_lost_cnt++;
            if (left_lost_cnt < 3)
                left_line = last_left;   // 补线
            else
                ;                        // 拐角，不补，left_line 保持 0
        }

        if (right_line != 0)
        {
            last_right = right_line;
            right_lost_cnt = 0;
        }
        else
        {
            right_lost_cnt++;
            if (right_lost_cnt < 3)
                right_line = last_right;
            else
                ;
        }*/


        left_line_save[i]  = (uint8)left_line;
        right_line_save[i] = (uint8)right_line;
        mid_line[i]        = (uint8)((left_line + right_line) / 2);
    }
}

/********************************
 *计算与实际中线的偏差
 *函数名float Deviation()；
 ********************************/
float Deviation()
{
    const int Actual_centerline=MT9V03X_1_W/2;
    float actual_start_line=MT9V03X_1_H-30;
    float actua_end_line=50;
    int sum_midline = 0;
    for(uint8 i=actual_start_line;i>=actua_end_line;i--)
    {
        sum_midline += (mid_line[i]-Actual_centerline);
    }
    float error=sum_midline/(actual_start_line-actua_end_line);
    return error/1.1;// 可调；比值越大，转向越灵敏；k越小，转向越柔和
}
/**********************************************************
 *
 *计算路口情况
 *imageindex[i][j+1]
**********************************************************/

struct canny_where
{
    int bbw ;
    int wwb ;
    int road[5] ;   
    int zuobiao[10][20];
};
struct canny_where up = {0};
struct canny_where down = {0};
struct canny_where left = {0};
struct canny_where right = {0};
//判断大小
int max(int a,int b)
{
    if(a>b)
    {
        return a;
        
    }
    else
    {
        return b;
    }

}
int min(int a,int b)
{
    if(a<b)
    {
        return a;
        
    }
    else
    {
        return b;
    }

}
int find_max(int arr[])
{
    int size=3;
    int max_val=arr[0];
    for(int i=1;i<size;i++)
    {
        if(arr[i]>max_val)
        {
            max_val=arr[i];
        }
    }
    return max_val;
}
int find_min(int arr[])
{
    int size=3;
    int min_val=arr[0];
    for(int i=1;i<size;i++)
    {
        if(arr[i]<min_val)
        {
            min_val=arr[i];
        }
    }
    return min_val;
}


void judge_num_where()
{
    memset(&up, 0, sizeof(up));
    memset(&down, 0, sizeof(down));
    memset(&left, 0, sizeof(left));
    memset(&right, 0, sizeof(right));
    //前进方向
    int up_num=0;
    for(int h=MT9V03X_1_H/2;h<MT9V03X_1_H/2+20;h+=5)//h为行，l为列
    {
        int found=0;
        for(int l=0;l<MT9V03X_1_W-1;l++)
        {
            
            if(imageindex[h][l]==255&&imageindex[h][l+1]==0)
            {
                up.zuobiao[up_num][found]=h;
                up.zuobiao[up_num][found+1]=l;
                found+=2;
                up.road[(h-MT9V03X_1_H/2)/3 ]++;
            }
            
        }
        up_num++;
    }   
    up.road[0]=find_max(up.road);
        //左转方向
        int left_num=0;
    
        for(int l=20;l<40;l+=5)
        {
            left.bbw=0;
            left.wwb=0;
            int found=0;
            //行发生黑白跳变
            for(int h=MT9V03X_1_H/2+10;h<MT9V03X_1_H/2+20;h++)
            {
            //黑->白跳变
                if(imageindex[h][l]==0&&imageindex[h+1][l]==255)
                {
                    left.zuobiao[left_num][found]=h;
                    left.zuobiao[left_num][found+1]=l;
                    found+=2;
                    left.bbw++;
                }
                //白->黑跳变
                if(imageindex[h][l]==255&&imageindex[h+1][l]==0)
                {
                    left.zuobiao[left_num][found]=h;
                    left.zuobiao[left_num][found+1]=l;
                    found+=2;
                    left.wwb++;
                }
            }
            left.road[left_num]=max(left.bbw,left.wwb);
            left_num++;
        }
        left.road[0]=find_max(left.road);
            // 右转
            int right_num=0;
            for(int l = MT9V03X_1_W/2+40; l > MT9V03X_1_W/2+20; l -= 5)
            {
                right.bbw=0;
                right.wwb=0;
                int found=0;
                for(int h=MT9V03X_1_H/2+10;h<MT9V03X_1_H/2+20;h++)
                {
                    //发生了黑白跳变
                    if(imageindex[h][l]==0&&imageindex[h+1][l]==255)
                    {
                        right.zuobiao[right_num][found]=h;
                        right.zuobiao[right_num][found+1]=l;
                        found+=2;
                        right.bbw++;

                    }
                    //白黑跳变
                    if(imageindex[h][l]==255&&imageindex[h+1][l]==0)
                    {
                        right.zuobiao[right_num][found]=h;
                        right.zuobiao[right_num][found+1]=l;
                        found+=2;
                        right.wwb++;
                    }
                }
                right.road[right_num]=max(right.bbw,right.wwb);
                right_num++;
            }
        right.road[0]=find_max(right.road);   
}

/*
 * 计算两点连线与图像中线的偏差

 需要x1，y1，x2，y2四个参数，分别为两点的坐标
 返回值为两点连线与图像中线的偏差
 偏差越大，说明两点连线与图像中线的偏离越大
*/
#include <math.h>
float calculate_deviation(int x1,int y1)
{
    int x2 = mid_line[MT9V03X_1_H - 1];    // 底行中线
    int y2 = 0;                            // y2-y1 永不为 0
    // 图像垂直中线
    float mid_x = (float)MT9V03X_1_W / 2.0f;   // 94.0
    //参考 y 坐标（图S像中间行）
    int y_ref = MT9V03X_1_H / 2;               // 60
    // 计算直线在 y_ref 处的 x 坐标
    //    (x-x1)/(x2-x1) = (y-y1)/(y2-y1)
    float x_line = (float)x1 + (float)(x2 - x1) * (float)(y_ref - y1) / (float)(y2 - y1);
    //偏差 = 直线x - 中线x
    float deviation = x_line - mid_x;
    return deviation;   
}
float weighted_error = 0;//偏差
int my_abs(int num) {
    return (num < 0) ? -num : num;
}
/*
识别前方是否存在大片黑作为辅助判断
*/
int judge_up_if_all_black()
{
    int contine_black=0;
    for(int h=MT9V03X_1_H/2+20;h>MT9V03X_1_H/2+10;h--)
    {   
        int white_count=0;
        for(int l=MT9V03X_1_W/2-20;l<MT9V03X_1_W/2+20;l++)
        {
            if(imageindex[h][l]==255)
            {
                white_count++;
            }
        }
        if(white_count<10)
        {
            contine_black++;
        }

    }
    if(contine_black>=3)
    {
        return 1;
    }
    else
    {
        return 0;
    }

}


int judge_where_better_left()
{
    //左转
    int contine_white = 0;
           for(int h = MT9V03X_1_H/2; h < MT9V03X_1_H/2 + 9; h++)
           {
               int count1 = 0;
               for(int l = 15; l < 25; l++)
               {
                   if(imageindex[h][l] == 255)
                   {
                       count1++;
                       if(count1 > 5)
                       {
                           contine_white++;   // 行满足连续白点条件
                           break;          // 跳出列循环，检测下一行
                       }
                   }
                   else
                   {
                       count1 = 0;
                   }
               }
           }
           if(contine_white >= 4)     // 至少4行满足，减少误判
               return 6;
           else
               return 0;
}
int jude_where_better_right()
{
    int contine_white = 0;
        for(int h = MT9V03X_1_H/2; h < MT9V03X_1_H/2 + 10; h++)
        {
            int count1 = 0;
            for(int l = 160; l < 175; l++)
            {
                if(imageindex[h][l] == 255)
                {
                    count1++;
                    if(count1 > 5)
                    {
                        contine_white++;   // 行满足连续白点条件
                        break;          // 跳出列循环，检测下一行
                    }
                }
                else
                {
                    count1 = 0;
                }
            }
        }
        if(contine_white >= 3)     // 至少4行满足，减少误判
            return 7;
        else
            return 0;
}


int judge_where()
{
    if(up.road[0]>0&&left.road[0]>=1&&right.road[0]==0)
    {
        if(left.zuobiao[0][0]-left.zuobiao[1][0]<=2||left.zuobiao[0][0]-left.zuobiao[2][0]<=2)
        {
            if(my_abs(left.zuobiao[0][2] - left.zuobiao[0][0]) >= 5&&judge_where_better_left()==6)
            {
                weighted_error = calculate_deviation(left.zuobiao[0][1],left.zuobiao[0][0]);
                buzzer_beep(30);
                return 3;//左转

            }
        }
    }
    if(up.road[0]>0&&left.road[0]==0&&right.road[0]>=1)
    {
        if(right.zuobiao[0][0]-right.zuobiao[1][0]<=2||right.zuobiao[0][0]-right.zuobiao[2][0]<=2)
        {
            if(my_abs(right.zuobiao[0][2] - right.zuobiao[0][0]) >= 5)
            {
                weighted_error = calculate_deviation(right.zuobiao[0][1],right.zuobiao[0][0]);
                buzzer_beep(30);
                return 4;//右转
            }
        }
    }
    if(up.road[0]>0&&left.road[0]>=1&&right.road[0]>=1)
    {
        if((distance >= 70 && distance <= 77)||(distance>=186&&distance<=190))
        {
            weighted_error = calculate_deviation(left.zuobiao[0][1],left.zuobiao[0][0]);
        }
        else
        {
            weighted_error = calculate_deviation(right.zuobiao[0][1],right.zuobiao[0][0]);
        }

        if(weighted_error>=60.0f) weighted_error=60.0f;
        if(weighted_error<=-60.0f) weighted_error=-60.0f;
        buzzer_beep(30);
        return 8; //十字口
    }
    /*if(up.road[0]>=0&&left.road[0]>=1&&right.road[0]>=1&&judge_up_if_all_black()==1)
    {
        return 9;
    }*/
    
    return 0;//直行
}
void ips114_show_line()
{
    for(int i=0;i<MT9V03X_1_H;i++)
    {
        uint16 y = (uint16)(i / 2);

        ips114_draw_point((uint16)(left_line_save[i]/2), y, RGB565_BLUE);
        ips114_draw_point((uint16)(right_line_save[i]/2), y, RGB565_RED);
        ips114_draw_point((uint16)(mid_line[i]/2), y, RGB565_YELLOW);
    }
}
void image_invocation()//图像处理
{
    threhold=Otsu(mt9v03x_image_1);
    BinaryZation(threhold);
    follow_bianjie(imageindex);
    judge_num_where();

}
void ips114_screen()
{
    ips114_show_int(0, MT9V03X_1_H/2+5,threhold, 3);//阈值
    ips114_show_float(0, MT9V03X_1_H/2+20, left_jidian, 3, 2);//显示左基点
    ips114_show_float(0, MT9V03X_1_H/2+35, right_jidian, 3, 2);//显示右基点

    ips114_show_int(MT9V03X_1_W/2+10,0,motor_L.encoder_speed,3);//左编码器数值
    ips114_show_int(MT9V03X_1_W/2+10,25,motor_R.encoder_speed,3);//右编码器数值
    //ips114_show_int(MT9V03X_1_W/2+10,45,judge_where(),2);//判断左右转

    ips114_show_int(MT9V03X_1_W/2+30, 45, up.road[0], 2);
    ips114_show_int(MT9V03X_1_W/2+30, 60, left.road[0], 2);
    ips114_show_int(MT9V03X_1_W/2+30, 75, right.road[0], 2);
    ips114_show_int(MT9V03X_1_W/2+30, 90, judge_where(), 2);

    ips114_show_float(MT9V03X_1_W/2+45, 0,distance_L, 3,2);
    ips114_show_float(MT9V03X_1_W/2+45, 20,distance_R, 3,2);
    ips114_show_float(MT9V03X_1_W/2+45,35,distance,4,2);

    //ips114_show_int(MT9V03X_1_W/2+30,105,judge_up_if_all_black(),1);

}


