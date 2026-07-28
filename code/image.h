/*
 * image.h
 *
 *  Created on: 2026锟斤拷6锟斤拷5锟斤拷
 *      Author: hp
 */

#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"
extern int threhold;
 extern float weighted_error;//偏锟斤拷


uint8 Otsu(uint8 imageindex[MT9V03X_1_H][MT9V03X_1_W]);
void BinaryZation(uint8 value);
void image_invocation();
void ips114_screen();
void ips114_show_line();
float Deviation();


typedef enum
{
    ROAD_NORMAL       = 0,
    ROAD_LEFT_BRANCH  = 3,   // 宸︿笁宀�
    ROAD_RIGHT_BRANCH = 4,   // 鍙充笁宀�
    ROAD_CROSS        = 8,   // 鍗佸瓧
    ROAD_T            = 9,   // T瀛�
} road_type_t;
#endif /* CODE_IMAGE_H_ */
