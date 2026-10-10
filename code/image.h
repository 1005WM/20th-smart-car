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
#define CROP_H  90      // 裁剪后高度
#define CROP_W  168  //宽度

uint8 Otsu(uint8 Image_Use[CROP_H][CROP_W]);
void BinaryZation(uint8 value);
void image_invocation();
void ips114_screen();
void ips114_show_line();
float Deviation();
void cut_image(void);
extern uint8 Image_Use[CROP_H][CROP_W];

extern volatile int road_order_cache;
extern volatile float road_weighted_error_cache;

typedef enum
{
    ROAD_NORMAL       = 0,
    ROAD_LEFT_BRANCH  = 3,
    ROAD_RIGHT_BRANCH = 4,
    ROAD_CROSS        = 8,
    ROAD_T            = 9,
} road_type_t;
#endif /* CODE_IMAGE_H_ */
