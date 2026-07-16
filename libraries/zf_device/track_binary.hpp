#ifndef _TRACK_BINARY_HPP__
#define _TRACK_BINARY_HPP__
#include "zf_common_headfile.hpp"
#include <stdint.h>
#include <string.h>

// /* 图像尺寸（根据实际情况修改） */
// #define UVC_H   60
// #define UVC_W   80

/* 种子点起始行（从图像底部往上数，通常取靠近车体的行） */
#define START_ROW  55   // 可根据实际调整

/* 状态枚举 */
enum {
    NO_PTS      = 0,
    LEFT_PTS    = 1,
    RIGHT_PTS   = 2,
    TWO_PTS     = 3
};

/* 全局变量声明（需在 .c 中定义） */
//extern uint8 image_two_value[UVC_H][UVC_W];   // 二值图：0=背景，1=赛道
extern uint8 seed_left[2];                     // 左边界种子点 (x, y)
extern uint8 seed_right[2];                    // 右边界种子点 (x, y)
extern uint8 mid_position;                     // 中线列坐标
extern uint8 seed_state;                       // 当前种子状态
extern uint8 width_base;                       // 基准赛道宽度（记忆值）

/* 函数声明 */
void seek_pts_seed(uint8 start_row, uint8 min_row);
uint8 findline_lefthand_binary(uint8 pts_array[][2]);
uint8 findline_righthand_binary(uint8 pts_array[][2]);
uint8 reduce_left_to_line(uint8 pts_in[][2], uint8 step, uint8 line_out[60]);
uint8 reduce_right_to_line(uint8 pts_in[][2], uint8 step, uint8 line_out[60]) ;
void map_line_to_display(const uint8 line_80x60[60], uint8 top_row, uint16 line_320[240],int xx) ;
#endif