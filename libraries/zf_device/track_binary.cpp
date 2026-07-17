#include "track_binary.hpp"
#include "my_image.hpp"
// /* 全局变量定义 */
extern uint8 image_two_value[UVC_H][UVC_W]; 
// uint8 seed_left[2]  = {0};
// uint8 seed_right[2] = {0};
// uint8 mid_position  = UVC_W / 2;
// uint8 seed_state    = NO_PTS;
// uint8 width_base    = 0;

// /* ==================== 辅助函数 ==================== */

// /**
//  * @brief 在二值图像的行上，从 start_x 开始沿 direction 方向搜索第一个跳变点（0→1 或 1→0）
//  * @param row       行号
//  * @param start_x   起始列
//  * @param direction 0=向左搜索，1=向右搜索
//  * @return 找到的列坐标，未找到返回0
//  */
// static uint8 find_edge_binary_row(uint8 row, uint8 start_x, uint8 direction) {
//     if (row >= UVC_H || start_x >= UVC_W) return 0;

//     if (direction == 1) {  // 向右
//         int16_t x = start_x;
//         while (x < UVC_W - 1) {
//             if (image_two_value[row][x] != image_two_value[row][x + 1]) {
//                 return x + 1;  // 返回跳变后的点
//             }
//             x++;
//         }
//     } else {  // 向左
//         int16_t x = start_x;
//         while (x > 0) {
//             if (image_two_value[row][x] != image_two_value[row][x - 1]) {
//                 return x - 1;
//             }
//             x--;
//         }
//     }
//     return 0;
// }

// // static uint8 find_edge_binary_row(uint8 row, uint8 start_x) {
// //     if (row >= UVC_H || start_x >= UVC_W) return 0;

// //         uint8 x = start_x;
// //         while (x < UVC_W - 1) {
// //             if (image_two_value[row][x] != image_two_value[row][x + 1]) {
// //                 return x + 1;  // 返回跳变后的点
// //             }
// //             x++;
// //         }
   
// //     return 0;
// // }

// /**
//  * @brief 在二值图像的列上，从 start_y 开始沿 direction 方向搜索第一个跳变点
//  * @param col       列号
//  * @param start_y   起始行
//  * @param direction 0=向下（y增加），1=向上（y减小）
//  * @return 找到的行坐标，未找到返回0
//  */
// static uint8 find_edge_binary_col(uint8 col, uint8 start_y, uint8 direction) {
//     if (col >= UVC_W || start_y >= UVC_H) return 0;

//     if (direction == 0) {  // 向下
//         uint8 y = start_y;
//         while (y < UVC_H - 1) {
//             if (image_two_value[y][col] != image_two_value[y + 1][col]) {
//                 return y + 1;
//             }
//             y++;
//         }
//     } else {  // 向上
//         int16_t y = start_y;
//         while (y > 0) {
//             if (image_two_value[y][col] != image_two_value[y - 1][col]) {
//                 return y - 1;
//             }
//             y--;
//         }
//     }
//     return 0;
// }

// /* ==================== 种子点搜索 ==================== */

// /**
//  * @brief 向上补充缺失的种子点（列方向搜索）
//  * @param point     待更新的点（输入起点，输出找到的新点）
//  * @param seed_kind 标识：LEFT_PTS 或 RIGHT_PTS
//  */
// static void seek_up_binary(uint8 point[2], uint8 seed_kind) {
//     uint8 col = point[0];
//     uint8 start_row = point[1];
//     if (col < 1 || col >= UVC_W - 1 || start_row < 1 || start_row >= UVC_H - 1)
//         return;

//     uint8 found_row = find_edge_binary_col(col, start_row, 1);  // 向上搜索
//     if (found_row > 0) {
//         if (seed_kind == LEFT_PTS) {
//             seed_left[0] = col;
//             seed_left[1] = found_row;
//         } else if (seed_kind == RIGHT_PTS) {
//             seed_right[0] = col;
//             seed_right[1] = found_row;
//         } else {
//             point[0] = col;
//             point[1] = found_row;
//         }
//     }
// }

// /**
//  * @brief 主种子点搜索：在 start_row 行从中线向两侧找左右边界
//  * @param start_row 起始行（通常靠近图像底部）
//  * @param min_row   允许向上搜索的最小行（防止搜索到图像顶部以外）
//  */
// void seek_pts_seed(uint8 start_row, uint8 min_row) {
//     if (start_row >= UVC_H || start_row < 1) {
//         seed_state = NO_PTS;
//         return;
//     }

//     uint8 result = NO_PTS;
//     seed_left[0]  = 0;
//     seed_left[1]  = start_row;
//     seed_right[0] = UVC_W-1;
//     seed_right[1] = start_row;

//     /* 从中线向左右找跳变点（即边界） */
//     uint8 left_x = find_edge_binary_row(start_row, mid_position, 0);  // 向左
//     if (left_x >= 2) {
//         seed_left[0] = left_x;
//         result += LEFT_PTS;
//     }

//     uint8 right_x = find_edge_binary_row(start_row, mid_position, 1); // 向右
//     if (right_x <= UVC_W - 2) {
//         seed_right[0] = right_x;
//         result += RIGHT_PTS;
//     }

//     /* 若只找到一侧，向上逐行补充另一侧 */
//     // if (result != TWO_PTS) {
//     //     if (result == LEFT_PTS) {
//     //         for (uint8 r = start_row - 1; r >= min_row; r--) {
//     //             right_x = find_edge_binary_row(r, mid_position, 1);
//     //             if (right_x <= UVC_W - 2) {
//     //                 seed_right[0] = right_x;
//     //                 seed_right[1] = r;
//     //                 result += RIGHT_PTS;
//     //                 break;
//     //             }
//     //         }
//     //     } else if (result == RIGHT_PTS) {
//     //         for (uint8 r = start_row - 1; r >= min_row; r--) {
//     //             left_x = find_edge_binary_row(r, mid_position, 0);
//     //             if (left_x >= 2) {
//     //                 seed_left[0] = left_x;
//     //                 seed_left[1] = r;
//     //                 result += LEFT_PTS;
//     //                 break;
//     //             }
//     //         }
//     //     }
//     // }

//     seed_state = result;
//     // if (result == TWO_PTS) {
//     //     if (width_base == 0)
//     //         width_base = seed_right[0] - seed_left[0];
//     //     mid_position = (seed_left[0] + seed_right[0]) / 2;
//     // } else if (result == LEFT_PTS) {
//     //     if (width_base != 0)
//     //         mid_position = seed_left[0] + width_base / 2;
//     // } else if (result == RIGHT_PTS) {
//     //     if (width_base != 0)
//     //         mid_position = seed_right[0] - width_base / 2;
//     // }
// }

// /* ==================== 左右手沿边线跟踪 ==================== */

// /* 方向表（与原始代码一致） */
// static const int8 dir_front[4][2] = {
//     {0, -1},   // 上
//     {1, 0},    // 右
//     {0, 1},    // 下
//     {-1, 0}    // 左
// };
// static const int8 dir_frontleft[4][2] = {
//     {-1, -1},  // 左上
//     {1, -1},   // 右上
//     {1, 1},    // 右下
//     {-1, 1}    // 左下
// };
// static const int8 dir_frontright[4][2] = {
//     {1, -1},   // 右上
//     {1, 1},    // 右下
//     {-1, 1},   // 左下
//     {-1, -1}   // 左上
// };

// /**
//  * @brief 左手沿边线跟踪（基于二值图）
//  * @param pts_array 输出轨迹点数组 [step][2]
//  * @return 跟踪到的点数
//  */
// uint8 findline_lefthand_binary(uint8 pts_array[][2]) {
//     uint8 step = 0, dir = 0, turn = 0, repeat = 0;
//     uint8 x = seed_left[0];
//     uint8 y = seed_left[1];

//     while (step < UVC_W - 2 &&
//            x > 2 && x < UVC_W - 2 &&
//            y > 2 && y < UVC_H - 2 &&
//            turn < 4) {

//         /* 防重复点（若回到上一帧的右种子点，计数） */
//         if (x == seed_right[0] && y == seed_right[1]) {
//             repeat++;
//             if (repeat > 3) { step = 0; break; }
//         }

//         if (step == 1) {
//             /* 记录第一点作为后续防重复参考 */
//             seed_right[0] = x;
//             seed_right[1] = y;
//         }

//         int8 nx = x + dir_front[dir][0];
//         int8 ny = y + dir_front[dir][1];
//         int8 flx = x + dir_frontleft[dir][0];
//         int8 fly = y + dir_frontleft[dir][1];

//         uint8 front_val     = image_two_value[ny][nx];
//         uint8 frontleft_val = image_two_value[fly][flx];

//         /* 二值图中：0=背景（黑色），1=赛道（白色）
//            原始算法中 front < thres 表示前方是背景，需要转向 */
//         if (front_val == 0) {
//             dir = (dir + 1) % 4;   // 顺时针转向
//             turn++;
//         } else if (frontleft_val == 0) {
//             x = nx; y = ny;        // 向前走一步
//             pts_array[step][0] = x;
//             pts_array[step][1] = y;
//             step++;
//             turn = 0;
//         } else {
//             x = flx; y = fly;      // 向前左方向走
//             pts_array[step][0] = x;
//             pts_array[step][1] = y;
//             step++;
//             turn = 0;
//             dir = (dir + 3) % 4;   // 逆时针回正
//         }
//     }

//     /* 更新种子点为跟踪终点 */
//     seed_left[0] = x;
//     seed_left[1] = y;
//     return step;
// }

// /**
//  * @brief 右手沿边线跟踪（基于二值图）
//  * @param pts_array 输出轨迹点数组 [step][2]
//  * @return 跟踪到的点数
//  */
// uint8 findline_righthand_binary(uint8 pts_array[][2]) {
//     uint8 step = 0, dir = 0, turn = 0, repeat = 0;
//     uint8 x = seed_right[0];
//     uint8 y = seed_right[1];

//     while (step < UVC_W - 2 &&
//            x > 2 && x < UVC_W - 2 &&
//            y > 2 && y < UVC_H - 2 &&
//            turn < 4) {

//         if (x == seed_left[0] && y == seed_left[1]) {
//             repeat++;
//             if (repeat > 3) { step = 0; break; }
//         }

//         if (step == 1) {
//             seed_left[0] = x;
//             seed_left[1] = y;
//         }

//         int8 nx = x + dir_front[dir][0];
//         int8 ny = y + dir_front[dir][1];
//         int8 frx = x + dir_frontright[dir][0];
//         int8 fry = y + dir_frontright[dir][1];

//         uint8 front_val      = image_two_value[ny][nx];
//         uint8 frontright_val = image_two_value[fry][frx];

//         if (front_val == 0) {
//             dir = (dir + 3) % 4;   // 右手转向相反（逆时针）
//             turn++;
//         } else if (frontright_val == 0) {
//             x = nx; y = ny;
//             pts_array[step][0] = x;
//             pts_array[step][1] = y;
//             step++;
//             turn = 0;
//         } else {
//             x = frx; y = fry;
//             pts_array[step][0] = x;
//             pts_array[step][1] = y;
//             step++;
//             turn = 0;
//             dir = (dir + 1) % 4;   // 顺时针回正
//         }
//     }

//     seed_right[0] = x;
//     seed_right[1] = y;
//     return step;
// }






uint8 seed_left[2]  = {0};
uint8 seed_right[2] = {0};
uint8 mid_position  = UVC_W / 2;
uint8 seed_state    = NO_PTS;
uint8 width_base    = 0;

/* ==================== 辅助函数 ==================== */


/**
 * @brief 将左边界点数组压缩为每行一个点（取最小x）
 * @param pts_in  输入点数组（来自 findline_lefthand_binary）
 * @param step    输入点数
 * @param pts_out 输出点数组（需保证容量至少 UVC_H）
 * @return 输出点数
 */
// uint8 reduce_left_per_row(uint8 pts_in[][2], uint8 step, uint8 pts_out[][2]) {
//     uint8 row_x[UVC_H];
//     uint8 row_valid[UVC_H] = {0};  // 标记该行是否已有左边界
//     memset(row_x, 0xFF, sizeof(row_x)); // 初始化为最大值，方便取最小值

//     // 遍历所有输入点，每行保留最小的x
//     for (uint8 i = 0; i < step; i++) {
//         uint8 y = pts_in[i][1];
//         uint8 x = pts_in[i][0];
//         if (y >= UVC_H) continue;
//         if (!row_valid[y] || x < row_x[y]) {
//             row_x[y] = x;
//             row_valid[y] = 1;
//         }
//     }

//     // 按行从小到大输出
//     uint8 count = 0;
//     for (uint8 y = 0; y < UVC_H; y++) {
//         if (row_valid[y]) {
//             pts_out[count][0] = row_x[y];
//             pts_out[count][1] = y;
//             count++;
//         }
//     }
//     return count;
// }

// /**
//  * @brief 将右边界点数组压缩为每行一个点（取最大x）
//  * @param pts_in  输入点数组（来自 findline_righthand_binary）
//  * @param step    输入点数
//  * @param pts_out 输出点数组（需保证容量至少 UVC_H）
//  * @return 输出点数
//  */
// uint8 reduce_right_per_row(uint8 pts_in[][2], uint8 step, uint8 pts_out[][2]) {
//     uint8 row_x[UVC_H];
//     uint8 row_valid[UVC_H] = {0};
//     memset(row_x, 0, sizeof(row_x)); // 初始化为0，方便取最大值

//     for (uint8 i = 0; i < step; i++) {
//         uint8 y = pts_in[i][1];
//         uint8 x = pts_in[i][0];
//         if (y >= UVC_H) continue;
//         if (!row_valid[y] || x > row_x[y]) {
//             row_x[y] = x;
//             row_valid[y] = 1;
//         }
//     }

//     uint8 count = 0;
//     for (uint8 y = 0; y < UVC_H; y++) {
//         if (row_valid[y]) {
//             pts_out[count][0] = row_x[y];
//             pts_out[count][1] = y;
//             count++;
//         }
//     }
//     return count;
// }

/**
 * @brief 将左边界点数组压缩为每行一个x值，存入一维数组，缺失行填0
 * @param pts_in  输入点数组（来自 findline_lefthand_binary）
 * @param step    输入点数
 * @param line_out 输出数组，长度UVC_H，line_out[y] = 左边界x坐标，0表示无边界（实际填充为0）
 * @return 最高有边界行的行号（最小y），若无边界返回UVC_H
 */
uint8 reduce_left_to_line(uint8 pts_in[][2], uint8 step, uint8 line_out[UVC_H]) {
    uint8 row_valid[UVC_H] = {0};
    // 初始化全部为0（左边界默认0）
    for (uint8 y = 0; y < UVC_H; y++) line_out[y] = 0;

    // 遍历输入点，每行保留最小的x
    for (uint8 i = 0; i < step; i++) {
        uint8 y = pts_in[i][1];
        uint8 x = pts_in[i][0];
        if (y >= UVC_H) continue;
        if (!row_valid[y] || x < line_out[y]) {
            line_out[y] = x;
            row_valid[y] = 1;
        }
    }

    // 查找最高有边界行
    uint8 top_row = UVC_H;
    for (uint8 y = 0; y < UVC_H; y++) {
        if (row_valid[y]) {
            top_row = y;
            break;
        }
    }
    return top_row;
}

/**
 * @brief 将右边界点数组压缩为每行一个x值，存入一维数组，缺失行填79
 * @param pts_in  输入点数组（来自 findline_righthand_binary）
 * @param step    输入点数
 * @param line_out 输出数组，长度UVC_H，line_out[y] = 右边界x坐标
 * @return 最高有边界行的行号（最小y），若无边界返回UVC_H
 */
uint8 reduce_right_to_line(uint8 pts_in[][2], uint8 step, uint8 line_out[UVC_H]) {
    uint8 row_valid[UVC_H] = {0};
    // 初始化全部为79（右边界默认79）
    for (uint8 y = 0; y < UVC_H; y++) line_out[y] = UVC_W - 1;

    for (uint8 i = 0; i < step; i++) {
        uint8 y = pts_in[i][1];
        uint8 x = pts_in[i][0];
        if (y >= UVC_H) continue;
        if (!row_valid[y] || x > line_out[y]) {
            line_out[y] = x;
            row_valid[y] = 1;
        }
    }

    uint8 top_row = UVC_H;
    for (uint8 y = 0; y < UVC_H; y++) {
        if (row_valid[y]) {
            top_row = y;
            break;
        }
    }
    return top_row;
}


#define DISPLAY_H 240
#define DISPLAY_W 320
#define SCALE 4

/**
 * @brief 将80x60的一维边界数组映射到320x240的一维显示数组
 * @param line_80x60  输入数组，长度UVC_H，line_80x60[y] = 边界x坐标（0~79）
 * @param top_row     最高有边界行（来自 reduce 函数的返回值）
 * @param line_320    输出数组，长度DISPLAY_H，line_320[r] = 显示x坐标（0~319），无效行填0xFFFF
 */
void map_line_to_display(const uint8 line_80x60[60], uint8 top_row, uint16 line_320[240],int xx) {
    // 初始化所有显示行为无效
    //if( xx=0)
    if( xx=0)
    {for (uint16 r = 0; r < DISPLAY_H; r++) {
        line_320[r] = 1;
    }}else 
    {for (uint16 r = 0; r < DISPLAY_H; r++) {
        line_320[r] = 319;
    }}

    if (top_row >= UVC_H) return; // 无有效边界

    // 从顶部有效行开始映射到底部
    for (uint8 y = top_row; y < UVC_H; y++) {
        uint8 x_small = line_80x60[y];
        uint16 x_disp = (uint16)x_small * SCALE;
        uint16 y_disp_start = (uint16)y * SCALE;
        for (uint16 j = 0; j < SCALE; j++) {
            uint16 r = y_disp_start + j;
            if (r < DISPLAY_H) {
                line_320[r] = x_disp;
            }
        }
    }
}

/**
 * @brief 在二值图像的行上，从 start_x 开始沿 direction 方向搜索第一个跳变点
 * @param row       行号
 * @param start_x   起始列
 * @param direction 0=向左搜索，1=向右搜索
 * @return 找到的列坐标，未找到返回0
 */
static uint8 find_edge_binary_row(uint8 row, uint8 start_x, uint8 direction) {
    if (row >= UVC_H || start_x >= UVC_W) return 0;

    if (direction == 1) {  // 向右
        uint8 x = start_x;
        while (x < UVC_W - 1) {
            if (image_two_value[row][x] != image_two_value[row][x + 1])
                return x + 1;
            x++;
        }
    } else {  // 向左
        int16_t x = start_x;
        while (x > 0) {
            if (image_two_value[row][x] != image_two_value[row][x - 1])
                return x - 1;
            x--;
        }
    }
    return 0;
}

/**
 * @brief 在二值图像的列上，从 start_y 开始沿 direction 方向搜索第一个跳变点
 * @param col       列号
 * @param start_y   起始行
 * @param direction 0=向下（y增加），1=向上（y减小）
 * @return 找到的行坐标，未找到返回0
 */
static uint8 find_edge_binary_col(uint8 col, uint8 start_y, uint8 direction) {
    if (col >= UVC_W || start_y >= UVC_H) return 0;

    if (direction == 0) {  // 向下
        uint8 y = start_y;
        while (y < UVC_H - 1) {
            if (image_two_value[y][col] != image_two_value[y + 1][col])
                return y + 1;
            y++;
        }
    } else {  // 向上
        int16_t y = start_y;
        while (y > 0) {
            if (image_two_value[y][col] != image_two_value[y - 1][col])
                return y - 1;
            y--;
        }
    }
    return 0;
}

/* ==================== 种子点搜索 ==================== */

/**
 * @brief 向上补充缺失的种子点（列方向搜索）
 * @param point     待更新的点（输入起点，输出找到的新点）
 * @param seed_kind 标识：LEFT_PTS 或 RIGHT_PTS
 */
static void seek_up_binary(uint8 point[2], uint8 seed_kind) {
    uint8 col = point[0];
    uint8 start_row = point[1];
    if (col < 1 || col >= UVC_W - 1 || start_row < 1 || start_row >= UVC_H - 1)
        return;

    uint8 found_row = find_edge_binary_col(col, start_row, 1);  // 向上搜索
    if (found_row > 0) {
        if (seed_kind == LEFT_PTS) {
            seed_left[0] = col;  seed_left[1] = found_row;
        } else if (seed_kind == RIGHT_PTS) {
            seed_right[0] = col; seed_right[1] = found_row;
        } else {
            point[0] = col; point[1] = found_row;
        }
    }
}

/**
 * @brief 主种子点搜索：在 start_row 行从中线向两侧找左右边界
 * @param start_row 起始行（通常靠近图像底部）
 * @param min_row   允许向上搜索的最小行（防止搜索到图像顶部以外）
 */
void seek_pts_seed(uint8 start_row, uint8 min_row) {
    if (start_row >= UVC_H || start_row < 1) {
        seed_state = NO_PTS;
        return;
    }

    uint8 result = NO_PTS;
    seed_left[0]  = 0;          seed_left[1]  = start_row;
    seed_right[0] = UVC_W;      seed_right[1] = start_row;

    uint8 left_x = find_edge_binary_row(start_row, mid_position, 0);
    if (left_x >= 2) { seed_left[0] = left_x; result += LEFT_PTS; }

    uint8 right_x = find_edge_binary_row(start_row, mid_position, 1);
    if (right_x <= UVC_W - 2) { seed_right[0] = right_x; result += RIGHT_PTS; }

    if (result != TWO_PTS) {
        if (result == LEFT_PTS) {
            for (uint8 r = start_row - 1; r >= min_row; r--) {
                right_x = find_edge_binary_row(r, mid_position, 1);
                if (right_x <= UVC_W - 2) {
                    seed_right[0] = right_x; seed_right[1] = r;
                    result += RIGHT_PTS; break;
                }
            }
        } else if (result == RIGHT_PTS) {
            for (uint8 r = start_row - 1; r >= min_row; r--) {
                left_x = find_edge_binary_row(r, mid_position, 0);
                if (left_x >= 2) {
                    seed_left[0] = left_x; seed_left[1] = r;
                    result += LEFT_PTS; break;
                }
            }
        }
    }

    seed_state = result;
    // if (result == TWO_PTS) {
    //     if (width_base == 0) width_base = seed_right[0] - seed_left[0];
    //     mid_position = (seed_left[0] + seed_right[0]) / 2;
    // } else if (result == LEFT_PTS) {
    //     if (width_base != 0) mid_position = seed_left[0] + width_base / 2;
    // } else if (result == RIGHT_PTS) {
    //     if (width_base != 0) mid_position = seed_right[0] - width_base / 2;
    // }
}

/* ==================== 左右手沿边线跟踪 ==================== */

/* 方向表 */
static const int8 dir_front[4][2] = {{0,-1},{1,0},{0,1},{-1,0}};
static const int8 dir_frontleft[4][2] = {{-1,-1},{1,-1},{1,1},{-1,1}};
static const int8 dir_frontright[4][2] = {{1,-1},{1,1},{-1,1},{-1,-1}};

/**
 * @brief 左手沿边线跟踪（适配白色=255，黑色=0）
 * @param pts_array 输出轨迹点数组 [step][2]
 * @return 跟踪到的点数
 */
uint8 findline_lefthand_binary(uint8 pts_array[][2]) {
    uint8 step = 0, dir = 0, turn = 0, repeat = 0;
    uint8 x = seed_left[0];
    uint8 y = seed_left[1];
    uint8 prev_x = 0, prev_y = 0;   // 独立的上一个点记录

    while (step < UVC_W - 2 &&
           x > 2 && x < UVC_W - 2 &&
           y > 2 && y < UVC_H - 2 &&
           turn < 4) {

        // 防重复：检查是否回到上一个点（自身历史）
        if (step > 0 && x == prev_x && y == prev_y) {
            repeat++;
            if (repeat > 3) { step = 0; break; }
        } else {
            repeat = 0;
        }

        // 记录当前点作为下一次比较的“上一个点”
        prev_x = x;
        prev_y = y;

        int8 nx = x + dir_front[dir][0];
        int8 ny = y + dir_front[dir][1];
        int8 flx = x + dir_frontleft[dir][0];
        int8 fly = y + dir_frontleft[dir][1];

        uint8 front_val     = image_two_value[ny][nx];
        uint8 frontleft_val = image_two_value[fly][flx];

        if (front_val != 255) {          // 前方是黑色（背景）
            dir = (dir + 1) % 4;
            turn++;
        } else if (frontleft_val != 255) { // 前左是黑色
            x = nx; y = ny;
            pts_array[step][0] = x;
            pts_array[step][1] = y;
            step++; turn = 0;
        } else {                           // 前左也是白色，向前左走
            x = flx; y = fly;
            pts_array[step][0] = x;
            pts_array[step][1] = y;
            step++; turn = 0;
            dir = (dir + 3) % 4;
        }
    }
    seed_left[0] = x; seed_left[1] = y;
    return step;
}

/**
 * @brief 右手沿边线跟踪（适配白色=255，黑色=0）
 * @param pts_array 输出轨迹点数组 [step][2]
 * @return 跟踪到的点数
 */
uint8 findline_righthand_binary(uint8 pts_array[][2]) {
    uint8 step = 0, dir = 0, turn = 0, repeat = 0;
    uint8 x = seed_right[0];
    uint8 y = seed_right[1];
    uint8 prev_x = 0, prev_y = 0;   // 独立的上一个点记录

    while (step < UVC_W - 2 &&
           x > 2 && x < UVC_W - 2 &&
           y > 2 && y < UVC_H - 2 &&
           turn < 4) {

        // 防重复：检查是否回到上一个点（自身历史）
        if (step > 0 && x == prev_x && y == prev_y) {
            repeat++;
            if (repeat > 3) { step = 0; break; }
        } else {
            repeat = 0;
        }

        prev_x = x;
        prev_y = y;

        int8 nx = x + dir_front[dir][0];
        int8 ny = y + dir_front[dir][1];
        int8 frx = x + dir_frontright[dir][0];
        int8 fry = y + dir_frontright[dir][1];

        uint8 front_val      = image_two_value[ny][nx];
        uint8 frontright_val = image_two_value[fry][frx];

        if (front_val != 255) {           // 前方是黑色
            dir = (dir + 3) % 4;
            turn++;
        } else if (frontright_val != 255) { // 前右是黑色
            x = nx; y = ny;
            pts_array[step][0] = x;
            pts_array[step][1] = y;
            step++; turn = 0;
        } else {                           // 前右也是白色
            x = frx; y = fry;
            pts_array[step][0] = x;
            pts_array[step][1] = y;
            step++; turn = 0;
            dir = (dir + 1) % 4;
        }
    }
    seed_right[0] = x; seed_right[1] = y;
    return step;
}
