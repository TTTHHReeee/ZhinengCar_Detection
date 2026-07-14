#ifndef _MY_IMAGE_HPP__
#define _MY_IMAGE_HPP__


/*宏定义 */
#define UVC_H 60
#define UVC_W 80
#define IMG_WHITE 255
#define IMG_BLACK 0
#define start_H 0
#define end_H 90
/*函数声明*/
int my_adapt_threshold(uint8 *image, uint16 col, uint16 row) ;
void Image_Binarization(int threshold);
void Image_Binarization2(int threshold);
void Longest_White_Column(uint8 Island_Flag_1);
void Get_Mid();

void shizibuxian(void);
void Identify();
void Island_Detect();
void banmaxian(void);
void chujie();
void Leftzhuantou(void);
void Rightzhuantou(void);
void zhuantou();
/*变量声明*/
extern uint8 image_two_value[UVC_H][UVC_W];
 extern uint8_t* gray_image ;

extern int threshold;
extern int8 Mid_Line[UVC_H];
extern int8 Left_Line [UVC_H];       
 extern int8 Right_Line[UVC_H];
 extern int Road_Wide[UVC_H];
extern int8_t Cross_Flag;
extern int8_t Island_State;
extern int8_t banmaxianFlag;
extern int8_t Chujie_Flag;
extern int8 Left_Island_Flag; //左右环岛标志
extern int8 Right_Island_Flag;//左右环岛标志
extern int8 DownRightFlag;
extern uint8 Right_Up_Point_finish_flag ;
extern uint8 Left_Up_Point_finish_flag ;
extern uint8 Right_Down_Point_finish_flag;
extern uint8 Left_Down_Point_finish_flag ;
extern uint8 Right_Down_Point[2];  // 0:x坐标(列), 1:y坐标(行)
extern uint8 Right_Up_Point[2];                        
extern uint8 Left_Down_Point[2];
extern uint8 Left_Up_Point[2];
extern int8 LeftzhuantouFlag;
extern int8 RightzhuantouFlag;
extern uint8 Standard_Road_Wide[UVC_H];
extern int Search_Stop_Line;
extern int8  ncnn_ctrl_flag;
#endif