#ifndef _MOTOR_CTRL_HPP__
#define _MOTOR_CTRL_HPP__

typedef struct{
float huandao_R2;
float huandao_R4;
float huandao_R6;
float huandao_L2;
float huandao_L4;
float huandao_L6;
float banmaxian_start;
float banmaxian_end;
float t=0.005;
float L;
float zuozhuantou_L;
float youzhuantou_L;
float podao_L;
float ncnn_L;

}motor_typedef;
extern motor_typedef motor;
void motor_control(int16 encoder_left,int16 encoder_right ,int8 Island_State,uint8 start_flag,int8 banmaxianFlag,int8 zhuantou_l,int8 zhuantou_r,int8 ncnn_id);

#endif
