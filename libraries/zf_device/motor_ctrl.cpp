#include "zf_common_typedef.hpp"
#include "motor_ctrl.hpp"


motor_typedef motor;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      编码器积分
//  @param      
//  @return     void
//  @note       单纯的编码器积分
//-------------------------------------------------------------------------------------------------------------------
void motor_control(int16 encoder_left,int16 encoder_right ,int8 Island_State,
    uint8 start_flag,int8 banmaxianFlag,int8 zhuantou_l,int8 zhuantou_r,int8 ncnn_id)
{   
    //motor.L+=(encoder_left+encoder_right)*1.0/2*motor.t;
    
if(Island_State==2){
    motor.huandao_L2+=(encoder_left+encoder_right)*1.0/2*motor.t;
}
else{
    motor.huandao_L2=0;
}

if(Island_State==6||Island_State==7||Island_State==8)
{
    motor.huandao_L6+=(encoder_left+encoder_right)*1.0/2*motor.t;
}
else
{
    motor.huandao_L6=0;
}

if(start_flag==1)
{

motor.banmaxian_start+=(encoder_left+encoder_right)*1.0/2*motor.t;

}

if(start_flag==0)
{
    motor.banmaxian_start=0;
}
if(banmaxianFlag==1)
{
motor.banmaxian_end+=(encoder_left+encoder_right)*1.0/2*motor.t;

}
if(zhuantou_l==2){
    motor.zuozhuantou_L+=(encoder_left+encoder_right)*1.0/2*motor.t;
}else{
    motor.zuozhuantou_L=0;
}
if(zhuantou_r==2){
    motor.youzhuantou_L+=(encoder_left+encoder_right)*1.0/2*motor.t;
}else{
    motor.youzhuantou_L=0;
}

if(ncnn_id!=0)
{
    motor.ncnn_L+=(abs(encoder_left)+abs(encoder_right))*1.0/2*motor.t;
}
else{
    motor.ncnn_L=0;
}
}


