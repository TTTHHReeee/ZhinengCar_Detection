#include "zf_common_typedef.hpp"
#include "my_imu.hpp"
#include <cmath>
#include "my_image.hpp"



#define Gyro_Range_Scale 14.3
#define DEG2RAD 3.14/180
#define RAD2DEG 180/3.14


ICMDatatypedef icmdata={0};
 

void AHRS_update(float Ax,float Ay,float Az,float Gx,float Gy,float Gz){
    float gradient_length_base=0.0003f;   //梯度下降基础步长
    float gradient_legth;               //梯度下降迭代步长
    float Op0,Op1,Op2,Op3;             //梯度下降算子
    float a;                           //运动状态估计因子
    static float Q0=0,Q1=0, Q2=0,Q3=1;    //最终解算出可用的四元数
    float ex,ey,ez;                   //误差计算
    float vx,vy,vz;                   //目标加速度
    float correct_x,correct_y,correct_z; //三轴陀螺仪修正量
    float ki=0.001,kp=0.001;            //pi控制器参数

    static float Gx_last,Gy_last,Gz_last;  //陀螺仪数据
    static float Gx_now,Gy_now,Gz_now;

    float Dt=0.005f;         //解算周期 s

    double normal_rate;       //归一化系数
    float S10,S11,S12,S13;
    float S20,S21,S22,S23;
    float S30,S31,S32,S33;
    float S40,S41,S42,S43;

    ki=0.001*Dt;
    kp=0.01;
   
    normal_rate=sqrt(Ax*Ax+Ay*Ay+Az*Az);
    Ax=Ax/normal_rate;
    Ay=Ay/normal_rate;
    Az=Az/normal_rate;

    //陀螺仪数据转换为弧度
    Gx_last=Gx_now;
    Gy_last=Gy_now;
    Gz_last=Gz_now;

    Gx_now=Gx/Gyro_Range_Scale;
    Gy_now=Gy/Gyro_Range_Scale;
    Gz_now=Gz/Gyro_Range_Scale;
    Gx_now*=DEG2RAD;
    Gy_now*=DEG2RAD;
    Gz_now*=DEG2RAD;

    //梯度算子计算
    Op0 = 4.0f*Q0*Q2*Q2+2.0f*Q2*Ax+4.0f*Q0*Q1*Q1-2.0f*Q1*Ay;
    Op1 = 4.0f*Q1*Q3*Q3-2.0f*Q3*Ax+4.0f*Q1*Q0*Q0-2.0f*Q0*Ay-4.0f*Q1+8.0f*Q1*Q1*Q1+8.0f*Q1*Q2*Q2+4.0f*Q1*Az;
    Op2 = 4.0f*Q2*Q0*Q0+2.0f*Q0*Ax+4.0f*Q2*Q3*Q3-2.0f*Q3*Ay-4.0f*Q2+8.0f*Q2*Q1*Q1+8.0f*Q2*Q2*Q2+4.0f*Q2*Az;
    Op3 = 4.0f*Q3*Q1*Q1-2.0f*Q1*Ax+4.0f*Q3*Q2*Q2-2.0f*Q2*Ay;

    //梯度算子归一化处理
    normal_rate=sqrt(Op0*Op0+Op1*Op1+Op2*Op2+Op3*Op3);
    Op0=Op0/normal_rate;
    Op1=Op1/normal_rate;
    Op2=Op2/normal_rate;
    Op3=Op3/normal_rate;

    //迭代步长计算
    a=1-1.0f/(sqrt(Ax*Ax+Ay*Ay+Az*Az));  //运动状态影响因子计算   计算非重力加速度
    //自适应步长/******************************/
    if(a<=0.8f){
    gradient_legth=gradient_length_base;
    }else{
    gradient_legth=gradient_length_base+a*Dt*sqrt(Gx_now*Gx_now+Gy_now*Gy_now+Gz_now*Gz_now);
    }

    //目标加速度计算
    vx=2.0f*(Q1*Q3-Q0*Q2);
    vy=2.0f*(Q0*Q1+Q2*Q3);
    vz=Q0*Q0-Q1*Q1-Q2*Q2+Q3*Q3;

    //误差计算
    ex=Ay*vz-Az*vy;
    ey=Az*vx-Ax*vz;
    ez=Ax*vy-Ay*vx;

    //误差积分
    static float Inter_x,Inter_y,Inter_z;
    Inter_x+=ex*ki;
    Inter_y+=ey*ki;
    Inter_z+=ez*ki;

    //陀螺仪修正量计算
    correct_x=ex*kp+Inter_x;
    correct_y=ey*kp+Inter_y;
    correct_z=ez*kp+Inter_z;

    //角速度修正
    Gx_now+=correct_x;
    Gy_now+=correct_y;
    Gz_now+=correct_z;

    //微分方程更新四元数   四阶龙格库塔法求解

    S10=-(Gx_last*Q1)/2.0f - (Gy_last*Q2)/2.0f - (Gz_last*Q3)/2.0f;
    S11=(Gx_last*Q0)/2.0f - (Gy_last*Q3)/2.0f - (Gz_last*Q2)/2.0f;
    S12=(Gx_last*Q3)/2.0f + (Gy_last*Q0)/2.0f - (Gz_last*Q1)/2.0f;
    S13=(Gy_last*Q1)/2.0f - (Gx_last*Q2)/2.0f + (Gz_last*Q0)/2.0f;


    S20=- (Gx_now/4 + Gx_last/4)*(Q1 + (Dt*S11)/2) - (Gy_now/4 + Gy_last/4)*(Q2 + (Dt*S12)/2) - (Gz_now/4 + Gz_last/4)*(Q3 + (Dt*S13)/2);
    S21= (Gx_now/4 + Gx_last/4)*(Q0 + (Dt*S10)/2) - (Gy_now/4 + Gy_last/4)*(Q3 + (Dt*S13)/2) - (Gz_now/4 + Gz_last/4)*(Q2 + (Dt*S12)/2);
    S22= (Gx_now/4 + Gx_last/4)*(Q3 + (Dt*S13)/2) + (Gy_now/4 + Gy_last/4)*(Q0 + (Dt*S10)/2) - (Gz_now/4 + Gz_last/4)*(Q1 + (Dt*S11)/2);
    S23= (Gy_now/4 + Gy_last/4)*(Q1 + (Dt*S11)/2) - (Gx_now/4 + Gx_last/4)*(Q2 + (Dt*S12)/2) + (Gz_now/4 + Gz_last/4)*(Q0 + (Dt*S10)/2);


    S30=- (Gx_now/4 + Gx_last/4)*(Q1 + (Dt*S21)/2) - (Gy_now/4 + Gy_last/4)*(Q2 + (Dt*S22)/2) - (Gz_now/4 + Gz_last/4)*(Q3 + (Dt*S23)/2);
    S31= (Gx_now/4 + Gx_last/4)*(Q0 + (Dt*S20)/2) - (Gy_now/4 + Gy_last/4)*(Q3 + (Dt*S23)/2) - (Gz_now/4 + Gz_last/4)*(Q2 + (Dt*S22)/2);
    S32= (Gx_now/4 + Gx_last/4)*(Q3 + (Dt*S23)/2) + (Gy_now/4 + Gy_last/4)*(Q0 + (Dt*S20)/2) - (Gz_now/4 + Gz_last/4)*(Q1 + (Dt*S21)/2);
    S33= (Gy_now/4 + Gy_last/4)*(Q1 + (Dt*S21)/2) - (Gx_now/4 + Gx_last/4)*(Q2 + (Dt*S22)/2) + (Gz_now/4 + Gz_last/4)*(Q0 + (Dt*S20)/2);


    S40=- (Gx_now*(Q1 + Dt*S31))/2 - (Gy_now*(Q2 + Dt*S32))/2 - (Gz_now*(Q3 + Dt*S33))/2;
    S41= (Gx_now*(Q0 + Dt*S30))/2 - (Gy_now*(Q3 + Dt*S33))/2 - (Gz_now*(Q2 + Dt*S32))/2;
    S42= (Gx_now*(Q3 + Dt*S33))/2 + (Gy_now*(Q0 + Dt*S30))/2 - (Gz_now*(Q1 + Dt*S31))/2;
    S43= (Gy_now*(Q1 + Dt*S31))/2 - (Gx_now*(Q2 + Dt*S32))/2 + (Gz_now*(Q0 + Dt*S30))/2;


    Q0 = Q0 + (-Q1 * Gx_now - Q2 * Gy_now - Q3 * Gz_now) * (0.5f * Dt);
    Q1 = Q1 + (Q0 * Gx_now + Q2 * Gz_now - Q3 * Gy_now) * (0.5f * Dt);
    Q2 = Q2 + (Q0 * Gy_now - Q1 * Gz_now + Q3 * Gx_now) * (0.5f * Dt);
    Q3 = Q3 + (Q0 * Gz_now + Q1 * Gy_now - Q2 * Gx_now) * (0.5f * Dt);







//  //梯度算子 再次对四元数进行修正
    Q0=Q0-gradient_legth*Op0;
    Q1=Q1-gradient_legth*Op1;
    Q2=Q2-gradient_legth*Op2;
    Q3=Q3-gradient_legth*Op3;

    //四元数 归一化
    normal_rate=sqrt(Q0*Q0+Q1*Q1+Q2*Q2+Q3*Q3);
    Q0=Q0/normal_rate;
    Q1=Q1/normal_rate;
    Q2=Q2/normal_rate;
    Q3=Q3/normal_rate;


    //磁力计数据有效  融合航向

    
    
 
  /*   if(Ramp_Flag!=0)
     {
        icmdata.Pitch= asin(2.0f * Q0* Q2-2.0f * Q1 * Q3) * RAD2DEG;
     } */
      
    if(Island_State==2||Island_State==3||Island_State==4||Island_State==5||Island_State==6||Island_State==7)
         {
    
    icmdata.huandao_yaw= atan2(2.0f * Q1 * Q2 + 2.0f * Q0* Q3, -2.0f * Q3* Q3 - 2.0f * Q2 * Q2 + 1.0f) * RAD2DEG;
  
    icmdata.huandao_yaw=icmdata.huandao_yaw-180;
//    if(icmdata.huandao_yaw>180) icmdata.huandao_yaw-=360;
//     if(icmdata.huandao_yaw<-180) icmdata.huandao_yaw+=360; 
            if(Left_Island_Flag==1)  
            {if(icmdata.huandao_yaw > 360.0f) {
                    icmdata.huandao_yaw -= 720.0f;  // 减去720度，确保在-360到360之间
                }
                if(icmdata.huandao_yaw < -360.0f) {
                    icmdata.huandao_yaw += 720.0f;  // 加上720度，确保在-360到360之间
                }
                icmdata.huandao_yaw+=360.0f;
            }
            if(Right_Island_Flag==1)
            {
            if(icmdata.huandao_yaw > 360.0f) {
                    icmdata.huandao_yaw -= 720.0f;  // 减去720度，确保在-360到360之间
                }
                if(icmdata.huandao_yaw < -360.0f) {
                    icmdata.huandao_yaw += 720.0f;  // 加上720度，确保在-360到360之间
                }

            }         

        }
         else
     {icmdata.huandao_yaw=0;
        Q0=0,Q1=0,Q2=0,Q3=1;
     }
  
}

