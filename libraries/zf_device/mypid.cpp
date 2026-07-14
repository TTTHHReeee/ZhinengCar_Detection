// #include "mypid.hpp"
// #include <cmath>
// #include "zf_common_headfile.hpp"
// int target_speed=0;



// int PID_V_L(int set_speed ,int speed)//pid控制电机转速

// {
//     volatile static int out;
//     volatile static int out_increment;
//     volatile static int ek,ek1;
//     // float kp=35,ki=3.7;
//   float kp=10,ki=0.2;
   
    
//     ek = set_speed - speed;
//     out_increment= (int)(kp*(ek-ek1) + ki*ek);
//     out+= out_increment;
//     ek1 = ek;
//     if(out>=8500)//限幅处理
//         {out=8500;}
//     else if(out<=-8500)
//         {out=-8500;}
//     return (int) out;
// }
// int PID_V_R(int set_speed ,int speed)//pid控制电机转速

// {
//     volatile static int out;
//     volatile static int out_increment;
//     volatile static int ek,ek1;
//     // float kp=35,ki=3.7;
//   float kp=10,ki=0.2;
//     /* if(Go==7)//正常运行状态使用的参数
//     {
//         //float P_L=30;
//         //float I_L=1.6;
//         kp = P_L;//一套pi足矣，速度拨动不会太大
//         ki = I_L;
//     }
//     else//发车阶段速度环要硬，不能超调晃动
//     {
//         kp = 20.0;//一套pi足矣，速度拨动不会太大
//         ki = 0.9;
//     } */
    
//     ek = set_speed - speed;
//     out_increment= (int)(kp*(ek-ek1) + ki*ek);
//     out+= out_increment;
//     ek1 = ek;
//     printf("PID_R: set=%d speed=%d ek=%d ek1=%d inc=%d out=%d\r\n",
//        set_speed, speed, ek, ek1, out_increment, out);
//     if(out>=8500)//限幅处理
//         {out=8500;}
//     else if(out<=-8500)
//        { out=-8500;}
//     return (int) out;
// }
