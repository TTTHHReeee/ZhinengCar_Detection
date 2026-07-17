
/*********************************************************************************************************************
* LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是LS2K0300 开源库的一部分
*
* LS2K0300 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-12-27        大W            first version
********************************************************************************************************************/

#include "zf_common_headfile.hpp"
#include "my_image.hpp"
#include "my_imu.hpp" 
#include <cmath>
#include "motor_ctrl.hpp"
#include "ncnn_demo.hpp" // 模型初始化与识别 逻辑
#include "findRedBox.hpp" //红色色块+roi框定 逻辑
#include "track_binary.hpp"

// 另外一端的IP地址
#define SERVER_IP "10.79.179.226"
// 端口号
#define PORT 8086


zf_device_ips200 ips200;        // 定义2寸IPS SPI屏幕的设备对象，用于屏幕的所有操作
ZfDeviceYUYV uvc_dev;         // 定义UVC免驱摄像头的设备对象，用于摄像头的所有操作
NcnnClassifier g_classifier;    // NCNN 分类器对象
//方向编码器
#define ENCODER_DIR_1_PATH           ZF_ENCODER_DIR_1
#define ENCODER_DIR_2_PATH           ZF_ENCODER_DIR_2

zf_driver_encoder encoder_dir_1(ENCODER_DIR_1_PATH);
zf_driver_encoder encoder_dir_2(ENCODER_DIR_2_PATH);
//drv8701
#define PWM_1_PATH        ZF_PWM_MOTOR_1
#define DIR_1_PATH        ZF_GPIO_MOTOR_1

#define PWM_2_PATH        ZF_PWM_MOTOR_2
#define DIR_2_PATH        ZF_GPIO_MOTOR_2

struct pwm_info drv8701e_pwm_1_info;
struct pwm_info drv8701e_pwm_2_info;

zf_driver_gpio  drv8701e_dir_1(DIR_1_PATH, O_RDWR);
zf_driver_gpio  drv8701e_dir_2(DIR_2_PATH, O_RDWR);
zf_driver_pwm   drv8701e_pwm_1(PWM_1_PATH);
zf_driver_pwm   drv8701e_pwm_2(PWM_2_PATH);

#define KEY_1_PATH        ZF_GPIO_KEY_1
#define KEY_2_PATH        ZF_GPIO_KEY_2
#define KEY_3_PATH        ZF_GPIO_KEY_3
#define KEY_4_PATH        ZF_GPIO_KEY_4

zf_driver_gpio  key_1(KEY_1_PATH, O_RDWR);
zf_driver_gpio  key_2(KEY_2_PATH, O_RDWR);
zf_driver_gpio  key_3(KEY_3_PATH, O_RDWR);
zf_driver_gpio  key_4(KEY_4_PATH, O_RDWR);

#define BEEP_PATH        ZF_GPIO_BEEP

zf_driver_gpio  beep(BEEP_PATH, O_RDWR);

int16 duty = 0;
int Speed_L=0;
int Speed_R=0;
int R_duty=0;
int L_duty=0;
bool dir = true;
int8_t stop_flag=0;
uint8 start_flag=0;
int8_t tuxiang_flag=0;
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR1_PWM_DUTY_MAX    (drv8701e_pwm_1_info.duty_max)       
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。 
#define MOTOR2_PWM_DUTY_MAX    (drv8701e_pwm_2_info.duty_max)        

#define MAX_DUTY        (30 )   // 最大 MAX_DUTY% 占空比
//定时器
zf_driver_pit pit_timer;
//用户变量
uint16 count = 0;               // 帧计数变量，用于控制图像显示模式切换的频率
uint8  show_flag = 0;  

int encoder_left;
int encoder_right;
float out_err=0;
float motor_err=0;
int encoder_flag=0;


//int target_speed=210;
 int target_speed_1=220;//220
float imu=0;
int8 image_ips=0;
#define ESC_PATH        ZF_PWM_ESC_1

struct pwm_info esc_info;
zf_driver_pwm   esc_pwm(ESC_PATH);
uint16 duty_S = 0;
int ncnn_id=0;
int frame_counter=1;
int dajin_counter=0;
int8 detected = 0;
 int8 ncnn_ctrl_id=0;

 int8 ncnn_stop=0;
 //int16 ncnn_ctrl_S=50;
cv::Rect red_rect;
cv::Point red_center;
cv::Point bottom_point;








// const int8 Weight[60]=//误差权重数组
//  {       0,0,0,0,0,0,0,0,0,0,
//          0,0,0,0,0,0,0,0,0,0,
//          0,0,0,0,0,0,0,
//          10,15,20,15,10,0,0,
//          0,0,0,
//          0,0,0,
//          0,0,0,0,0,0,0,0,0,0,
//          0,0,0,0,0,0,0,0,0,0,
         
         
         
         
//          };
         const uint8 Weight[60]=//误差权重数组200
 {       0,0,0,0,0,0,0,0,0,
         0,0,0,0,0,0,0,0,0,
         0,0,0,
         10,15,20,15,10,0,0,
         0,0,
         0,
         0,0,0,0,0,0,0,0,0,0,
         0,0,0,0,0,0,0,0,0,0,
         
         
         
         
        };
const int8 Weight_ncnn[60]=//误差权重数组
 {       0,0,0,0,0,0,0,0,0,0,
         0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
         0,0,0,0,0,0,0,0,
         10,15,20,15,10,
         0,0,0,0,0,0,0,0,0,
         0,0,0,0,0,0,0,0,0,0,
  };    
//-------------------------------------------------------------------------------------------------------------------
// 函数简介 全局变量-IMU六轴/九轴传感器原始数据存储变量
// 参数说明 无
// 返回参数 无
// 使用示例 直接赋值与读取，均为int16整型原始采集值
// 备注信息 acc:三轴加速度  gyro:三轴陀螺仪  mag:三轴磁力计(仅963RA有数据)
//-------------------------------------------------------------------------------------------------------------------
int16 imu_acc_x,imu_acc_y,imu_acc_z;
int16 imu_gyro_x,imu_gyro_y,imu_gyro_z;
int16 imu_mag_x,imu_mag_y,imu_mag_z;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 全局变量-定义IMU传感器设备对象
// 参数说明 无
// 返回参数 无
// 使用示例 imu_dev.init(); imu_dev.get_acc_x();
// 备注信息 封装IMU初始化/数据读取/设备类型识别等所有相关属性和成员函数
//-------------------------------------------------------------------------------------------------------------------
zf_device_imu imu_dev;

/*********示波器代码**********/
uint8 image_copy[UVC_H][UVC_W];
//-------------------------------------------------------------------------------------------------------------------
// 函数简介 全局对象-TCP客户端设备对象
// 参数说明 无
// 返回参数 无
// 使用示例 tcp_client_dev.init(SERVER_IP, PORT);
// 备注信息 封装TCP客户端的初始化、收发数据等成员函数
//-------------------------------------------------------------------------------------------------------------------
zf_driver_tcp_client tcp_client_dev;

uint32  read_len = 0;
uint8   recv_buff[1024];
uint8   temp_str[] = "seekfree this is tcp_client demo.\r\n";
uint8   read_str[] = "read data:\r\n";

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 TCP发送数据 全局包装函数
// 参数说明 buf 要发送的数据缓冲区指针, len 要发送的数据字节长度
// 返回参数 uint32 实际成功发送的字节数
// 使用示例 供seekfree_assistant_interface_init调用，无需手动调用
// 备注信息 封装tcp_client_dev.send_data成员函数，适配普通函数指针格式要求
//-------------------------------------------------------------------------------------------------------------------
uint32 tcp_send_wrap(const uint8 *buf, uint32 len)
{
    return tcp_client_dev.send_data(buf, len);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 TCP接收数据 全局包装函数
// 参数说明 buf 接收数据的缓冲区指针, len 最大可接收的字节长度
// 返回参数 uint32 实际成功接收的字节数
// 使用示例 供seekfree_assistant_interface_init调用，无需手动调用
// 备注信息 封装tcp_client_dev.read_data成员函数，适配普通函数指针格式要求
//-------------------------------------------------------------------------------------------------------------------
uint32 tcp_read_wrap( uint8 *buf, uint32 len)
{
    return tcp_client_dev.read_data(buf, len);
}

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

void cleanup()
{
    // 需要先停止定时器线程，后面才能稳定关闭电机，电调，舵机等
    pit_timer.stop();
    printf("程序异常退出，执行清理操作\n");
    // 关闭电机
    drv8701e_pwm_1.set_duty(0);   
    drv8701e_pwm_2.set_duty(0);  
    esc_pwm.set_duty(0);  
}


/*********示波器代码**********/


int PID_V_L(int set_speed ,int speed)//pid控制电机转速

{
    volatile static int out;
    volatile static int out_increment;
    volatile static int ek,ek1;
    // float kp=35,ki=3.7;
  float kp=90,ki=15;
//     if((ncnn_ctrl_flag==1||ncnn_ctrl_flag==3||detected==2)&&ncnn_ctrl_flag!=2)
//   {
//     kp=90;ki=25;
//   }
    
    ek = set_speed - speed;
    out_increment= (int)(kp*(ek-ek1) + ki*ek);
    out+= out_increment;
    ek1 = ek;
   
    
    if(out>=9999)//限幅处理
        {out=9999;}
    else if(out<=-9999)
        {out=-9999;}
    //      if(ncnn_ctrl_flag!=0)
    // {printf("PID_L: set=%d speed=%d ek=%d ek1=%d inc=%d out=%d\n",
    //    set_speed, speed, ek, ek1, out_increment, out);}
    return (int) out;
}
int PID_V_R(int set_speed ,int speed)//pid控制电机转速

{
    volatile static int out;
    volatile static int out_increment;
    volatile static int ek,ek1;
    // float kp=35,ki=3.7;
  float kp=90,ki=15;
//   if(ncnn_ctrl_flag==1||ncnn_ctrl_flag==3)
//   {
//     kp=90;ki=25;
//   }
    /* if(Go==7)//正常运行状态使用的参数
    {
        //float P_L=30;
        //float I_L=1.6;
        kp = P_L;//一套pi足矣，速度拨动不会太大
        ki = I_L;
    }
    else//发车阶段速度环要硬，不能超调晃动
    {
        kp = 20.0;//一套pi足矣，速度拨动不会太大
        ki = 0.9;
    } */
    
    ek = set_speed - speed;
    out_increment= (int)(kp*(ek-ek1) + ki*ek);
    out+= out_increment;
    ek1 = ek;
    
  
    if(out>=9999)//限幅处理
        {out=9999;}
    else if(out<=-9999)
       { out=-9999;}
    //    if(ncnn_ctrl_flag!=0)
    // {printf("PID_R: set=%d speed=%d ek=%d ek1=%d inc=%d out=%d\n",
    //    set_speed, speed, ek, ek1, out_increment, out);}
    return (int) out;
}

float PID_motor(float err)//pid控制差速转向

{
    volatile static float out;
    volatile static float integral;
    volatile static float ek,ek1;
    

        float kp=err*err*0.013+8;float kd=0;float k=0.33;
        if(kp>=13)kp=13;
        // if(ncnn_ctrl_flag==1||ncnn_ctrl_flag==3)
        // {
        //     kp=15;
        // }
        ek1 = ek;
        ek = err;
        integral+=ek;     
     
      
     out=kp*ek+kd*(ek-ek1)-(imu_gyro_z/16.4)*k;
	
    if(out>=125)out=125;
    if(out<=-125)out=-125;
    // if(ncnn_ctrl_flag==1||ncnn_ctrl_flag==3)
    // {
    // if(out>=80)out=80;
    // if(out<=-80)out=-80;
    // }
    return out;
}

// float PID_motor(float err)//pid控制差速转向

// {
//     volatile static float out;
//     volatile static float integral;
//     volatile static float ek,ek1;
    
//     // if((ncnn_ctrl_flag==1||ncnn_ctrl_flag==3)&&motor.ncnn_L<=20)
//     //     {
//     //     float kp=err*err*0+6;float kd=0;float k=0.25;
//     //    // if(kp>=7)kp=7;
        
//     //     ek1 = ek;
//     //     ek = err;
//     //     integral+=ek;   
     
      
//     //  out=kp*ek+kd*(ek-ek1)-(imu_gyro_z/16.4)*k;
	
//     // if(out>=70)out=70;
//     // if(out<=-70)out=-70;
//     // }
//     // else 
//     // {
//         float kp=err*err*0.01+5;float kd=0;float k=0.25;
//         if(kp>=6)kp=6;
        
//         ek1 = ek;
//         ek = err;
//         integral+=ek;   
     
      
//      out=kp*ek+kd*(ek-ek1)-(imu_gyro_z/16.4)*k;
// 	if(out>=95)out=95;
//     if(out<=-95)out=-95;
//     //  }
//     return out;
// }
float ncnn_get_error()
{  
int16 weight_count=0;
   float error=0;
   int i;
    //常规误差
        
        if(ncnn_ctrl_flag==1)//右拐
        {
            for(i=UVC_H-1;i>=0;i--)//常规误差计算
        {
        error+=(UVC_W/2-Right_Line[i])*Weight_ncnn[i];//右移1位，等效除2
            weight_count+=Weight_ncnn[i];
            //*Weight[i-20]
        }
        }
        else if(ncnn_ctrl_flag==3)//左拐
        {
             for(i=UVC_H-1;i>=0;i--)//常规误差计算
        {
        error+=(UVC_W/2-Left_Line[i])*Weight_ncnn[i];//右移1位，等效除2
            weight_count+=Weight_ncnn[i];
            //*Weight[i-20]
        }
         }
        error=error/weight_count;
        
        return error;
        
    }
int8_t Shape=-1;//1是左拐，2是直行，3是右拐 
// void ncnn_ctrl()
//  {     
    
//    if(Shape=-1&&ncnn_ctrl_id!=0)
//     {if(Search_Stop_Line>=50&&abs(out_err)<=5)//必定是直道，或者斜率小的s弯
//     {
//         Shape=2;
//     }
//     else if(Search_Stop_Line<50&&out_err>5)//大概率左拐
//     {
//         Shape=1;
//     }
//      else if(Search_Stop_Line<50&&out_err<-5)//大概率右拐
//     {
//         Shape=3;
//     }
//     else//没有明显的赛道特征，比如弯道但是误差小，或者小s弯白列长但是误差稍微大一点，或者是出弯的时候
//     {
//         Shape=0;
//     }
//         printf("%d.\r\n",Shape);
//     }
//   if(ncnn_ctrl_id!=0)
//  {   
//     if(Shape==2)//直行
// {
//     if(ncnn_ctrl_id==3)//左绕
// {   ncnn_ctrl_flag=3;
//     out_err=ncnn_get_error();
//     // if(motor.ncnn_L<=15)
//     // {out_err+=70;}

//     if(motor.ncnn_L>=70&&abs(out_err)<=10)
    
//     {
//         ncnn_ctrl_flag=0;
//         detected=0;
//         ncnn_ctrl_id=0;
//         Shape=-1;
//     }
    
// }
// else if(ncnn_ctrl_id==2)
// {   
//     ncnn_ctrl_flag=2;

//     if(motor.ncnn_L>=30)
    
//     {
//         ncnn_ctrl_flag=0;
//         detected=0;
//         ncnn_ctrl_id=0;
//         Shape=-1;
//     }
   

// }
// else if(ncnn_ctrl_id==1)
//     {
//         ncnn_ctrl_flag=1;
//           out_err=ncnn_get_error();
//         //   if(motor.ncnn_L<=15)
//         // {out_err-=70;}
//         if(motor.ncnn_L>=70&&abs(out_err)<=10)
        
//         {
//             ncnn_ctrl_flag=0;
//             detected=0;
//             ncnn_ctrl_id=0;
//             Shape=-1;
//         }
        
//     }
// }
// if(Shape==1)//左拐
// {
//     if(ncnn_ctrl_id==3)//左绕
// {   ncnn_ctrl_flag=3;
//     out_err=ncnn_get_error();
//     // if(motor.ncnn_L<=15)
//     // {out_err+=70;}

//     if(motor.ncnn_L>=70&&abs(out_err)<=10)
    
//     {
//         ncnn_ctrl_flag=0;
//         detected=0;
//         ncnn_ctrl_id=0;
//         Shape=-1;
//     }
    
// }
// else if(ncnn_ctrl_id==2)
// {   
//     ncnn_ctrl_flag=2;

//     if(motor.ncnn_L>=30)
    
//     {
//         ncnn_ctrl_flag=0;
//         detected=0;
//         ncnn_ctrl_id=0;
//         Shape=-1;
//     }
   

// }
// else if(ncnn_ctrl_id==1)
//     {
//         ncnn_ctrl_flag=1;
//           out_err=ncnn_get_error();
//         //   if(motor.ncnn_L<=15)
//         // {out_err-=70;}
//         if(motor.ncnn_L>=70&&abs(out_err)<=10)
        
//         {
//             ncnn_ctrl_flag=0;
//             detected=0;
//             ncnn_ctrl_id=0;
//             Shape=-1;
//         }
        
//     }
// }
// if(Shape==3)//右拐
// {
//     if(ncnn_ctrl_id==3)//左绕
// {   ncnn_ctrl_flag=3;
//     out_err=ncnn_get_error();
//     // if(motor.ncnn_L<=15)
//     // {out_err+=70;}

//     if(motor.ncnn_L>=70&&abs(out_err)<=10)
    
//     {
//         ncnn_ctrl_flag=0;
//         detected=0;
//         ncnn_ctrl_id=0;
//         Shape=-1;
//     }
    
// }
// else if(ncnn_ctrl_id==2)
// {   
//     ncnn_ctrl_flag=2;

//     if(motor.ncnn_L>=30)
    
//     {
//         ncnn_ctrl_flag=0;
//         detected=0;
//         ncnn_ctrl_id=0;
//         Shape=-1;
//     }
   

// }
// else if(ncnn_ctrl_id==1)
//     {
//         ncnn_ctrl_flag=1;
//           out_err=ncnn_get_error();
//         //   if(motor.ncnn_L<=15)
//         // {out_err-=70;}
//         if(motor.ncnn_L>=70&&abs(out_err)<=10)
        
//         {
//             ncnn_ctrl_flag=0;
//             detected=0;
//             ncnn_ctrl_id=0;
//             Shape=-1;
//         }
        
//     }
// }
//     if(Shape==0)//大概率是小幅度弯道
// {
//     if(ncnn_ctrl_id==3)//左绕
// {   ncnn_ctrl_flag=3;
//     out_err=ncnn_get_error();
//     // if(motor.ncnn_L<=15)
//     // {out_err+=70;}

//     if(motor.ncnn_L>=70&&abs(out_err)<=10)
    
//     {
//         ncnn_ctrl_flag=0;
//         detected=0;
//         ncnn_ctrl_id=0;
//         Shape=-1;
//     }
    
// }
// else if(ncnn_ctrl_id==2)
// {   
//     ncnn_ctrl_flag=2;

//     if(motor.ncnn_L>=30)
    
//     {
//         ncnn_ctrl_flag=0;
//         detected=0;
//         ncnn_ctrl_id=0;
//         Shape=-1;
//     }
   

// }
// else if(ncnn_ctrl_id==1)
//     {
//         ncnn_ctrl_flag=1;
//           out_err=ncnn_get_error();
//         //   if(motor.ncnn_L<=15)
//         // {out_err-=70;}
//         if(motor.ncnn_L>=70&&abs(out_err)<=10)
        
//         {
//             ncnn_ctrl_flag=0;
//             detected=0;
//             ncnn_ctrl_id=0;
//             Shape=-1;
//         }
        
//     }
// }

//  }

// } 
// void ncnn_ctrl_motor()
// {   if(ncnn_ctrl_flag==1||ncnn_ctrl_flag==3)
//    { if(Shape==1)//赛道左
//   {if(ncnn_ctrl_flag==3) //左拐
//    {if(motor.ncnn_L<=10)
//          {motor_err+=20;}
//    }   
//    if(ncnn_ctrl_flag==1) //右拐
//    {if(motor.ncnn_L<=10)
//          {motor_err-=20;}
//    }}
      
//    else if(Shape==2)//赛道直
//   {if(ncnn_ctrl_flag==3) //左拐
//    {if(motor.ncnn_L<=10)
//          {motor_err+=20;}
//    }   
//    if(ncnn_ctrl_flag==1) //右拐
//    {if(motor.ncnn_L<=10)
//          {motor_err-=20;}
//    }} 
//    else if(Shape==3)//赛道右
//   {if(ncnn_ctrl_flag==3) //左拐
//    {if(motor.ncnn_L<=10)
//          {motor_err+=20;}
//    }   
//    if(ncnn_ctrl_flag==1) //右拐
//    {if(motor.ncnn_L<=10)
//          {motor_err-=20;}
//    }}
//     else if(Shape==0)//大概率曲率小的弯道
//   {if(ncnn_ctrl_flag==3) //左拐
//    {if(motor.ncnn_L<=10)
//          {motor_err+=20;}
//    }   
//    if(ncnn_ctrl_flag==1) //右拐
//    {if(motor.ncnn_L<=10)
//          {motor_err-=20;}
//    }} }

// } 


void ncnn_ctrl()
 {     
    
   
  if(ncnn_ctrl_id!=0)
 {   
    if(Shape==2)//直行
{
    if(ncnn_ctrl_id==3)//左绕
{   ncnn_ctrl_flag=3;
    out_err=ncnn_get_error();
    // if(motor.ncnn_L<=15)
    // {out_err+=70;}

    if(motor.ncnn_L>=70)
    
    {
        ncnn_ctrl_flag=0;
        detected=0;
        ncnn_ctrl_id=0;
        Shape=-1;
    }
    
}
else if(ncnn_ctrl_id==2)
{   
    ncnn_ctrl_flag=2;

    if(motor.ncnn_L>=30)
    
    {
        ncnn_ctrl_flag=0;
        detected=0;
        ncnn_ctrl_id=0;
        Shape=-1;
    }
   

}
else if(ncnn_ctrl_id==1)
    {
        ncnn_ctrl_flag=1;
          out_err=ncnn_get_error();
        //   if(motor.ncnn_L<=15)
        // {out_err-=70;}
        if(motor.ncnn_L>=70)
        
        {
            ncnn_ctrl_flag=0;
            detected=0;
            ncnn_ctrl_id=0;
            Shape=-1;
        }
        
    }
}
else if(Shape==1)//左拐
{
    if(ncnn_ctrl_id==3)//左绕
{   ncnn_ctrl_flag=3;
    out_err=ncnn_get_error();
    // if(motor.ncnn_L<=15)
    // {out_err+=70;}

    if(motor.ncnn_L>=70)
    
    {
        ncnn_ctrl_flag=0;
        detected=0;
        ncnn_ctrl_id=0;
        Shape=-1;
    }
    
}
else if(ncnn_ctrl_id==2)
{   
    ncnn_ctrl_flag=2;

    if(motor.ncnn_L>=30)
    
    {
        ncnn_ctrl_flag=0;
        detected=0;
        ncnn_ctrl_id=0;
        Shape=-1;
    }
   

}
else if(ncnn_ctrl_id==1)
    {
        ncnn_ctrl_flag=1;
          out_err=ncnn_get_error();
        //   if(motor.ncnn_L<=15)
        // {out_err-=70;}
        if(motor.ncnn_L>=70)
        
        {
            ncnn_ctrl_flag=0;
            detected=0;
            ncnn_ctrl_id=0;
            Shape=-1;
        }
        
    }
}

else if(Shape==3)//右拐
{
    if(ncnn_ctrl_id==3)//左绕
{   ncnn_ctrl_flag=3;
    out_err=ncnn_get_error();
    // if(motor.ncnn_L<=15)
    // {out_err+=70;}

    if(motor.ncnn_L>=70)
    
    {
        ncnn_ctrl_flag=0;
        detected=0;
        ncnn_ctrl_id=0;
        Shape=-1;
    }
    
}
else if(ncnn_ctrl_id==2)
{   
    ncnn_ctrl_flag=2;

    if(motor.ncnn_L>=30)
    
    {
        ncnn_ctrl_flag=0;
        detected=0;
        ncnn_ctrl_id=0;
        Shape=-1;
    }
   

}
else if(ncnn_ctrl_id==1)
    {
        ncnn_ctrl_flag=1;
          out_err=ncnn_get_error();
        //   if(motor.ncnn_L<=15)
        // {out_err-=70;}
        if(motor.ncnn_L>=70)
        
        {
            ncnn_ctrl_flag=0;
            detected=0;
            ncnn_ctrl_id=0;
            Shape=-1;
        }
        
    }
}
    if(Shape==0)//大概率是小幅度弯道
{
    if(ncnn_ctrl_id==3)//左绕
{   ncnn_ctrl_flag=3;
    out_err=ncnn_get_error();
    // if(motor.ncnn_L<=15)
    // {out_err+=70;}

    if(motor.ncnn_L>=70)
    
    {
        ncnn_ctrl_flag=0;
        detected=0;
        ncnn_ctrl_id=0;
        Shape=-1;
    }
    
}
else if(ncnn_ctrl_id==2)
{   
    ncnn_ctrl_flag=2;

    if(motor.ncnn_L>=30)
    
    {
        ncnn_ctrl_flag=0;
        detected=0;
        ncnn_ctrl_id=0;
        Shape=-1;
    }
   

}
else if(ncnn_ctrl_id==1)
    {
        ncnn_ctrl_flag=1;
          out_err=ncnn_get_error();
        //   if(motor.ncnn_L<=15)
        // {out_err-=70;}
        if(motor.ncnn_L>=70)
        
        {
            ncnn_ctrl_flag=0;
            detected=0;
            ncnn_ctrl_id=0;
            Shape=-1;
        }
        
    }
}

 }

} 
void ncnn_ctrl_motor()
{   if(ncnn_ctrl_flag==1||ncnn_ctrl_flag==3)
   { if(Shape==1)//赛道左
  {if(ncnn_ctrl_flag==3) //左拐
   {if(motor.ncnn_L<=10)
         {motor_err+=20;}
   }   
   if(ncnn_ctrl_flag==1) //右拐
   {if(motor.ncnn_L<=10)
         {motor_err-=20;}
   }}
      
   else if(Shape==2)//赛道直
  {if(ncnn_ctrl_flag==3) //左拐
   {if(motor.ncnn_L<=10)
         {motor_err+=40;}
   }   
   if(ncnn_ctrl_flag==1) //右拐
   {if(motor.ncnn_L<=10)
         {motor_err-=40;}
   }} 
   else if(Shape==3)//赛道右
  {if(ncnn_ctrl_flag==3) //左拐
   {if(motor.ncnn_L<=10)
         {motor_err+=20;}
   }   
   if(ncnn_ctrl_flag==1) //右拐
   {if(motor.ncnn_L<=10)
         {motor_err-=20;}
   }}
    else if(Shape==0)//大概率曲率小的弯道
  {if(ncnn_ctrl_flag==3) //左拐
   {if(motor.ncnn_L<=10)
         {motor_err+=20;}
   }   
   if(ncnn_ctrl_flag==1) //右拐
   {if(motor.ncnn_L<=10)
         {motor_err-=20;}
   }} }

} 

/********按键滤波********/

// 定义4个按键的滤波器和上次状态
static uint8_t key_filter[4] = {0, 0, 0, 0};
static uint8_t key_last[4] = {0, 0, 0, 0};
static uint8_t key_pressed[4] = {0, 0, 0, 0};  // 按下事件标志

// 按键扫描函数
void process_keys(void)
{
    // 读取4个按键电平
    uint8_t key_levels[4];
    key_levels[0] = (key_1.get_level() == 1);
    key_levels[1] = (key_2.get_level() == 1);
    key_levels[2] = (key_3.get_level() == 1);
    key_levels[3] = (key_4.get_level() == 1);
    
    for(int i = 0; i < 4; i++) {
        // 移位滤波：将新电平移入滤波器的低位
        key_filter[i] = (key_filter[i] << 1) | key_levels[i];
        
        // 检测上升沿：连续3次为1，且上次状态为0
        if(((key_filter[i] & 0x07) == 0x07) && (key_last[i] == 0)) {
            key_pressed[i] = 1;  // 设置按下事件标志
        }
        
        // 更新稳定状态
        if((key_filter[i] & 0x07) == 0x07) {  // 稳定按下
            key_last[i] = 1;
        } else if((key_filter[i] & 0x07) == 0x00) {  // 稳定释放
            key_last[i] = 0;
        }
    }
    
    // 处理按键事件
    if(key_pressed[0]) {  // key1
       key_pressed[0] = 0;
       start_flag=!start_flag;
       banmaxianFlag=0;
       Chujie_Flag=0;
       Island_State=0;
       Left_Island_Flag=0; //左右环岛标志
       Right_Island_Flag=0; 
       LeftzhuantouFlag=0;
       RightzhuantouFlag=0; 
       ncnn_ctrl_flag=0;
       ncnn_ctrl_id=0;
       ncnn_stop=0;      
    }
    
    if(key_pressed[1]) {  // key2
        key_pressed[1] = 0;
         tuxiang_flag=!tuxiang_flag;
       
    }
    
    if(key_pressed[2]) {  // key3
        key_pressed[2] = 0;
        image_ips=!image_ips;
        ips200.clear();    
        
        // 添加key3的处理逻辑
    }
    
    if(key_pressed[3]) {  // key4
        key_pressed[3] = 0;
      
        // 添加key4的处理逻辑
    }
}
/*--------角点显示函数-------*/
void Point_Image(void)
{

if(Right_Up_Point_finish_flag==1)
{
    ips200.draw_point(Right_Up_Point[0],Right_Up_Point[1],RGB565_YELLOW);
    ips200.draw_point(Right_Up_Point[0]-1,Right_Up_Point[1]-1,RGB565_YELLOW);
    ips200.draw_point(Right_Up_Point[0],Right_Up_Point[1]-1,RGB565_YELLOW);
    ips200.draw_point(Right_Up_Point[0]+1,Right_Up_Point[1]-1,RGB565_YELLOW);
    ips200.draw_point(Right_Up_Point[0]-1,Right_Up_Point[1],RGB565_YELLOW);
    ips200.draw_point(Right_Up_Point[0]+1,Right_Up_Point[1],RGB565_YELLOW);
    ips200.draw_point(Right_Up_Point[0]-1,Right_Up_Point[1]+1,RGB565_YELLOW);
    ips200.draw_point(Right_Up_Point[0],Right_Up_Point[1]+1,RGB565_YELLOW);
    ips200.draw_point(Right_Up_Point[0]+1,Right_Up_Point[1]+1,RGB565_YELLOW);
}

if(Left_Up_Point_finish_flag==1)
{
    ips200.draw_point(Left_Up_Point[0],Left_Up_Point[1],RGB565_YELLOW);
    ips200.draw_point(Left_Up_Point[0]-1,Left_Up_Point[1]-1,RGB565_YELLOW);
    ips200.draw_point(Left_Up_Point[0],Left_Up_Point[1]-1,RGB565_YELLOW);
    ips200.draw_point(Left_Up_Point[0]+1,Left_Up_Point[1]-1,RGB565_YELLOW);
    ips200.draw_point(Left_Up_Point[0]-1,Left_Up_Point[1],RGB565_YELLOW);
    ips200.draw_point(Left_Up_Point[0]+1,Left_Up_Point[1],RGB565_YELLOW);
    ips200.draw_point(Left_Up_Point[0]-1,Left_Up_Point[1]+1,RGB565_YELLOW);
    ips200.draw_point(Left_Up_Point[0],Left_Up_Point[1]+1,RGB565_YELLOW);
    ips200.draw_point(Left_Up_Point[0]+1,Left_Up_Point[1]+1,RGB565_YELLOW);
}

if(Right_Down_Point_finish_flag==1)
{
    ips200.draw_point(Right_Down_Point[0],Right_Down_Point[1],RGB565_YELLOW);
    ips200.draw_point(Right_Down_Point[0]-1,Right_Down_Point[1]-1,RGB565_YELLOW);
    ips200.draw_point(Right_Down_Point[0],Right_Down_Point[1]-1,RGB565_YELLOW);
    ips200.draw_point(Right_Down_Point[0]+1,Right_Down_Point[1]-1,RGB565_YELLOW);
    ips200.draw_point(Right_Down_Point[0]-1,Right_Down_Point[1],RGB565_YELLOW);
    ips200.draw_point(Right_Down_Point[0]+1,Right_Down_Point[1],RGB565_YELLOW);
    ips200.draw_point(Right_Down_Point[0]-1,Right_Down_Point[1]+1,RGB565_YELLOW);
    ips200.draw_point(Right_Down_Point[0],Right_Down_Point[1]+1,RGB565_YELLOW);
    ips200.draw_point(Right_Down_Point[0]+1,Right_Down_Point[1]+1,RGB565_YELLOW);
}

if(Left_Down_Point_finish_flag==1)
{
    ips200.draw_point(Left_Down_Point[0],Left_Down_Point[1],RGB565_YELLOW);
    ips200.draw_point(Left_Down_Point[0]-1,Left_Down_Point[1]-1,RGB565_YELLOW);
    ips200.draw_point(Left_Down_Point[0],Left_Down_Point[1]-1,RGB565_YELLOW);
    ips200.draw_point(Left_Down_Point[0]+1,Left_Down_Point[1]-1,RGB565_YELLOW);
    ips200.draw_point(Left_Down_Point[0]-1,Left_Down_Point[1],RGB565_YELLOW);
    ips200.draw_point(Left_Down_Point[0]+1,Left_Down_Point[1],RGB565_YELLOW);
    ips200.draw_point(Left_Down_Point[0]-1,Left_Down_Point[1]+1,RGB565_YELLOW);
    ips200.draw_point(Left_Down_Point[0],Left_Down_Point[1]+1,RGB565_YELLOW);
    ips200.draw_point(Left_Down_Point[0]+1,Left_Down_Point[1]+1,RGB565_YELLOW);
}

}


void banmaxianjifen()
{
    if(motor.banmaxian_start>=100)
    {
        start_flag=2;
        motor.banmaxian_start=0;

    }
    if(motor.banmaxian_end>=40)
    {
        banmaxianFlag=2;
        motor.banmaxian_end=0;
        start_flag=0;
    }
}

void ICM_OneOrderFilter(void) {
   
        imu_acc_x = imu_dev.get_acc_x();
        imu_acc_y = imu_dev.get_acc_y();
        imu_acc_z = imu_dev.get_acc_z();

        imu_gyro_x = imu_dev.get_gyro_x();
        imu_gyro_y = imu_dev.get_gyro_y();
        imu_gyro_z = imu_dev.get_gyro_z();
        
        AHRS_update((float)imu_acc_x,(float)imu_acc_y,(float)imu_acc_z,(float)imu_gyro_x,(float)imu_gyro_y,(float)imu_gyro_z);

}


  void ips_tuxiang()
{
     uint8 i,j; int color = 0;
    
    
    for(i=0;i<=UVC_H-1;i++)
    {   //ips200_draw_point(Mid_Line[i], i,  0xF800); 
        /* ips200_draw_point(Mid_Line[i], i,  0xF800); 
        ips200_draw_point(Right_Line[i], i,  0x001F);
        ips200_draw_point(Left_Line[i], i,  0x07E0); */
        for(j=0;j<UVC_W;j++)//灰度图的数据只做判断，不进行更改，二值化图像放在了新数组中
        {
                if(image_two_value[i][j] ==0)
            {
                color = (RGB565_BLACK);
            }
            else         
            {
                color = (RGB565_WHITE);
            }
                // uint16_t r = (image_two_value[i][j] >> 3) & 0b11111;
                // uint16_t g = (image_two_value[i][j] >> 2) & 0b111111;
                // uint16_t b = (image_two_value[i][j] >> 3) & 0b11111;
                //color = (r << 11) | (g << 5) | (b << 0); 
                ips200.draw_point(j, i,  color);
                
        }
    }  
        
     


        for(i=0;i<=59;i++)
        {
        
        ips200.draw_point(Right_Line[i], i,  0x001F);
        ips200.draw_point(Left_Line[i], i,  0x07E0); ips200.draw_point(Mid_Line[i], i,  0xF800); 
        }  
    
    }



float get_error()
{  
int16 weight_count=0;
   float error=0;
   int i;
    //常规误差
        
        if(ncnn_ctrl_flag==1||ncnn_ctrl_flag==3)
        {
            for(i=UVC_H-1;i>=0;i--)//常规误差计算
        {
        error+=(UVC_W/2-Mid_Line[i])*Weight_ncnn[i];//右移1位，等效除2
            weight_count+=Weight_ncnn[i];
            //*Weight[i-20]
        }
        }
        else{
             for(i=UVC_H-1;i>=0;i--)//常规误差计算
        {
        error+=(UVC_W/2-Mid_Line[i])*Weight[i];//右移1位，等效除2
            weight_count+=Weight[i];
            //*Weight[i-20]
        }
         }
        error=error/weight_count;
        
        return error;
        
    }


void pit_callback(void)
{      
    int target_speed=target_speed_1;
    imu_gyro_x = imu_dev.get_gyro_x();
    // if(Island_State!=0) 
    //    {
        ICM_OneOrderFilter();
    // }
    //    else{icmdata.huandao_yaw=0;} 

    encoder_left = encoder_dir_1.get_count();
    encoder_right = -encoder_dir_2.get_count();
    encoder_dir_1.clear_count();
    encoder_dir_2.clear_count();
    motor_control(encoder_left , encoder_right , Island_State,start_flag,banmaxianFlag,LeftzhuantouFlag,RightzhuantouFlag,ncnn_ctrl_flag);
    // if(ncnn_ctrl_flag!=0)
    // {
    //     target_speed=70;
    // }
    // if(ncnn_stop==1)
    // {
    //     target_speed=0;
    // }
    // if( detected==1||detected==2)
    // {
    //     target_speed=0;     
    // }
    // if(ncnn_ctrl_flag==3)
    // {   if(motor.ncnn_L<=30)
    //     target_speed=100;
    //     else 
    //     target_speed=100;
    // }
    //  if(ncnn_ctrl_flag==1)
    // {   if(motor.ncnn_L<=30)
    //     target_speed=100;
    //     else 
    //     target_speed=100;
    // }
    
   motor_err=PID_motor(out_err);
    ncnn_ctrl_motor();
    if(motor_err>=0)
    {Speed_L=target_speed-motor_err;
    Speed_R=target_speed+0.4*motor_err;}
    if(motor_err<0)
    {Speed_L=target_speed-0.4*motor_err;
    Speed_R=target_speed+motor_err;}

    if(ncnn_ctrl_flag==1||ncnn_ctrl_flag==3)
    {
    if(motor_err>=0)
    {Speed_L=target_speed-motor_err;
    Speed_R=target_speed+0.4*motor_err;}
    if(motor_err<0)
    {Speed_L=target_speed-0.4*motor_err;
    Speed_R=target_speed+motor_err;}

     }
   
   

    if(start_flag==0||tuxiang_flag==1||banmaxianFlag==2||Chujie_Flag==1)
    {
        Speed_L=0;
        Speed_R=0;
        duty_S=500;
    }
    else if(duty_S<=900)
    {duty_S++;}
   
    
    if(duty_S<900)
    {Speed_L=0;
   Speed_R=0;
    }

    esc_pwm.set_duty(duty_S);
     L_duty=PID_V_L(Speed_L ,encoder_left);
     R_duty=PID_V_R(Speed_R ,encoder_right);
    
    if(R_duty >= 0)                                                           // 正转
        {
            drv8701e_dir_1.set_level(1);                                      // DIR输出高电平
            drv8701e_pwm_1.set_duty(R_duty );       // 计算占空比

            // drv8701e_dir_2.set_level(1);                                      // DIR输出高电平
            // drv8701e_pwm_2.set_duty(duty );       // 计算占空比
        }
        else
        {
            drv8701e_dir_1.set_level(0);                                      // DIR输出低电平
            drv8701e_pwm_1.set_duty(-R_duty );      // 计算占空比

            // drv8701e_dir_2.set_level(0);                                      // DIR输出低电平
            // drv8701e_pwm_2.set_duty(-duty );      // 计算占空比

        }
     
        
         if(L_duty >= 0)                                                           // 正转
        {
            

            drv8701e_dir_2.set_level(1);                                      // DIR输出高电平
            drv8701e_pwm_2.set_duty(L_duty );       // 计算占空比
        }
        else
        {
          

            drv8701e_dir_2.set_level(0);                                      // DIR输出低电平
            drv8701e_pwm_2.set_duty(-L_duty );      // 计算占空比

        }

}

int main(int, char**) 
{   
    //ncnn初始化
    bool ncnn_ok = g_classifier.ncnn_Init();
    if (!ncnn_ok) {
        printf("[Classifier] 模型加载失败!\n");
        return -1;
    }
// 初始化2寸IPS屏幕，传入帧缓冲设备路径，完成屏幕底层驱动、显示区域初始化
    ips200.init(FB_PATH);
    drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);
    drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info); 
    esc_pwm.get_dev_info(&esc_info);
    imu_dev.init();//陀螺仪初始化
    // 注册清理函数
    atexit(cleanup);

    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);
    // 初始化UVC免驱摄像头，传入USB摄像头设备路径
    // 初始化返回值小于0表示初始化失败，直接退出程序
    if (!uvc_dev.init("/dev/video0", 320, 240, 90)) 
    {
    fprintf(stderr, "YUYV camera init failed\n");
    return -1;
    }


    
    // // 初始化TCP客户端
    // if(tcp_client_dev.init(SERVER_IP, PORT) == 0)
    // {
    //     printf("tcp_client ok\r\n");
    // }
    // else
    // {
    //     printf("tcp_client error\r\n");
    //     return -1;
    // }
    // // 修复核心：传入包装函数，替代原有的成员函数直接传参
    // seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);
    
    //  seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], UVC_W ,UVC_H);
    // // 配置X轴巡线边界信息，Y坐标由图像高度自动匹配
    // seekfree_assistant_camera_boundary_config(X_BOUNDARY, UVC_H, Left_Line, Mid_Line, Right_Line, NULL, NULL ,NULL);
  //初始化中断 
    zf_driver_pit pit_timer;
    pit_timer.init_ms(5, pit_callback);
    //printf("进入主循环\n"); fflush(stdout);
    printf("初始化赛道边界缓存\n");
    init_lane_bounds();
    printf("初始化赛道边界缓存完成\n");
    while(1)
    {
//============================================================================================
//============================================================================================
        // 阻塞等待摄像头完成新一帧图像的采集和刷新
        // 返回值等于0表示成功获取到新的一帧图像数据
        if(uvc_dev.wait_frame() != nullptr)
        {
            
                        /*
            初始化
            */
            /*
            直接获取YUYV图像
            */
            // 获取当前帧 YUYV 原始数据（零拷贝，不阻塞）
            const unsigned char *yuyv_data = uvc_dev.get_yuyv_data();
            if (!yuyv_data) continue;

            // 包装为 cv::Mat，不复制数据，仅引用
            cv::Mat yuyv_mat(240, 320, CV_8UC2, const_cast<unsigned char*>(yuyv_data));
            // 创建检测结果结构体，保存色块信息和roi信息等
            DetectionResult target;
            cv::Mat roi_rgb;
            bool roi_valid = false;
            uint16 right_disp[240];
            uint16 left_disp[240];
            uint16 mid_disp[240];
            cv::Mat bgr;

            //uvc_dev.get_both_images_manual(gray_image, rgb_image, UVC_WIDTH, UVC_HEIGHT);
           // ips200.displayimage_rgb565(rgb_image, UVC_WIDTH, UVC_HEIGHT);
            //*gray_data=uvc_obj.get_gray_image_ptr();
            // yuyv = uvc_dev.wait_frame();
            // if (!yuyv) return 0;
            gray_image = uvc_dev.get_gray_image_ptr();
            
            
            threshold =  my_adapt_threshold(gray_image, 320,240) ;
            
            Image_Binarization(threshold);

            //Image_Binarization(threshold);
            //ips200.show_int(10, 20, threshold, 3);
            Longest_White_Column(0);
            Identify();
            shizibuxian();
            //ips200.displayimage_gray(&image_two_value[0][0], UVC_WIDTH, 60);
            Get_Mid(); 
            if(ncnn_ctrl_flag==0)
            {Island_Detect();}
            banmaxianjifen();
            if(start_flag==2)banmaxian();
            //zhuantou();
            if(ncnn_ctrl_flag==0)
            {chujie();}
 

            


            for (uint8 row = 0; row < UVC_H; row++) {
                // 最左边4列（列0~3）
                for (uint8 col = 0; col < 4; col++) {
                    image_two_value[row][col] = 0;
                }
                // 最右边4列（列UVC_W-4 ~ UVC_W-1）
                for (uint8 col = UVC_W - 4; col < UVC_W; col++) {
                    image_two_value[row][col] = 0;
                }
            }
            uint8 left_pts[UVC_W][2];       // 最多UVC_W个点（80）
            uint8 right_pts[UVC_W][2];
            seek_pts_seed(START_ROW, 10);
            uint8 red_stop_line=0;
            
            uint8 right_step = findline_righthand_binary(right_pts);
            uint8 right_pts_final[UVC_H];
            uint8 right_final_step = reduce_right_to_line(right_pts, right_step, right_pts_final);
            
            map_line_to_display(right_pts_final, right_final_step, right_disp,1);
            
            uint8 left_step  = findline_lefthand_binary(left_pts);
            uint8 left_pts_final[UVC_H];  // 过滤后的结果（每行一个点）
            uint8 left_final_step = reduce_left_to_line(left_pts, left_step, left_pts_final);

            map_line_to_display(left_pts_final, left_final_step, left_disp,0);

            if(right_final_step>=left_final_step)
            {
                red_stop_line=right_final_step*4;

            }
            else 
            {
                red_stop_line=left_final_step*4;
            }
            for(uint8 i=red_stop_line;i<=239;i++)
            {
                mid_disp[i]=(left_disp[i]+right_disp[i])/2;
            }
           /*模型识别*/
            if (ncnn_ok)
            {   //detected=0;
                 //if(ncnn_ctrl_flag==0&&ncnn_ctrl_id==0)
                  
                    
                    //if(frame_counter++ % 2 == 0)
                    //{   
                        //rgb_image = uvc_dev.get_rgb_image_ptr();
                        
                        //detected = g_classifier.Red_Detection(red_rect, red_center, bottom_point);
                        

                        // if(detected==1) 
                        // {   ncnn_stop=1;
                        //     printf("\n=====减速环节=====\n red_center: %d, %d\n", red_center.x, red_center.y);
                        // }
                    
                        update_lane_bounds(left_disp, right_disp);
                        //set_lane_search_bottom(red_stop_line * 4);
                        update_lane_search_top(red_stop_line );
                        // 检测红色色块 → BEV PCA → 回投原图角点
                         //printf("准备检测红色色块\n");
                        target = findRedBoxAndIpm(yuyv_mat, kRot, kInvRot, target);
                        //printf("检测完成\n");
                        ncnn_id=0;
                        /*
                        红色色块是直接检测的，所以判断的返回值只有target.found=tur/faulse, 但是可以控制红色种子扫描的范围
                        - 默认SEARCH_LEFT = 75;EARCH_RIGHT = 285;EARCH_TOP = 20;SEARCH_BOTTOM = 130;
                        - 在findRedBox.cpp顶部设置
                        */
                        if(target.found)
                        {    
                            printf("已检测到目标色块\n");   

                    //     L_duty=PID_V_L(-100 ,encoder_left);
                    //     R_duty=PID_V_R(-100 ,encoder_right);
                    // if(R_duty >= 0)                                                           // 正转
                    //     {   drv8701e_dir_1.set_level(1);                                      // DIR输出高电平
                    //         drv8701e_pwm_1.set_duty(R_duty );}
                    //     else
                    //     {   drv8701e_dir_1.set_level(0);                                      // DIR输出低电平
                    //         drv8701e_pwm_1.set_duty(-R_duty ); }
                    //     if(L_duty >= 0)                                                           // 正转
                    //     {drv8701e_dir_2.set_level(1);                                      // DIR输出高电平
                    //         drv8701e_pwm_2.set_duty(L_duty ); }
                    //     else
                    //     {   drv8701e_dir_2.set_level(0);                                      // DIR输出低电平
                    //         drv8701e_pwm_2.set_duty(-L_duty );}
                            if(abs(out_err)<=10)//必定是直道，或者斜率小的s弯Search_Stop_Line>=50&&
                            {
                                Shape=2;
                            }
                            else if(out_err>10)//大概率左拐Search_Stop_Line<50&&
                            {
                                Shape=1;
                            }
                            else if(out_err<-10)//大概率右拐Search_Stop_Line<50&&
                            {
                                Shape=3;
                            }
                            else//没有明显的赛道特征，比如弯道但是误差小，或者小s弯白列长但是误差稍微大一点，或者是出弯的时候
                            {
                                Shape=0;
                            }
                            printf("%d.\r\n",Shape);
                            if(target.flag==1){
                                {
                                cv::Mat roi_rgb_direct = warp_yuyv_to_bev_rgb(yuyv_mat, target.RAW_pts, BEV_W, BEV_H);
                                ncnn_id = g_classifier.Main_Detection(roi_rgb_direct);
                                }
                            }else{
                                roi_valid = expand_roi_from_red(yuyv_mat, target, kInvRot, target.IpmRec_pts, target.RawRec_pts);
                                if(roi_valid){
                                    roi_rgb = warp_yuyv_to_bev_rgb(yuyv_mat, target.RawRec_pts , BEV_W, BEV_H);
                                    ncnn_id = g_classifier.Main_Detection(roi_rgb);
                                }
                            }
                        }else{
                        printf("未检测到目标色块\n");
                        }
                        if(ncnn_id!=0)
                        {   
                            ncnn_ctrl_id=ncnn_id;
                            detected=0;
                        }
                        
                
               
            }
        
    
        

            
             
            if(ncnn_ctrl_flag!=1&&ncnn_ctrl_flag!=3)
            {out_err=get_error();}
            if(ncnn_ctrl_flag==1)//右拐  
    {
        for(int i=0;i<UVC_H-Search_Stop_Line;i++)
        {
            Right_Line[i]=0;
        }

    } 
    if(ncnn_ctrl_flag==3)//左拐  
    {
        for(int i=0;i<UVC_H-Search_Stop_Line;i++)
        {
            Left_Line[i]=79;
        }

    } 
            if(ncnn_ctrl_flag==3)
            {
                for(int i=0;i<=59;i++)
                {
                    Mid_Line[i]=Left_Line[i];
                }
            }
            if(ncnn_ctrl_flag==1)
            {
                for(int i=0;i<=59;i++)
                {
                    Mid_Line[i]=Right_Line[i];
                }
            }
            ncnn_ctrl();
            //printf("%d.\r\n",Search_Stop_Line);
            // if(detected==2)
            // {       
            //     // if(ncnn_ctrl_flag==1||ncnn_ctrl_flag==3)
            //     //     {
            //     //     motor_err=PID_motor(out_err);
            //     //     if(motor_err>=0)
            //     //     {Speed_L=100-motor_err;
            //     //     Speed_R=100+1*motor_err;}
            //     //     if(motor_err<0)
            //     //     {Speed_L=100-1*motor_err;
            //     //     Speed_R=100+motor_err;}

                    
            //     //         L_duty=PID_V_L(Speed_L ,encoder_left);
            //     //         R_duty=PID_V_R(Speed_R ,encoder_right);
            //     //         if(R_duty >= 0)                                                           // 正转
            //     //         {   drv8701e_dir_1.set_level(1);                                      // DIR输出高电平
            //     //             drv8701e_pwm_1.set_duty(R_duty );}
            //     //         else
            //     //         {   drv8701e_dir_1.set_level(0);                                      // DIR输出低电平
            //     //             drv8701e_pwm_1.set_duty(-R_duty ); }
            //     //         if(L_duty >= 0)                                                           // 正转
            //     //         {drv8701e_dir_2.set_level(1);                                      // DIR输出高电平
            //     //             drv8701e_pwm_2.set_duty(L_duty ); }
            //     //         else
            //     //         {   drv8701e_dir_2.set_level(0);                                      // DIR输出低电平
            //     //             drv8701e_pwm_2.set_duty(-L_duty );}
            //     //     }

            // }
           
            // if (ncnn_ok) {
            //     detected = g_classifier.Red_Detection(red_rect, red_center);
            //     ncnn_id=0;
            //    // printf("red_center: %d, %d\n", red_center.x, red_center.y);
            //     if(detected)
            //     {
            //         cv::Mat roi_rgb = g_classifier.roi_Extract(red_rect);
            //         ncnn_id= g_classifier.Main_Detection(roi_rgb); // class_id 为推理结果,初始值为e. 1: supply 2: vehicle 3: weapon
            //     }
            // }
            //1右拐3左拐，2直行
            // if(ncnn_id!=0)
            // {
            //     Chujie_Flag=1;
            // }

            
            /*模型识别*/

            if( tuxiang_flag==1)
            {   if(image_ips==0)
                {
                ips_tuxiang();
                ips200.show_float(0,170,out_err,3,3);
                ips200.show_float(50,170,icmdata.huandao_yaw,3,3);
                ips200.show_int(0,150,Island_State,3);
                ips200.show_int(50,150,ncnn_ctrl_flag,3);
                ips200.show_int(100,150,red_center.y,3);
                ips200.show_int(150,150,detected,3);
                ips200.show_int(200,150,ncnn_ctrl_flag,3);
                ips200.show_int(220,150,threshold,3);
                }
                if(image_ips==1)
                {   /*tuxiang*/
                   //printf("准备显示图像\n");
                    cv::cvtColor(yuyv_mat, bgr, cv::COLOR_YUV2BGR_YUYV); 
                    if (!bgr.empty()) {
                        //printf("开始显示图像1");
                        //红色种子搜索范围
                        /*
                        cv::line(bgr, cv::Point(35, 20), cv::Point(285, 20), cv::Scalar(255, 255, 255)); //左上到右上
                        cv::line(bgr, cv::Point(35, 20), cv::Point(35, 130), cv::Scalar(255, 255, 255)); //左上到左下
                        cv::line(bgr, cv::Point(35, 130), cv::Point(285, 130), cv::Scalar(255, 255, 255)); //左下到右下
                        cv::line(bgr, cv::Point(285, 20), cv::Point(285, 130), cv::Scalar(255, 255, 255)); //右上到右下
                        */
                        cv::line(bgr, cv::Point(0, SEARCH_TOP), cv::Point(340, SEARCH_TOP), cv::Scalar(255, 255, 255)); 
                        cv::line(bgr, cv::Point(0, SEARCH_BOTTOM), cv::Point(340, SEARCH_BOTTOM), cv::Scalar(255, 255, 255)); 
                     
                        //printf("开始显示图像2\n");
                        //显示目标框
                        if (target.found) {
                            draw_rotated_quad(bgr, target.RAW_pts, cv::Scalar(255, 0, 0));
                            if (roi_valid)
                                draw_rotated_quad(bgr, target.RawRec_pts, cv::Scalar(0, 255, 0));
                        }
                        //printf("显示图像2完成\n");
                        //printf("开始显示BEV\n");
                        if (!roi_rgb.empty()) {
                            cv::Mat bev_bgr;
                            cv::cvtColor(roi_rgb, bev_bgr, cv::COLOR_RGB2BGR);
                            int pw = bev_bgr.cols < (320 - BEV_W) ? bev_bgr.cols : 320 - BEV_W;
                            int ph = bev_bgr.rows < BEV_H ? bev_bgr.rows : BEV_H;
                            bev_bgr(cv::Rect(0, 0, pw, ph)).copyTo(bgr(cv::Rect(320 - BEV_W, 0, pw, ph)));
                        }
                        //printf("显示BEV完成\n");
                        //printf("开始显示全部图像\n");
                        std::vector<uint16_t> buf(240 * 320);
                        for (int dy = 0; dy < 320; dy++) {
                            uint16_t* dst_row = buf.data() + dy * 240;
                            for (int dx = 0; dx < 240; dx++) {
                                int sx = dy;
                                int sy = 239 - dx;
                                const uint8_t* src_pixel = bgr.ptr<uint8_t>(sy) + sx * 3;
                                uint8_t b = src_pixel[0];
                                uint8_t g = src_pixel[1];
                                uint8_t r = src_pixel[2];
                                dst_row[dx] = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
                            }
                        }

                        //printf("显示全部图像完成\n");
                        ips200.show_rgb565_image(0, 0, buf.data(), 240, 320, 240, 320, 0);
                        for (uint16 r = 0; r < 240; r++) {
                            if (left_disp[r] != 0xFFFF) {
                                ips200.draw_point(239 - r, left_disp[r], 0xF800);
                            }
                            if (right_disp[r] != 0xFFFF) {
                                ips200.draw_point(239 - r, right_disp[r], 0x001F);
                            }
                        }
                        for(uint8 i=red_stop_line;i<=239;i++)
                        {
                            ips200.draw_point(239 - i, mid_disp[i], 0x001F);
                        }
                        
                    }
                }
                }
                  
       
//============================================================================================
//============================================================================================
        //printf("imu = %d\n", imu_gyro_x);
        // if(Island_State!=0||Cross_Flag!=0||ncnn_ctrl_flag!=0)//Island_State!=0||Cross_Flag!=0||LeftzhuantouFlag!=0||RightzhuantouFlag!=0||ncnn_ctrl_flag!=0
        // {
        //     beep.set_level(1);

        // }
        // else{

        //     beep.set_level(0);
        // }
            
        //
       process_keys();
    //    memcpy(image_copy[0], image_two_value[0], UVC_W * UVC_H);

    //     // 核心发送函数：将灰度图像/边界信息 通过TCP网络发送到逐飞助手上位机
    //     seekfree_assistant_camera_send();
    //    for(int i=0;i<89;i++)
    //    {
    //     printf("%d.\r\n", Road_Wide[i]);

    //     }
            // seekfree_assistant_oscilloscope_data.data[0]=encoder_right;
            // seekfree_assistant_oscilloscope_data.data[1]=Speed_R;
            // seekfree_assistant_oscilloscope_data.data[2]=encoder_left;
            // seekfree_assistant_oscilloscope_data.data[3]=Speed_L;
            // // 设置本次需要发送几个通道的数据
            // seekfree_assistant_oscilloscope_data.channel_num = 4;
          
            // printf("zf_encoder_left = %d.\r\n", encoder_left);
            // printf("zf_encoder_right = %d.\r\n", encoder_right);
            // for(int8 i=0;i<=59;i++)
            // {}
            //     printf("%d.\r\n",Road_Wide[i]);

            // }motor_err
            // if(ncnn_ctrl_flag!=0)
            // {
            // printf("motor_err = %f.\r\n", motor_err);    
            // }
            //printf("huandao=%d.\r\n",Island_State);
            //seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    
    }
    }
    return 0;
   
}

