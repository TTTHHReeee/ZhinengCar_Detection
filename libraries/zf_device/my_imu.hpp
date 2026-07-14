#ifndef _MY_IMU_HPP__
#define _MY_IMU_HPP__

void AHRS_update(float Ax,float Ay,float Az,float Gx,float Gy,float Gz);

//ICM数据
typedef struct{
    float ACCEL_X;
    float ACCEL_Y;
    float ACCEL_Z;
    float GYRO_X;
    float GYRO_Y;
    float GYRO_Z;
    //角度
    float Pitch;
    float Yaw;        //环岛角度
    float Roll;
    float huandao_yaw;
    float youhuandaoYaw;
    float zuohuandaoYaw;
    float podaopitch;
    float Yaw_right_T;
    float Yaw_zuohuan1;
    float Yaw_zuohuan2;
    int Yaw_youhuan1;
    float Yaw_youhuan2;
    float Yaw_zouhuan3;
    float Yaw_sancha;
    float Yaw_incheku;
    float Yaw_duanlu;
    float Yaw_bizhang;
    //角速度
    float PitchVelocity;
    float RollVelocity;
    float YawVelocity;
    float YawVelocity_offset;//偏航角速度偏置
}ICMDatatypedef;


extern ICMDatatypedef icmdata;


#endif
