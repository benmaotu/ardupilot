#include "Copter.h"

//#include <cmath>
//#include <math.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>


//#include "AP_MotorsMatrix.h"

//定义矩阵运算中时间T的值
#define T 0.1
#define g 9.81  //定义重力加速度


/*
float get_euler_roll();
float get_euler_pitch();
float get_euler_yaw();
float get_euler_high();
*/


float Copter::KF_Try(float ResrcData,float ProcessNoise_Q,float MeasureNoise_R)
{   
    //R，Q 分别为测量噪声和过程噪声
    float R = MeasureNoise_R;
    float Q = ProcessNoise_Q;
    //x_last 为 x(k-1|k-1);  x_mid 为 x(k|k-1);  x_now 为 x(k|k);
    static float x_last;

    float x_mid = x_last;
    float x_now;
    //P表示状态协方差矩阵
    static float p_last;

    float p_mid;
    float p_now;
    float kg;

    x_mid = x_last;//x(k|k-1) = x(k-1|k-1)
    p_mid = p_last+Q;//p(k|k-1) = p(k-1|k-1)+Q
    //kg是卡尔曼增益
    kg = p_mid/ (p_mid + R);//kg = p(k|k-1)/(p(k|k-1)+R)
    x_now = x_mid+ kg*(ResrcData-x_mid);//x(k|k) = x(k|k-1)+kg*(ResrcData-x(k|k-1))

    p_now = (1-kg)*p_mid;//p(k|k) = (1-kg)*p(k|k-1)

    p_last = p_now;
    x_last = x_now;

    return x_now;
}

//滤波器更新函数
float Copter::update_KF_Try()
{
    float h;
    float h_KF;
    h = barometer.get_altitude() * 100.0f;
    h_KF = KF_Try(h,0.004,1.0);
    return h_KF;
}

float Zin_R[4];
float Uin_R[4];
float Uin_R_PID[3];
float X_real_R[8];
float X_real_R_d[3];
float error_number;
int16_t Uin_R_motor[AP_MOTORS_MAX_NUM_MOTORS];
int T_U = 0;
int16_t M_U;

void Copter::IMM_KF_Update()
{
    Zin_R[0] = barometer.get_altitude();
    Zin_R[1] = quaternion.get_euler_roll();
    Zin_R[2] = quaternion.get_euler_pitch();
    Zin_R[3] = quaternion.get_euler_yaw(); 

    //hal.console->printf("%f\n",ahrs.roll_sensor);
    

    //*X_real_R_d = attitude_control->rate_controller_run_IMM(); //该函数自行添加，为了获取陀螺仪角速度
    X_real_R_d[0] = attitude_control->rate_controller_run_IMM().x;
    X_real_R_d[1] = attitude_control->rate_controller_run_IMM().y;
    X_real_R_d[2] = attitude_control->rate_controller_run_IMM().z;

    X_real_R[0] = barometer.get_altitude();
    X_real_R[1] = barometer.get_climb_rate();
    X_real_R[2] = quaternion.get_euler_roll();
    X_real_R[3] = X_real_R_d[0];
    X_real_R[4] = quaternion.get_euler_pitch();
    X_real_R[5] = X_real_R_d[1];
    X_real_R[6] = quaternion.get_euler_yaw();
    X_real_R[7] = X_real_R_d[2];
    
    //*Uin_R_PID = attitude_control->rate_controller_run_IMM_PID(); //该函数自行添加，为了获取PID输入
    //Uin_R[1] = Uin_R_PID[0];
    //Uin_R[2] = Uin_R_PID[1];
    //Uin_R[3] = Uin_R_PID[2];
    //Uin_R[0] = pos_control->get_alt_error();

    //float Uin_R[4] = {1920,1865,1964,1996};

    Uin_R[0] = Uin_R_0_IMM;
    Uin_R[1] = Uin_R_1_IMM;
    Uin_R[2] = Uin_R_2_IMM;
    Uin_R[3] = Uin_R_3_IMM;

    hal.console->printf("%f,%f,%f\n",attitude_control->rate_controller_run_IMM().x,attitude_control->rate_controller_run_IMM().y,attitude_control->rate_controller_run_IMM().z);
    

    //float Zin_Test[4] = {1.5,9.65,75.3,66};
    //float Uin_Test[4] = {12.6,8.487,0.02,41.5};
    //float X_Test[8] = {12.53,1.3,5.6,4.1,7.8,1.2,231,2};

    //error_number = IMM_KF(Zin_R,Uin_R,X_real_R);
    //error_number = IMM_KF(Zin_Test,Uin_Test,X_Test);
}

