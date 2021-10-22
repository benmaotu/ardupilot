#include "Copter.h"

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