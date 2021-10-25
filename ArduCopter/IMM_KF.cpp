#include "Copter.h"

//定义矩阵运算中时间T的值
#define T 0.0025

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
//
//
//多模型参数的初始化
//
//
//定义故障模型矩阵faultmodel
float faultmodel[4][5] = {
    {0,1,0,0,0},
    {0,0,1,0,0},
    {0,0,0,1,0},
    {0,0,0,0,1}
};
float temp = 4;
float N = 5; //模型个数

//定义模型概率转移矩阵
float prob[5][5] = {
    {0.95,0.125,0.125,0.125,0.125},
    {0.125,0.95,0.125,0.125,0.125},
    {0.125,0.125,0.95,0.125,0.125},
    {0.125,0.125,0.125,0.95,0.125},
    {0.125,0.125,0.125,0.125,0.95},
};

//定义系统状态转移矩阵A
float A[8][8] = {
    {1,T,0,0,0,0,0,0},
    {0,1,0,0,0,0,0,0},
    {0,0,1,T,0,0,0,0},
    {0,0,0,1,0,0,0,0},
    {0,0,0,0,1,T,0,0},
    {0,0,0,0,0,1,0,0},
    {0,0,0,0,0,0,1,T},
    {0,0,0,0,0,0,0,1}
};

//定义系统矩阵B
float b=175;
float m=1.42;
float l=0.20;
float IX=0.03;
float IY=0.03;
float IZ=0.04;
float d=4.025;

float temp1 = T*T*b/(2*m);
float temp2 = T*b/m;
float temp3 = b*l*T*T/(2*IX);
float temp4 = b*l*T/IX;
float temp5 = b*l*T*T/(2*IY);
float temp6 = b*l*T/IY;
float temp7 = d*T*T/(2*IZ);
float temp8 = d*T/IZ;

float B[8][4] = {
        { temp1, temp1, temp1, temp1},
        { temp2, temp2, temp2, temp2},
        {     0,-temp3,     0, temp3},
        {     0,-temp4,     0, temp4},
        { temp5,     0,-temp5,     0},
        { temp6,     0,-temp6,     0},
        {-temp7, temp7,-temp7, temp7},
        {-temp8, temp8,-temp8, temp8}
    };

//定义观测矩阵
float H[4][8] = {
    {1,0,0,0,0,0,0,0},
    {0,0,1,0,0,0,0,0},
    {0,0,0,0,1,0,0,0},
    {0,0,0,0,0,0,1,0},
};

//初始化X_next,P_next,Mu_next
float X_next[8][5] = {0};
float P_next[8][8][5] = {0};
float Mu_next[5] = {0};
//
//
//输入交互
//
//
float Copter::IMM_KF(float Zin[4],float Uin[4],float X_real[8],float t,float X_last[8][5],float P_last[8][8][5],float Mu_last[5])
{
    int i,j,k;
    if(Mu_last[5]={0})
    {
        X_last[8][5] = {0};
        for(i=0;i<8;i++)
        {
            for(j=0;j<8;j++)
            {
                for(k=0;k<5;k++)
                    P_last[i][j][k] = 1;
            }
        }
        for(i=0;i<5;i++)
            Mu_last[i] = 0.2;
    }
}