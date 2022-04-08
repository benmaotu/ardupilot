#include "Copter.h"
#include "math.h"
#include <GCS_MAVLink/GCS.h>

//#define plus
#define X

float T = 0.303;
//float T = 0.1;                 //定义函数运行周期，将函数放入100Hz循环//3.3Hz
//float kt = 9.122e-6;            //定义无人机升力系数
//float kq = 1.133e-7;            //定义无人机反扭力系数
float kt = 1.308e-6;
float kq = 1.4533e-7;

float m_hexa = 1.8;                 //定义无人机质量
float g_e = 9.81;                //定义重力加速度
float d_hexa = 0.275;               //定义无人机机臂长度
float Ixx = 3.627e-2;            //定义x轴转动惯量
float Iyy = 3.627e-2;            //定义y轴转动惯量
float Izz = 6.780e-2;            //定义z轴转动惯量

int8_t t_run = 0;//定义非初始化值的运行次数

//定义模型概率转移矩阵
float p_i_j[7][7] = {
    {0.95, 0.0083, 0.0083, 0.0083, 0.0083, 0.0083, 0.0083},
    {0.0083, 0.95 ,0.0083, 0.0083, 0.0083, 0.0083, 0.0083},
    {0.0083, 0.0083, 0.95 ,0.0083, 0.0083, 0.0083, 0.0083},
    {0.0083, 0.0083, 0.0083, 0.95, 0.0083, 0.0083, 0.0083},
    {0.0083, 0.0083, 0.0083, 0.0083, 0.95, 0.0083, 0.0083},
    {0.0083, 0.0083, 0.0083, 0.0083, 0.0083, 0.95, 0.0083},
    {0.0083, 0.0083, 0.0083, 0.0083, 0.0083, 0.0083, 0.95}
};

float B_g[8] = {
    T*T*g_e/2.0f, g_e*T, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

//定义状态转移矩阵
float A_State[8][8] = {
    {1, T, 0, 0, 0, 0, 0, 0},
    {0, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 1, T, 0, 0, 0, 0},
    {0, 0, 0, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, T, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, T},
    {0, 0, 0, 0, 0, 0, 0, 1}
};

//定义输入矩阵
float a_roll    =   kt*d_hexa/Ixx;
float a_pitch   =   kt*d_hexa/Iyy;
float a_yaw     =   kq/Izz;

#ifdef X

    float B_State[8][6] = {
    {kt*T*T/(2.0f*m_hexa),   kt*T*T/(2.0f*m_hexa),    kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa),      kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa)},
    {kt*T/m_hexa,            kt*T/m_hexa,             kt*T/m_hexa,              kt*T/m_hexa,               kt*T/m_hexa,              kt*T/m_hexa},
    {-a_roll*T*T/2.0f,       a_roll*T*T/2.0f,         a_roll*T*T*0.5/2.0f,      -a_roll*T*T*0.5f/2.0f,     -a_roll*T*T*0.5f/2.0f,    a_roll*T*T*0.5f/2.0f},
    {-a_roll*T,              a_roll*T,                a_roll*T*0.5,             -a_roll*T*0.5f,            -a_roll*T*0.5f,           a_roll*T*0.5f},
    {0,                      0,                       a_pitch*T*T*0.866f/2.0f,  -a_pitch*T*T*0.866f/2.0f,  a_pitch*T*T*0.866f/2.0f,  -a_pitch*T*T*0.866f/2.f},
    {0,                      0,                       a_pitch*T*0.866f,         -a_pitch*T*0.866f,         a_pitch*T*0.866f,         -a_pitch*T*0.866f},
    {-a_yaw*T*T/2.0f,        a_yaw*T*T/2.0f,          -a_yaw*T*T/2.0f,          -a_yaw*T*T/2.0f,            a_yaw*T*T/2.0f,           a_yaw*T*T/2.0f},
    {-a_yaw*T,               a_yaw*T,                 -a_yaw*T,                 -a_yaw*T,                   a_yaw*T,                  a_yaw*T}
};

float B_State_1[8][6] = {
    {0,   kt*T*T/(2.0f*m_hexa),    kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa),      kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa)},
    {0,   kt*T/m_hexa,             kt*T/m_hexa,              kt*T/m_hexa,               kt*T/m_hexa,              kt*T/m_hexa},
    {0,   a_roll*T*T/2.0f,         a_roll*T*T*0.5/2.0f,      -a_roll*T*T*0.5f/2.0f,     -a_roll*T*T*0.5f/2.0f,    a_roll*T*T*0.5f/2.0f},
    {0,   a_roll*T,                a_roll*T*0.5,             -a_roll*T*0.5f,            -a_roll*T*0.5f,           a_roll*T*0.5f},
    {0,   0,                       a_pitch*T*T*0.866f/2.0f,  -a_pitch*T*T*0.866f/2.0f,  a_pitch*T*T*0.866f/2.0f,  -a_pitch*T*T*0.866f/2.f},
    {0,   0,                       a_pitch*T*0.866f,         -a_pitch*T*0.866f,         a_pitch*T*0.866f,         -a_pitch*T*0.866f},
    {0,   a_yaw*T*T/2.0f,          -a_yaw*T*T/2.0f,         -a_yaw*T*T/2.0f,            a_yaw*T*T/2.0f,           a_yaw*T*T/2.0f},
    {0,   a_yaw*T,                 -a_yaw*T,                -a_yaw*T,                   a_yaw*T,                  a_yaw*T}
};

float B_State_2[8][6] = {
    {kt*T*T/(2.0f*m_hexa),   0,    kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa),      kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa)},
    {kt*T/m_hexa,            0,    kt*T/m_hexa,              kt*T/m_hexa,               kt*T/m_hexa,              kt*T/m_hexa},
    {-a_roll*T*T/2.0f,       0,    a_roll*T*T*0.5/2.0f,      -a_roll*T*T*0.5f/2.0f,     -a_roll*T*T*0.5f/2.0f,    a_roll*T*T*0.5f/2.0f},
    {-a_roll*T,              0,    a_roll*T*0.5,             -a_roll*T*0.5f,            -a_roll*T*0.5f,           a_roll*T*0.5f},
    {0,                      0,    a_pitch*T*T*0.866f/2.0f,  -a_pitch*T*T*0.866f/2.0f,  a_pitch*T*T*0.866f/2.0f,  -a_pitch*T*T*0.866f/2.f},
    {0,                      0,    a_pitch*T*0.866f,         -a_pitch*T*0.866f,         a_pitch*T*0.866f,         -a_pitch*T*0.866f},
    {-a_yaw*T*T/2.0f,        0,    -a_yaw*T*T/2.0f,         -a_yaw*T*T/2.0f,            a_yaw*T*T/2.0f,           a_yaw*T*T/2.0f},
    {-a_yaw*T,               0,    -a_yaw*T,                -a_yaw*T,                   a_yaw*T,                  a_yaw*T}
};

float B_State_3[8][6] = {
   {kt*T*T/(2.0f*m_hexa),   kt*T*T/(2.0f*m_hexa),     0,     kt*T*T/(2.0f*m_hexa),      kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa)},
    {kt*T/m_hexa,            kt*T/m_hexa,             0,     kt*T/m_hexa,               kt*T/m_hexa,              kt*T/m_hexa},
    {-a_roll*T*T/2.0f,       a_roll*T*T/2.0f,         0,     -a_roll*T*T*0.5f/2.0f,     -a_roll*T*T*0.5f/2.0f,    a_roll*T*T*0.5f/2.0f},
    {-a_roll*T,              a_roll*T,                0,     -a_roll*T*0.5f,            -a_roll*T*0.5f,           a_roll*T*0.5f},
    {0,                      0,                       0,     -a_pitch*T*T*0.866f/2.0f,  a_pitch*T*T*0.866f/2.0f,  -a_pitch*T*T*0.866f/2.f},
    {0,                      0,                       0,     -a_pitch*T*0.866f,         a_pitch*T*0.866f,         -a_pitch*T*0.866f},
    {-a_yaw*T*T/2.0f,        a_yaw*T*T/2.0f,          0,     -a_yaw*T*T/2.0f,            a_yaw*T*T/2.0f,           a_yaw*T*T/2.0f},
    {-a_yaw*T,               a_yaw*T,                 0,     -a_yaw*T,                   a_yaw*T,                  a_yaw*T}
};

float B_State_4[8][6] = {
    {kt*T*T/(2.0f*m_hexa),   kt*T*T/(2.0f*m_hexa),    kt*T*T/(2.0f*m_hexa),     0,      kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa)},
    {kt*T/m_hexa,            kt*T/m_hexa,             kt*T/m_hexa,              0,      kt*T/m_hexa,              kt*T/m_hexa},
    {-a_roll*T*T/2.0f,       a_roll*T*T/2.0f,         a_roll*T*T*0.5/2.0f,      0,      -a_roll*T*T*0.5f/2.0f,    a_roll*T*T*0.5f/2.0f},
    {-a_roll*T,              a_roll*T,                a_roll*T*0.5,             0,      -a_roll*T*0.5f,           a_roll*T*0.5f},
    {0,                      0,                       a_pitch*T*T*0.866f/2.0f,  0,      a_pitch*T*T*0.866f/2.0f,  -a_pitch*T*T*0.866f/2.f},
    {0,                      0,                       a_pitch*T*0.866f,         0,      a_pitch*T*0.866f,         -a_pitch*T*0.866f},
    {-a_yaw*T*T/2.0f,        a_yaw*T*T/2.0f,          -a_yaw*T*T/2.0f,          0,      a_yaw*T*T/2.0f,           a_yaw*T*T/2.0f},
    {-a_yaw*T,               a_yaw*T,                 -a_yaw*T,                 0,      a_yaw*T,                  a_yaw*T}
};

float B_State_5[8][6] = {
   {kt*T*T/(2.0f*m_hexa),   kt*T*T/(2.0f*m_hexa),    kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa),       0,     kt*T*T/(2.0f*m_hexa)},
    {kt*T/m_hexa,            kt*T/m_hexa,             kt*T/m_hexa,              kt*T/m_hexa,               0,     kt*T/m_hexa},
    {-a_roll*T*T/2.0f,       a_roll*T*T/2.0f,         a_roll*T*T*0.5/2.0f,      -a_roll*T*T*0.5f/2.0f,     0,     a_roll*T*T*0.5f/2.0f},
    {-a_roll*T,              a_roll*T,                a_roll*T*0.5,             -a_roll*T*0.5f,            0,     a_roll*T*0.5f},
    {0,                      0,                       a_pitch*T*T*0.866f/2.0f,  -a_pitch*T*T*0.866f/2.0f,  0,     -a_pitch*T*T*0.866f/2.f},
    {0,                      0,                       a_pitch*T*0.866f,         -a_pitch*T*0.866f,         0,     -a_pitch*T*0.866f},
    {-a_yaw*T*T/2.0f,        a_yaw*T*T/2.0f,          -a_yaw*T*T/2.0f,          -a_yaw*T*T/2.0f,            0,     a_yaw*T*T/2.0f},
    {-a_yaw*T,               a_yaw*T,                 -a_yaw*T,                 -a_yaw*T,                   0,     a_yaw*T}
};

float B_State_6[8][6] = {
    {kt*T*T/(2.0f*m_hexa),   kt*T*T/(2.0f*m_hexa),    kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa),      kt*T*T/(2.0f*m_hexa),     0},
    {kt*T/m_hexa,            kt*T/m_hexa,             kt*T/m_hexa,              kt*T/m_hexa,               kt*T/m_hexa,              0},
    {-a_roll*T*T/2.0f,       a_roll*T*T/2.0f,         a_roll*T*T*0.5/2.0f,      -a_roll*T*T*0.5f/2.0f,     -a_roll*T*T*0.5f/2.0f,    0},
    {-a_roll*T,              a_roll*T,                a_roll*T*0.5,             -a_roll*T*0.5f,            -a_roll*T*0.5f,           0},
    {0,                      0,                       a_pitch*T*T*0.866f/2.0f,  -a_pitch*T*T*0.866f/2.0f,  a_pitch*T*T*0.866f/2.0f,  0},
    {0,                      0,                       a_pitch*T*0.866f,         -a_pitch*T*0.866f,         a_pitch*T*0.866f,         0},
    {-a_yaw*T*T/2.0f,        a_yaw*T*T/2.0f,          -a_yaw*T*T/2.0f,          -a_yaw*T*T/2.0f,            a_yaw*T*T/2.0f,           0},
    {-a_yaw*T,               a_yaw*T,                 -a_yaw*T,                 -a_yaw*T,                   a_yaw*T,                  0}
};

#else

    float B_State[8][6] = {
    {kt*T*T/(2.0f*m_hexa),   kt*T*T/(2.0f*m_hexa),    kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa),      kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa)},
    {kt*T/m_hexa,            kt*T/m_hexa,             kt*T/m_hexa,              kt*T/m_hexa,               kt*T/m_hexa,              kt*T/m_hexa},
    {0.0f,                   0.0f,                    a_roll*T*T*0.866f/2.0f,   -a_roll*T*T*0.866f/2.0f,   a_roll*T*T*0.866f/2.0f,   -a_roll*T*T*0.866f/2.0f},
    {0.0f,                   0.0f,                    a_roll*T*0.866f,          -a_roll*T*0.866f,          a_roll*T*0.866f,          -a_roll*T*0.866f},
    {a_pitch*T*T/2.0f,       -a_pitch*T*T/2.0f,       -a_pitch*T*T*0.5f/2.0f,   a_pitch*T*T*0.5f/2.0f,     a_pitch*T*T*0.5f/2.0f,    -a_pitch*T*T*0.5f/2.f},
    {a_pitch*T,              -a_pitch*T,              -a_pitch*T*0.5f,          a_pitch*T*0.5f,            a_pitch*T*0.5f,           -a_pitch*T*0.5f},
    {-a_yaw*T*T/2.0f,        a_yaw*T*T/2.0f,          -a_yaw*T*T/2.0f,          a_yaw*T*T/2.0f,            a_yaw*T*T/2.0f,           -a_yaw*T*T/2.0f},
    {-a_yaw*T,               a_yaw*T,                 -a_yaw*T,                 a_yaw*T,                   a_yaw*T,                  -a_yaw*T}
};

float B_State_1[8][6] = {
    {0,   kt*T*T/(2.0f*m_hexa),    kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa),      kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa)},
    {0,   kt*T/m_hexa,             kt*T/m_hexa,              kt*T/m_hexa,               kt*T/m_hexa,              kt*T/m_hexa},
    {0,   0.0f,                    a_roll*T*T*0.866f/2.0f,   -a_roll*T*T*0.866f/2.0f,   a_roll*T*T*0.866f/2.0f,   -a_roll*T*T*0.866f/2.0f},
    {0,   0.0f,                    a_roll*T*0.866f,          -a_roll*T*0.866f,          a_roll*T*0.866f,          -a_roll*T*0.866f},
    {0,   -a_pitch*T*T/2.0f,       -a_pitch*T*T*0.5f/2.0f,   a_pitch*T*T*0.5f/2.0f,     a_pitch*T*T*0.5f/2.0f,    -a_pitch*T*T*0.5f/2.f},
    {0,   -a_pitch*T,              -a_pitch*T*0.5f,          a_pitch*T*0.5f,            a_pitch*T*0.5f,           -a_pitch*T*0.5f},
    {0,   a_yaw*T*T/2.0f,          -a_yaw*T*T/2.0f,          a_yaw*T*T/2.0f,            a_yaw*T*T/2.0f,           -a_yaw*T*T/2.0f},
    {0,   a_yaw*T,                 -a_yaw*T,                 a_yaw*T,                   a_yaw*T,                  -a_yaw*T}
};

float B_State_2[8][6] = {
    {kt*T*T/(2.0f*m_hexa),   0,    kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa),      kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa)},
    {kt*T/m_hexa,            0,    kt*T/m_hexa,              kt*T/m_hexa,               kt*T/m_hexa,              kt*T/m_hexa},
    {0.0f,                   0,    a_roll*T*T*0.866f/2.0f,   -a_roll*T*T*0.866f/2.0f,   a_roll*T*T*0.866f/2.0f,   -a_roll*T*T*0.866f/2.0f},
    {0.0f,                   0,    a_roll*T*0.866f,          -a_roll*T*0.866f,          a_roll*T*0.866f,          -a_roll*T*0.866f},
    {a_pitch*T*T/2.0f,       0,    -a_pitch*T*T*0.5f/2.0f,   a_pitch*T*T*0.5f/2.0f,     a_pitch*T*T*0.5f/2.0f,    -a_pitch*T*T*0.5f/2.f},
    {a_pitch*T,              0,    -a_pitch*T*0.5f,          a_pitch*T*0.5f,            a_pitch*T*0.5f,           -a_pitch*T*0.5f},
    {-a_yaw*T*T/2.0f,        0,    -a_yaw*T*T/2.0f,          a_yaw*T*T/2.0f,            a_yaw*T*T/2.0f,           -a_yaw*T*T/2.0f},
    {-a_yaw*T,               0,    -a_yaw*T,                 a_yaw*T,                   a_yaw*T,                  -a_yaw*T}
};

float B_State_3[8][6] = {
    {kt*T*T/(2.0f*m_hexa),   kt*T*T/(2.0f*m_hexa),    0,   kt*T*T/(2.0f*m_hexa),      kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa)},
    {kt*T/m_hexa,            kt*T/m_hexa,             0,   kt*T/m_hexa,               kt*T/m_hexa,              kt*T/m_hexa},
    {0.0f,                   0.0f,                    0,   -a_roll*T*T*0.866f/2.0f,   a_roll*T*T*0.866f/2.0f,   -a_roll*T*T*0.866f/2.0f},
    {0.0f,                   0.0f,                    0,   -a_roll*T*0.866f,          a_roll*T*0.866f,          -a_roll*T*0.866f},
    {a_pitch*T*T/2.0f,       -a_pitch*T*T/2.0f,       0,   a_pitch*T*T*0.5f/2.0f,     a_pitch*T*T*0.5f/2.0f,    -a_pitch*T*T*0.5f/2.f},
    {a_pitch*T,              -a_pitch*T,              0,   a_pitch*T*0.5f,            a_pitch*T*0.5f,           -a_pitch*T*0.5f},
    {-a_yaw*T*T/2.0f,        a_yaw*T*T/2.0f,          0,   a_yaw*T*T/2.0f,            a_yaw*T*T/2.0f,           -a_yaw*T*T/2.0f},
    {-a_yaw*T,               a_yaw*T,                 0,   a_yaw*T,                   a_yaw*T,                  -a_yaw*T}
};

float B_State_4[8][6] = {
    {kt*T*T/(2.0f*m_hexa),   kt*T*T/(2.0f*m_hexa),    kt*T*T/(2.0f*m_hexa),     0,   kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa)},
    {kt*T/m_hexa,            kt*T/m_hexa,             kt*T/m_hexa,              0,   kt*T/m_hexa,              kt*T/m_hexa},
    {0.0f,                   0.0f,                    a_roll*T*T*0.866f/2.0f,   0,   a_roll*T*T*0.866f/2.0f,   -a_roll*T*T*0.866f/2.0f},
    {0.0f,                   0.0f,                    a_roll*T*0.866f,          0,   a_roll*T*0.866f,          -a_roll*T*0.866f},
    {a_pitch*T*T/2.0f,       -a_pitch*T*T/2.0f,       -a_pitch*T*T*0.5f/2.0f,   0,   a_pitch*T*T*0.5f/2.0f,    -a_pitch*T*T*0.5f/2.f},
    {a_pitch*T,              -a_pitch*T,              -a_pitch*T*0.5f,          0,   a_pitch*T*0.5f,           -a_pitch*T*0.5f},
    {-a_yaw*T*T/2.0f,        a_yaw*T*T/2.0f,          -a_yaw*T*T/2.0f,          0,   a_yaw*T*T/2.0f,           -a_yaw*T*T/2.0f},
    {-a_yaw*T,               a_yaw*T,                 -a_yaw*T,                 0,   a_yaw*T,                  -a_yaw*T}
};

float B_State_5[8][6] = {
    {kt*T*T/(2.0f*m_hexa),   kt*T*T/(2.0f*m_hexa),    kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa),      0,   kt*T*T/(2.0f*m_hexa)},
    {kt*T/m_hexa,            kt*T/m_hexa,             kt*T/m_hexa,              kt*T/m_hexa,               0,   kt*T/m_hexa},
    {0.0f,                   0.0f,                    a_roll*T*T*0.866f/2.0f,   -a_roll*T*T*0.866f/2.0f,   0,   -a_roll*T*T*0.866f/2.0f},
    {0.0f,                   0.0f,                    a_roll*T*0.866f,          -a_roll*T*0.866f,          0,   -a_roll*T*0.866f},
    {a_pitch*T*T/2.0f,       -a_pitch*T*T/2.0f,       -a_pitch*T*T*0.5f/2.0f,   a_pitch*T*T*0.5f/2.0f,     0,   -a_pitch*T*T*0.5f/2.f},
    {a_pitch*T,              -a_pitch*T,              -a_pitch*T*0.5f,          a_pitch*T*0.5f,            0,   -a_pitch*T*0.5f},
    {-a_yaw*T*T/2.0f,        a_yaw*T*T/2.0f,          -a_yaw*T*T/2.0f,          a_yaw*T*T/2.0f,            0,   -a_yaw*T*T/2.0f},
    {-a_yaw*T,               a_yaw*T,                 -a_yaw*T,                 a_yaw*T,                   0,   -a_yaw*T}
};

float B_State_6[8][6] = {
    {kt*T*T/(2.0f*m_hexa),   kt*T*T/(2.0f*m_hexa),    kt*T*T/(2.0f*m_hexa),     kt*T*T/(2.0f*m_hexa),      kt*T*T/(2.0f*m_hexa),     0},
    {kt*T/m_hexa,            kt*T/m_hexa,             kt*T/m_hexa,              kt*T/m_hexa,               kt*T/m_hexa,              0},
    {0.0f,                   0.0f,                    a_roll*T*T*0.866f/2.0f,   -a_roll*T*T*0.866f/2.0f,   a_roll*T*T*0.866f/2.0f,   0},
    {0.0f,                   0.0f,                    a_roll*T*0.866f,          -a_roll*T*0.866f,          a_roll*T*0.866f,          0},
    {a_pitch*T*T/2.0f,       -a_pitch*T*T/2.0f,       -a_pitch*T*T*0.5f/2.0f,   a_pitch*T*T*0.5f/2.0f,     a_pitch*T*T*0.5f/2.0f,    0},
    {a_pitch*T,              -a_pitch*T,              -a_pitch*T*0.5f,          a_pitch*T*0.5f,            a_pitch*T*0.5f,           0},
    {-a_yaw*T*T/2.0f,        a_yaw*T*T/2.0f,          -a_yaw*T*T/2.0f,          a_yaw*T*T/2.0f,            a_yaw*T*T/2.0f,           0},
    {-a_yaw*T,               a_yaw*T,                 -a_yaw*T,                 a_yaw*T,                   a_yaw*T,                  0}
};

#endif



//定义输出矩阵
float C_State[4][8] = {
    {1, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 0}
};

//迭代初始化
float Mu_last[7] = {0};

float Mu_next[7];

//float x_last[8][7];
float x_last_0[8];float x_last_1[8];float x_last_2[8];float x_last_3[8];
float x_last_4[8];float x_last_5[8];float x_last_6[8];

float P_last_0[8][8];float P_last_1[8][8];float P_last_2[8][8];float P_last_3[8][8];
float P_last_4[8][8];float P_last_5[8][8];float P_last_6[8][8];                     //P_last为过程方差矩阵

float u_i_j[7][7];

//float x_mix[8][7];
float x_mix_0[8];float x_mix_1[8];float x_mix_2[8];float x_mix_3[8];
float x_mix_4[8];float x_mix_5[8];float x_mix_6[8];

float P_mix_0[8][8];float P_mix_1[8][8];float P_mix_2[8][8];float P_mix_3[8][8];
float P_mix_4[8][8];float P_mix_5[8][8];float P_mix_6[8][8];

//float x_pre[8][7];
float x_pre_0[8];float x_pre_1[8];float x_pre_2[8];float x_pre_3[8];
float x_pre_4[8];float x_pre_5[8];float x_pre_6[8];

float x_next_0[8];float x_next_1[8];float x_next_2[8];float x_next_3[8];
float x_next_4[8];float x_next_5[8];float x_next_6[8];

float P_pre_0[8][8];float P_pre_1[8][8];float P_pre_2[8][8];float P_pre_3[8][8];
float P_pre_4[8][8];float P_pre_5[8][8];float P_pre_6[8][8];

float P_next_0[8][8];float P_next_1[8][8];float P_next_2[8][8];float P_next_3[8][8];
float P_next_4[8][8];float P_next_5[8][8];float P_next_6[8][8];

float eye[8][8] = {//定义单位矩阵
    {1,0,0,0,0,0,0,0},
    {0,1,0,0,0,0,0,0},
    {0,0,1,0,0,0,0,0},
    {0,0,0,1,0,0,0,0},
    {0,0,0,0,1,0,0,0},
    {0,0,0,0,0,1,0,0},
    {0,0,0,0,0,0,1,0},
    {0,0,0,0,0,0,0,1},
};

float k_gain_0[8][4];float k_gain_1[8][4];float k_gain_2[8][4];float k_gain_3[8][4];
float k_gain_4[8][4];float k_gain_5[8][4];float k_gain_6[8][4];

//float z_res[4][7];
float z_res_0[4];float z_res_1[4];float z_res_2[4];float z_res_3[4];
float z_res_4[4];float z_res_5[4];float z_res_6[4];

float S_0[4][4],S_1[4][4],S_2[4][4],S_3[4][4],S_4[4][4],S_5[4][4],S_6[4][4];

int8_t model_number = 10;



int8_t i,j;
//----------------------------------------------------计算观测方差--------------------------------------------------------------------
    //先计算C*P_pre
    float C_P_0[4][8],C_P_1[4][8],C_P_2[4][8],C_P_3[4][8],C_P_4[4][8],C_P_5[4][8],C_P_6[4][8];

//------------------------------------------求解观测方差S的逆---------------------------------------------------------------
    float S_inv_0[4][4],S_inv_1[4][4],S_inv_2[4][4],S_inv_3[4][4],S_inv_4[4][4],S_inv_5[4][4],S_inv_6[4][4];

//---------------------------------------------求解卡尔曼增益---------------------------------------------------------------------
    //先计算P*C'
    float P_C_0[8][4],P_C_1[8][4],P_C_2[8][4],P_C_3[8][4],P_C_4[8][4],P_C_5[8][4],P_C_6[8][4];

//--------------------------------------计算P_next--------------------------------------------------------------------------------
    //先计算E-KC
    float E_KC_0[8][8];float E_KC_1[8][8];float E_KC_2[8][8];float E_KC_3[8][8];float E_KC_4[8][8];float E_KC_5[8][8];float E_KC_6[8][8];

//-------------------------------------------------模型可能性计算----------------------------------------------------------------
    //先计算Z_res'*S_inv*Z_res
    //先计算Z_res'*S_inv
    float ZT_SI_0[4];float ZT_SI_1[4];float ZT_SI_2[4];float ZT_SI_3[4];float ZT_SI_4[4];float ZT_SI_5[4];float ZT_SI_6[4];
    //再计算ZT_SI*Z_res
    float ZSZ_0;float ZSZ_1;float ZSZ_2;float ZSZ_3;float ZSZ_4;float ZSZ_5;float ZSZ_6;
    
    //计算S的行列式
    float S_0_det;float S_1_det;float S_2_det;float S_3_det;float S_4_det;float S_5_det; float S_6_det;

    //计算模型可能性
    float Mu_pre[7];

//-------------------------------------------------概率更新----------------------------------------------------------------------
    float C_IMM;
    float C_IMM_b[7];
