#include "Copter.h"

#include <cmath>
#include <math.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

//定义矩阵运算中时间T的值
#define T 0.0025
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

/*
//定义模型概率转移矩阵
float prob[5][5] = {
    {0.95,0.125,0.125,0.125,0.125},
    {0.125,0.95,0.125,0.125,0.125},
    {0.125,0.125,0.95,0.125,0.125},
    {0.125,0.125,0.125,0.95,0.125},
    {0.125,0.125,0.125,0.125,0.95},
};
*/

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

//计算[0,-gt,0,0,0,0,0,0]'*ones(1,5)
float gT_o[8][5] = {
                    {   0,   0,   0,   0,   0},
                    {-g*T,-g*T,-g*T,-g*T,-g*T},
                    {   0,   0,   0,   0,   0},
                    {   0,   0,   0,   0,   0},
                    {   0,   0,   0,   0,   0}
    };

//定义观测矩阵
float H[4][8] = {
    {1,0,0,0,0,0,0,0},
    {0,0,1,0,0,0,0,0},
    {0,0,0,0,1,0,0,0},
    {0,0,0,0,0,0,1,0},
};

//定义过程噪声协方差矩阵
float Q_KF[8][8] = {
    {0.000001,0,0,0,0,0,0,0},
    {0,0.000001,0,0,0,0,0,0},
    {0,0,0.000000000001,0,0,0,0,0},
    {0,0,0,0.000000000001,0,0,0,0},
    {0,0,0,0,0.000000000001,0,0,0},
    {0,0,0,0,0,0.000000000001,0,0},
    {0,0,0,0,0,0,0.000000000001,0},
    {0,0,0,0,0,0,0,0.000000000001},
};

//定义测量噪声协方差矩阵
float R_KF[4][4] = {
    {0.000001,0,0,0},
    {0,0.000000000001,0,0},
    {0,0,0.000000000001,0},
    {0,0,0,0.000000000001}
};

//定义8阶单位矩阵
float Eye[8][8] = {
    {1,0,0,0,0,0,0,0},
    {0,1,0,0,0,0,0,0},
    {0,0,1,0,0,0,0,0},
    {0,0,0,1,0,0,0,0},
    {0,0,0,0,1,0,0,0},
    {0,0,0,0,0,1,0,0},
    {0,0,0,0,0,0,1,0},
    {0,0,0,0,0,0,0,1},
};

//初始化X_next,P_next,Mu_next
float X_next[8][5] = {0};
float P_next[8][8][5] = {0};
float Mu_next[5] = {0};

int i,j,k,ii;
float M_o[5][5];
float X_mix[8][5];
float P_mix[8][8][5];
float x_1[8][5][5];
float x_11[8][8][5][5];
float P_1[8][8][5];

float Mu_last[5] = {0};
float X_last[8][5];
float P_last[8][8][5];

float X_pre[8][5];
float A_m_X_last[8][5];
float B_m_Uin[8];
float B_m_Uin_o[8][5];
float diag_Uin[4][4] = {0};
float diag_Uin_m_fault[4][5];
float B_m_diag_Uin_m_fault[8][5]; //计算X_pre所用参数

float Z_res[4][5];
float Zin_o[4][5];
float H_m_X_pre[4][5]; //计算Z_res所用参数

float P_pre[8][8][5] = {0};
float K_gain[8][4][5] = {0};
//float mu[5] = {0};
float A_m_P_last[8][8][5]; //计算P_pre所用参数

float S[4][4][5]; //观测方差
float H_m_P_pre[4][8][5];
float S_0[4][4];
float S_1[4][4];
float S_2[4][4];
float S_3[4][4];
float S_4[4][4]; //计算S

float S_0_inv[4][4];
float S_1_inv[4][4];
float S_2_inv[4][4];
float S_3_inv[4][4];
float S_4_inv[4][4];
float S_rev[4][4][5]; //计算S逆

float P_pre_m_H_T[8][4][5]; //计算K_gain

float K_m_Z_res[8][5]; //计算X_next

float E_Sub_KH[8][8][5]; //计算P_next

float bais[5];
float Z_T_S[5][4]; //计算bais

float mu[5]; //计算mu

float sum_mu; //计算Mu_next

float IMM_Model; //输出模型


float Copter::IMM_KF(float Zin[4],float Uin[4],float X_real[8])
{
    if((Mu_last[0]+Mu_last[1]+Mu_last[2]+Mu_last[3]+Mu_last[4])<0.01) //判断Mu_last是否为0
    {
        for(i=0;i<5;i++)
        {
            for(j=0;j<8;j++)
            {
                X_last[j][i]= {0};
            }
        }
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

    //
    //
    //并行滤波
    //
    //
    //------------------------------计算X_pre----------------------------------------------------------
    for(i=0;i<8;i++) //计算A*X_last
    {
        for(j=0;j<5;j++)
        {
            A_m_X_last[i][j] = A[i][0]*X_last[0][j]+A[i][1]*X_last[1][j]+A[i][2]*X_last[2][j]+A[i][3]*X_last[3][j]+
                               A[i][4]*X_last[4][j]+A[i][5]*X_last[5][j]+A[i][6]*X_last[6][j]+A[i][7]*X_last[7][j];
        }
    }

    for(i=0;i<8;i++) //计算B*Uin
    {
        B_m_Uin[i] = B[i][0]*Uin[0]+B[i][1]*Uin[1]+B[i][2]*Uin[2]+B[i][3]*Uin[3];
    }

    for(i=0;i<8;i++) //计算B*Uin*ones(1,5)
    {
        for(j=0;j<5;j++)
        {
            B_m_Uin_o[i][j] = B_m_Uin[i];
        }
    }

    for(i=0;i<4;i++) //计算diag(Uin)
    {
        diag_Uin[i][i] = Uin[i];
    }

    for(i=0;i<4;i++) //计算diag(Uin)*faultmodel(4,5)
    {
        for(j=0;j<5;j++)
        {
            diag_Uin_m_fault[i][j] = diag_Uin[i][0]*faultmodel[0][j]+diag_Uin[i][1]*faultmodel[1][j]+
                                     diag_Uin[i][2]*faultmodel[2][j]+diag_Uin[i][3]*faultmodel[3][j];
        }
    }

    for(i=0;i<8;i++) //计算B*diag(Uin)*faultmodel(4,5)
    {
        for(j=0;j<5;j++)
        {
            B_m_diag_Uin_m_fault[i][j] = B[i][0]*diag_Uin_m_fault[0][j]+B[i][1]*diag_Uin_m_fault[1][j]+
                                         B[i][2]*diag_Uin_m_fault[2][j]+B[i][3]*diag_Uin_m_fault[3][j];
        }
    }

    for(i=0;i<8;i++)
    {
        for(j=0;j<5;j++)
        {
            X_pre[i][j] = A_m_X_last[i][j]+B_m_Uin_o[i][j]-B_m_diag_Uin_m_fault[i][j]+gT_o[i][j];
        }
    }
    //-----------------------------------------------------------------------------------------------
    //-------------------------计算Z_res-------------------------------------------------------------
    for(i=0;i<4;i++) //计算ZIn*ones(1,5)
    {
        for(j=0;j<5;j++)
        {
            Zin_o[i][j] = Zin[i];
        }
    }

    for(i=0;i<4;i++)
    {
        for(j=0;j<5;j++)
        {
            H_m_X_pre[i][j] = H[i][0]*X_pre[0][j]+H[i][1]*X_pre[0][j]+H[i][2]*X_pre[0][j]+H[i][3]*X_pre[0][j]+
                              H[i][4]*X_pre[0][j]+H[i][5]*X_pre[0][j]+H[i][6]*X_pre[0][j]+H[i][7]*X_pre[0][j];
        }
    }

    for(i=0;i<4;i++)
    {
        for(j=0;j<5;j++)
        {
            Z_res[i][j] = Zin_o[i][j]-H_m_X_pre[i][j];
        }
    }
    //----------------------------------------------------------------------------------------------------
    //----------------------------------计算P_pre----------------------------------------------------------
    for(i=0;i<5;i++) //先计算A*P_mix(:,:,i)
    {
        for(j=0;j<8;j++)
        {
            for(k=0;k<8;k++)
            {
                A_m_P_last[j][k][i] = A[j][0]*P_last[0][k][i]+A[j][1]*P_last[1][k][i]+A[j][2]*P_last[2][k][i]+A[j][3]*P_last[3][k][i]+
                                     A[j][4]*P_last[4][k][i]+A[j][5]*P_last[5][k][i]+A[j][6]*P_last[6][k][i]+A[j][7]*P_last[7][k][i];
            }
        }
    }

    for(i=0;i<5;i++) //计算P_pre(:,:,i)=A*P_mix(:,:,i)*A'+Q
    {
        for(j=0;j<8;j++)
        {
            for(k=0;k<8;k++)
            {
               P_pre[j][k][i] = A_m_P_last[j][0][i]*A[k][0]+A_m_P_last[j][1][i]*A[k][1]+A_m_P_last[j][2][i]*A[k][2]+A_m_P_last[j][0][3]*A[k][3]+
                                A_m_P_last[j][4][i]*A[k][4]+A_m_P_last[j][5][i]*A[k][5]+A_m_P_last[j][6][i]*A[k][6]+A_m_P_last[j][7][i]*A[k][7]+Q_KF[j][k];
            }
        }
    }
    //------------------------------------------------------------------------------------------------
    //-------------------------------------计算S-------------------------------------------------------
    for(i=0;i<5;i++) //先计算H*P_pre(:,:,i)
    {
        for(j=0;j<4;j++)
        {
            for(k=0;k<8;k++)
            {
                H_m_P_pre[j][k][i] = H[j][0]*P_pre[0][k][i]+H[j][1]*P_pre[1][k][i]+H[j][2]*P_pre[2][k][i]+H[j][3]*P_pre[3][k][i]+
                                     H[j][4]*P_pre[4][k][i]+H[j][5]*P_pre[5][k][i]+H[j][6]*P_pre[6][k][i]+H[j][7]*P_pre[7][k][i];
            }
        }
    }

    for(i=0;i<5;i++) //计算S=H*P_pre(:,:,i)*H'+R
    {
        for(j=0;j<4;j++)
        {
            for(k=0;k<4;k++)
            {
               S[j][k][i] = H_m_P_pre[j][0][i]*H[k][0]+H_m_P_pre[j][1][i]*H[k][1]+H_m_P_pre[j][2][i]*H[k][2]+H_m_P_pre[j][3][i]*H[k][3]+
                            H_m_P_pre[j][4][i]*H[k][4]+H_m_P_pre[j][5][i]*H[k][5]+H_m_P_pre[j][6][i]*H[k][6]+H_m_P_pre[j][7][i]*H[k][7]+R_KF[j][k];
            }
        }
    }

    for(j=0;j<4;j++)
    {
        for(k=0;k<4;k++)
        {
           S_0[j][k] = S[j][k][0];
           S_1[j][k] = S[j][k][1];
           S_2[j][k] = S[j][k][2];
           S_3[j][k] = S[j][k][3];
           S_4[j][k] = S[j][k][4];
        }
    }        

    //------------------------------------------------------------------------------------------------
    //------------------------计算S的逆矩阵------------------------------------------------------------
    inverse4x4(*S_0,*S_0_inv);
    inverse4x4(*S_1,*S_1_inv);
    inverse4x4(*S_2,*S_2_inv);
    inverse4x4(*S_3,*S_3_inv);
    inverse4x4(*S_4,*S_4_inv);
 
    for(j=0;j<4;j++)
    {
        for(k=0;k<4;k++)
        {
           S_rev[j][k][0] = S_0_inv[j][k];
           S_rev[j][k][1] = S_1_inv[j][k];
           S_rev[j][k][2] = S_2_inv[j][k];
           S_rev[j][k][3] = S_3_inv[j][k];
           S_rev[j][k][4] = S_4_inv[j][k];
        }
    }   
    //------------------------------------------------------------------------------------------    
    //--------------------------计算K_gain卡尔曼增益--------------------------------------------- 
    for(i=0;i<5;i++) //计算P_pre(:,:,i)*H'
    {
        for(j=0;j<8;j++)
        {
            for(k=0;k<4;k++)
            {
               P_pre_m_H_T[j][k][i] = P_pre[j][0][i]*H[k][0]+P_pre[j][1][i]*H[k][1]+P_pre[j][2][i]*H[k][2]+P_pre[j][3][i]*H[k][3]+
                                      P_pre[j][4][i]*H[k][4]+P_pre[j][5][i]*H[k][5]+P_pre[j][6][i]*H[k][6]+P_pre[j][7][i]*H[k][7];
            }
        }
    }

    for(i=0;i<5;i++) //计算K(:,:,i)=P_pre(:,:,i)*H'*S_rev
    {
        for(j=0;j<8;j++)
        {
            for(k=0;k<4;k++)
            {
               K_gain[j][k][i] = P_pre_m_H_T[j][0][i]*S_rev[0][k][i]+P_pre_m_H_T[j][1][i]*S_rev[1][k][i]+
                                 P_pre_m_H_T[j][2][i]*S_rev[2][k][i]+P_pre_m_H_T[j][3][i]*S_rev[3][k][i];
            }
        }
    }
    //-----------------------------------------------------------------------------------------
    //---------------------------计算X_next-----------------------------------------------------
    for(i=0;i<5;i++) //先计算K(:,:,i)*Z_res(:,i)
    {
        for(j=0;j<8;j++)
        {
                K_m_Z_res[j][i] = K_gain[j][0][i]*Z_res[0][i]+K_gain[j][1][i]*Z_res[1][i]+
                                  K_gain[j][2][i]*Z_res[2][i]+K_gain[j][3][i]*Z_res[3][i];
        }
    }

    for(i=0;i<5;i++) //计算X_next = X_pre + K*Z_res
    {
        for(j=0;j<8;j++)
        {
            X_next[j][i] = X_pre[j][i]+K_m_Z_res[j][i];
        }
    }
    //----------------------------------------------------------------------------------------
    //--------------------------计算P_next----------------------------------------------------
    for(i=0;i<5;i++) //计算eye(8)-K(:,:,i)*H
    {
        for(j=0;j<8;j++)
        {
            for(k=0;k<8;k++)
            {
                 E_Sub_KH[j][k][i] = Eye[j][k]-(K_gain[j][0][i]*H[0][k]+K_gain[j][1][i]*H[1][k]+
                                                   K_gain[j][2][i]*H[2][k]+K_gain[j][3][i]*H[3][k]);
            }
        }
    }

    for(i=0;i<5;i++) //计算P_next = (eye(8)-K*H)*P_pre
    {
        for(j=0;j<8;j++)
        {
            for(k=0;k<8;k++)
            {
                P_next[j][k][i] = E_Sub_KH[j][0][i]*P_pre[0][k][i]+E_Sub_KH[j][1][i]*P_pre[1][k][i]+
                                  E_Sub_KH[j][2][i]*P_pre[2][k][i]+E_Sub_KH[j][3][i]*P_pre[3][k][i]+
                                  E_Sub_KH[j][4][i]*P_pre[4][k][i]+E_Sub_KH[j][5][i]*P_pre[5][k][i]+
                                  E_Sub_KH[j][6][i]*P_pre[6][k][i]+E_Sub_KH[j][6][i]*P_pre[6][k][i];
            }
        }
    }
    //------------------------------------------------------------------------------------------
    //--------------------------------计算bais---------------------------------------------------
    for(i=0;i<5;i++)
    {
        for(j=0;j<4;j++) //先计算Z_res*S_rev
        {
            Z_T_S[i][j] = Z_res[0][i]*S_rev[0][j][i]+Z_res[1][i]*S_rev[1][j][i]+
                          Z_res[2][i]*S_rev[2][j][i]+Z_res[3][i]*S_rev[3][j][i];
        }
    }

    for(i=0;i<5;i++) //再计算bais = Z_res'*S_rev*Z_res
    {
        bais[i] = Z_T_S[i][0]*Z_res[0][i]+Z_T_S[i][1]*Z_res[1][i]+
                  Z_T_S[i][2]*Z_res[2][i]+Z_T_S[i][3]*Z_res[3][i];
    }
    //-----------------------------------------------------------------------------------------
    //-----------------------------计算mu------------------------------------------------------
    for(i=0;i<5;i++)
    {
        mu[i] = 1/((bais[i]+0.00001)*(bais[i]+0.0001));
    }
    //-----------------------------------------------------------------------------------------
    //
    //
    //融合输出
    //
    //
    //----------------------------计算Mu_next--------------------------------------------------
    sum_mu = mu[0]+mu[1]+mu[2]+mu[3]+mu[4];
    for(i=0;i<5;i++)
    {
        Mu_next[i] = mu[i]/sum_mu;
    }

    if((Mu_next[0]+Mu_next[1]+Mu_next[2]+Mu_next[3]+Mu_next[4])<0.0001)
    {
        for(i=0;i<5;i++)
        {
            Mu_next[i] = 0.2;
        }
    }
    //----------------------------------------------------------------------------------------
    if(Mu_next[0]>0.5)
        IMM_Model = 0;
    else if(Mu_next[1]>0.5)
        IMM_Model = 1;
    else if(Mu_next[2]>0.5)
        IMM_Model = 2;
    else if(Mu_next[3]>0.5)
        IMM_Model = 3;
    else if(Mu_next[4]>0.5)
        IMM_Model = 4;
    else
        IMM_Model = 0;
    //----------------------------------------------------------------------------------------
    memcpy(Mu_last,Mu_next,sizeof(Mu_next));
    memcpy(P_last,P_next,sizeof(P_next));
    memcpy(X_last,X_next,sizeof(X_next));

    return IMM_Model;

}

float Copter::IMM_KF_Update()
{
    float Zin_R[4];
    //float Uin_R[4];
    float X_real_R[8];
    float X_real_R_d[3];
    Zin_R[0] = barometer.get_altitude();
    Zin_R[1] = quaternion.get_euler_roll();
    Zin_R[2] = quaternion.get_euler_pitch();
    Zin_R[3] = quaternion.get_euler_yaw(); 
    *X_real_R_d = attitude_control->rate_controller_run_IMM(); //该函数自行添加，为了获取陀螺仪角速度

    X_real_R[0] = barometer.get_altitude();
    X_real_R[1] = barometer.get_climb_rate();
    X_real_R[2] = quaternion.get_euler_roll();
    X_real_R[3] = X_real_R_d[0];
    X_real_R[4] = quaternion.get_euler_pitch();
    X_real_R[5] = X_real_R_d[1];
    X_real_R[6] = quaternion.get_euler_yaw();
    X_real_R[7] = X_real_R_d[2];

    

    return Zin_R[1]+Zin_R[2]+Zin_R[3]+Zin_R[0]+X_real_R[1];
}

