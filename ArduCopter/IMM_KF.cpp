#include "IMM_KF.h"

float U_IMM[6];
float x_IMM[8];
float Z_IMM[4];

void Copter::State_refresh(){
    float z,phi,theta,psai;
    float z_dot,phi_dot,theta_dot,psai_dot;
    int8_t number_IMM;

    z     = inertial_nav.get_altitude();
    phi   = attitude_control->get_att_target_euler_cd().x;
    theta = attitude_control->get_att_target_euler_cd().y;
    psai  = attitude_control->get_att_target_euler_cd().z;

//
//在AC_AttitudeControl_Multi.cpp中增加rate_controller_run_IMM函数，以获取角速度信息
//
    z_dot     = inertial_nav.get_velocity_z();
    phi_dot   = attitude_control->rate_controller_run_IMM().x;
    theta_dot = attitude_control->rate_controller_run_IMM().y;
    psai_dot  = attitude_control->rate_controller_run_IMM().z;

    x_IMM[0] = z/100.0f;
    x_IMM[1] = z_dot/100.0f;
    x_IMM[2] = (phi/100.0f)/180.0f*3.14;
    x_IMM[3] = phi_dot;
    x_IMM[4] = (theta/100.0f)/180.0f*3.14;
    x_IMM[5] = theta_dot;
    x_IMM[6] = (psai/100.0f)/180.0f*3.14;
    x_IMM[7] = psai_dot;

    Z_IMM[0] = z/100.0f;
    Z_IMM[1] = (phi/100.0f)/180.0f*3.14;
    Z_IMM[2] = (theta/100.0f)/180.0f*3.14;
    Z_IMM[3] = (psai/100.0f)/180.0f*3.14;

    //hal.console->printf("%.4f,%.4f,%.4f\n",x_IMM[2],x_IMM[4],x_IMM[6]);

    float x_1_IMM[8] = {5, 0, 0, 0, 0, 0, 0, 0};
    float U_1_IMM[6] = {1560,1560,1560,1560,1560,1560};
    float Z_1_IMM[4] = {5, 0, 0, 0};
    number_IMM = IMM_Kalman_Filter(x_1_IMM,U_1_IMM,Z_1_IMM);
    hal.console->printf("%d",number_IMM);

    /* if(x_IMM[0]>3.0f){
        number_IMM = IMM_Kalman_Filter(x_IMM,U_IMM,Z_IMM);

        hal.console->printf("%d",number_IMM);
    } */

}

//----------------------------------------------------------------------------------------------------------------------------------
//计算四阶方阵行列式
float Copter::get_mat_det(float a[4][4])
{
    float a_det,b_det,c_det,d_det;

    a_det = a[0][0]*(a[1][1]*a[2][2]*a[3][3]
                    -a[1][1]*a[2][3]*a[3][2]
                    -a[1][2]*a[2][1]*a[3][3]
                    +a[1][2]*a[2][3]*a[3][1]
                    +a[1][3]*a[2][1]*a[3][2]
                    -a[1][3]*a[2][2]*a[3][1]);

    b_det = a[0][1]*(-a[1][0]*a[2][2]*a[3][3]
                     +a[1][0]*a[2][3]*a[3][2]
                     +a[1][2]*a[2][0]*a[3][3]
                     -a[1][2]*a[2][3]*a[3][0]
                     -a[1][3]*a[2][0]*a[3][2]
                     +a[1][3]*a[2][2]*a[3][0]);

    c_det = a[0][2]*( a[1][0]*a[2][1]*a[3][3]
                     -a[1][0]*a[2][3]*a[3][1]
                     -a[1][1]*a[2][0]*a[3][3]
                     +a[1][1]*a[2][3]*a[3][0]
                     +a[1][3]*a[2][0]*a[3][1]
                     -a[1][3]*a[2][1]*a[3][0]);

    d_det = a[0][3]*(-a[1][0]*a[2][1]*a[3][2]
                     +a[1][0]*a[2][2]*a[3][1]
                     +a[1][1]*a[2][0]*a[3][2]
                     -a[1][1]*a[2][2]*a[3][0]
                     -a[1][2]*a[2][0]*a[3][1]
                     +a[1][2]*a[2][1]*a[3][0]);

    return a_det+b_det+c_det+d_det;

}

//----------------------------------------------------------------------------------------------------------------------------------


//
// 编写多模型滤波函数
// 

//交互式多模型卡尔曼滤波函数
int Copter::IMM_Kalman_Filter(float x_real[8],float U_in[6],float z_real[4])
{
    //初次运行时的初始化
    if(Mu_last[0]+Mu_last[1]+Mu_last[2]+Mu_last[3]+Mu_last[4]+Mu_last[5]+Mu_last[6] <0.1 ){
        for(i=0;i<8;i++){
            x_last_0[i] = x_real[i];x_last_1[i] = x_real[i];x_last_2[i] = x_real[i];x_last_3[i] = x_real[i];
            x_last_4[i] = x_real[i];x_last_5[i] = x_real[i];x_last_6[i] = x_real[i];
        }

        for(i=0;i<8;i++){
            for(j=0;j<8;j++){
                if(i==j){
                    P_last_0[i][j] = 1.0f; P_last_1[i][j] = 1.0f; P_last_2[i][j] = 1.0f; P_last_3[i][j] = 1.0f;
                    P_last_4[i][j] = 1.0f; P_last_5[i][j] = 1.0f; P_last_6[i][j] = 1.0f;
                }else{
                    P_last_0[i][j] = 0.0f; P_last_1[i][j] = 0.0f; P_last_2[i][j] = 0.0f; P_last_3[i][j] = 0.0f;
                    P_last_4[i][j] = 0.0f; P_last_5[i][j] = 0.0f; P_last_6[i][j] = 0.0f;
                }
            }
        }

        for(i=0;i<7;i++){
            Mu_last[i] = 0.143;
        }
    }

    //
    //计算uij----------------------------------------------------------------------------------------------------------------
    //
   for(j=0;j<7;j++){
       for(i=0;i<7;i++){
           u_i_j[i][j] = p_i_j[i][j]*Mu_last[i]/(p_i_j[0][j]*Mu_last[0]+p_i_j[1][j]*Mu_last[1]+p_i_j[2][j]*Mu_last[2]+
                                                 p_i_j[3][j]*Mu_last[3]+p_i_j[4][j]*Mu_last[4]+p_i_j[5][j]*Mu_last[5]+
                                                 p_i_j[6][j]*Mu_last[6]);
       }
   }
    //-----------------------------------------------------输入交互-----------------------------------------------------------------

    for(i=0;i<7;i++){
        x_mix_0[i] = x_last_0[i]*u_i_j[0][0] +x_last_1[i]*u_i_j[1][0] +x_last_2[i]*u_i_j[2][0] +x_last_3[i]*u_i_j[3][0] +
                     x_last_4[i]*u_i_j[4][0] +x_last_5[i]*u_i_j[5][0] +x_last_6[i]*u_i_j[6][0];

        x_mix_1[i] = x_last_0[i]*u_i_j[0][1] +x_last_1[i]*u_i_j[1][1] +x_last_2[i]*u_i_j[2][1] +x_last_3[i]*u_i_j[3][1] +
                     x_last_4[i]*u_i_j[4][1] +x_last_5[i]*u_i_j[5][1] +x_last_6[i]*u_i_j[6][1];

        x_mix_2[i] = x_last_0[i]*u_i_j[0][2] +x_last_1[i]*u_i_j[1][2] +x_last_2[i]*u_i_j[2][2] +x_last_3[i]*u_i_j[3][2] +
                     x_last_4[i]*u_i_j[4][2] +x_last_5[i]*u_i_j[5][2] +x_last_6[i]*u_i_j[6][2];

        x_mix_3[i] = x_last_0[i]*u_i_j[0][3] +x_last_1[i]*u_i_j[1][3] +x_last_2[i]*u_i_j[2][3] +x_last_3[i]*u_i_j[3][3] +
                     x_last_4[i]*u_i_j[4][3] +x_last_5[i]*u_i_j[5][3] +x_last_6[i]*u_i_j[6][3];

        x_mix_4[i] = x_last_0[i]*u_i_j[0][4] +x_last_1[i]*u_i_j[1][4] +x_last_2[i]*u_i_j[2][4] +x_last_3[i]*u_i_j[3][4] +
                     x_last_4[i]*u_i_j[4][4] +x_last_5[i]*u_i_j[5][4] +x_last_6[i]*u_i_j[6][4];
    
        x_mix_5[i] = x_last_0[i]*u_i_j[0][5] +x_last_1[i]*u_i_j[1][5] +x_last_2[i]*u_i_j[2][5] +x_last_3[i]*u_i_j[3][5] +
                     x_last_4[i]*u_i_j[4][5] +x_last_5[i]*u_i_j[5][5] +x_last_6[i]*u_i_j[6][5];
    
        x_mix_6[i] = x_last_0[i]*u_i_j[0][6] +x_last_1[i]*u_i_j[1][6] +x_last_2[i]*u_i_j[2][6] +x_last_3[i]*u_i_j[3][6] +
                     x_last_4[i]*u_i_j[4][6] +x_last_5[i]*u_i_j[5][6] +x_last_6[i]*u_i_j[6][6];
    }


    for(i=0;i<8;i++){
        for(j=0;j<8;j++){
            P_mix_0[i][j] = (P_last_0[i][j]+(x_last_0[i]-x_mix_0[i])*(x_last_0[j]-x_mix_0[j]))*u_i_j[0][0]
                           +(P_last_1[i][j]+(x_last_1[i]-x_mix_0[i])*(x_last_1[j]-x_mix_0[j]))*u_i_j[1][0]
                           +(P_last_2[i][j]+(x_last_2[i]-x_mix_0[i])*(x_last_2[j]-x_mix_0[j]))*u_i_j[2][0]
                           +(P_last_3[i][j]+(x_last_3[i]-x_mix_0[i])*(x_last_3[j]-x_mix_0[j]))*u_i_j[3][0]
                           +(P_last_4[i][j]+(x_last_4[i]-x_mix_0[i])*(x_last_4[j]-x_mix_0[j]))*u_i_j[4][0]
                           +(P_last_5[i][j]+(x_last_5[i]-x_mix_0[i])*(x_last_5[j]-x_mix_0[j]))*u_i_j[5][0]
                           +(P_last_6[i][j]+(x_last_6[i]-x_mix_0[i])*(x_last_6[j]-x_mix_0[j]))*u_i_j[6][0];

            P_mix_1[i][j] = (P_last_0[i][j]+(x_last_0[i]-x_mix_1[i])*(x_last_0[j]-x_mix_1[j]))*u_i_j[0][1]
                           +(P_last_1[i][j]+(x_last_1[i]-x_mix_1[i])*(x_last_1[j]-x_mix_1[j]))*u_i_j[1][1]
                           +(P_last_2[i][j]+(x_last_2[i]-x_mix_1[i])*(x_last_2[j]-x_mix_1[j]))*u_i_j[2][1]
                           +(P_last_3[i][j]+(x_last_3[i]-x_mix_1[i])*(x_last_3[j]-x_mix_1[j]))*u_i_j[3][1]
                           +(P_last_4[i][j]+(x_last_4[i]-x_mix_1[i])*(x_last_4[j]-x_mix_1[j]))*u_i_j[4][1]
                           +(P_last_5[i][j]+(x_last_5[i]-x_mix_1[i])*(x_last_5[j]-x_mix_1[j]))*u_i_j[5][1]
                           +(P_last_6[i][j]+(x_last_6[i]-x_mix_1[i])*(x_last_6[j]-x_mix_1[j]))*u_i_j[6][1];

            P_mix_2[i][j] = (P_last_0[i][j]+(x_last_0[i]-x_mix_2[i])*(x_last_0[j]-x_mix_2[j]))*u_i_j[0][2]
                           +(P_last_1[i][j]+(x_last_1[i]-x_mix_2[i])*(x_last_1[j]-x_mix_2[j]))*u_i_j[1][2]
                           +(P_last_2[i][j]+(x_last_2[i]-x_mix_2[i])*(x_last_2[j]-x_mix_2[j]))*u_i_j[2][2]
                           +(P_last_3[i][j]+(x_last_3[i]-x_mix_2[i])*(x_last_3[j]-x_mix_2[j]))*u_i_j[3][2]
                           +(P_last_4[i][j]+(x_last_4[i]-x_mix_2[i])*(x_last_4[j]-x_mix_2[j]))*u_i_j[4][2]
                           +(P_last_5[i][j]+(x_last_5[i]-x_mix_2[i])*(x_last_5[j]-x_mix_2[j]))*u_i_j[5][2]
                           +(P_last_6[i][j]+(x_last_6[i]-x_mix_2[i])*(x_last_6[j]-x_mix_2[j]))*u_i_j[6][2];

            P_mix_3[i][j] = (P_last_0[i][j]+(x_last_0[i]-x_mix_3[i])*(x_last_0[j]-x_mix_3[j]))*u_i_j[0][3]
                           +(P_last_1[i][j]+(x_last_1[i]-x_mix_3[i])*(x_last_1[j]-x_mix_3[j]))*u_i_j[1][3]
                           +(P_last_2[i][j]+(x_last_2[i]-x_mix_3[i])*(x_last_2[j]-x_mix_3[j]))*u_i_j[2][3]
                           +(P_last_3[i][j]+(x_last_3[i]-x_mix_3[i])*(x_last_3[j]-x_mix_3[j]))*u_i_j[3][3]
                           +(P_last_4[i][j]+(x_last_4[i]-x_mix_3[i])*(x_last_4[j]-x_mix_3[j]))*u_i_j[4][3]
                           +(P_last_5[i][j]+(x_last_5[i]-x_mix_3[i])*(x_last_5[j]-x_mix_3[j]))*u_i_j[5][3]
                           +(P_last_6[i][j]+(x_last_6[i]-x_mix_3[i])*(x_last_6[j]-x_mix_3[j]))*u_i_j[6][3];

            P_mix_4[i][j] = (P_last_0[i][j]+(x_last_0[i]-x_mix_4[i])*(x_last_0[j]-x_mix_4[j]))*u_i_j[0][4]
                           +(P_last_1[i][j]+(x_last_1[i]-x_mix_4[i])*(x_last_1[j]-x_mix_4[j]))*u_i_j[1][4]
                           +(P_last_2[i][j]+(x_last_2[i]-x_mix_4[i])*(x_last_2[j]-x_mix_4[j]))*u_i_j[2][4]
                           +(P_last_3[i][j]+(x_last_3[i]-x_mix_4[i])*(x_last_3[j]-x_mix_4[j]))*u_i_j[3][4]
                           +(P_last_4[i][j]+(x_last_4[i]-x_mix_4[i])*(x_last_4[j]-x_mix_4[j]))*u_i_j[4][4]
                           +(P_last_5[i][j]+(x_last_5[i]-x_mix_4[i])*(x_last_5[j]-x_mix_4[j]))*u_i_j[5][4]
                           +(P_last_6[i][j]+(x_last_6[i]-x_mix_4[i])*(x_last_6[j]-x_mix_4[j]))*u_i_j[6][4];

            P_mix_5[i][j] = (P_last_0[i][j]+(x_last_0[i]-x_mix_5[i])*(x_last_0[j]-x_mix_5[j]))*u_i_j[0][5]
                           +(P_last_1[i][j]+(x_last_1[i]-x_mix_5[i])*(x_last_1[j]-x_mix_5[j]))*u_i_j[1][5]
                           +(P_last_2[i][j]+(x_last_2[i]-x_mix_5[i])*(x_last_2[j]-x_mix_5[j]))*u_i_j[2][5]
                           +(P_last_3[i][j]+(x_last_3[i]-x_mix_5[i])*(x_last_3[j]-x_mix_5[j]))*u_i_j[3][5]
                           +(P_last_4[i][j]+(x_last_4[i]-x_mix_5[i])*(x_last_4[j]-x_mix_5[j]))*u_i_j[4][5]
                           +(P_last_5[i][j]+(x_last_5[i]-x_mix_5[i])*(x_last_5[j]-x_mix_5[j]))*u_i_j[5][5]
                           +(P_last_6[i][j]+(x_last_6[i]-x_mix_5[i])*(x_last_6[j]-x_mix_5[j]))*u_i_j[6][5];

            P_mix_6[i][j] = (P_last_0[i][j]+(x_last_0[i]-x_mix_6[i])*(x_last_0[j]-x_mix_6[j]))*u_i_j[0][6]
                           +(P_last_1[i][j]+(x_last_1[i]-x_mix_6[i])*(x_last_1[j]-x_mix_6[j]))*u_i_j[1][6]
                           +(P_last_2[i][j]+(x_last_2[i]-x_mix_6[i])*(x_last_2[j]-x_mix_6[j]))*u_i_j[2][6]
                           +(P_last_3[i][j]+(x_last_3[i]-x_mix_6[i])*(x_last_3[j]-x_mix_6[j]))*u_i_j[3][6]
                           +(P_last_4[i][j]+(x_last_4[i]-x_mix_6[i])*(x_last_4[j]-x_mix_6[j]))*u_i_j[4][6]
                           +(P_last_5[i][j]+(x_last_5[i]-x_mix_6[i])*(x_last_5[j]-x_mix_6[j]))*u_i_j[5][6]
                           +(P_last_6[i][j]+(x_last_6[i]-x_mix_6[i])*(x_last_6[j]-x_mix_6[j]))*u_i_j[6][6];
        }
    }

    //
    //并行滤波开始----------------------------------------------------------------------------------------------------------
    //

   for(i=0;i<8;i++){
       x_pre_0[i] = A_State[i][0]*x_mix_0[0]+A_State[i][1]*x_mix_0[1]+A_State[i][2]*x_mix_0[2]+A_State[i][3]*x_mix_0[3]
                   +A_State[i][4]*x_mix_0[4]+A_State[i][5]*x_mix_0[5]+A_State[i][6]*x_mix_0[6]+A_State[i][7]*x_mix_0[7]

                   +B_State[i][0]*U_in[0]+B_State[i][1]*U_in[1]+B_State[i][2]*U_in[2]
                   +B_State[i][3]*U_in[3]+B_State[i][4]*U_in[4]+B_State[i][5]*U_in[5]
                   
                   -B_g[i];
   }

   for(i=0;i<8;i++){
       x_pre_1[i] = A_State[i][0]*x_mix_1[0]+A_State[i][1]*x_mix_1[1]+A_State[i][2]*x_mix_1[2]+A_State[i][3]*x_mix_1[3]
                   +A_State[i][4]*x_mix_1[4]+A_State[i][5]*x_mix_1[5]+A_State[i][6]*x_mix_1[6]+A_State[i][7]*x_mix_1[7]

                   +B_State_1[i][0]*U_in[0]+B_State_1[i][1]*U_in[1]+B_State_1[i][2]*U_in[2]
                   +B_State_1[i][3]*U_in[3]+B_State_1[i][4]*U_in[4]+B_State_1[i][5]*U_in[5]
                   
                   -B_g[i];
   }

   for(i=0;i<8;i++){
       x_pre_2[i] = A_State[i][0]*x_mix_2[0]+A_State[i][1]*x_mix_2[1]+A_State[i][2]*x_mix_2[2]+A_State[i][3]*x_mix_2[3]
                   +A_State[i][4]*x_mix_2[4]+A_State[i][5]*x_mix_2[5]+A_State[i][6]*x_mix_2[6]+A_State[i][7]*x_mix_2[7]

                   +B_State_2[i][0]*U_in[0]+B_State_2[i][1]*U_in[1]+B_State_2[i][2]*U_in[2]
                   +B_State_2[i][3]*U_in[3]+B_State_2[i][4]*U_in[4]+B_State_2[i][5]*U_in[5]
                   
                   -B_g[i];
   }

   for(i=0;i<8;i++){
       x_pre_3[i] = A_State[i][0]*x_mix_3[0]+A_State[i][1]*x_mix_3[1]+A_State[i][2]*x_mix_3[2]+A_State[i][3]*x_mix_3[3]
                   +A_State[i][4]*x_mix_3[4]+A_State[i][5]*x_mix_3[5]+A_State[i][6]*x_mix_3[6]+A_State[i][7]*x_mix_3[7]

                   +B_State_3[i][0]*U_in[0]+B_State_3[i][1]*U_in[1]+B_State_3[i][2]*U_in[2]
                   +B_State_3[i][3]*U_in[3]+B_State_3[i][4]*U_in[4]+B_State_3[i][5]*U_in[5]
                   
                   -B_g[i];
   }

   for(i=0;i<8;i++){
       x_pre_4[i] = A_State[i][0]*x_mix_4[0]+A_State[i][1]*x_mix_4[1]+A_State[i][2]*x_mix_4[2]+A_State[i][3]*x_mix_4[3]
                   +A_State[i][4]*x_mix_4[4]+A_State[i][5]*x_mix_4[5]+A_State[i][6]*x_mix_4[6]+A_State[i][7]*x_mix_4[7]

                   +B_State_4[i][0]*U_in[0]+B_State_4[i][1]*U_in[1]+B_State_4[i][2]*U_in[2]
                   +B_State_4[i][3]*U_in[3]+B_State_4[i][4]*U_in[4]+B_State_4[i][5]*U_in[5]
                   
                   -B_g[i];
   }

   for(i=0;i<8;i++){
       x_pre_5[i] = A_State[i][0]*x_mix_5[0]+A_State[i][1]*x_mix_5[1]+A_State[i][2]*x_mix_5[2]+A_State[i][3]*x_mix_5[3]
                   +A_State[i][4]*x_mix_5[4]+A_State[i][5]*x_mix_5[5]+A_State[i][6]*x_mix_5[6]+A_State[i][7]*x_mix_5[7]

                   +B_State_5[i][0]*U_in[0]+B_State_5[i][1]*U_in[1]+B_State_5[i][2]*U_in[2]
                   +B_State_5[i][3]*U_in[3]+B_State_5[i][4]*U_in[4]+B_State_5[i][5]*U_in[5]
                   
                   -B_g[i];
   }

   for(i=0;i<8;i++){
       x_pre_6[i] = A_State[i][0]*x_mix_6[0]+A_State[i][1]*x_mix_6[1]+A_State[i][2]*x_mix_6[2]+A_State[i][3]*x_mix_6[3]
                   +A_State[i][4]*x_mix_6[4]+A_State[i][5]*x_mix_6[5]+A_State[i][6]*x_mix_6[6]+A_State[i][7]*x_mix_6[7]

                   +B_State_6[i][0]*U_in[0]+B_State_6[i][1]*U_in[1]+B_State_6[i][2]*U_in[2]
                   +B_State_6[i][3]*U_in[3]+B_State_6[i][4]*U_in[4]+B_State_6[i][5]*U_in[5]
                   
                   -B_g[i];
   }

    //-------------------------------------------------------计算残差---------------------------------------------------------------------
    for(i=0;i<3;i++){
        z_res_0[i] = z_real[i]-(C_State[i][0]*x_pre_0[0]+C_State[i][1]*x_pre_0[1]+C_State[i][2]*x_pre_0[2]+C_State[i][3]*x_pre_0[3]
                               +C_State[i][4]*x_pre_0[4]+C_State[i][5]*x_pre_0[5]+C_State[i][6]*x_pre_0[6]+C_State[i][7]*x_pre_0[7]);

        z_res_1[i] = z_real[i]-(C_State[i][0]*x_pre_1[0]+C_State[i][1]*x_pre_1[1]+C_State[i][2]*x_pre_1[2]+C_State[i][3]*x_pre_1[3]
                               +C_State[i][4]*x_pre_1[4]+C_State[i][5]*x_pre_1[5]+C_State[i][6]*x_pre_1[6]+C_State[i][7]*x_pre_1[7]);

        z_res_2[i] = z_real[i]-(C_State[i][0]*x_pre_2[0]+C_State[i][1]*x_pre_2[1]+C_State[i][2]*x_pre_2[2]+C_State[i][3]*x_pre_2[3]
                               +C_State[i][4]*x_pre_2[4]+C_State[i][5]*x_pre_2[5]+C_State[i][6]*x_pre_2[6]+C_State[i][7]*x_pre_2[7]);

        z_res_3[i] = z_real[i]-(C_State[i][0]*x_pre_3[0]+C_State[i][1]*x_pre_3[1]+C_State[i][2]*x_pre_3[2]+C_State[i][3]*x_pre_3[3]
                               +C_State[i][4]*x_pre_3[4]+C_State[i][5]*x_pre_3[5]+C_State[i][6]*x_pre_3[6]+C_State[i][7]*x_pre_3[7]);

        z_res_4[i] = z_real[i]-(C_State[i][0]*x_pre_4[0]+C_State[i][1]*x_pre_4[1]+C_State[i][2]*x_pre_4[2]+C_State[i][3]*x_pre_4[3]
                               +C_State[i][4]*x_pre_4[4]+C_State[i][5]*x_pre_4[5]+C_State[i][6]*x_pre_4[6]+C_State[i][7]*x_pre_4[7]);

        z_res_5[i] = z_real[i]-(C_State[i][0]*x_pre_5[0]+C_State[i][1]*x_pre_5[1]+C_State[i][2]*x_pre_5[2]+C_State[i][3]*x_pre_5[3]
                               +C_State[i][4]*x_pre_5[4]+C_State[i][5]*x_pre_5[5]+C_State[i][6]*x_pre_5[6]+C_State[i][7]*x_pre_5[7]);

        z_res_6[i] = z_real[i]-(C_State[i][0]*x_pre_6[0]+C_State[i][1]*x_pre_6[1]+C_State[i][2]*x_pre_6[2]+C_State[i][3]*x_pre_6[3]
                               +C_State[i][4]*x_pre_6[4]+C_State[i][5]*x_pre_6[5]+C_State[i][6]*x_pre_6[6]+C_State[i][7]*x_pre_6[7]);
    }

    //------------------------------------------------计算过程方差------------------------------------------------------------------------
    //先计算A*P_mix
    float A_P_0[8][8], A_P_1[8][8], A_P_2[8][8], A_P_3[8][8], A_P_4[8][8], A_P_5[8][8],A_P_6[8][8];
    for(i=0;i<8;i++){
        for(j=0;j<8;j++){
            A_P_0[i][j] = A_State[i][0]*P_mix_0[0][j]+A_State[i][1]*P_mix_0[1][j]+A_State[i][2]*P_mix_0[2][j]+A_State[i][3]*P_mix_0[3][j]
                         +A_State[i][4]*P_mix_0[4][j]+A_State[i][5]*P_mix_0[5][j]+A_State[i][6]*P_mix_0[6][j]+A_State[i][7]*P_mix_0[7][j];

            A_P_1[i][j] = A_State[i][0]*P_mix_1[0][j]+A_State[i][1]*P_mix_1[1][j]+A_State[i][2]*P_mix_1[2][j]+A_State[i][3]*P_mix_1[3][j]
                         +A_State[i][4]*P_mix_1[4][j]+A_State[i][5]*P_mix_1[5][j]+A_State[i][6]*P_mix_1[6][j]+A_State[i][7]*P_mix_1[7][j];

            A_P_2[i][j] = A_State[i][0]*P_mix_2[0][j]+A_State[i][1]*P_mix_2[1][j]+A_State[i][2]*P_mix_2[2][j]+A_State[i][3]*P_mix_2[3][j]
                         +A_State[i][4]*P_mix_2[4][j]+A_State[i][5]*P_mix_2[5][j]+A_State[i][6]*P_mix_2[6][j]+A_State[i][7]*P_mix_2[7][j];

            A_P_3[i][j] = A_State[i][0]*P_mix_3[0][j]+A_State[i][1]*P_mix_3[1][j]+A_State[i][2]*P_mix_3[2][j]+A_State[i][3]*P_mix_3[3][j]
                         +A_State[i][4]*P_mix_3[4][j]+A_State[i][5]*P_mix_3[5][j]+A_State[i][6]*P_mix_3[6][j]+A_State[i][7]*P_mix_3[7][j];

            A_P_4[i][j] = A_State[i][0]*P_mix_4[0][j]+A_State[i][1]*P_mix_4[1][j]+A_State[i][2]*P_mix_4[2][j]+A_State[i][3]*P_mix_4[3][j]
                         +A_State[i][4]*P_mix_4[4][j]+A_State[i][5]*P_mix_4[5][j]+A_State[i][6]*P_mix_4[6][j]+A_State[i][7]*P_mix_4[7][j];

            A_P_5[i][j] = A_State[i][0]*P_mix_5[0][j]+A_State[i][1]*P_mix_5[1][j]+A_State[i][2]*P_mix_5[2][j]+A_State[i][3]*P_mix_5[3][j]
                         +A_State[i][4]*P_mix_5[4][j]+A_State[i][5]*P_mix_5[5][j]+A_State[i][6]*P_mix_5[6][j]+A_State[i][7]*P_mix_5[7][j];

            A_P_6[i][j] = A_State[i][0]*P_mix_6[0][j]+A_State[i][1]*P_mix_6[1][j]+A_State[i][2]*P_mix_6[2][j]+A_State[i][3]*P_mix_6[3][j]
                         +A_State[i][4]*P_mix_6[4][j]+A_State[i][5]*P_mix_6[5][j]+A_State[i][6]*P_mix_6[6][j]+A_State[i][7]*P_mix_6[7][j];
        }
    }

    //再计算A*P*A'
    float A_P_A_0[8][8],A_P_A_1[8][8],A_P_A_2[8][8],A_P_A_3[8][8],A_P_A_4[8][8],A_P_A_5[8][8],A_P_A_6[8][8];

    for(i=0;i<8;i++){
        for(j=0;j<8;j++){
            A_P_A_0[i][j] = A_P_0[i][0]*A_State[j][0]+A_P_0[i][1]*A_State[j][1]+A_P_0[i][2]*A_State[j][2]+A_P_0[i][3]*A_State[j][3]
                           +A_P_0[i][4]*A_State[j][4]+A_P_0[i][5]*A_State[j][5]+A_P_0[i][6]*A_State[j][6]+A_P_0[i][7]*A_State[j][7];

            A_P_A_1[i][j] = A_P_1[i][0]*A_State[j][0]+A_P_1[i][1]*A_State[j][1]+A_P_1[i][2]*A_State[j][2]+A_P_1[i][3]*A_State[j][3]
                           +A_P_1[i][4]*A_State[j][4]+A_P_1[i][5]*A_State[j][5]+A_P_1[i][6]*A_State[j][6]+A_P_1[i][7]*A_State[j][7];

            A_P_A_2[i][j] = A_P_2[i][0]*A_State[j][0]+A_P_2[i][1]*A_State[j][1]+A_P_2[i][2]*A_State[j][2]+A_P_2[i][3]*A_State[j][3]
                           +A_P_2[i][4]*A_State[j][4]+A_P_2[i][5]*A_State[j][5]+A_P_2[i][6]*A_State[j][6]+A_P_2[i][7]*A_State[j][7];

            A_P_A_3[i][j] = A_P_3[i][0]*A_State[j][0]+A_P_3[i][1]*A_State[j][1]+A_P_3[i][2]*A_State[j][2]+A_P_3[i][3]*A_State[j][3]
                           +A_P_3[i][4]*A_State[j][4]+A_P_3[i][5]*A_State[j][5]+A_P_3[i][6]*A_State[j][6]+A_P_3[i][7]*A_State[j][7];

            A_P_A_4[i][j] = A_P_4[i][0]*A_State[j][0]+A_P_4[i][1]*A_State[j][1]+A_P_4[i][2]*A_State[j][2]+A_P_4[i][3]*A_State[j][3]
                           +A_P_4[i][4]*A_State[j][4]+A_P_4[i][5]*A_State[j][5]+A_P_4[i][6]*A_State[j][6]+A_P_4[i][7]*A_State[j][7];

            A_P_A_5[i][j] = A_P_5[i][0]*A_State[j][0]+A_P_5[i][1]*A_State[j][1]+A_P_5[i][2]*A_State[j][2]+A_P_5[i][3]*A_State[j][3]
                           +A_P_5[i][4]*A_State[j][4]+A_P_5[i][5]*A_State[j][5]+A_P_5[i][6]*A_State[j][6]+A_P_5[i][7]*A_State[j][7];

            A_P_A_6[i][j] = A_P_6[i][0]*A_State[j][0]+A_P_6[i][1]*A_State[j][1]+A_P_6[i][2]*A_State[j][2]+A_P_6[i][3]*A_State[j][3]
                           +A_P_6[i][4]*A_State[j][4]+A_P_6[i][5]*A_State[j][5]+A_P_6[i][6]*A_State[j][6]+A_P_6[i][7]*A_State[j][7];
        }
    }

    //计算P_pre,即A*P*A'+Q,先尝试不加噪声

    for(i=0;i<8;i++){
        for(j=0;j<8;j++){
            P_pre_0[i][j] = A_P_A_0[i][j];
            P_pre_1[i][j] = A_P_A_1[i][j];
            P_pre_2[i][j] = A_P_A_2[i][j];
            P_pre_3[i][j] = A_P_A_3[i][j];
            P_pre_4[i][j] = A_P_A_4[i][j];
            P_pre_5[i][j] = A_P_A_5[i][j];
            P_pre_6[i][j] = A_P_A_6[i][j];
        }
    }
    
    //----------------------------------------------------计算观测方差--------------------------------------------------------------------
    for(i=0;i<4;i++){//先计算C*P_pre
        for(j=0;j<8;j++){
            C_P_0[i][j] = C_State[i][0]*P_pre_0[0][j]+C_State[i][1]*P_pre_0[1][j]+C_State[i][2]*P_pre_0[2][j]+C_State[i][3]*P_pre_0[3][j]
                         +C_State[i][4]*P_pre_0[4][j]+C_State[i][5]*P_pre_0[5][j]+C_State[i][6]*P_pre_0[6][j]+C_State[i][7]*P_pre_0[7][j];

            C_P_1[i][j] = C_State[i][0]*P_pre_1[0][j]+C_State[i][1]*P_pre_1[1][j]+C_State[i][2]*P_pre_1[2][j]+C_State[i][3]*P_pre_1[3][j]
                         +C_State[i][4]*P_pre_1[4][j]+C_State[i][5]*P_pre_1[5][j]+C_State[i][6]*P_pre_1[6][j]+C_State[i][7]*P_pre_1[7][j];

            C_P_2[i][j] = C_State[i][0]*P_pre_2[0][j]+C_State[i][1]*P_pre_2[1][j]+C_State[i][2]*P_pre_2[2][j]+C_State[i][3]*P_pre_2[3][j]
                         +C_State[i][4]*P_pre_2[4][j]+C_State[i][5]*P_pre_2[5][j]+C_State[i][6]*P_pre_2[6][j]+C_State[i][7]*P_pre_2[7][j];

            C_P_3[i][j] = C_State[i][0]*P_pre_3[0][j]+C_State[i][1]*P_pre_3[1][j]+C_State[i][2]*P_pre_3[2][j]+C_State[i][3]*P_pre_3[3][j]
                         +C_State[i][4]*P_pre_3[4][j]+C_State[i][5]*P_pre_3[5][j]+C_State[i][6]*P_pre_3[6][j]+C_State[i][7]*P_pre_3[7][j];

            C_P_4[i][j] = C_State[i][0]*P_pre_4[0][j]+C_State[i][1]*P_pre_4[1][j]+C_State[i][2]*P_pre_4[2][j]+C_State[i][3]*P_pre_4[3][j]
                         +C_State[i][4]*P_pre_4[4][j]+C_State[i][5]*P_pre_4[5][j]+C_State[i][6]*P_pre_4[6][j]+C_State[i][7]*P_pre_4[7][j];

            C_P_5[i][j] = C_State[i][0]*P_pre_5[0][j]+C_State[i][1]*P_pre_5[1][j]+C_State[i][2]*P_pre_5[2][j]+C_State[i][3]*P_pre_5[3][j]
                         +C_State[i][4]*P_pre_5[4][j]+C_State[i][5]*P_pre_5[5][j]+C_State[i][6]*P_pre_5[6][j]+C_State[i][7]*P_pre_5[7][j];

            C_P_6[i][j] = C_State[i][0]*P_pre_6[0][j]+C_State[i][1]*P_pre_6[1][j]+C_State[i][2]*P_pre_6[2][j]+C_State[i][3]*P_pre_6[3][j]
                         +C_State[i][4]*P_pre_6[4][j]+C_State[i][5]*P_pre_6[5][j]+C_State[i][6]*P_pre_6[6][j]+C_State[i][7]*P_pre_6[7][j];
        }
    }
    //再计算C*P*C'
    for(i=0;i<4;i++){
        for(j=4;j<4;j++){
            S_0[i][j] = C_P_0[i][0]*C_State[j][0]+C_P_0[i][1]*C_State[j][1]+C_P_0[i][2]*C_State[j][2]+C_P_0[i][3]*C_State[j][3]
                       +C_P_0[i][4]*C_State[j][4]+C_P_0[i][5]*C_State[j][5]+C_P_0[i][6]*C_State[j][6]+C_P_0[i][7]*C_State[j][7];

            S_1[i][j] = C_P_1[i][0]*C_State[j][0]+C_P_1[i][1]*C_State[j][1]+C_P_1[i][2]*C_State[j][2]+C_P_1[i][3]*C_State[j][3]
                       +C_P_1[i][4]*C_State[j][4]+C_P_1[i][5]*C_State[j][5]+C_P_1[i][6]*C_State[j][6]+C_P_1[i][7]*C_State[j][7];

            S_2[i][j] = C_P_2[i][0]*C_State[j][0]+C_P_2[i][1]*C_State[j][1]+C_P_2[i][2]*C_State[j][2]+C_P_2[i][3]*C_State[j][3]
                       +C_P_2[i][4]*C_State[j][4]+C_P_2[i][5]*C_State[j][5]+C_P_2[i][6]*C_State[j][6]+C_P_2[i][7]*C_State[j][7];

            S_3[i][j] = C_P_3[i][0]*C_State[j][0]+C_P_3[i][1]*C_State[j][1]+C_P_3[i][2]*C_State[j][2]+C_P_3[i][3]*C_State[j][3]
                       +C_P_3[i][4]*C_State[j][4]+C_P_3[i][5]*C_State[j][5]+C_P_3[i][6]*C_State[j][6]+C_P_3[i][7]*C_State[j][7];

            S_4[i][j] = C_P_4[i][0]*C_State[j][0]+C_P_4[i][1]*C_State[j][1]+C_P_4[i][2]*C_State[j][2]+C_P_4[i][3]*C_State[j][3]
                       +C_P_4[i][4]*C_State[j][4]+C_P_4[i][5]*C_State[j][5]+C_P_4[i][6]*C_State[j][6]+C_P_4[i][7]*C_State[j][7];

            S_5[i][j] = C_P_5[i][0]*C_State[j][0]+C_P_5[i][1]*C_State[j][1]+C_P_5[i][2]*C_State[j][2]+C_P_5[i][3]*C_State[j][3]
                       +C_P_5[i][4]*C_State[j][4]+C_P_5[i][5]*C_State[j][5]+C_P_5[i][6]*C_State[j][6]+C_P_5[i][7]*C_State[j][7];

            S_6[i][j] = C_P_6[i][0]*C_State[j][0]+C_P_6[i][1]*C_State[j][1]+C_P_6[i][2]*C_State[j][2]+C_P_6[i][3]*C_State[j][3]
                       +C_P_6[i][4]*C_State[j][4]+C_P_6[i][5]*C_State[j][5]+C_P_6[i][6]*C_State[j][6]+C_P_6[i][7]*C_State[j][7];
        }
    }
    
    inverse4x4(*S_0,*S_inv_0);
    inverse4x4(*S_1,*S_inv_1);
    inverse4x4(*S_2,*S_inv_2);
    inverse4x4(*S_3,*S_inv_3);
    inverse4x4(*S_4,*S_inv_4);
    inverse4x4(*S_5,*S_inv_5);
    inverse4x4(*S_6,*S_inv_6);

    //---------------------------------------------求解卡尔曼增益---------------------------------------------------------------------
    //先计算P*C'

    for(i=0;i<8;i++){
        for(j=0;j<4;j++){
            P_C_0[i][j] = P_pre_0[i][0]*C_State[j][0]+P_pre_0[i][1]*C_State[j][1]+P_pre_0[i][2]*C_State[j][2]+P_pre_0[i][3]*C_State[j][3]
                         +P_pre_0[i][4]*C_State[j][4]+P_pre_0[i][5]*C_State[j][5]+P_pre_0[i][6]*C_State[j][6]+P_pre_0[i][7]*C_State[j][7];

            P_C_1[i][j] = P_pre_1[i][0]*C_State[j][0]+P_pre_1[i][1]*C_State[j][1]+P_pre_1[i][2]*C_State[j][2]+P_pre_1[i][3]*C_State[j][3]
                         +P_pre_1[i][4]*C_State[j][4]+P_pre_1[i][5]*C_State[j][5]+P_pre_1[i][6]*C_State[j][6]+P_pre_1[i][7]*C_State[j][7];

            P_C_2[i][j] = P_pre_2[i][0]*C_State[j][0]+P_pre_2[i][1]*C_State[j][1]+P_pre_2[i][2]*C_State[j][2]+P_pre_2[i][3]*C_State[j][3]
                         +P_pre_2[i][4]*C_State[j][4]+P_pre_2[i][5]*C_State[j][5]+P_pre_2[i][6]*C_State[j][6]+P_pre_2[i][7]*C_State[j][7];

            P_C_3[i][j] = P_pre_3[i][0]*C_State[j][0]+P_pre_3[i][1]*C_State[j][1]+P_pre_3[i][2]*C_State[j][2]+P_pre_3[i][3]*C_State[j][3]
                         +P_pre_3[i][4]*C_State[j][4]+P_pre_3[i][5]*C_State[j][5]+P_pre_3[i][6]*C_State[j][6]+P_pre_3[i][7]*C_State[j][7];

            P_C_4[i][j] = P_pre_4[i][0]*C_State[j][0]+P_pre_4[i][1]*C_State[j][1]+P_pre_4[i][2]*C_State[j][2]+P_pre_4[i][3]*C_State[j][3]
                         +P_pre_4[i][4]*C_State[j][4]+P_pre_4[i][5]*C_State[j][5]+P_pre_4[i][6]*C_State[j][6]+P_pre_4[i][7]*C_State[j][7];

            P_C_5[i][j] = P_pre_5[i][0]*C_State[j][0]+P_pre_5[i][1]*C_State[j][1]+P_pre_5[i][2]*C_State[j][2]+P_pre_5[i][3]*C_State[j][3]
                         +P_pre_5[i][4]*C_State[j][4]+P_pre_5[i][5]*C_State[j][5]+P_pre_5[i][6]*C_State[j][6]+P_pre_5[i][7]*C_State[j][7];

            P_C_6[i][j] = P_pre_6[i][0]*C_State[j][0]+P_pre_6[i][1]*C_State[j][1]+P_pre_6[i][2]*C_State[j][2]+P_pre_6[i][3]*C_State[j][3]
                         +P_pre_6[i][4]*C_State[j][4]+P_pre_6[i][5]*C_State[j][5]+P_pre_6[i][6]*C_State[j][6]+P_pre_6[i][7]*C_State[j][7];
        }
    }

    //再计算P*C'*S_inv
    for(i=0;i<8;i++){
        for(j=0;j<4;j++){
            k_gain_0[i][j] = P_C_0[i][0]*S_inv_0[0][j]+P_C_0[i][1]*S_inv_0[1][j]+P_C_0[i][2]*S_inv_0[2][j]+P_C_0[i][3]*S_inv_0[3][j];

            k_gain_1[i][j] = P_C_1[i][0]*S_inv_1[0][j]+P_C_1[i][1]*S_inv_1[1][j]+P_C_1[i][2]*S_inv_1[2][j]+P_C_1[i][3]*S_inv_1[3][j];

            k_gain_2[i][j] = P_C_2[i][0]*S_inv_2[0][j]+P_C_2[i][1]*S_inv_2[1][j]+P_C_2[i][2]*S_inv_2[2][j]+P_C_2[i][3]*S_inv_2[3][j];

            k_gain_3[i][j] = P_C_3[i][0]*S_inv_3[0][j]+P_C_3[i][1]*S_inv_3[1][j]+P_C_3[i][2]*S_inv_3[2][j]+P_C_3[i][3]*S_inv_3[3][j];

            k_gain_4[i][j] = P_C_4[i][0]*S_inv_4[0][j]+P_C_4[i][1]*S_inv_4[1][j]+P_C_4[i][2]*S_inv_4[2][j]+P_C_4[i][3]*S_inv_4[3][j];

            k_gain_5[i][j] = P_C_5[i][0]*S_inv_5[0][j]+P_C_5[i][1]*S_inv_5[1][j]+P_C_5[i][2]*S_inv_5[2][j]+P_C_5[i][3]*S_inv_5[3][j];

            k_gain_6[i][j] = P_C_6[i][0]*S_inv_6[0][j]+P_C_6[i][1]*S_inv_6[1][j]+P_C_6[i][2]*S_inv_6[2][j]+P_C_6[i][3]*S_inv_6[3][j];
        }
    }

    //------------------------------------计算滤波器的输出状态和方差--------------------------------------------------------------------
    for(i=0;i<8;i++){
        x_next_0[i] = x_pre_0[i]+k_gain_0[i][0]*z_res_0[0]+k_gain_0[i][1]*z_res_0[1]
                                +k_gain_0[i][2]*z_res_0[2]+k_gain_0[i][3]*z_res_0[3];

        x_next_1[i] = x_pre_1[i]+k_gain_1[i][0]*z_res_1[0]+k_gain_1[i][1]*z_res_1[1]
                                +k_gain_1[i][2]*z_res_1[2]+k_gain_1[i][3]*z_res_1[3];

        x_next_2[i] = x_pre_2[i]+k_gain_2[i][0]*z_res_2[0]+k_gain_2[i][1]*z_res_2[1]
                                +k_gain_2[i][2]*z_res_2[2]+k_gain_2[i][3]*z_res_2[3];

        x_next_3[i] = x_pre_3[i]+k_gain_3[i][0]*z_res_3[0]+k_gain_3[i][1]*z_res_3[1]
                                +k_gain_3[i][2]*z_res_3[2]+k_gain_3[i][3]*z_res_3[3];

        x_next_4[i] = x_pre_4[i]+k_gain_4[i][0]*z_res_4[0]+k_gain_4[i][1]*z_res_4[1]
                                +k_gain_4[i][2]*z_res_4[2]+k_gain_4[i][3]*z_res_4[3];

        x_next_5[i] = x_pre_5[i]+k_gain_5[i][0]*z_res_5[0]+k_gain_5[i][1]*z_res_5[1]
                                +k_gain_5[i][2]*z_res_5[2]+k_gain_5[i][3]*z_res_5[3];

        x_next_6[i] = x_pre_6[i]+k_gain_6[i][0]*z_res_6[0]+k_gain_6[i][1]*z_res_6[1]
                                +k_gain_6[i][2]*z_res_6[2]+k_gain_6[i][3]*z_res_6[3];
    }

    //--------------------------------------计算P_next--------------------------------------------------------------------------------
    //先计算E-KC

    for(i=0;i<8;i++){
        for(j=0;j<8;j++){
            E_KC_0[i][j] = eye[i][j] - (k_gain_0[i][0]*C_State[0][j]+k_gain_0[i][1]*C_State[1][j]
                                       +k_gain_0[i][2]*C_State[2][j]+k_gain_0[i][3]*C_State[3][j]);

            E_KC_1[i][j] = eye[i][j] - (k_gain_1[i][0]*C_State[0][j]+k_gain_1[i][1]*C_State[1][j]
                                       +k_gain_1[i][2]*C_State[2][j]+k_gain_1[i][3]*C_State[3][j]);

            E_KC_2[i][j] = eye[i][j] - (k_gain_2[i][0]*C_State[0][j]+k_gain_2[i][1]*C_State[1][j]
                                       +k_gain_2[i][2]*C_State[2][j]+k_gain_2[i][3]*C_State[3][j]);

            E_KC_3[i][j] = eye[i][j] - (k_gain_3[i][0]*C_State[0][j]+k_gain_3[i][1]*C_State[1][j]
                                       +k_gain_3[i][2]*C_State[2][j]+k_gain_3[i][3]*C_State[3][j]);

            E_KC_4[i][j] = eye[i][j] - (k_gain_4[i][0]*C_State[0][j]+k_gain_4[i][1]*C_State[1][j]
                                       +k_gain_4[i][2]*C_State[2][j]+k_gain_4[i][3]*C_State[3][j]);

            E_KC_5[i][j] = eye[i][j] - (k_gain_5[i][0]*C_State[0][j]+k_gain_5[i][1]*C_State[1][j]
                                       +k_gain_5[i][2]*C_State[2][j]+k_gain_5[i][3]*C_State[3][j]);

            E_KC_6[i][j] = eye[i][j] - (k_gain_6[i][0]*C_State[0][j]+k_gain_6[i][1]*C_State[1][j]
                                       +k_gain_6[i][2]*C_State[2][j]+k_gain_6[i][3]*C_State[3][j]);
        }
    }
    //再计算P_next
    for(i=0;i<8;i++){
        for(j=0;j<8;j++){
            P_next_0[i][j] = E_KC_0[i][0]*P_pre_0[0][j]+E_KC_0[i][1]*P_pre_0[1][j]+E_KC_0[i][2]*P_pre_0[2][j]+E_KC_0[i][3]*P_pre_0[3][j]
                            +E_KC_0[i][4]*P_pre_0[4][j]+E_KC_0[i][5]*P_pre_0[5][j]+E_KC_0[i][6]*P_pre_0[6][j]+E_KC_0[i][7]*P_pre_0[7][j];

            P_next_1[i][j] = E_KC_1[i][0]*P_pre_1[0][j]+E_KC_1[i][1]*P_pre_1[1][j]+E_KC_1[i][2]*P_pre_1[2][j]+E_KC_1[i][3]*P_pre_1[3][j]
                            +E_KC_1[i][4]*P_pre_1[4][j]+E_KC_1[i][5]*P_pre_1[5][j]+E_KC_1[i][6]*P_pre_1[6][j]+E_KC_1[i][7]*P_pre_1[7][j];

            P_next_2[i][j] = E_KC_2[i][0]*P_pre_2[0][j]+E_KC_2[i][1]*P_pre_2[1][j]+E_KC_2[i][2]*P_pre_2[2][j]+E_KC_2[i][3]*P_pre_2[3][j]
                            +E_KC_2[i][4]*P_pre_2[4][j]+E_KC_2[i][5]*P_pre_2[5][j]+E_KC_2[i][6]*P_pre_2[6][j]+E_KC_2[i][7]*P_pre_2[7][j];

            P_next_3[i][j] = E_KC_3[i][0]*P_pre_3[0][j]+E_KC_3[i][1]*P_pre_3[1][j]+E_KC_3[i][2]*P_pre_3[2][j]+E_KC_3[i][3]*P_pre_3[3][j]
                            +E_KC_3[i][4]*P_pre_3[4][j]+E_KC_3[i][5]*P_pre_3[5][j]+E_KC_3[i][6]*P_pre_3[6][j]+E_KC_3[i][7]*P_pre_3[7][j];

            P_next_4[i][j] = E_KC_4[i][0]*P_pre_4[0][j]+E_KC_4[i][1]*P_pre_4[1][j]+E_KC_4[i][2]*P_pre_4[2][j]+E_KC_4[i][3]*P_pre_4[3][j]
                            +E_KC_4[i][4]*P_pre_4[4][j]+E_KC_4[i][5]*P_pre_4[5][j]+E_KC_4[i][6]*P_pre_4[6][j]+E_KC_4[i][7]*P_pre_4[7][j];

            P_next_5[i][j] = E_KC_5[i][0]*P_pre_5[0][j]+E_KC_5[i][1]*P_pre_5[1][j]+E_KC_5[i][2]*P_pre_5[2][j]+E_KC_5[i][3]*P_pre_5[3][j]
                            +E_KC_5[i][4]*P_pre_5[4][j]+E_KC_5[i][5]*P_pre_5[5][j]+E_KC_5[i][6]*P_pre_5[6][j]+E_KC_5[i][7]*P_pre_5[7][j];

            P_next_6[i][j] = E_KC_6[i][0]*P_pre_6[0][j]+E_KC_6[i][1]*P_pre_6[1][j]+E_KC_6[i][2]*P_pre_6[2][j]+E_KC_6[i][3]*P_pre_6[3][j]
                            +E_KC_6[i][4]*P_pre_6[4][j]+E_KC_6[i][5]*P_pre_6[5][j]+E_KC_6[i][6]*P_pre_6[6][j]+E_KC_6[i][7]*P_pre_6[7][j];
        }
    }
    //-------------------------------------------------模型可能性计算----------------------------------------------------------------
    //先计算Z_res'*S_inv*Z_res
    //先计算Z_res'*S_inv

    for(i=0;i<4;i++){
        ZT_SI_0[i] = z_res_0[0]*S_inv_0[0][i]+z_res_0[1]*S_inv_0[1][i]+z_res_0[2]*S_inv_0[2][i]+z_res_0[3]*S_inv_0[3][i];

        ZT_SI_1[i] = z_res_1[0]*S_inv_1[0][i]+z_res_1[1]*S_inv_1[1][i]+z_res_1[2]*S_inv_1[2][i]+z_res_1[3]*S_inv_1[3][i];

        ZT_SI_2[i] = z_res_2[0]*S_inv_2[0][i]+z_res_2[1]*S_inv_2[1][i]+z_res_2[2]*S_inv_2[2][i]+z_res_2[3]*S_inv_2[3][i];

        ZT_SI_3[i] = z_res_3[0]*S_inv_3[0][i]+z_res_3[1]*S_inv_3[1][i]+z_res_3[2]*S_inv_3[2][i]+z_res_3[3]*S_inv_3[3][i];

        ZT_SI_4[i] = z_res_4[0]*S_inv_4[0][i]+z_res_4[1]*S_inv_4[1][i]+z_res_4[2]*S_inv_4[2][i]+z_res_4[3]*S_inv_4[3][i];

        ZT_SI_5[i] = z_res_5[0]*S_inv_5[0][i]+z_res_5[1]*S_inv_5[1][i]+z_res_5[2]*S_inv_5[2][i]+z_res_5[3]*S_inv_5[3][i];

        ZT_SI_6[i] = z_res_6[0]*S_inv_6[0][i]+z_res_6[1]*S_inv_6[1][i]+z_res_6[2]*S_inv_6[2][i]+z_res_6[3]*S_inv_6[3][i];
    }

    //再计算ZT_SI*Z_res

    ZSZ_0 = ZT_SI_0[0]*z_res_0[0]+ZT_SI_0[1]*z_res_0[1]+ZT_SI_0[2]*z_res_0[2]+ZT_SI_0[3]*z_res_0[3];

    ZSZ_1 = ZT_SI_1[0]*z_res_1[0]+ZT_SI_1[1]*z_res_1[1]+ZT_SI_1[2]*z_res_1[2]+ZT_SI_1[3]*z_res_1[3];

    ZSZ_2 = ZT_SI_2[0]*z_res_2[0]+ZT_SI_2[1]*z_res_2[1]+ZT_SI_2[2]*z_res_2[2]+ZT_SI_2[3]*z_res_2[3];

    ZSZ_3 = ZT_SI_3[0]*z_res_3[0]+ZT_SI_3[1]*z_res_3[1]+ZT_SI_3[2]*z_res_3[2]+ZT_SI_3[3]*z_res_3[3];

    ZSZ_4 = ZT_SI_4[0]*z_res_4[0]+ZT_SI_4[1]*z_res_4[1]+ZT_SI_4[2]*z_res_4[2]+ZT_SI_4[3]*z_res_4[3];

    ZSZ_5 = ZT_SI_5[0]*z_res_5[0]+ZT_SI_5[1]*z_res_5[1]+ZT_SI_5[2]*z_res_5[2]+ZT_SI_5[3]*z_res_5[3];

    ZSZ_6 = ZT_SI_6[0]*z_res_6[0]+ZT_SI_6[1]*z_res_6[1]+ZT_SI_6[2]*z_res_6[2]+ZT_SI_6[3]*z_res_6[3];
    
    //计算指数exp();
    float exp_0 = expf(-0.5*ZSZ_0);
    float exp_1 = expf(-0.5*ZSZ_1);
    float exp_2 = expf(-0.5*ZSZ_2);
    float exp_3 = expf(-0.5*ZSZ_3);
    float exp_4 = expf(-0.5*ZSZ_4);
    float exp_5 = expf(-0.5*ZSZ_5);
    float exp_6 = expf(-0.5*ZSZ_6);
    
    //计算S的行列式
    S_0_det = get_mat_det(S_0);
    S_1_det = get_mat_det(S_1);
    S_2_det = get_mat_det(S_2);
    S_3_det = get_mat_det(S_3);
    S_4_det = get_mat_det(S_4);
    S_5_det = get_mat_det(S_5);
    S_6_det = get_mat_det(S_6);

    //计算模型可能性
    Mu_pre[0] = exp_0/(sqrtl(2*3.14*S_0_det));
    Mu_pre[1] = exp_1/(sqrtl(2*3.14*S_1_det));
    Mu_pre[2] = exp_2/(sqrtl(2*3.14*S_2_det));
    Mu_pre[3] = exp_3/(sqrtl(2*3.14*S_3_det));
    Mu_pre[4] = exp_4/(sqrtl(2*3.14*S_4_det));
    Mu_pre[5] = exp_5/(sqrtl(2*3.14*S_5_det));
    Mu_pre[6] = exp_6/(sqrtl(2*3.14*S_6_det));
    
    //-------------------------------------------------概率更新----------------------------------------------------------------------
    for(j=0;j<7;j++){
        C_IMM_b[j] = p_i_j[0][j]*Mu_last[0]+p_i_j[1][j]*Mu_last[1]+p_i_j[2][j]*Mu_last[2]+p_i_j[3][j]*Mu_last[3]
                    +p_i_j[4][j]*Mu_last[4]+p_i_j[5][j]*Mu_last[5]+p_i_j[6][j]*Mu_last[6];
    }

    C_IMM = Mu_pre[0]*C_IMM_b[0]+Mu_pre[1]*C_IMM_b[1]+Mu_pre[2]*C_IMM_b[2]+Mu_pre[3]*C_IMM_b[3]+Mu_pre[4]*C_IMM_b[4]
           +Mu_pre[5]*C_IMM_b[5]+Mu_pre[6]*C_IMM_b[6];

    for(i=0;i<7;i++){
        Mu_next[i] = Mu_pre[i]*C_IMM_b[i]/C_IMM;
    }

    //寻找可能性最大的模型概率
    if(Mu_next[0]>=0.5){
        model_number = 0;
    }else if(Mu_next[1]>=0.5){
        model_number = 1;
    }else if (Mu_next[2]>=0.5)
    {
        model_number = 2;
    }else if (Mu_next[3]>=0.5)
    {
        model_number = 3;
    }else if (Mu_next[4]>=0.5)
    {
        model_number = 4;
    }else if (Mu_next[5]>=0.5)
    {
        model_number = 5;
    }else if (Mu_next[6]>=0.5)
    {
        model_number = 6;
    }else{
        model_number = 0;
    }
    
    //更新下一时刻运算的初始值

    for(i=0;i<7;i++){
        Mu_last[i] = Mu_next[i];
    }

    for(i=0;i<8;i++){
        x_last_0[i] = x_next_0[i];
        x_last_1[i] = x_next_1[i];
        x_last_2[i] = x_next_2[i];
        x_last_3[i] = x_next_3[i];
        x_last_4[i] = x_next_4[i];
        x_last_5[i] = x_next_5[i];
        x_last_6[i] = x_next_6[i];
    }

    for(i=0;i<8;i++){
        for(j=0;j<8;j++){
            P_last_0[i][j] = P_next_0[i][j];
            P_last_1[i][j] = P_next_1[i][j];
            P_last_2[i][j] = P_next_2[i][j];
            P_last_3[i][j] = P_next_3[i][j];
            P_last_4[i][j] = P_next_4[i][j];
            P_last_5[i][j] = P_next_5[i][j];
            P_last_6[i][j] = P_next_6[i][j];
        }
    }

    return model_number;
}
