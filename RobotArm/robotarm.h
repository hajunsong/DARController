
#ifndef ROBOTARM_H
#define ROBOTARM_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>

#include "numerical.h"

using namespace std;

enum Module{FAR_V1=1, FAR_V2, FAR_V3, FAR_V4};

class Body
{
public:
    Body();
    Body(double psi, double theta, double phi, double sijp_x, double sijp_y, double sijp_z);
    ~Body();

    double qi, qi_dot, qi_ddot;
    double mi, Ixx, Iyy, Izz, Ixy, Iyz, Izx;

    // orientation
    double Ai[9], Aijpp[9], Ai_Cij[9], Cij[9], u_vec[3];
    // position
    double sij[3], sijp[3], ri[3], re[3], Ae[9], re_dot[3], we[3], ori[3], ori_zyx[3], tool_offset[3];
    // jacobian
    double Jvi[3], Jwi[3], re_qi[3], Ae_qi[9], Ai_Cij_Aijpp[9], Aijpp_qi[9], Cij_Aijpp[9];
    double oi[3], zi[3], zit[9];
    double Ai_q1[9], Ai_q2[9], Ai_q3[9], Ai_q4[9], Ai_q5[9], Ai_q6[9];
    double ri_q1[3], ri_q2[3], ri_q3[3], ri_q4[3], ri_q5[3], ri_q6[3];
    double sij_q1[3], sij_q2[3], sij_q3[3], sij_q4[3], sij_q5[3], sij_q6[3];
    double Ai_q1_sijp[3], Ai_q2_sijp[3], Ai_q3_sijp[3], Ai_q4_sijp[3], Ai_q5_sijp[3], Ai_q6_sijp[3];
    double rict_dot_wi[3], rit_dot_Hi[3], rit_Hi_dot[3];

    double Ai_Cij_Aijpp_qi[9], A6_qi[9], r6_qi[3], Ae_qi_31, Ae_qi_32, Ae_qi_33, Ae_qi_21, Ae_qi_11, roll_qi, pitch_qi, yaw_qi;
    // velocity state
    double Hi[3], rit[9], Bi[6], Yih[6];
    // cartesian velocity
    double Ti[36], Yib[6], ri_dot[3], wi[3], wit[9], rhoip[3], rhoi[3], ric[3], ric_dot[3];
    // mass & force
    double Cii[9], Ai_Cii[9], Jic[9], Jip[9], rit_dot[9], rict_dot[9], rict[9], Mih[36], fic[3], tic[3], Qih[6], Qih_g[6], Qih_c[6];
    double rict_fic[3], rict_rict_dot_wi[3], Jic_wi[3], wit_Jic_wi[3];
    // velocity coupling
    double Hi_dot[3], Di[6], Di_sum[6];
    // system EQM
    double Ki[36], Li[6], Li_g[6], Li_c[6], Ki_Di[6], Ki_Di_sum[6], Ki_Bi[6], temp_Bi_Ki_Bi;
    // DOB residual
    double y, yp, Ta, r_hat, K, p_linear, p_rotate, p, y_old, yp_old, alpha, f_cut, time_zone[3], r_hat_filter;
    int filter_indx;

    static void ang2mat(double ang_z1, double ang_x, double ang_z2, double* mat, bool deg_flag = true);
};

class RobotArm
{
public:
    RobotArm(uint numbody, uint DOF, double step_size, unsigned char ver);
    ~RobotArm();
#ifdef FILEIO_H_
    void run_kinematics();
    void run_inverse_kinematics();
    void run_dynamics();
#endif
    void run_kinematics(double *q, double *cur_pose);
    int run_inverse_kinematics(double* cur_joint, double* des_pose, double* res_joint, double* res_pose);
    void run_dynamics(double *q, double *q_dot, double *q_ddot, double *end_pose);
    void gravity_compensation(double *q, double *q_dot, double *torque);
    void disturbance_observer(double *q, double *q_dot, double *torque, double *residual);

    static void rpy2mat(double yaw, double pitch, double roll, double *mat);
    static void zyx2mat(double rz, double ry, double rx, double *mat);
    static void mat2rpy(double mat[9], double ori[3]);
    static void mat2zyx(double mat[9], double ori[3]);
    static void mat_to_axis_angle(double R_init[9], double R_final[9], double r[3], double *theta);
    static void axis_angle_to_mat(double r[3], double theta, double mat[9]);

    Body *body;
    uint num_body, dof;

    static void mat(double *mat_1, double *mat_2, uint row_1, uint col_1, uint row_2, uint col_2, double *mat_3);
    static void mat(double *mat_1, double *vec_2, uint row_1, uint col_1, uint row_2, double *vec_3);
    static void mat(const double *mat_1, double *vec_2, uint row_1, uint col_1, uint row_2, double *vec_3);

    double J[6*6], Jw[6*3], Jv[6*3];
    double re_q1[3], re_q2[3], re_q3[3], re_q4[3], re_q5[3], re_q6[3];
    double x_qi[6], y_qi[6], z_qi[6];
    double Ae_q1[9], Ae_q2[9], Ae_q3[9], Ae_q4[9], Ae_q5[9], Ae_q6[9];

    void jacobian();
    void jacobian_zyx();

    void set_tool_offset(double tool_offset[3]);
    void set_tool_offset(double tool_x, double tool_y, double tool_z);

    Numerical *numeric;

private:
    inline void tilde(double *a, double *b) {
        *(b++) = 0;     *(b++) = -a[2];	*(b++) = a[1];
        *(b++) = a[2];	*(b++) = 0;     *(b++) = -a[0];
        *(b++) = -a[1];	*(b++) = a[0];	*(b++) = 0;
    }

    double DH[6*4];

    double Ae_31, Ae_32, Ae_33, Ae_21, Ae_11, Ae_32_33, Ae_32_33_2, Ae_21_11;

    double *PH, *PH_pos, *PH_ori, *delta_q, *JD;
    double *M, *Q, *Q_c, *Q_g;

    // system variable
    double start_time, end_time, h, t_current;
    double g;

    // file
    char file_name[256];
    FILE *fp;

    double lamda;

    void kinematics();
    int inverse_kinematics(double pos_d[3], double ori_d[3]=NULL);
    void dynamics();
    void save_data();
    void residual();
    void high_pass_filter(double cur, double *timeZone, double *cur_filter, double ts, double f_cut);

    // High Pass Filter
    double w_cut, tau, tau_ts, a1, b0, b1, a2, b2, sum0;
    unsigned int filter_indx;

    double roll_q_temp1, roll_q_temp2, roll_q_temp3, roll_q_temp4;
    double pitch_q_temp1, pitch_q_temp2, pitch_q_temp3, pitch_q_temp4;
    double yaw_q_temp1, yaw_q_temp2, yaw_q_temp3, yaw_q_temp4;

    int indx[6];
    double fac[6*6];
};

#endif // ROBOTARM_H
