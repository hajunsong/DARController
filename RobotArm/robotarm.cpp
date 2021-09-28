#include "robotarm.h"

static double temp = 0;
static double Rz1[9] = {0,}, Rx[9] = {0,}, Rz2[9] = {0,}, Rz1Rx[9] = {0,};
static double R_yaw[9] = {0,}, R_pitch[9] = {0,}, R_roll[9] = {0,}, R_yaw_R_pitch[9] = {0,};
static double R_z[9] = {0,}, R_y[9] = {0,}, R_x[9] = {0,}, R_z_R_y[9] = {0,};
static double R[9] = {0,};
static double m00 = 0, m01 = 0, m02 = 0, m10 = 0, m11 = 0, m12 = 0, m20 = 0, m21 = 0, m22 = 0;
static double angle = 0, x = 0, y = 0, z = 0;
static double epsilon = 0, epsilon2 = 0;
static double xx = 0, yy = 0, zz = 0, xy = 0, xz = 0, yz = 0;
static double s = 0, c = 0, t = 0, magnitude = 0;
static double goal_reach = 0, epsilon_pos = 0, epsilon_ang = 0;
static double pos_d[3] = {0,}, ori_d[3] = {0,};
static double errmax = 0, errtol = 0, alpha = 0, err[6] = {0,}, qdot[6] = {0,};
static double temp3[9] = {0,};
static double *fac_mat, *qddot;
static int *indx_array;

Body::Body(){
    u_vec[0] = 0;
    u_vec[1] = 0;
    u_vec[2] = 1;
}

Body::~Body(){}

void Body::ang2mat(double ang_z1, double ang_x, double ang_z2, double *mat, bool deg_flag)
{
    if (deg_flag){
        ang_z1 = ang_z1*M_PI/180.0;
        ang_x = ang_x*M_PI/180.0;
        ang_z2 = ang_z2*M_PI/180.0;
    }

    memset(Rz1, 0, sizeof(double)*9);
    memset(Rx, 0, sizeof(double)*9);
    memset(Rz2, 0, sizeof(double)*9);
    memset(Rz1Rx, 0, sizeof(double)*9);

    Rz1[0] = cos(ang_z1);
    Rz1[1] = -sin(ang_z1);
    Rz1[3] = sin(ang_z1);
    Rz1[4] = cos(ang_z1);
    Rz1[8] = 1;

    Rx[0] = 1;
    Rx[4] = cos(ang_x);
    Rx[5] = -sin(ang_x);
    Rx[7] = sin(ang_x);
    Rx[8] = cos(ang_x);

    Rz2[0] = cos(ang_z2);
    Rz2[1] = -sin(ang_z2);
    Rz2[3] = sin(ang_z2);
    Rz2[4] = cos(ang_z2);
    Rz2[8] = 1;

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            temp = 0;
            for(int k = 0; k < 3; k++){
                temp += Rz1[i*3+k]*Rx[k*3+j];
            }
            Rz1Rx[i*3+j] = temp;
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            temp = 0;
            for(int k = 0; k < 3; k++){
                temp += Rz1Rx[i*3+k]*Rz2[k*3+j];
            }
            mat[i*3+j] = temp;
        }
    }
}

void RobotArm::mat(double *mat_1, double *mat_2, uint row_1, uint col_1, uint row_2, uint col_2, double *mat_3){
    if (col_1 != row_2){
        printf("please check matrix size\n");
        return;
    }

    for(uint i = 0; i < row_1; i++){
        for(uint j = 0; j < col_2; j++){
            temp = 0;
            for(uint k = 0; k < row_2; k++){
                temp += mat_1[i*col_1 + k]*mat_2[k*col_2 + j];
            }
            mat_3[i*col_2 + j] = temp;
        }
    }
}

void RobotArm::mat(double *mat_1, double *vec_2, uint row_1, uint col_1, uint row_2, double *vec_3){
    if (col_1 != row_2){
        printf("please check matrix size\n");
        return;
    }

    for(uint i = 0; i < row_1; i++){
        temp = 0;
        for(uint j = 0; j < row_2; j++){
            temp += mat_1[i*col_1 + j]*vec_2[j];
        }
        vec_3[i] = temp;
    }
}

void RobotArm::mat(const double *mat_1, double *vec_2, uint row_1, uint col_1, uint row_2, double *vec_3){
    if (col_1 != row_2){
        printf("please check matrix size\n");
        return;
    }

    for(uint i = 0; i < row_1; i++){
        temp = 0;
        for(uint j = 0; j < row_2; j++){
            temp += mat_1[i*col_1 + j]*vec_2[j];
        }
        vec_3[i] = temp;
    }
}

void RobotArm::rpy2mat(double yaw, double pitch, double roll, double *mat)
{
//    double R_yaw[9] = {cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1};
//    double R_pitch[9] = {cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch)};
//    double R_roll[9] = {1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll)};
//    double R_yaw_R_pitch[9] = {0,};

    memset(R_yaw, 0, sizeof(double)*9);
    memset(R_pitch, 0, sizeof(double)*9);
    memset(R_roll, 0, sizeof(double)*9);
    memset(R_yaw_R_pitch, 0, sizeof(double)*9);

    R_yaw[0] = cos(yaw);
    R_yaw[1] = -sin(yaw);
    R_yaw[3] = sin(yaw);
    R_yaw[4] = cos(yaw);
    R_yaw[8] = 1;

    R_pitch[0] = cos(pitch);
    R_pitch[2] = sin(pitch);
    R_pitch[4] = 1;
    R_pitch[6] = -sin(pitch);
    R_pitch[8] = cos(pitch);

    R_roll[0] = 1;
    R_roll[4] = cos(roll);
    R_roll[5] = -sin(roll);
    R_roll[7] = sin(roll);
    R_roll[8] = cos(roll);

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            temp = 0;
            for(int k = 0; k < 3; k++){
                temp += R_yaw[i*3+k]*R_pitch[k*3+j];
            }
            R_yaw_R_pitch[i*3+j] = temp;
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            temp = 0;
            for(int k = 0; k < 3; k++){
                temp += R_yaw_R_pitch[i*3+k]*R_roll[k*3+j];
            }
            mat[i*3+j] = temp;
        }
    }
}

void RobotArm::zyx2mat(double rz, double ry, double rx, double *mat)
{
//    double R_z[9] = {cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1};
//    double R_y[9] = {cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry)};
//    double R_x[9] = {1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx)};
//    double R_yaw_R_pitch[9] = {0,};

    memset(R_z, 0, sizeof(double)*9);
    memset(R_y, 0, sizeof(double)*9);
    memset(R_x, 0, sizeof(double)*9);
    memset(R_z_R_y, 0, sizeof(double)*9);

    R_z[0] = cos(rz);
    R_z[1] = -sin(rz);
    R_z[3] = sin(rz);
    R_z[4] = cos(rz);
    R_z[8] = 1;

    R_y[0] = cos(ry);
    R_y[2] = sin(ry);
    R_y[4] = 1;
    R_y[6] = -sin(ry);
    R_y[8] = cos(ry);

    R_x[0] = 1;
    R_x[4] = cos(rx);
    R_x[5] = -sin(rx);
    R_x[7] = sin(rx);
    R_x[8] = cos(rx);

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            temp = 0;
            for(int k = 0; k < 3; k++){
                temp += R_z[i*3+k]*R_y[k*3+j];
            }
            R_z_R_y[i*3+j] = temp;
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            temp = 0;
            for(int k = 0; k < 3; k++){
                temp += R_z_R_y[i*3+k]*R_x[k*3+j];
            }
            mat[i*3+j] = temp;
        }
    }
}

void RobotArm::mat_to_axis_angle(double R_init[], double R_final[], double r[], double *theta)
{
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            temp = 0;
            for(int k = 0; k < 3; k++){
                temp += R_init[k*3+i]*R_final[k*3+j];
            }
            R[i*3+j] = temp;
        }
    }

    m00 = R[0]; m01 = R[1]; m02 = R[2];
    m10 = R[3]; m11 = R[4]; m12 = R[5];
    m20 = R[6]; m21 = R[7]; m22 = R[8];

    //double angle, x, y, z; // variables for result
    epsilon = 0.01; // margin to allow for rounding errors
    epsilon2 = 0.1; // margin to distinguish between 0 and 180 degrees

    if (abs(m01 - m10) < epsilon && abs(m02 - m20) < epsilon && abs(m12 - m21) < epsilon){
        // singularity found
        // first check for identity matrix which must have +1 for all terms
        // in leading diagonal and zero in other terms
        if (abs(m01 + m10) < epsilon2 && abs(m02 + m20) < epsilon2 && abs(m12 + m21) < epsilon2 && abs(m00 + m11 + m22 - 3) < epsilon2){
            angle = 0;
            x = 1;
            y = 0;
            z = 0;
        }
        else{
            angle = M_PI;
            xx = (m00 + 1)/2;
            yy = (m11 + 1)/2;
            zz = (m22 + 1)/2;
            xy = (m01 + m10)/4;
            xz = (m02 + m20)/4;
            yz = (m12 + m21)/4;
            if (xx > yy && xx > zz){ // m00 is the largest diagonal term
                if (xx < epsilon){
                    x = 0;
                    y = 0.7071;
                    z = 0.7071;
                }
                else{
                    x = sqrt(xx);
                    y = xy/x;
                    z = xz/x;
                }
            }
            else if(yy > zz){ // m11 is the largest diagonal term
                if (yy < epsilon){
                    x = 0.7071;
                    y = 0;
                    z = 0.7071;
                }
                else{
                    y = sqrt(yy);
                    x = xy/y;
                    z = yz/y;
                }
            }
            else{ // m22 is the largest diagonal term so base result on this
                if (zz < epsilon){
                    x = 0.7071;
                    y = 0.7071;
                    z = 0;
                }
                else{
                    z = sqrt(zz);
                    x = xz/z;
                    y = yz/z;
                }
            }
        }
    }
    else {
        // as we have reached here there are no signularites so we can handle normally
        double s = sqrt((m21 - m12)*(m21 - m12) + (m02 - m20)*(m02 - m20) + (m10 - m01)*(m10 - m01));
        if (abs(s) < 0.001){
            s = 1;
        }
        // prevent divide by zero, should not happen if matrix is orthogonal and should be
        // caought by singularity test above, but I've left it in just in case
        angle = acos((m00 + m11 + m22 - 1)/2);
        x = (m21 - m12)/s;
        y = (m02 - m20)/s;
        z = (m10 - m01)/s;
    }

    *theta = angle;
    r[0] = x;
    r[1] = y;
    r[2] = z;
}

void RobotArm::axis_angle_to_mat(double r[], double angle, double mat[])
{
    c = cos(angle);
    s = sin(angle);
    t = 1.0 - c;
    x = r[0];
    y = r[1];
    z = r[2];
    // if axis is not already normalised then uncomment this
    magnitude = sqrt(x*x + y*y + z*z);
    if (magnitude <=0 && magnitude >=0) {
        return;
    }
    x = x/magnitude;
    y = y/magnitude;
    z = z/magnitude;

//    double m00, m01, m02, m10, m11, m12, m20, m21, m22;

//    m00 = c + x*x*t;
//    m11 = c + y*y*t;
//    m22 = c + z*z*t;

//    double tmp1 = x*y*t;
//    double tmp2 = z*s;
//    m10 = tmp1 + tmp2;
//    m01 = tmp1 - tmp2;
//    tmp1 = x*z*t;
//    tmp2 = y*s;
//    m20 = tmp1 - tmp2;
//    m02 = tmp1 + tmp2;
//    tmp1 = y*z*t;
//    tmp2 = x*s;
//    m21 = tmp1 + tmp2;
//    m12 = tmp1 - tmp2;

//    mat[0*3 + 0] = m00; mat[0*3 + 1] = m01; mat[0*3 + 2] = m02;
//    mat[1*3 + 0] = m10; mat[1*3 + 1] = m11; mat[1*3 + 2] = m12;
//    mat[2*3 + 0] = m20; mat[2*3 + 1] = m21; mat[2*3 + 2] = m22;

    mat[0] = t*x*x + c;     mat[1] = t*x*y - z*s;   mat[2] = t*x*z + y*s;
    mat[3] = t*x*y + z*s;   mat[4] = t*y*y + c;     mat[5] = t*y*z - x*s;
    mat[6] = t*x*z - y*s;   mat[7] = t*y*z + x*s;   mat[8] = t*z*z + c;
}

void RobotArm::mat2rpy(double mat[], double ori[])
{
    ori[0] = atan2(mat[2 * 3 + 1], mat[2 * 3 + 2]);
    ori[1] = atan2(-mat[2 * 3 + 0], sqrt(pow(mat[2 * 3 + 1], 2.0) + pow(mat[2 * 3 + 2], 2.0)));
    ori[2] = atan2(mat[1 * 3 + 0], mat[0 * 3 + 0]);
}

void RobotArm::mat2zyx(double mat[], double ori[])
{
    if(mat[6] < 1){
        if(mat[6] > -1){
            ori[1] = asin(-mat[6]);
            ori[2] = atan2(mat[3], mat[0]);
            ori[0] = atan2(mat[7], mat[8]);
        }
        else{
            // m20 = -1
            //            printf("Not a unique solution : thetaX - thetaZ = atan2(-m12, m11)\n");
            ori[1] = M_PI_2;
            ori[2] = -atan2(-mat[5], mat[4]);
            ori[0] = 0;
        }
    }
    else{
        // m20 = 1;
        //        printf("Not a unique solution : thetaX + thetaZ = atan2(-m12, m11)\n");
        ori[1] = -M_PI_2;
        ori[2] = atan2(-mat[5], mat[4]);
        ori[0] = 0;
    }
}

RobotArm::RobotArm(uint numbody, uint DOF, double step_size, unsigned char ver) {
    num_body = numbody;
    dof = DOF;

    PH = new double[dof];
    PH_pos = new double[3 * num_body];
    PH_ori = new double[3 * num_body];
    delta_q = new double[dof];
    JD = new double[dof * num_body];

    M = new double[num_body*num_body];
    Q = new double[num_body];
    Q_c = new double[num_body];
    Q_g = new double[num_body];

    body = new Body[num_body+1];

    lamda = 0.0001;

    // read data
    start_time = 0;
    h = step_size;
    g = -9.80665;

    if(ver == FAR_V1){
        // DH paramter
        // | Link | alpha(deg) |  a(mm)  |  d(mm)   | theta(deg) |
        // |=====================================================|
        // |  1   |    -90     |  0      |   0      |     90     |
        // |  2   |     0      |  151.75 |   0      |    -90     |
        // |  3   |     0      |  150    |   0      |     0      |
        // |  4   |     90     |  86.75  |   0      |     90     |
        // |  5   |     90     |  0      |   -20.25 |     90     |
        // |  6   |     0      |  0      |   102.5  |     0      |

        DH[0] = -90;    DH[1] = 0;          DH[2] = 0;          DH[3] = 90;
        DH[4] = 0;      DH[5] = 0.15175;    DH[6] = 0;          DH[7] = -90;
        DH[8] = 0;      DH[9] = 0.150;      DH[10] = 0;         DH[11] = 0;
        DH[12] = 90;    DH[13] = 0.08675;   DH[14] = 0;         DH[15] = 90;
        DH[16] = 90;    DH[17] = 0;         DH[18] = -0.02025;  DH[19] = 90;
        DH[20] = 0;     DH[21] = 0;         DH[22] = 0.1025;    DH[23] = 0;

        // body 0 variable
        body[0].Ai[0] = 1; body[0].Ai[1] = 0; body[0].Ai[2] = 0;
        body[0].Ai[3] = 0; body[0].Ai[4] = 1; body[0].Ai[5] = 0;
        body[0].Ai[6] = 0; body[0].Ai[7] = 0; body[0].Ai[8] = 1;

        body[0].ri[0] = 0; body[0].ri[1] = 0; body[0].ri[2] = 0;

        // preliminary work
        memset(body[0].Yih, 0, sizeof(double)*6);
        memset(body[0].wi, 0, sizeof(double)*3);
        memset(body[0].wit, 0, sizeof(double)*9);

        Body::ang2mat(0, 0, 0, body[0].Cij);
        body[0].sijp[0] = 0; body[0].sijp[1] = 0; body[0].sijp[2] = 0;

        body[0].ri_dot[0] = 0; body[0].ri_dot[1] = 0; body[0].ri_dot[2] = 0;
        body[0].wi[0] = 0; body[0].wi[1] = 0; body[0].wi[2] = 0;

        body[0].u_vec[0] = 0; body[0].u_vec[1] = 0; body[0].u_vec[2] = 1;

        // body 1 variables
        Body::ang2mat(DH[0*4+3], DH[0*4+0], 0, body[1].Cij);
        body[1].sijp[0] = 0; body[1].sijp[1] = 0; body[1].sijp[2] = 0;

        Body::ang2mat(-M_PI_2, M_PI_2, 0, body[1].Cii, false);
        body[1].rhoip[0] = -0.00189888; body[1].rhoip[1] = -1.44683e-8; body[1].rhoip[2] = -0.0234351;
        body[1].mi = 6.33612131907843e-002;
        body[1].Ixx = 5.46760988093101e-005;
        body[1].Iyy = 4.11897872591055e-005;
        body[1].Izz = 2.28294446378339e-005;
        body[1].Ixy = 1.16933891602143e-011;
        body[1].Iyz = 1.72355337398552e-006;
        body[1].Izx = 3.03099214889948e-011;
        body[1].Jip[0] = body[1].Ixx; body[1].Jip[1] = body[1].Ixy; body[1].Jip[2] = body[1].Izx;
        body[1].Jip[3] = body[1].Ixy; body[1].Jip[4] = body[1].Iyy; body[1].Jip[5] = body[1].Iyz;
        body[1].Jip[6] = body[1].Izx; body[1].Jip[7] = body[1].Iyz; body[1].Jip[8] = body[1].Izz;
        body[1].u_vec[0] = 0; body[1].u_vec[1] = 0; body[1].u_vec[2] = 1;

        // body 2 variables
        Body::ang2mat(DH[1*4+3], DH[1*4+0], 0, body[2].Cij);
        body[2].sijp[0] = 0; body[2].sijp[1] = -DH[1*4+1]; body[2].sijp[2] = 0;

        Body::ang2mat(M_PI, 0, 0, body[2].Cii, false);
        body[2].rhoip[0] = -0.000462227; body[2].rhoip[1] = -0.0427355; body[2].rhoip[2] = 0.000759913;
        body[2].mi = 0.291144481135948;
        body[2].Ixx = 6.84357146533933e-004;
        body[2].Iyy = 1.19767708650701e-004;
        body[2].Izz = 6.2201207394514e-004;
        body[2].Ixy = -7.26891485430593e-006;
        body[2].Iyz = 5.05996626479478e-006;
        body[2].Izx = 1.80750423403909e-007;
        body[2].Jip[0] = body[2].Ixx; body[2].Jip[1] = body[2].Ixy; body[2].Jip[2] = body[2].Izx;
        body[2].Jip[3] = body[2].Ixy; body[2].Jip[4] = body[2].Iyy; body[2].Jip[5] = body[2].Iyz;
        body[2].Jip[6] = body[2].Izx; body[2].Jip[7] = body[2].Iyz; body[2].Jip[8] = body[2].Izz;
        body[2].u_vec[0] = 0; body[2].u_vec[1] = 0; body[2].u_vec[2] = 1;

        // body 3 variables
        Body::ang2mat(DH[2*4+3], DH[2*4+0], 0, body[3].Cij);
        body[3].sijp[0] = DH[2*4+1]; body[3].sijp[1] = 0; body[3].sijp[2] = 0;

        Body::ang2mat(-M_PI_2, 0, 0, body[3].Cii, false);
        body[3].rhoip[0] = 0.075; body[3].rhoip[1] = 0; body[3].rhoip[2] = 0.000807364;
        body[3].mi = 0.416638668104345;
        body[3].Ixx = 1.45776042402133e-003;
        body[3].Iyy = 1.15949266176089e-004;
        body[3].Izz = 1.44207442743259e-003;
        body[3].Ixy = -2.14630188922107e-014;
        body[3].Iyz = -4.86620428197596e-019;
        body[3].Izx = -5.85663447574856e-020;
        body[3].Jip[0] = body[3].Ixx; body[3].Jip[1] = body[3].Ixy; body[3].Jip[2] = body[3].Izx;
        body[3].Jip[3] = body[3].Ixy; body[3].Jip[4] = body[3].Iyy; body[3].Jip[5] = body[3].Iyz;
        body[3].Jip[6] = body[3].Izx; body[3].Jip[7] = body[3].Iyz; body[3].Jip[8] = body[3].Izz;
        body[3].u_vec[0] = 0; body[3].u_vec[1] = 0; body[3].u_vec[2] = 1;

        // body 4 variables
        Body::ang2mat(DH[3*4+3], DH[3*4+0], 0, body[4].Cij);
        body[4].sijp[0] = 0; body[4].sijp[1] = DH[3*4+1]; body[4].sijp[2] = 0;

        Body::ang2mat(-M_PI_2, 0, 0, body[4].Cii, false);
        body[4].rhoip[0] = 0.000749752; body[4].rhoip[1] = 0.0609445; body[4].rhoip[2] = 0.000415268;
        body[4].mi = 0.228993914748238;
        body[4].Ixx = 7.74704754240776e-005;
        body[4].Iyy = 2.01161940464821e-004;
        body[4].Izz = 1.88922599914013e-004;
        body[4].Ixy = 2.91568180110741e-006;
        body[4].Iyz = 7.12741866241557e-008;
        body[4].Izx = -2.20466982640091e-006;
        body[4].Jip[0] = body[4].Ixx; body[4].Jip[1] = body[4].Ixy; body[4].Jip[2] = body[4].Izx;
        body[4].Jip[3] = body[4].Ixy; body[4].Jip[4] = body[4].Iyy; body[4].Jip[5] = body[4].Iyz;
        body[4].Jip[6] = body[4].Izx; body[4].Jip[7] = body[4].Iyz; body[4].Jip[8] = body[4].Izz;
        body[4].u_vec[0] = 0; body[4].u_vec[1] = 0; body[4].u_vec[2] = 1;

        // body 5 variables
        Body::ang2mat(DH[4*4+3], DH[4*4+0], 0, body[5].Cij);
        body[5].sijp[0] = 0; body[5].sijp[1] = 0; body[5].sijp[2] = DH[4*4+2];

        Body::ang2mat(M_PI, M_PI_2, 0, body[5].Cii, false);
        body[5].rhoip[0] = 0.0555687; body[5].rhoip[1] = 0; body[5].rhoip[2] = -0.000237633;
        body[5].mi = 0.204137411295743;
        body[5].Ixx = 9.4696526893192e-005;
        body[5].Iyy = 7.92107777080459e-005;
        body[5].Izz = 1.38821213983018e-004;
        body[5].Ixy = -2.02238967554624e-005;
        body[5].Iyz = -1.20999283701959e-015;
        body[5].Izx = 1.87131808263915e-015;
        body[5].Jip[0] = body[5].Ixx; body[5].Jip[1] = body[5].Ixy; body[5].Jip[2] = body[5].Izx;
        body[5].Jip[3] = body[5].Ixy; body[5].Jip[4] = body[5].Iyy; body[5].Jip[5] = body[5].Iyz;
        body[5].Jip[6] = body[5].Izx; body[5].Jip[7] = body[5].Iyz; body[5].Jip[8] = body[5].Izz;
        body[5].u_vec[0] = 0; body[5].u_vec[1] = 0; body[5].u_vec[2] = 1;

        // body 6 variables
        Body::ang2mat(DH[5*4+3], DH[5*4+0], 0, body[6].Cij);
        body[6].sijp[0] = 0; body[6].sijp[1] = 0; body[6].sijp[2] = DH[5*4+2];

        Body::ang2mat(M_PI_2, M_PI_2, -M_PI_2, body[6].Cii, false);
        body[6].rhoip[0] = 5.39394e-10; body[6].rhoip[1] = 3.33671e-8; body[6].rhoip[2] = 0.089384;
        body[6].mi = 2.08363885223627e-002;
        body[6].Ixx = 2.66302463617021e-006;
        body[6].Iyy = 1.56637607668211e-006;
        body[6].Izz = 1.88187616526518e-006;
        body[6].Ixy = 2.4095425326714e-012;
        body[6].Iyz = 2.738802635816e-013;
        body[6].Izx = 9.27461478843821e-014;
        body[6].Jip[0] = body[6].Ixx; body[6].Jip[1] = body[6].Ixy; body[6].Jip[2] = body[6].Izx;
        body[6].Jip[3] = body[6].Ixy; body[6].Jip[4] = body[6].Iyy; body[6].Jip[5] = body[6].Iyz;
        body[6].Jip[6] = body[6].Izx; body[6].Jip[7] = body[6].Iyz; body[6].Jip[8] = body[6].Izz;
        body[6].u_vec[0] = 0; body[6].u_vec[1] = 0; body[6].u_vec[2] = 1;
    }
    else if(ver == FAR_V2){
        // DH paramter
        // | Link | alpha(deg) |  a(mm)  |  d(mm)   | theta(deg) |
        // |=====================================================|
        // |  1   |    -90     |  0      |   0      |    -90     |
        // |  2   |     0      |  164.25 |   16.50  |    -90     |
        // |  3   |     180    |  170    |   0      |     0      |
        // |  4   |     90     |  65.25  |   0      |     90     |
        // |  5   |     90     |  0      |  -16.75  |     90     |
        // |  6   |     0      |  0      |   84     |     0      |

       DH[0] = -90;    DH[1] = 0;          DH[2] = 0;          DH[3] = -90;
       DH[4] = 0;      DH[5] = 0.16425;    DH[6] = -0.0165;    DH[7] = -90;
       DH[8] = 180;    DH[9] = 0.170;      DH[10] = 0;         DH[11] = 0;
       DH[12] = 90;    DH[13] = 0.06525;   DH[14] = 0;         DH[15] = 90;
       DH[16] = 90;    DH[17] = 0;         DH[18] = -0.01675;  DH[19] = 90;
       DH[20] = 0;     DH[21] = 0;         DH[22] = 0.084;     DH[23] = 0;

       // body 0 variable
        body[0].Ai[0] = 1; body[0].Ai[1] = 0; body[0].Ai[2] = 0;
        body[0].Ai[3] = 0; body[0].Ai[4] = 1; body[0].Ai[5] = 0;
        body[0].Ai[6] = 0; body[0].Ai[7] = 0; body[0].Ai[8] = 1;

        body[0].ri[0] = 0; body[0].ri[1] = 0; body[0].ri[2] = 0;

        // preliminary work
        memset(body[0].Yih, 0, sizeof(double)*6);
        memset(body[0].wi, 0, sizeof(double)*3);
        memset(body[0].wit, 0, sizeof(double)*9);

        Body::ang2mat(0, 0, 0, body[0].Cij);
        body[0].sijp[0] = 0; body[0].sijp[1] = 0; body[0].sijp[2] = 0;

        body[0].ri_dot[0] = 0; body[0].ri_dot[1] = 0; body[0].ri_dot[2] = 0;
        body[0].wi[0] = 0; body[0].wi[1] = 0; body[0].wi[2] = 0;

        body[0].u_vec[0] = 0; body[0].u_vec[1] = 0; body[0].u_vec[2] = 1;

        // body 1 variables
        Body::ang2mat(DH[0*4+3], DH[0*4+0], 0, body[1].Cij);
        body[1].sijp[0] = 0; body[1].sijp[1] = 0; body[1].sijp[2] = 0;

        Body::ang2mat(0, 0, 0, body[1].Cii, false);
        body[1].rhoip[0] = -0.000135874; body[1].rhoip[1] = -1.068e-12; body[1].rhoip[2] = -0.0246531;
        body[1].mi = 7.08684533226054e-002;
        body[1].Ixx = 2.15089916203442e-005;
        body[1].Iyy = 3.65070109080493e-005;
        body[1].Izz = 2.83989018839352e-005;
        body[1].Ixy = -1.24014643551512e-017;
        body[1].Iyz = -1.12222253664962e-010;
        body[1].Izx = 3.33296217281281e-008;
        body[1].Jip[0] = body[1].Ixx; body[1].Jip[1] = body[1].Ixy; body[1].Jip[2] = body[1].Izx;
        body[1].Jip[3] = body[1].Ixy; body[1].Jip[4] = body[1].Iyy; body[1].Jip[5] = body[1].Iyz;
        body[1].Jip[6] = body[1].Izx; body[1].Jip[7] = body[1].Iyz; body[1].Jip[8] = body[1].Izz;
        body[1].u_vec[0] = 0; body[1].u_vec[1] = 0; body[1].u_vec[2] = 1;

        body[1].r_hat = 0;
        body[1].y = 0;
        body[1].yp = 0;
        body[1].K = 200;
        body[1].p_linear = 0;
        body[1].p_rotate = 0;
        body[1].p = 0;
        body[1].y_old = 0;
        body[1].yp_old = 0;
        body[1].f_cut=1000;
        memset(body[1].time_zone, 0, sizeof(float)*3);
        body[1].filter_indx = 0;

        // body 2 variables
        Body::ang2mat(DH[1*4+3], DH[1*4+0], 0, body[2].Cij);
        body[2].sijp[0] = DH[1*4+2]; body[2].sijp[1] = -DH[1*4+1]; body[2].sijp[2] = 0;

        Body::ang2mat(0, M_PI_2, M_PI_2, body[2].Cii, false);
        body[2].rhoip[0] = 0.00264336; body[2].rhoip[1] = -0.0319009; body[2].rhoip[2] = -0.000524792;
        body[2].mi = 0.233270004294732;
        body[2].Ixx = 4.08019849963512e-004;
        body[2].Iyy = 4.36730722674176e-004;
        body[2].Izz = 8.71040040349363e-005;
        body[2].Ixy = -3.23596533711006e-007;
        body[2].Iyz = -4.58645443586337e-005;
        body[2].Izx = -2.40916440855742e-006;
        body[2].Jip[0] = body[2].Ixx; body[2].Jip[1] = body[2].Ixy; body[2].Jip[2] = body[2].Izx;
        body[2].Jip[3] = body[2].Ixy; body[2].Jip[4] = body[2].Iyy; body[2].Jip[5] = body[2].Iyz;
        body[2].Jip[6] = body[2].Izx; body[2].Jip[7] = body[2].Iyz; body[2].Jip[8] = body[2].Izz;
        body[2].u_vec[0] = 0; body[2].u_vec[1] = 0; body[2].u_vec[2] = 1;

        body[2].r_hat = 0;
        body[2].y = 0;
        body[2].yp = 0;
        body[2].K = 200;
        body[2].p_linear = 0;
        body[2].p_rotate = 0;
        body[2].p = 0;
        body[2].y_old = 0;
        body[2].yp_old = 0;
        body[2].f_cut=1000;
        memset(body[2].time_zone, 0, sizeof(float)*3);
        body[2].filter_indx = 0;

        // body 3 variables
        Body::ang2mat(DH[2*4+3], DH[2*4+0], 0, body[3].Cij);
        body[3].sijp[0] = DH[2*4+1]; body[3].sijp[1] = 0; body[3].sijp[2] = 0;

        Body::ang2mat(M_PI_2,M_PI_2,M_PI_2, body[3].Cii, false);
        body[3].rhoip[0] = 0.0668431; body[3].rhoip[1] = -4.49044e-11; body[3].rhoip[2] = -0.000574255;
        body[3].mi = 0.294733648136712;
        body[3].Ixx = 1.33438729955757e-003;
        body[3].Iyy = 1.35236609017727e-003;
        body[3].Izz = 6.10851857303522e-005;
        body[3].Ixy = -1.65934500194573e-013;
        body[3].Iyz = 5.8944693629749e-013;
        body[3].Izx = -1.46477397988517e-006;
        body[3].Jip[0] = body[3].Ixx; body[3].Jip[1] = body[3].Ixy; body[3].Jip[2] = body[3].Izx;
        body[3].Jip[3] = body[3].Ixy; body[3].Jip[4] = body[3].Iyy; body[3].Jip[5] = body[3].Iyz;
        body[3].Jip[6] = body[3].Izx; body[3].Jip[7] = body[3].Iyz; body[3].Jip[8] = body[3].Izz;
        body[3].u_vec[0] = 0; body[3].u_vec[1] = 0; body[3].u_vec[2] = 1;

        body[3].r_hat = 0;
        body[3].y = 0;
        body[3].yp = 0;
        body[3].K = 200;
        body[3].p_linear = 0;
        body[3].p_rotate = 0;
        body[3].p = 0;
        body[3].y_old = 0;
        body[3].yp_old = 0;
        body[3].f_cut=1000;
        memset(body[3].time_zone, 0, sizeof(float)*3);
        body[3].filter_indx = 0;

        // body 4 variables
        Body::ang2mat(DH[3*4+3], DH[3*4+0], 0, body[4].Cij);
        body[4].sijp[0] = 0; body[4].sijp[1] = DH[3*4+1]; body[4].sijp[2] = 0;

        Body::ang2mat(M_PI_2,M_PI_2,-M_PI_2, body[4].Cii, false);
        body[4].rhoip[0] = 0.000488263; body[4].rhoip[1] = 0.0465912; body[4].rhoip[2] = 3.24848e-5;
        body[4].mi = 0.108749563323237;
        body[4].Ixx = 5.19451711277109e-005;
        body[4].Iyy = 2.17677195227188e-005;
        body[4].Izz = 5.3423100843467e-005;
        body[4].Ixy = -5.50565486810879e-008;
        body[4].Iyz = -4.1508751024039e-007;
        body[4].Izx = -1.72489029231613e-009;
        body[4].Jip[0] = body[4].Ixx; body[4].Jip[1] = body[4].Ixy; body[4].Jip[2] = body[4].Izx;
        body[4].Jip[3] = body[4].Ixy; body[4].Jip[4] = body[4].Iyy; body[4].Jip[5] = body[4].Iyz;
        body[4].Jip[6] = body[4].Izx; body[4].Jip[7] = body[4].Iyz; body[4].Jip[8] = body[4].Izz;
        body[4].u_vec[0] = 0; body[4].u_vec[1] = 0; body[4].u_vec[2] = 1;

        body[4].r_hat = 0;
        body[4].y = 0;
        body[4].yp = 0;
        body[4].K = 200;
        body[4].p_linear = 0;
        body[4].p_rotate = 0;
        body[4].p = 0;
        body[4].y_old = 0;
        body[4].yp_old = 0;
        body[4].f_cut=1000;
        memset(body[4].time_zone, 0, sizeof(float)*3);
        body[4].filter_indx = 0;

        // body 5 variables
        Body::ang2mat(DH[4*4+3], DH[4*4+0], 0, body[5].Cij);
        body[5].sijp[0] = 0; body[5].sijp[1] = 0; body[5].sijp[2] = DH[4*4+2];

        Body::ang2mat(-M_PI_2, 0, 0, body[5].Cii, false);
        body[5].rhoip[0] = 0.0449512; body[5].rhoip[1] = -1.30501e-12; body[5].rhoip[2] = -0.00250684;
        body[5].mi = 0.110204790536652;
        body[5].Ixx = 5.65872649539517e-005;
        body[5].Iyy = 3.21370607982722e-005;
        body[5].Izz = 3.83261110287993e-005;
        body[5].Ixy = 3.11849020965568e-015;
        body[5].Iyz = 2.2933899377825e-006;
        body[5].Izx = 1.98601435820104e-015;
        body[5].Jip[0] = body[5].Ixx; body[5].Jip[1] = body[5].Ixy; body[5].Jip[2] = body[5].Izx;
        body[5].Jip[3] = body[5].Ixy; body[5].Jip[4] = body[5].Iyy; body[5].Jip[5] = body[5].Iyz;
        body[5].Jip[6] = body[5].Izx; body[5].Jip[7] = body[5].Iyz; body[5].Jip[8] = body[5].Izz;
        body[5].u_vec[0] = 0; body[5].u_vec[1] = 0; body[5].u_vec[2] = 1;

        body[5].r_hat = 0;
        body[5].y = 0;
        body[5].yp = 0;
        body[5].K = 200;
        body[5].p_linear = 0;
        body[5].p_rotate = 0;
        body[5].p = 0;
        body[5].y_old = 0;
        body[5].yp_old = 0;
        body[5].f_cut=1000;
        memset(body[5].time_zone, 0, sizeof(float)*3);
        body[5].filter_indx = 0;

        // body 6 variables
        Body::ang2mat(DH[5*4+3], DH[5*4+0], 0, body[6].Cij);
        body[6].sijp[0] = 0; body[6].sijp[1] = 0; body[6].sijp[2] = DH[5*4+2];

        Body::ang2mat(M_PI, M_PI_2, 0, body[6].Cii, false);
        body[6].rhoip[0] = 0.00178007; body[6].rhoip[1] = -0.00579632; body[6].rhoip[2] = 0.0781584;
        body[6].mi = 4.33236347828486e-002;
        body[6].Ixx = 1.6349828551908e-004;
        body[6].Iyy = 1.64408120008878e-004;
        body[6].Izz = 3.19367458327637e-006;
        body[6].Ixy = 6.41534880621834e-008;
        body[6].Iyz = 2.49712406360536e-007;
        body[6].Izx = -8.84019788476247e-006;
        body[6].Jip[0] = body[6].Ixx; body[6].Jip[1] = body[6].Ixy; body[6].Jip[2] = body[6].Izx;
        body[6].Jip[3] = body[6].Ixy; body[6].Jip[4] = body[6].Iyy; body[6].Jip[5] = body[6].Iyz;
        body[6].Jip[6] = body[6].Izx; body[6].Jip[7] = body[6].Iyz; body[6].Jip[8] = body[6].Izz;
        body[6].u_vec[0] = 0; body[6].u_vec[1] = 0; body[6].u_vec[2] = 1;
//        body[6].tool_offset[0] = 0;  body[6].tool_offset[1] = 0.1501;    body[6].tool_offset[2] = -0.005;

        body[6].r_hat = 0;
        body[6].y = 0;
        body[6].yp = 0;
        body[6].K = 200;
        body[6].p_linear = 0;
        body[6].p_rotate = 0;
        body[6].p = 0;
        body[6].y_old = 0;
        body[6].yp_old = 0;
        body[6].f_cut=1000;
        memset(body[6].time_zone, 0, sizeof(float)*3);
        body[6].filter_indx = 0;

//        memset(body[6].tool_offset, 0, sizeof(double)*3);
//        body[6].sijp[0] += body[6].tool_offset[0];
//        body[6].sijp[1] += body[6].tool_offset[1];
//        body[6].sijp[2] += body[6].tool_offset[2];


//        Body::ang2mat(DH[5*4+3], DH[5*4+0], 0, body[6].Cij);
//        body[6].sijp[0] = 0; body[6].sijp[1] = 0; body[6].sijp[2] = DH[5*4+2];

//        Body::ang2mat(M_PI, M_PI_2, 0, body[6].Cii, false);
//        body[6].rhoip[0] = 4.25231e-8; body[6].rhoip[1] = 0.00049999; body[6].rhoip[2] = 0.0756101;
//        body[6].mi = 1.06900256777816e-002;
//        body[6].Ixx = 5.4230201398644e-007;
//        body[6].Iyy = 8.27349038593228e-007;
//        body[6].Izz = 6.80541319418483e-007;
//        body[6].Ixy = 1.68734917731254e-012;
//        body[6].Iyz = 1.25961065157397e-013;
//        body[6].Izx = 3.11703037783589e-013;
//        body[6].Jip[0] = body[6].Ixx; body[6].Jip[1] = body[6].Ixy; body[6].Jip[2] = body[6].Izx;
//        body[6].Jip[3] = body[6].Ixy; body[6].Jip[4] = body[6].Iyy; body[6].Jip[5] = body[6].Iyz;
//        body[6].Jip[6] = body[6].Izx; body[6].Jip[7] = body[6].Iyz; body[6].Jip[8] = body[6].Izz;
//        body[6].u_vec[0] = 0; body[6].u_vec[1] = 0; body[6].u_vec[2] = 1;
    }
    else if(ver == FAR_V3){
        // DH paramter
        // | Link | alpha(deg) |  a(mm)  |  d(mm)   | theta(deg) |
        // |=====================================================|
        // |  1   |    -90     |  0      |   0      |    -90     |
        // |  2   |     0      |  164.25 |   0      |    -90     |
        // |  3   |     180    |  170    |   0      |     0      |
        // |  4   |     90     |  65.2457|   0      |     90     |
        // |  5   |     90     |  0      |  -16.2558|     90     |
        // |  6   |     0      |  0      |   88.9972|     0      |

        DH[0] = -90;    DH[1] =  0;         DH[2] =   0;         DH[3] = -90;
        DH[4] =   0;    DH[5] =  0.16425;   DH[6] =   0;         DH[7] = -90;
        DH[8] = 180;    DH[9] =  0.170;     DH[10] =  0;         DH[11] =  0;
        DH[12] = 90;    DH[13] = 0.0652457; DH[14] =  0;         DH[15] = 90;
        DH[16] = 90;    DH[17] = 0;         DH[18] = -0.0162558; DH[19] = 90;
        DH[20] =  0;    DH[21] = 0;         DH[22] =  0.0889972; DH[23] =  0;

        // body 0 variable
        body[0].Ai[0] = 1; body[0].Ai[1] = 0; body[0].Ai[2] = 0;
        body[0].Ai[3] = 0; body[0].Ai[4] = 1; body[0].Ai[5] = 0;
        body[0].Ai[6] = 0; body[0].Ai[7] = 0; body[0].Ai[8] = 1;

        body[0].Cij[0] = 1; body[0].Cij[1] = 0; body[0].Cij[2] = 0;
        body[0].Cij[3] = 0; body[0].Cij[4] = 1; body[0].Cij[5] = 0;
        body[0].Cij[6] = 0; body[0].Cij[7] = 0; body[0].Cij[8] = 1;

        body[0].ri[0] = 0; body[0].ri[1] = 0; body[0].ri[2] = 0;

        // preliminary work
        memset(body[0].Yih, 0, sizeof(double)*6);
        memset(body[0].wi, 0, sizeof(double)*3);
        memset(body[0].wit, 0, sizeof(double)*9);

        Body::ang2mat(0, 0, 0, body[0].Cij);
        body[0].sijp[0] = 0; body[0].sijp[1] = 0; body[0].sijp[2] = 0;

        body[0].ri_dot[0] = 0; body[0].ri_dot[1] = 0; body[0].ri_dot[2] = 0;
        body[0].wi[0] = 0; body[0].wi[1] = 0; body[0].wi[2] = 0;

        body[0].u_vec[0] = 0; body[0].u_vec[1] = 0; body[0].u_vec[2] = 1;

        // body 1 variables
        Body::ang2mat(DH[0*4+3], DH[0*4+0], 0, body[1].Cij);
        body[1].sijp[0] = 0; body[1].sijp[1] = 0; body[1].sijp[2] = 0;

        Body::ang2mat(0, M_PI_2, 0, body[1].Cii, false);
        body[1].rhoip[0] = 0.000110511;
        body[1].rhoip[1] = -1.16167e-8;
        body[1].rhoip[2] = -0.0187838;
        body[1].mi = 0.153104343087778;
        body[1].Ixx = 6.96678741977276e-005;
        body[1].Iyy = 1.12136062524532e-004;
        body[1].Izz = 1.20113478363185e-004;
        body[1].Ixy = 6.85012902792771e-008;
        body[1].Iyz = 1.30749783280943e-011;
        body[1].Izx = 2.701372946453e-009;
        body[1].Jip[0] = body[1].Ixx; body[1].Jip[1] = body[1].Ixy; body[1].Jip[2] = body[1].Izx;
        body[1].Jip[3] = body[1].Ixy; body[1].Jip[4] = body[1].Iyy; body[1].Jip[5] = body[1].Iyz;
        body[1].Jip[6] = body[1].Izx; body[1].Jip[7] = body[1].Iyz; body[1].Jip[8] = body[1].Izz;
        body[1].u_vec[0] = 0; body[1].u_vec[1] = 0; body[1].u_vec[2] = 1;

        body[1].r_hat = 0;
        body[1].y = 0;
        body[1].yp = 0;
        body[1].K = 200;
        body[1].p_linear = 0;
        body[1].p_rotate = 0;
        body[1].p = 0;
        body[1].y_old = 0;
        body[1].yp_old = 0;
        body[1].f_cut=1000;
        memset(body[1].time_zone, 0, sizeof(float)*3);
        body[1].filter_indx = 0;

        // body 2 variables
        Body::ang2mat(DH[1*4+3], DH[1*4+0], 0, body[2].Cij);
        body[2].sijp[0] = DH[1*4+2]; body[2].sijp[1] = -DH[1*4+1]; body[2].sijp[2] = 0;

        Body::ang2mat(M_PI_2, M_PI_2, M_PI_2, body[2].Cii, false);
        body[2].rhoip[0] = 0.00103404;
        body[2].rhoip[1] = -0.0555052;
        body[2].rhoip[2] = -4.99696e-5;
        body[2].mi = 0.741242292221872;
        body[2].Ixx = 2.1034196117051e-003;
        body[2].Iyy = 5.22299087086884e-004;
        body[2].Izz = 2.38975745676555e-003;
        body[2].Ixy = 1.3560795940288e-005;
        body[2].Iyz = -3.32194031774024e-005;
        body[2].Izx = 4.10393311075504e-008;
        body[2].Jip[0] = body[2].Ixx; body[2].Jip[1] = body[2].Ixy; body[2].Jip[2] = body[2].Izx;
        body[2].Jip[3] = body[2].Ixy; body[2].Jip[4] = body[2].Iyy; body[2].Jip[5] = body[2].Iyz;
        body[2].Jip[6] = body[2].Izx; body[2].Jip[7] = body[2].Iyz; body[2].Jip[8] = body[2].Izz;
        body[2].u_vec[0] = 0; body[2].u_vec[1] = 0; body[2].u_vec[2] = 1;

        body[2].r_hat = 0;
        body[2].y = 0;
        body[2].yp = 0;
        body[2].K = 200;
        body[2].p_linear = 0;
        body[2].p_rotate = 0;
        body[2].p = 0;
        body[2].y_old = 0;
        body[2].yp_old = 0;
        body[2].f_cut=1000;
        memset(body[2].time_zone, 0, sizeof(float)*3);
        body[2].filter_indx = 0;

        // body 3 variables
        Body::ang2mat(DH[2*4+3], DH[2*4+0], 0, body[3].Cij);
        body[3].sijp[0] = DH[2*4+1]; body[3].sijp[1] = 0; body[3].sijp[2] = 0;

        Body::ang2mat(-M_PI_2,M_PI_2,M_PI_2, body[3].Cii, false);
        body[3].rhoip[0] = 0.0684422;
        body[3].rhoip[1] = 0.000513196;
        body[3].rhoip[2] = -0.000415251;
        body[3].mi = 0.554447311677615;
        body[3].Ixx = 2.26605992866374e-003;
        body[3].Iyy = 2.30239764187232e-003;
        body[3].Izz = 1.59720819620979e-004;
        body[3].Ixy = -6.14645398504087e-008;
        body[3].Iyz = 3.79269133429595e-006;
        body[3].Izx = 2.47499181937338e-006;
        body[3].Jip[0] = body[3].Ixx; body[3].Jip[1] = body[3].Ixy; body[3].Jip[2] = body[3].Izx;
        body[3].Jip[3] = body[3].Ixy; body[3].Jip[4] = body[3].Iyy; body[3].Jip[5] = body[3].Iyz;
        body[3].Jip[6] = body[3].Izx; body[3].Jip[7] = body[3].Iyz; body[3].Jip[8] = body[3].Izz;
        body[3].u_vec[0] = 0; body[3].u_vec[1] = 0; body[3].u_vec[2] = 1;

        body[3].r_hat = 0;
        body[3].y = 0;
        body[3].yp = 0;
        body[3].K = 200;
        body[3].p_linear = 0;
        body[3].p_rotate = 0;
        body[3].p = 0;
        body[3].y_old = 0;
        body[3].yp_old = 0;
        body[3].f_cut=1000;
        memset(body[3].time_zone, 0, sizeof(float)*3);
        body[3].filter_indx = 0;

        // body 4 variables
        Body::ang2mat(DH[3*4+3], DH[3*4+0], 0, body[4].Cij);
        body[4].sijp[0] = 0; body[4].sijp[1] = DH[3*4+1]; body[4].sijp[2] = 0;

        Body::ang2mat(-0.0762709,M_PI_2,-M_PI_2, body[4].Cii, false);
        body[4].rhoip[0] = -0.000293313;
        body[4].rhoip[1] = 0.0465082;
        body[4].rhoip[2] = 0.000198586;
        body[4].mi = 0.236016152954683;
        body[4].Ixx = 1.45905112959314e-004;
        body[4].Iyy = 1.61294325145748e-004;
        body[4].Izz = 6.87040338242159e-005;
        body[4].Ixy = 1.3939720862584e-007;
        body[4].Iyz = -7.64663380262862e-006;
        body[4].Izx = 4.48718859748069e-007;
        body[4].Jip[0] = body[4].Ixx; body[4].Jip[1] = body[4].Ixy; body[4].Jip[2] = body[4].Izx;
        body[4].Jip[3] = body[4].Ixy; body[4].Jip[4] = body[4].Iyy; body[4].Jip[5] = body[4].Iyz;
        body[4].Jip[6] = body[4].Izx; body[4].Jip[7] = body[4].Iyz; body[4].Jip[8] = body[4].Izz;
        body[4].u_vec[0] = 0; body[4].u_vec[1] = 0; body[4].u_vec[2] = 1;

        body[4].r_hat = 0;
        body[4].y = 0;
        body[4].yp = 0;
        body[4].K = 200;
        body[4].p_linear = 0;
        body[4].p_rotate = 0;
        body[4].p = 0;
        body[4].y_old = 0;
        body[4].yp_old = 0;
        body[4].f_cut=1000;
        memset(body[4].time_zone, 0, sizeof(float)*3);
        body[4].filter_indx = 0;

        // body 5 variables
        Body::ang2mat(DH[4*4+3], DH[4*4+0], 0, body[5].Cij);
        body[5].sijp[0] = 0; body[5].sijp[1] = 0; body[5].sijp[2] = DH[4*4+2];

        Body::ang2mat(-1.56096, 1.64707, 0, body[5].Cii, false);
        body[5].rhoip[0] = 0.0461179;
        body[5].rhoip[1] = -0.0127205;
        body[5].rhoip[2] = -0.00513714;
        body[5].mi = 0.431128576046632;
        body[5].Ixx = 2.52382582624056e-004;
        body[5].Iyy = 2.7649832000486e-004;
        body[5].Izz = 2.77696751567096e-004;
        body[5].Ixy = 6.42650948265985e-006;
        body[5].Iyz = -1.18465047588535e-005;
        body[5].Izx = 1.9884316920465e-005;
        body[5].Jip[0] = body[5].Ixx; body[5].Jip[1] = body[5].Ixy; body[5].Jip[2] = body[5].Izx;
        body[5].Jip[3] = body[5].Ixy; body[5].Jip[4] = body[5].Iyy; body[5].Jip[5] = body[5].Iyz;
        body[5].Jip[6] = body[5].Izx; body[5].Jip[7] = body[5].Iyz; body[5].Jip[8] = body[5].Izz;
        body[5].u_vec[0] = 0; body[5].u_vec[1] = 0; body[5].u_vec[2] = 1;

        body[5].r_hat = 0;
        body[5].y = 0;
        body[5].yp = 0;
        body[5].K = 200;
        body[5].p_linear = 0;
        body[5].p_rotate = 0;
        body[5].p = 0;
        body[5].y_old = 0;
        body[5].yp_old = 0;
        body[5].f_cut=1000;
        memset(body[5].time_zone, 0, sizeof(float)*3);
        body[5].filter_indx = 0;

        // body 6 variables
        Body::ang2mat(DH[5*4+3], DH[5*4+0], 0, body[6].Cij);
        body[6].sijp[0] = 0; body[6].sijp[1] = 0; body[6].sijp[2] = DH[5*4+2];

        Body::ang2mat(1.76182, 3.06469, 3.0136, body[6].Cii, false);
        body[6].rhoip[0] = 0.000285815;
        body[6].rhoip[1] = 0.00408477;
        body[6].rhoip[2] = 0.0828791;
        body[6].mi = 6.95238304501428e-002;
        body[6].Ixx = 2.3856149716382e-005;
        body[6].Iyy = 1.54234398164e-004;
        body[6].Izz = 1.73835472035582e-004;
        body[6].Ixy = -5.1743115435324e-005;
        body[6].Iyz = 8.52575927879209e-007;
        body[6].Izx = 2.14709465974145e-006;
        body[6].Jip[0] = body[6].Ixx; body[6].Jip[1] = body[6].Ixy; body[6].Jip[2] = body[6].Izx;
        body[6].Jip[3] = body[6].Ixy; body[6].Jip[4] = body[6].Iyy; body[6].Jip[5] = body[6].Iyz;
        body[6].Jip[6] = body[6].Izx; body[6].Jip[7] = body[6].Iyz; body[6].Jip[8] = body[6].Izz;
        body[6].u_vec[0] = 0; body[6].u_vec[1] = 0; body[6].u_vec[2] = 1;
//        body[6].tool_offset[0] = 0;  body[6].tool_offset[1] = 0.1501;    body[6].tool_offset[2] = -0.005;

        body[6].r_hat = 0;
        body[6].y = 0;
        body[6].yp = 0;
        body[6].K = 200;
        body[6].p_linear = 0;
        body[6].p_rotate = 0;
        body[6].p = 0;
        body[6].y_old = 0;
        body[6].yp_old = 0;
        body[6].f_cut=1000;
        memset(body[6].time_zone, 0, sizeof(float)*3);
        body[6].filter_indx = 0;

//        memset(body[6].tool_offset, 0, sizeof(double)*3);
//        body[6].sijp[0] += body[6].tool_offset[0];
//        body[6].sijp[1] += body[6].tool_offset[1];
//        body[6].sijp[2] += body[6].tool_offset[2];


//        Body::ang2mat(DH[5*4+3], DH[5*4+0], 0, body[6].Cij);
//        body[6].sijp[0] = 0; body[6].sijp[1] = 0; body[6].sijp[2] = DH[5*4+2];

//        Body::ang2mat(M_PI, M_PI_2, 0, body[6].Cii, false);
//        body[6].rhoip[0] = 4.25231e-8; body[6].rhoip[1] = 0.00049999; body[6].rhoip[2] = 0.0756101;
//        body[6].mi = 1.06900256777816e-002;
//        body[6].Ixx = 5.4230201398644e-007;
//        body[6].Iyy = 8.27349038593228e-007;
//        body[6].Izz = 6.80541319418483e-007;
//        body[6].Ixy = 1.68734917731254e-012;
//        body[6].Iyz = 1.25961065157397e-013;
//        body[6].Izx = 3.11703037783589e-013;
//        body[6].Jip[0] = body[6].Ixx; body[6].Jip[1] = body[6].Ixy; body[6].Jip[2] = body[6].Izx;
//        body[6].Jip[3] = body[6].Ixy; body[6].Jip[4] = body[6].Iyy; body[6].Jip[5] = body[6].Iyz;
//        body[6].Jip[6] = body[6].Izx; body[6].Jip[7] = body[6].Iyz; body[6].Jip[8] = body[6].Izz;
//        body[6].u_vec[0] = 0; body[6].u_vec[1] = 0; body[6].u_vec[2] = 1;
    }
    else if(ver == FAR_V4){
        // DH paramter
        // | Link | alpha(deg) |  a(mm)  |  d(mm)   | theta(deg) |
        // |=====================================================|
        // |  1   |    -90     |    0    |    0     |    -90     |
        // |  2   |      0     |  164.25 |    0     |    -90     |
        // |  3   |    180     |  170    |    0     |      0     |
        // |  4   |     90     |   65.25 |    0     |     90     |
        // |  5   |     90     |    0    |  -16.25  |     90     |
        // |  6   |      0     |    0    |   93.25  |      0     |

        DH[0] = -90;    DH[1] =  0;         DH[2] =   0;         DH[3] = -90;
        DH[4] =   0;    DH[5] =  0.16425;   DH[6] =   0;         DH[7] = -90;
        DH[8] = 180;    DH[9] =  0.170;     DH[10] =  0;         DH[11] =  0;
        DH[12] = 90;    DH[13] = 0.06525;   DH[14] =  0;         DH[15] = 90;
        DH[16] = 90;    DH[17] = 0;         DH[18] = -0.01625;   DH[19] = 90;
        DH[20] =  0;    DH[21] = 0;         DH[22] =  0.09325;   DH[23] =  0;

        // body 0 variable
        body[0].Ai[0] = 1; body[0].Ai[1] = 0; body[0].Ai[2] = 0;
        body[0].Ai[3] = 0; body[0].Ai[4] = 1; body[0].Ai[5] = 0;
        body[0].Ai[6] = 0; body[0].Ai[7] = 0; body[0].Ai[8] = 1;

        body[0].Cij[0] = 1; body[0].Cij[1] = 0; body[0].Cij[2] = 0;
        body[0].Cij[3] = 0; body[0].Cij[4] = 1; body[0].Cij[5] = 0;
        body[0].Cij[6] = 0; body[0].Cij[7] = 0; body[0].Cij[8] = 1;

        body[0].ri[0] = 0; body[0].ri[1] = 0; body[0].ri[2] = 0;

        // preliminary work
        memset(body[0].Yih, 0, sizeof(double)*6);
        memset(body[0].wi, 0, sizeof(double)*3);
        memset(body[0].wit, 0, sizeof(double)*9);

        Body::ang2mat(0, 0, 0, body[0].Cij);
        body[0].sijp[0] = 0; body[0].sijp[1] = 0; body[0].sijp[2] = 0;

        body[0].ri_dot[0] = 0; body[0].ri_dot[1] = 0; body[0].ri_dot[2] = 0;
        body[0].wi[0] = 0; body[0].wi[1] = 0; body[0].wi[2] = 0;

        body[0].u_vec[0] = 0; body[0].u_vec[1] = 0; body[0].u_vec[2] = 1;

        // body 1 variables
        Body::ang2mat(DH[0*4+3], DH[0*4+0], 0, body[1].Cij);
        body[1].sijp[0] = 0; body[1].sijp[1] = 0; body[1].sijp[2] = 0;

        Body::ang2mat(0, M_PI_2, 0, body[1].Cii, false);
        body[1].rhoip[0] = 0.000110511;
        body[1].rhoip[1] = -1.16167e-8;
        body[1].rhoip[2] = -0.0187838;
        body[1].mi = 0.153104343087778;
        body[1].Ixx = 6.96678741977276e-005;
        body[1].Iyy = 1.12136062524532e-004;
        body[1].Izz = 1.20113478363185e-004;
        body[1].Ixy = 6.85012902792771e-008;
        body[1].Iyz = 1.30749783280943e-011;
        body[1].Izx = 2.701372946453e-009;
        body[1].Jip[0] = body[1].Ixx; body[1].Jip[1] = body[1].Ixy; body[1].Jip[2] = body[1].Izx;
        body[1].Jip[3] = body[1].Ixy; body[1].Jip[4] = body[1].Iyy; body[1].Jip[5] = body[1].Iyz;
        body[1].Jip[6] = body[1].Izx; body[1].Jip[7] = body[1].Iyz; body[1].Jip[8] = body[1].Izz;
        body[1].u_vec[0] = 0; body[1].u_vec[1] = 0; body[1].u_vec[2] = 1;

        body[1].r_hat = 0;
        body[1].y = 0;
        body[1].yp = 0;
        body[1].K = 200;
        body[1].p_linear = 0;
        body[1].p_rotate = 0;
        body[1].p = 0;
        body[1].y_old = 0;
        body[1].yp_old = 0;
        body[1].f_cut=1000;
        memset(body[1].time_zone, 0, sizeof(float)*3);
        body[1].filter_indx = 0;

        // body 2 variables
        Body::ang2mat(DH[1*4+3], DH[1*4+0], 0, body[2].Cij);
        body[2].sijp[0] = DH[1*4+2]; body[2].sijp[1] = -DH[1*4+1]; body[2].sijp[2] = 0;

        Body::ang2mat(M_PI_2, M_PI_2, M_PI_2, body[2].Cii, false);
        body[2].rhoip[0] = 0.00103404;
        body[2].rhoip[1] = -0.0555052;
        body[2].rhoip[2] = -4.99696e-5;
        body[2].mi = 0.741242292221872;
        body[2].Ixx = 2.1034196117051e-003;
        body[2].Iyy = 5.22299087086884e-004;
        body[2].Izz = 2.38975745676555e-003;
        body[2].Ixy = 1.3560795940288e-005;
        body[2].Iyz = -3.32194031774024e-005;
        body[2].Izx = 4.10393311075504e-008;
        body[2].Jip[0] = body[2].Ixx; body[2].Jip[1] = body[2].Ixy; body[2].Jip[2] = body[2].Izx;
        body[2].Jip[3] = body[2].Ixy; body[2].Jip[4] = body[2].Iyy; body[2].Jip[5] = body[2].Iyz;
        body[2].Jip[6] = body[2].Izx; body[2].Jip[7] = body[2].Iyz; body[2].Jip[8] = body[2].Izz;
        body[2].u_vec[0] = 0; body[2].u_vec[1] = 0; body[2].u_vec[2] = 1;

        body[2].r_hat = 0;
        body[2].y = 0;
        body[2].yp = 0;
        body[2].K = 200;
        body[2].p_linear = 0;
        body[2].p_rotate = 0;
        body[2].p = 0;
        body[2].y_old = 0;
        body[2].yp_old = 0;
        body[2].f_cut=1000;
        memset(body[2].time_zone, 0, sizeof(float)*3);
        body[2].filter_indx = 0;

        // body 3 variables
        Body::ang2mat(DH[2*4+3], DH[2*4+0], 0, body[3].Cij);
        body[3].sijp[0] = DH[2*4+1]; body[3].sijp[1] = 0; body[3].sijp[2] = 0;

        Body::ang2mat(-M_PI_2,M_PI_2,M_PI_2, body[3].Cii, false);
        body[3].rhoip[0] = 0.0684422;
        body[3].rhoip[1] = 0.000513196;
        body[3].rhoip[2] = -0.000415251;
        body[3].mi = 0.554447311677615;
        body[3].Ixx = 2.26605992866374e-003;
        body[3].Iyy = 2.30239764187232e-003;
        body[3].Izz = 1.59720819620979e-004;
        body[3].Ixy = -6.14645398504087e-008;
        body[3].Iyz = 3.79269133429595e-006;
        body[3].Izx = 2.47499181937338e-006;
        body[3].Jip[0] = body[3].Ixx; body[3].Jip[1] = body[3].Ixy; body[3].Jip[2] = body[3].Izx;
        body[3].Jip[3] = body[3].Ixy; body[3].Jip[4] = body[3].Iyy; body[3].Jip[5] = body[3].Iyz;
        body[3].Jip[6] = body[3].Izx; body[3].Jip[7] = body[3].Iyz; body[3].Jip[8] = body[3].Izz;
        body[3].u_vec[0] = 0; body[3].u_vec[1] = 0; body[3].u_vec[2] = 1;

        body[3].r_hat = 0;
        body[3].y = 0;
        body[3].yp = 0;
        body[3].K = 200;
        body[3].p_linear = 0;
        body[3].p_rotate = 0;
        body[3].p = 0;
        body[3].y_old = 0;
        body[3].yp_old = 0;
        body[3].f_cut=1000;
        memset(body[3].time_zone, 0, sizeof(float)*3);
        body[3].filter_indx = 0;

        // body 4 variables
        Body::ang2mat(DH[3*4+3], DH[3*4+0], 0, body[4].Cij);
        body[4].sijp[0] = 0; body[4].sijp[1] = DH[3*4+1]; body[4].sijp[2] = 0;

        Body::ang2mat(-0.0762709,M_PI_2,-M_PI_2, body[4].Cii, false);
        body[4].rhoip[0] = -0.000293313;
        body[4].rhoip[1] = 0.0465082;
        body[4].rhoip[2] = 0.000198586;
        body[4].mi = 0.236016152954683;
        body[4].Ixx = 1.45905112959314e-004;
        body[4].Iyy = 1.61294325145748e-004;
        body[4].Izz = 6.87040338242159e-005;
        body[4].Ixy = 1.3939720862584e-007;
        body[4].Iyz = -7.64663380262862e-006;
        body[4].Izx = 4.48718859748069e-007;
        body[4].Jip[0] = body[4].Ixx; body[4].Jip[1] = body[4].Ixy; body[4].Jip[2] = body[4].Izx;
        body[4].Jip[3] = body[4].Ixy; body[4].Jip[4] = body[4].Iyy; body[4].Jip[5] = body[4].Iyz;
        body[4].Jip[6] = body[4].Izx; body[4].Jip[7] = body[4].Iyz; body[4].Jip[8] = body[4].Izz;
        body[4].u_vec[0] = 0; body[4].u_vec[1] = 0; body[4].u_vec[2] = 1;

        body[4].r_hat = 0;
        body[4].y = 0;
        body[4].yp = 0;
        body[4].K = 200;
        body[4].p_linear = 0;
        body[4].p_rotate = 0;
        body[4].p = 0;
        body[4].y_old = 0;
        body[4].yp_old = 0;
        body[4].f_cut=1000;
        memset(body[4].time_zone, 0, sizeof(float)*3);
        body[4].filter_indx = 0;

        // body 5 variables
        Body::ang2mat(DH[4*4+3], DH[4*4+0], 0, body[5].Cij);
        body[5].sijp[0] = 0; body[5].sijp[1] = 0; body[5].sijp[2] = DH[4*4+2];

        Body::ang2mat(-1.56096, 1.64707, 0, body[5].Cii, false);
        body[5].rhoip[0] = 0.0461179;
        body[5].rhoip[1] = -0.0127205;
        body[5].rhoip[2] = -0.00513714;
        body[5].mi = 0.431128576046632;
        body[5].Ixx = 2.52382582624056e-004;
        body[5].Iyy = 2.7649832000486e-004;
        body[5].Izz = 2.77696751567096e-004;
        body[5].Ixy = 6.42650948265985e-006;
        body[5].Iyz = -1.18465047588535e-005;
        body[5].Izx = 1.9884316920465e-005;
        body[5].Jip[0] = body[5].Ixx; body[5].Jip[1] = body[5].Ixy; body[5].Jip[2] = body[5].Izx;
        body[5].Jip[3] = body[5].Ixy; body[5].Jip[4] = body[5].Iyy; body[5].Jip[5] = body[5].Iyz;
        body[5].Jip[6] = body[5].Izx; body[5].Jip[7] = body[5].Iyz; body[5].Jip[8] = body[5].Izz;
        body[5].u_vec[0] = 0; body[5].u_vec[1] = 0; body[5].u_vec[2] = 1;

        body[5].r_hat = 0;
        body[5].y = 0;
        body[5].yp = 0;
        body[5].K = 200;
        body[5].p_linear = 0;
        body[5].p_rotate = 0;
        body[5].p = 0;
        body[5].y_old = 0;
        body[5].yp_old = 0;
        body[5].f_cut=1000;
        memset(body[5].time_zone, 0, sizeof(float)*3);
        body[5].filter_indx = 0;

        // body 6 variables
        Body::ang2mat(DH[5*4+3], DH[5*4+0], 0, body[6].Cij);
        body[6].sijp[0] = 0; body[6].sijp[1] = 0; body[6].sijp[2] = DH[5*4+2];

        Body::ang2mat(1.76182, 3.06469, 3.0136, body[6].Cii, false);
        body[6].rhoip[0] = 0.000285815;
        body[6].rhoip[1] = 0.00408477;
        body[6].rhoip[2] = 0.0828791;
        body[6].mi = 6.95238304501428e-002;
        body[6].Ixx = 2.3856149716382e-005;
        body[6].Iyy = 1.54234398164e-004;
        body[6].Izz = 1.73835472035582e-004;
        body[6].Ixy = -5.1743115435324e-005;
        body[6].Iyz = 8.52575927879209e-007;
        body[6].Izx = 2.14709465974145e-006;
        body[6].Jip[0] = body[6].Ixx; body[6].Jip[1] = body[6].Ixy; body[6].Jip[2] = body[6].Izx;
        body[6].Jip[3] = body[6].Ixy; body[6].Jip[4] = body[6].Iyy; body[6].Jip[5] = body[6].Iyz;
        body[6].Jip[6] = body[6].Izx; body[6].Jip[7] = body[6].Iyz; body[6].Jip[8] = body[6].Izz;
        body[6].u_vec[0] = 0; body[6].u_vec[1] = 0; body[6].u_vec[2] = 1;

        body[6].r_hat = 0;
        body[6].y = 0;
        body[6].yp = 0;
        body[6].K = 200;
        body[6].p_linear = 0;
        body[6].p_rotate = 0;
        body[6].p = 0;
        body[6].y_old = 0;
        body[6].yp_old = 0;
        body[6].f_cut=1000;
        memset(body[6].time_zone, 0, sizeof(float)*3);
        body[6].filter_indx = 0;
    }
    numeric = new Numerical();
}

RobotArm::~RobotArm() {
    delete[] PH;
    delete[] PH_pos;
    delete[] PH_ori;
    delete[] delta_q;
    //    delete[] J;
    delete[] JD;

    delete[] M;
    delete[] Q;
    delete[] Q_c;
    delete[] Q_g;

    delete[] body;
    delete numeric;
}

#ifdef FILEIO_H_
void RobotArm::run_kinematics()
{
    sprintf(file_name, "../FAR_Analysis/data/evaluation_motion_cpp.txt");
    fp = fopen(file_name, "w+");

    vector<double> ref_data;
    uint row = 2501;
    uint col = 14;
    load_data("../FAR_Analysis/data/evaluation_motion_recurdyn.txt", &ref_data, "\t");

    for(uint indx = 0; indx < row; indx++) {
        for (uint i = 1; i <= 6; i++) {
            body[i].qi = ref_data[indx*col + i + 1];
        }

        kinematics();

        save_data();

        printf("Time : %.3f[s]\n", static_cast<double>(t_current));

        t_current += h;
    }

    ref_data.clear();
    fclose(fp);
}
#endif

void RobotArm::set_tool_offset(double tool_offset[])
{
    body[6].sijp[0] = tool_offset[0];
    body[6].sijp[1] = tool_offset[1];
    body[6].sijp[2] = DH[5*4+2] + tool_offset[2];
}

void RobotArm::set_tool_offset(double tool_x, double tool_y, double tool_z)
{
    body[6].sijp[0] = tool_x;
    body[6].sijp[1] = tool_y;
    body[6].sijp[2] = DH[5*4+2] + tool_z;
}

void RobotArm::run_kinematics(double *q, double *pose){
    for(int i = 0; i < 6; i++){
        body[i+1].qi = q[i];
    }
    body[6].qi = q[5];
//        printf("present q : %f, %f, %f, %f, %f, %f\n",
//                  body[1].qi, body[2].qi, body[3].qi, body[4].qi, body[5].qi, body[6].qi);

//    double rpy[3] = {1.5708, 0, -2.0944};
//    double mat_rpy[9] = {0,};
//    double zyx[3] = {0,};
//    rpy2mat(rpy[2], rpy[1], rpy[0], mat_rpy);
//    mat2zyx(mat_rpy, zyx);
//    rt_printf("%f, %f, %f\n", zyx[0], zyx[1], zyx[2]);

    kinematics();

    //    double ori[3] = {0,};
    //    mat2rpy(body[num_body].Ae, ori);

    pose[0] = body[6].re[0];
    pose[1] = body[6].re[1];
    pose[2] = body[6].re[2];
    pose[3] = body[6].ori_zyx[0];
    pose[4] = body[6].ori_zyx[1];
    pose[5] = body[6].ori_zyx[2];

//    rt_printf("%f, %f, %f, %f, %f, %f\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

//    if(pose_zyx != nullptr){
//        pose_zyx[0] = body[6].re[0];
//        pose_zyx[1] = body[6].re[1];
//        pose_zyx[2] = body[6].re[2];
//        pose_zyx[3] = body[6].ori_zyx[0];
//        pose_zyx[4] = body[6].ori_zyx[1];
//        pose_zyx[5] = body[6].ori_zyx[2];
//    }
}

#ifdef FILEIO_H_
void RobotArm::run_inverse_kinematics() {
    sprintf(file_name, "../FAR_Analysis/data/evaluation_motion_cpp.txt");
    fp = fopen(file_name, "w+");

    vector<double> ref_data;
    uint row = 2501;
    uint col = 14;
    load_data("../FAR_Analysis/data/evaluation_motion_recurdyn.txt", &ref_data, "\t");
    for (uint i = 1; i <= 6; i++) {
        body[i].qi = ref_data[0*col + i + 1];
    }

    double pos_d[3], ori_d[3];
    for (uint indx = 0; indx < row; indx++) {
        pos_d[0] = ref_data[indx*col + 8];
        pos_d[1] = ref_data[indx*col + 9];
        pos_d[2] = ref_data[indx*col + 10];
        ori_d[0] = ref_data[indx*col + 11];
        ori_d[1] = ref_data[indx*col + 12];
        ori_d[2] = ref_data[indx*col + 13];

        kinematics();

        inverse_kinematics(pos_d, ori_d);

        save_data();

        printf("Time : %.3f[s]\n", static_cast<double>(t_current));

        t_current += h;
    }

    ref_data.clear();
    fclose(fp);
}
#endif

int RobotArm::run_inverse_kinematics(double* input_q, double* des_pose, double* cur_joint, double* cur_pose){
    goal_reach = false;

    epsilon_pos = 0.05;
    epsilon_ang = 1;

    for (uint i = 1; i <= num_body; i++) {
        body[i].qi = input_q[i - 1];
    }

    //    for(uint i = 0; i < 5; i++){
    pos_d[0] = des_pose[0];
    pos_d[1] = des_pose[1];
    pos_d[2] = des_pose[2];

    memset(temp3, 0, sizeof(double)*9);
    rpy2mat(des_pose[5], des_pose[4], des_pose[3], temp3);
    mat2zyx(temp3, ori_d);

    kinematics();

    int err = inverse_kinematics(pos_d, ori_d);

    for(uint i = 1; i <= num_body; i++){
        cur_joint[i - 1] = body[i].qi;
    }

    //        kinematics();
    //    double ori[3] = {0,};
    //    mat2rpy(body[num_body].Ae, ori);
    cur_pose[0] = body[num_body].re[0];
    cur_pose[1] = body[num_body].re[1];
    cur_pose[2] = body[num_body].re[2];
    cur_pose[3] = body[num_body].ori_zyx[0];
    cur_pose[4] = body[num_body].ori_zyx[1];
    cur_pose[5] = body[num_body].ori_zyx[2];

    return err;

    //        double pos = sqrt(pow(des_pose[0] - cur_pose[0], 2) + pow(des_pose[1] - cur_pose[1], 2) + pow(des_pose[2] - cur_pose[2], 2));
    //        double ang_r = abs(des_pose[3] - cur_pose[3]);
    //        double ang_p = abs(des_pose[4] - cur_pose[4]);
    //        double ang_y = abs(des_pose[5] - cur_pose[5]);

    //        printf("[IK]pos : %f\n", pos);
    //        printf("[IK]ang_r : %f\t ang_p : %f\t ang_y : %f\n", ang_r, ang_p, ang_y);

    //        if (pos < epsilon_pos && ang_r < epsilon_ang && ang_p < epsilon_ang && ang_y < epsilon_ang){
    //            goal_reach = true;
    //            printf("[IK]iteration : %d\n", i);
    //            break;
    //        }
    //        else{
    //            goal_reach = false;
    //        }
    //    }
}

void RobotArm::run_dynamics(double *q, double *q_dot, double *q_ddot, double *end_pose)
{
    for(int i = 0; i < 6; i++){
        body[i+1].qi = q[i];
        body[i+1].qi_dot = q_dot[i];
    }

    run_kinematics(q, end_pose);
    dynamics();

    for(int i = 0; i < 6; i++){
        q_ddot[i] = body[i+1].qi_ddot;
    }
}

#ifdef FILEIO_H_
void RobotArm::run_dynamics(){
    sprintf(file_name, "../FAR_Analysis/data/evaluation_dynamics_cpp.txt");
    fp = fopen(file_name, "w+");

    vector<double> ref_data;
    uint row = 2501;
    uint col = 20;
    load_data("../FAR_Analysis/data/evaluation_dynamics_recurdyn.txt", &ref_data, "\t");

    for (uint indx = 0; indx < row; indx++) {
        for (uint i = 0; i <= 6; i++) {
            body[i].qi = ref_data[indx*col + i + 1];
            body[i].qi_dot = ref_data[indx*col + i + 1 + 6];
        }

        kinematics();
        dynamics();

        save_data();

        printf("Time : %.3f[s]\n", static_cast<double>(t_current));

        t_current += h;
    }

    ref_data.clear();
    fclose(fp);
}
#endif

void RobotArm::gravity_compensation(double *q, double *q_dot, double *torque){
    for(uint i = 1; i <= num_body; i++){
        body[i].qi = q[i - 1];
        body[i].qi_dot = q_dot[i - 1];
    }

//    rt_printf("current_q  : %f, %f, %f, %f, %f, %f\n",
//              q[0], q[1], q[2], q[3], q[4], q[5]);
//    rt_printf("current_dq : %f, %f, %f, %f, %f, %f\n",
//              q_dot[0], q_dot[1], q_dot[2], q_dot[3], q_dot[4], q_dot[5]);

    kinematics();
    dynamics();

    for(uint i = 0; i < 6; i++){
        torque[i] = -Q[i];
    }
}

void RobotArm::disturbance_observer(double *q, double *q_dot, double *torque, double *dist){
    for(unsigned int i = 0; i < num_body; i++){
        body[i + 1].qi = q[i];
        body[i + 1].qi_dot = q_dot[i];
        body[i + 1].Ta = torque[i];
    }

    body[0].K = 80;
    body[1].K = 80;
    body[2].K = 80;
    body[3].K = 80;
    body[4].K = 80;
    body[5].K = 80;

    body[0].f_cut = 100;
    body[1].f_cut = 100;
    body[2].f_cut = 100;
    body[3].f_cut = 100;
    body[4].f_cut = 100;
    body[5].f_cut = 100;

    kinematics();
    dynamics();
    residual();

    for(unsigned int i = 1; i <= num_body; i++){
        high_pass_filter(body[i].r_hat, body[i].time_zone, &body[i].r_hat_filter, h, body[i].f_cut);
    }

    for(unsigned int i = 0; i < num_body; i++){
        dist[i] = body[i + 1].r_hat_filter;
    }
}

void RobotArm::kinematics()
{
    for (uint indx = 1; indx <= num_body; indx++) {
        body[indx].Aijpp[0] = cos(body[indx].qi);   body[indx].Aijpp[1] = -sin(body[indx].qi);  body[indx].Aijpp[2] = 0;
        body[indx].Aijpp[3] = sin(body[indx].qi);   body[indx].Aijpp[4] =  cos(body[indx].qi);  body[indx].Aijpp[5] = 0;
        body[indx].Aijpp[6] = 0;                    body[indx].Aijpp[7] = 0;                    body[indx].Aijpp[8] = 1;

        mat(body[indx-1].Ai, body[indx-1].Cij, 3, 3, 3, 3, body[indx].Ai_Cij);
        mat(body[indx].Ai_Cij, body[indx].Aijpp, 3, 3, 3, 3, body[indx].Ai);
        mat(body[indx].Ai_Cij, body[indx].u_vec, 3, 3, 3, body[indx].zi);
        mat(body[indx-1].Cij, body[indx].Aijpp, 3, 3, 3,3, body[indx].Cij_Aijpp);

        // position
        mat(body[indx-1].Ai, body[indx-1].sijp, 3, 3, 3, body[indx-1].sij);
        for (uint i = 0; i < 3; i++){
            body[indx].ri[i] = body[indx-1].ri[i] + body[indx-1].sij[i];
        }
    }

    // End point
    //    Body *body_end = &body[num_body];
    mat(body[num_body].Ai, body[num_body].sijp, 3, 3, 3, body[num_body].sij);
    for (uint i = 0; i < 3; i++){
        body[num_body].re[i] = body[num_body].ri[i] + body[num_body].sij[i];
    }

    mat(body[num_body].Ai, body[num_body].Cij, 3, 3, 3, 3, body[num_body].Ae);

    mat2rpy(body[num_body].Ae, body[num_body].ori);
    mat2zyx(body[num_body].Ae, body[num_body].ori_zyx);
}

int RobotArm::inverse_kinematics(double des_pos[3], double des_ang[3]) {
    errmax = 0;
    errtol = 1e-3;

    alpha = 1/4.0;
    for(int i = 0; i < 1/alpha; i++){
        for(int j = 0; j < 3; j++){
            err[j] = des_pos[j] - body[num_body].re[j];
            err[j + 3] = des_ang[j] - body[num_body].ori_zyx[j];
        }

        jacobian_zyx();

        numeric->ludcmp(J, 6, indx, 0.0, fac);
        numeric->lubksb(fac, 6, indx, err, qdot);

        for (uint j = 0; j < 6; j++){
            delta_q[j] = qdot[j]*alpha;
        }

        for (uint j = 0; j < num_body; j++) {
            body[j + 1].qi += delta_q[j];
        }

        kinematics();

        errmax = err[0];
        for(uint i = 1; i < num_body;i++){
            errmax = errmax > abs(err[i]) ? errmax : abs(err[i]);
        }

    }
//    printf("[IK]Err Max : %E\n", errmax);

    if(errmax < errtol){
        return 0;
    }
    else{
        return 1;
    }


    //    Body *body_end = &body[num_body];
    //    for (uint i = 0; i < 3; i++) {
    //        PH_pos[i] = des_pos[i] - body_end->re[i];
    //        PH_ori[i] = des_ang[i] - body_end->ori[i];
    //    }

    //    for (uint i = 0; i < 3; i++) {
    //        PH[i] = PH_pos[i];
    //        PH[i + 3] = PH_ori[i];
    //    }

    //#if 0
    //    jacobian();

    //    double *U, *s, *V;
    //    U = new double[dof * dof];
    //    s = new double[MIN(dof, num_body)];
    //    V = new double[num_body*num_body];

    //    numeric->svdcmp(J, dof, num_body, U, s, V);

    //    memset(JD, 0, sizeof(double) * num_body*dof);
    //    double *temp = new double[num_body*dof];
    //    double lamda = 1e-5;
    //    for (uint i = 0; i < dof; i++) {
    //        for (uint j = 0; j < num_body; j++) {
    //            for (uint k = 0; k < dof; k++) {
    //                temp[j * dof + k] = V[j * num_body + i] * U[k * num_body + i];
    //            }
    //        }
    //        for (uint j = 0; j < num_body; j++) {
    //            for (uint k = 0; k < dof; k++) {
    //                JD[j * dof + k] += (s[i] / (s[i]*s[i] +lamda*lamda))*(temp[j * dof + k]);
    //            }
    //        }
    //    }

    //    delete[] s;
    //    delete[] U;
    //    delete[] V;
    //    delete[] temp;


    //    memset(delta_q, 0, sizeof(double) * 6);
    //    for (uint i = 0; i < num_body; i++) {
    //        for (uint j = 0; j < num_body; j++) {
    //            delta_q[i] += JD[i * num_body + j] * PH[j];
    //        }
    //    }
    //#else

    //    int *indx = new int[6];
    //    double *fac = new double[6*6];
    //    double errmax = 0;
    //    int NRcount = 0;

    //    do{
    //        jacobian();

    //        numeric->ludcmp(J, 6, indx, 0.0, fac);
    //        memset(delta_q, 0, sizeof(double) * 6);
    //        numeric->lubksb(fac, 6, indx, PH, delta_q);

    //        for (uint i = 0; i < num_body; i++) {
    //            body[i + 1].qi += delta_q[i];
    //        }

    //        kinematics();

    //        for (uint i = 0; i < 3; i++) {
    //            PH_pos[i] = des_pos[i] - body[num_body].re[i];
    //            PH_ori[i] = des_ang[i] - body[num_body].ori[i];
    //        }

    //        for (uint i = 0; i < 3; i++) {
    //            PH[i] = PH_pos[i];
    //            PH[i + 3] = PH_ori[i];
    //        }

    //        errmax = PH[0];
    //        for(uint i = 1; i < num_body;i++){
    //            errmax = errmax > abs(PH[i]) ? errmax : abs(PH[i]);
    //        }

    //        NRcount++;
    //    }while(errmax > 1e-1 && NRcount < 10);

    //    rt_printf("[IK]Err Max : %E\t : Iteration : %d\n", errmax, NRcount);

    //    delete[] indx;
    //    delete[] fac;

    //    if (NRcount == 10){
    //        return 0;
    //    }
    //    else{
    //        return 1;
    //    }
    //#endif
}

void RobotArm::jacobian()
{
//    double *Jv = new double[3 * num_body];
//    double *Jw = new double[3 * num_body];

    memset(Jv, 0, sizeof(double)*3*num_body);
    memset(Jw, 0, sizeof(double)*3*num_body);

//    Body *body0, *body1;
//    Body *body_end = &body[num_body];

#if 0
    //    A[9] = {cos(M_PI), -sin(M_PI), 0, sin(M_PI), cos(M_PI), 0, 0, 0, 1};
//    A[0] = cos(M_PI);
//    A[1] = -sin(M_PI);
//    A[2] = 0;
//    A[3] = sin(M_PI);
//    A[4] = cos(M_PI);
//    A[5] = 0;
//    A[6] = 0;
//    A[7] = 0;
//    A[8] = 1;

    for (uint indx = 1; indx <= num_body; indx++){
        for(uint i = 0; i < 3; i++){
            body[indx].oi[i] = body[num_body].re[i] - body[indx].ri[i];
        }
        tilde(body[indx].zi, body[indx].zit);
        mat(body[indx].zit, body[indx].oi, 3, 3, 3, body[indx].Jvi);

        for (uint i = 0; i < 3; i++){
            Jv[i*num_body + (indx - 1)] = body[indx].Jvi[i];
            Jw[i*num_body + (indx - 1)] = body[indx].Jwi[i];
        }
    }
#elif 0
    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        body1->Aijpp_qi[0] = -sin(body1->qi); body1->Aijpp_qi[1] = -cos(body1->qi); body1->Aijpp_qi[2] = 0;
        body1->Aijpp_qi[3] =  cos(body1->qi); body1->Aijpp_qi[4] = -sin(body1->qi); body1->Aijpp_qi[5] = 0;
        body1->Aijpp_qi[6] = 0; body1->Aijpp_qi[7] = 0; body1->Aijpp_qi[8] = 0;
    }

    for(uint indx = 1; indx <= num_body; indx++){
        memset(body[indx].Ae_qi, 0, sizeof(double)*9);
        memset(body[indx].re_qi, 0, sizeof(double)*3);
        for(uint indx2 = indx; indx2 <= num_body; indx2++){
            if (indx == indx2){
                mat(body[indx2].Ai_Cij, body[indx2].Aijpp_qi, 3, 3, 3, 3, body[indx2].Ai_Cij_Aijpp_qi);
                for(uint i = 0; i < 9; i++){
                    body[indx].Ae_qi[i] += body[indx2].Ai_Cij_Aijpp_qi[i];
                }
            }
            else{
                mat(body[indx].Ae_qi, body[indx2 - 1].Cij, 3, 3, 3, 3, body[indx2].Ae_qi_Cij);
                mat(body[indx2].Ae_qi_Cij, body[indx2].Aijpp, 3, 3, 3, 3, body[indx].Ae_qi_Cij_Aijpp);
                memcpy(body[indx].Ae_qi, body[indx].Ae_qi_Cij_Aijpp, sizeof(double)*9);
            }
            if (indx2 < num_body){
                mat(body[indx].Ae_qi, body[indx2].sijp, 3, 3, 3, body[indx2].Ae_qi_sijp);
                for(uint i = 0; i < 3; i++){
                    body[indx].re_qi[i] += body[indx2].Ae_qi_sijp[i];
                }
            }
        }
    }

    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        mat(body1->Ae_qi, body_end->sijp, 3, 3, 3, body1->Ae_qi_end);
        if (indx < num_body){
            for(uint i = 0; i < 3; i++){
                body1->Jvi[i] = body1->re_qi[i] + body1->Ae_qi_end[i];
            }
        }
        else{
            for(uint i = 0; i < 3; i++){
                body1->Jvi[i] = body1->Ae_qi_end[i];
            }
        }
        for (uint i = 0; i < 3; i++){
            Jv[i*num_body + indx - 1] = body1->Jvi[i];
        }
    }

    Ae_31 = body_end->Ae[6];
    Ae_32 = body_end->Ae[7];
    Ae_33 = body_end->Ae[8];
    Ae_21 = body_end->Ae[3];
    Ae_11 = body_end->Ae[0];

    Ae_32_33 = Ae_32/Ae_33;
    body[1].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[1].roll_qi_2 = ((body[1].Ae_qi[7]*Ae_33 - Ae_32*body[1].Ae_qi[8])/(pow(Ae_33, 2))); body[1].roll_qi = body[1].roll_qi_1*body[1].roll_qi_2;
    body[2].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[2].roll_qi_2 = ((body[2].Ae_qi[7]*Ae_33 - Ae_32*body[2].Ae_qi[8])/(pow(Ae_33, 2))); body[2].roll_qi = body[2].roll_qi_1*body[2].roll_qi_2;
    body[3].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[3].roll_qi_2 = ((body[3].Ae_qi[7]*Ae_33 - Ae_32*body[3].Ae_qi[8])/(pow(Ae_33, 2))); body[3].roll_qi = body[3].roll_qi_1*body[3].roll_qi_2;
    body[4].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[4].roll_qi_2 = ((body[4].Ae_qi[7]*Ae_33 - Ae_32*body[4].Ae_qi[8])/(pow(Ae_33, 2))); body[4].roll_qi = body[4].roll_qi_1*body[4].roll_qi_2;
    body[5].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[5].roll_qi_2 = ((body[5].Ae_qi[7]*Ae_33 - Ae_32*body[5].Ae_qi[8])/(pow(Ae_33, 2))); body[5].roll_qi = body[5].roll_qi_1*body[5].roll_qi_2;
    body[6].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[6].roll_qi_2 = ((body[6].Ae_qi[7]*Ae_33 - Ae_32*body[6].Ae_qi[8])/(pow(Ae_33, 2))); body[6].roll_qi = body[6].roll_qi_1*body[6].roll_qi_2;

    Ae_32_33_2 = pow(Ae_32, 2) + pow(Ae_33, 2);
    body[1].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[1].pitch_qi_2 = (-body[1].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[1].Ae_qi[8] + Ae_33*body[1].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[1].pitch_qi = body[1].pitch_qi_1*body[1].pitch_qi_2;
    body[2].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[2].pitch_qi_2 = (-body[2].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[2].Ae_qi[8] + Ae_33*body[2].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[2].pitch_qi = body[2].pitch_qi_1*body[2].pitch_qi_2;
    body[3].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[3].pitch_qi_2 = (-body[3].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[3].Ae_qi[8] + Ae_33*body[3].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[3].pitch_qi = body[3].pitch_qi_1*body[3].pitch_qi_2;
    body[4].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[4].pitch_qi_2 = (-body[4].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[4].Ae_qi[8] + Ae_33*body[4].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[4].pitch_qi = body[4].pitch_qi_1*body[4].pitch_qi_2;
    body[5].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[5].pitch_qi_2 = (-body[5].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[5].Ae_qi[8] + Ae_33*body[5].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[5].pitch_qi = body[5].pitch_qi_1*body[5].pitch_qi_2;
    body[6].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[6].pitch_qi_2 = (-body[6].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[6].Ae_qi[8] + Ae_33*body[6].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[6].pitch_qi = body[6].pitch_qi_1*body[6].pitch_qi_2;

    Ae_21_11 = Ae_21/Ae_11;
    body[1].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[1].yaw_qi_2 = ((body[1].Ae_qi[3]*Ae_11 - Ae_21*body[1].Ae_qi[0])/(pow(Ae_11, 2))); body[1].yaw_qi = body[1].yaw_qi_1*body[1].yaw_qi_2;
    body[2].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[2].yaw_qi_2 = ((body[2].Ae_qi[3]*Ae_11 - Ae_21*body[2].Ae_qi[0])/(pow(Ae_11, 2))); body[2].yaw_qi = body[2].yaw_qi_1*body[2].yaw_qi_2;
    body[3].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[3].yaw_qi_2 = ((body[3].Ae_qi[3]*Ae_11 - Ae_21*body[3].Ae_qi[0])/(pow(Ae_11, 2))); body[3].yaw_qi = body[3].yaw_qi_1*body[3].yaw_qi_2;
    body[4].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[4].yaw_qi_2 = ((body[4].Ae_qi[3]*Ae_11 - Ae_21*body[4].Ae_qi[0])/(pow(Ae_11, 2))); body[4].yaw_qi = body[4].yaw_qi_1*body[4].yaw_qi_2;
    body[5].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[5].yaw_qi_2 = ((body[5].Ae_qi[3]*Ae_11 - Ae_21*body[5].Ae_qi[0])/(pow(Ae_11, 2))); body[5].yaw_qi = body[5].yaw_qi_1*body[5].yaw_qi_2;
    body[6].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[6].yaw_qi_2 = ((body[6].Ae_qi[3]*Ae_11 - Ae_21*body[6].Ae_qi[0])/(pow(Ae_11, 2))); body[6].yaw_qi = body[6].yaw_qi_1*body[6].yaw_qi_2;

    body[1].Jwi[0] = body[1].roll_qi; body[1].Jwi[1] = body[1].pitch_qi; body[1].Jwi[2] = body[1].yaw_qi;
    body[2].Jwi[0] = body[2].roll_qi; body[2].Jwi[1] = body[2].pitch_qi; body[2].Jwi[2] = body[2].yaw_qi;
    body[3].Jwi[0] = body[3].roll_qi; body[3].Jwi[1] = body[3].pitch_qi; body[3].Jwi[2] = body[3].yaw_qi;
    body[4].Jwi[0] = body[4].roll_qi; body[4].Jwi[1] = body[4].pitch_qi; body[4].Jwi[2] = body[4].yaw_qi;
    body[5].Jwi[0] = body[5].roll_qi; body[5].Jwi[1] = body[5].pitch_qi; body[5].Jwi[2] = body[5].yaw_qi;
    body[6].Jwi[0] = body[6].roll_qi; body[6].Jwi[1] = body[6].pitch_qi; body[6].Jwi[2] = body[6].yaw_qi;

    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        for (uint i = 0; i < 3; i++){
            Jw[i*num_body + indx - 1] = body1->Jwi[i];
        }
    }


#elif 1
    for (uint indx = 1; indx <= num_body; indx++) {
//        body0 = &body[indx - 1];
//        body1 = &body[indx];
        mat(body[indx-1].Cij, body[indx].Aijpp, 3, 3, 3, 3, body[indx-1].Cij_Aijpp);

//        double *Aijpp_qi_ptr = body[indx].Aijpp_qi;
//        *(Aijpp_qi_ptr++) = -sin(body[indx].qi);	*(Aijpp_qi_ptr++) = -cos(body[indx].qi);	*(Aijpp_qi_ptr++) = 0;
//        *(Aijpp_qi_ptr++) =  cos(body[indx].qi);	*(Aijpp_qi_ptr++) = -sin(body[indx].qi);	*(Aijpp_qi_ptr++) = 0;
//        *(Aijpp_qi_ptr++) = 0;                  *(Aijpp_qi_ptr++) = 0;                  *(Aijpp_qi_ptr) = 0;
        memset(body[indx].Aijpp_qi, 0, sizeof(double)*9);
        body[indx].Aijpp_qi[0] = -sin(body[indx].qi);
        body[indx].Aijpp_qi[1] = -cos(body[indx].qi);
        body[indx].Aijpp_qi[3] =  cos(body[indx].qi);
        body[indx].Aijpp_qi[4] = -sin(body[indx].qi);

        mat(body[indx].Ai_Cij, body[indx].Aijpp_qi, 3, 3, 3, 3, body[indx].Ai_Cij_Aijpp_qi);
    }

    double temp1[9] = { 0, }, temp2[3] = {0,};
    for (uint indx = 1; indx <= num_body; indx++) {
//        body0 = &body[indx - 1];
//        body1 = &body[indx];
        memset(body[indx].A6_qi, 0, sizeof(double) * 9);
        memset(body[indx].r6_qi, 0, sizeof(double) * 3);
        for (uint indx2 = indx; indx2 <= num_body; indx2++) {
            if (indx2 == indx) {
                for(uint i = 0; i < 9;i++){
                    body[indx].A6_qi[i] += body[indx].Ai_Cij_Aijpp_qi[i];
                }
            }
            else {
                mat(body[indx].A6_qi, body[indx2 - 1].Cij_Aijpp, 3, 3, 3, 3, temp1);
                memcpy(body[indx].A6_qi, temp1, sizeof(double) * 9);
            }
            if (indx2 < num_body) {
                mat(body[indx].A6_qi, body[indx2].sijp, 3, 3, 3, temp2);
                for(uint i = 0; i < 3; i++){
                    body[indx].r6_qi[i] += temp2[i];
                }
            }
        }

        mat(body[indx].A6_qi, body[num_body].Cij, 3, 3, 3, 3, body[indx].Ae_qi);
        mat(body[indx].A6_qi, body[num_body].sijp, 3, 3, 3, body[indx].re_qi);
        for (uint i = 0; i < 3; i++){
            body[indx].re_qi[i] += body[indx].r6_qi[i];
        }

        for (uint i = 0; i < 3; i++){
            Jv[i*num_body + indx - 1] = body[indx].re_qi[i];
        }
    }

    Ae_31 = body[num_body].Ae[6];
    Ae_32 = body[num_body].Ae[7];
    Ae_33 = body[num_body].Ae[8];
    Ae_21 = body[num_body].Ae[3];
    Ae_11 = body[num_body].Ae[0];

    roll_q_temp1 = Ae_32 * Ae_32 + Ae_33 * Ae_33;
    roll_q_temp2 = sqrt(roll_q_temp1);
    roll_q_temp3 = Ae_33 + roll_q_temp2;
    roll_q_temp4 = roll_q_temp2 * (roll_q_temp1 + Ae_33*roll_q_temp2);

    pitch_q_temp1 = sqrt(Ae_32*Ae_32 + Ae_33*Ae_33);
    pitch_q_temp2 = Ae_31 * Ae_31 + pitch_q_temp1 * pitch_q_temp1;
    pitch_q_temp3 = sqrt(pitch_q_temp2);
    pitch_q_temp4 = pitch_q_temp3 * (pitch_q_temp2 + pitch_q_temp1 * pitch_q_temp3);

    yaw_q_temp1 = Ae_21 * Ae_21 + Ae_11 * Ae_11;
    yaw_q_temp2 = sqrt(yaw_q_temp1);
    yaw_q_temp3 = Ae_11 + yaw_q_temp2;
    yaw_q_temp4 = yaw_q_temp2 * (yaw_q_temp1 + Ae_11*yaw_q_temp2);

    for (uint indx = 1; indx <= num_body; indx++) {
//        body1 = &body[indx];
        body[indx].Ae_qi_31 = body[indx].Ae_qi[6];
        body[indx].Ae_qi_32 = body[indx].Ae_qi[7];
        body[indx].Ae_qi_33 = body[indx].Ae_qi[8];
        body[indx].Ae_qi_21 = body[indx].Ae_qi[3];
        body[indx].Ae_qi_11 = body[indx].Ae_qi[0];

        body[indx].roll_qi = (roll_q_temp3*(body[indx].Ae_qi_32*Ae_33 - Ae_32*body[indx].Ae_qi_33)) / roll_q_temp4;
        body[indx].pitch_qi = -((pitch_q_temp3 + pitch_q_temp1)*(body[indx].Ae_qi_31*pitch_q_temp1 - Ae_31 * (Ae_32*body[indx].Ae_qi_32 + Ae_33 * body[indx].Ae_qi_33)/pitch_q_temp1))/ pitch_q_temp4;
        body[indx].yaw_qi = (yaw_q_temp3*(body[indx].Ae_qi_21*Ae_11 - Ae_21*body[indx].Ae_qi_11)) / yaw_q_temp4;

        Jw[0 * num_body + indx - 1] = body[indx].roll_qi;
        Jw[1 * num_body + indx - 1] = body[indx].pitch_qi;
        Jw[2 * num_body + indx - 1] = body[indx].yaw_qi;
    }
#endif

    memcpy(J, Jv, sizeof(double) * 3 * num_body);
    memcpy(J + 3 * num_body, Jw, sizeof(double) * 3 * num_body);

//    delete[] Jv;
//    delete[] Jw;
}

void RobotArm::jacobian_zyx()
{
    //    rt_printf("1[J]present_q : %f, %f, %f, %f, %f, %f\n", body[1].qi, body[2].qi, body[3].qi, body[4].qi, body[5].qi, body[6].qi);
    body[1].Aijpp_qi[0] = -sin(body[1].qi); body[1].Aijpp_qi[1] = -cos(body[1].qi); body[1].Aijpp_qi[2] = 0;
    body[1].Aijpp_qi[3] =  cos(body[1].qi); body[1].Aijpp_qi[4] = -sin(body[1].qi); body[1].Aijpp_qi[5] = 0;
    body[1].Aijpp_qi[6] = 0;                body[1].Aijpp_qi[7] = 0;                body[1].Aijpp_qi[8] = 0;

    body[2].Aijpp_qi[0] = -sin(body[2].qi); body[2].Aijpp_qi[1] = -cos(body[2].qi); body[2].Aijpp_qi[2] = 0;
    body[2].Aijpp_qi[3] =  cos(body[2].qi); body[2].Aijpp_qi[4] = -sin(body[2].qi); body[2].Aijpp_qi[5] = 0;
    body[2].Aijpp_qi[6] = 0;                body[2].Aijpp_qi[7] = 0;                body[2].Aijpp_qi[8] = 0;

    body[3].Aijpp_qi[0] = -sin(body[3].qi); body[3].Aijpp_qi[1] = -cos(body[3].qi); body[3].Aijpp_qi[2] = 0;
    body[3].Aijpp_qi[3] =  cos(body[3].qi); body[3].Aijpp_qi[4] = -sin(body[3].qi); body[3].Aijpp_qi[5] = 0;
    body[3].Aijpp_qi[6] = 0;                body[3].Aijpp_qi[7] = 0;                body[3].Aijpp_qi[8] = 0;

    body[4].Aijpp_qi[0] = -sin(body[4].qi); body[4].Aijpp_qi[1] = -cos(body[4].qi); body[4].Aijpp_qi[2] = 0;
    body[4].Aijpp_qi[3] =  cos(body[4].qi); body[4].Aijpp_qi[4] = -sin(body[4].qi); body[4].Aijpp_qi[5] = 0;
    body[4].Aijpp_qi[6] = 0;                body[4].Aijpp_qi[7] = 0;                body[4].Aijpp_qi[8] = 0;

    body[5].Aijpp_qi[0] = -sin(body[5].qi); body[5].Aijpp_qi[1] = -cos(body[5].qi); body[5].Aijpp_qi[2] = 0;
    body[5].Aijpp_qi[3] =  cos(body[5].qi); body[5].Aijpp_qi[4] = -sin(body[5].qi); body[5].Aijpp_qi[5] = 0;
    body[5].Aijpp_qi[6] = 0;                body[5].Aijpp_qi[7] = 0;                body[5].Aijpp_qi[8] = 0;

    body[6].Aijpp_qi[0] = -sin(body[6].qi); body[6].Aijpp_qi[1] = -cos(body[6].qi); body[6].Aijpp_qi[2] = 0;
    body[6].Aijpp_qi[3] =  cos(body[6].qi); body[6].Aijpp_qi[4] = -sin(body[6].qi); body[6].Aijpp_qi[5] = 0;
    body[6].Aijpp_qi[6] = 0;                body[6].Aijpp_qi[7] = 0;                body[6].Aijpp_qi[8] = 0;

    //    rt_printf("[J]body[1].Ajipp_qi\n%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n\n",
    //              body[1].Aijpp_qi[0], body[1].Aijpp_qi[1], body[1].Aijpp_qi[2],
    //            body[1].Aijpp_qi[3], body[1].Aijpp_qi[4], body[1].Aijpp_qi[5],
    //            body[1].Aijpp_qi[6], body[1].Aijpp_qi[7], body[1].Aijpp_qi[8]);

    mat(body[1].Ai_Cij, body[1].Aijpp_qi, 3, 3, 3, 3, body[1].Ai_q1);

    mat(body[1].Ai_q1, body[2].Cij_Aijpp, 3, 3, 3, 3, body[2].Ai_q1);
    mat(body[2].Ai_Cij, body[2].Aijpp_qi, 3, 3, 3, 3, body[2].Ai_q2);

    mat(body[2].Ai_q1, body[3].Cij_Aijpp, 3, 3, 3, 3, body[3].Ai_q1);
    mat(body[2].Ai_q2, body[3].Cij_Aijpp, 3, 3, 3, 3, body[3].Ai_q2);
    mat(body[3].Ai_Cij, body[3].Aijpp_qi, 3, 3, 3, 3, body[3].Ai_q3);

    mat(body[3].Ai_q1, body[4].Cij_Aijpp, 3, 3, 3, 3, body[4].Ai_q1);
    mat(body[3].Ai_q2, body[4].Cij_Aijpp, 3, 3, 3, 3, body[4].Ai_q2);
    mat(body[3].Ai_q3, body[4].Cij_Aijpp, 3, 3, 3, 3, body[4].Ai_q3);
    mat(body[4].Ai_Cij, body[4].Aijpp_qi, 3, 3, 3, 3, body[4].Ai_q4);

    mat(body[4].Ai_q1, body[5].Cij_Aijpp, 3, 3, 3, 3, body[5].Ai_q1);
    mat(body[4].Ai_q2, body[5].Cij_Aijpp, 3, 3, 3, 3, body[5].Ai_q2);
    mat(body[4].Ai_q3, body[5].Cij_Aijpp, 3, 3, 3, 3, body[5].Ai_q3);
    mat(body[4].Ai_q4, body[5].Cij_Aijpp, 3, 3, 3, 3, body[5].Ai_q4);
    mat(body[5].Ai_Cij, body[5].Aijpp_qi, 3, 3, 3, 3, body[5].Ai_q5);

    mat(body[5].Ai_q1, body[6].Cij_Aijpp, 3, 3, 3, 3, body[6].Ai_q1);
    mat(body[5].Ai_q2, body[6].Cij_Aijpp, 3, 3, 3, 3, body[6].Ai_q2);
    mat(body[5].Ai_q3, body[6].Cij_Aijpp, 3, 3, 3, 3, body[6].Ai_q3);
    mat(body[5].Ai_q4, body[6].Cij_Aijpp, 3, 3, 3, 3, body[6].Ai_q4);
    mat(body[5].Ai_q5, body[6].Cij_Aijpp, 3, 3, 3, 3, body[6].Ai_q5);
    mat(body[6].Ai_Cij, body[6].Aijpp_qi, 3, 3, 3, 3, body[6].Ai_q6);

    mat(body[1].Ai_q1, body[1].sijp, 3, 3, 3, body[1].sij_q1);
    for(int i = 0; i < 3; i++){
        body[2].ri_q1[i] = body[1].sij_q1[i];
    }

    mat(body[2].Ai_q1, body[2].sijp, 3, 3, 3, body[2].sij_q1);
    mat(body[2].Ai_q2, body[2].sijp, 3, 3, 3, body[2].sij_q2);
    for(int i = 0; i < 3; i++){
        body[3].ri_q1[i] = body[2].ri_q1[i] + body[2].sij_q1[i];
        body[3].ri_q2[i] = body[2].sij_q2[i];
    }

    mat(body[3].Ai_q1, body[3].sijp, 3, 3, 3, body[3].sij_q1);
    mat(body[3].Ai_q2, body[3].sijp, 3, 3, 3, body[3].sij_q2);
    mat(body[3].Ai_q3, body[3].sijp, 3, 3, 3, body[3].sij_q3);
    for(int i = 0; i < 3; i++){
        body[4].ri_q1[i] = body[3].ri_q1[i] + body[3].sij_q1[i];
        body[4].ri_q2[i] = body[3].ri_q2[i] + body[3].sij_q2[i];
        body[4].ri_q3[i] = body[3].sij_q3[i];
    }

    mat(body[4].Ai_q1, body[4].sijp, 3, 3, 3, body[4].sij_q1);
    mat(body[4].Ai_q2, body[4].sijp, 3, 3, 3, body[4].sij_q2);
    mat(body[4].Ai_q3, body[4].sijp, 3, 3, 3, body[4].sij_q3);
    mat(body[4].Ai_q4, body[4].sijp, 3, 3, 3, body[4].sij_q4);
    for(int i = 0; i < 3; i++){
        body[5].ri_q1[i] = body[4].ri_q1[i] + body[4].sij_q1[i];
        body[5].ri_q2[i] = body[4].ri_q2[i] + body[4].sij_q2[i];
        body[5].ri_q3[i] = body[4].ri_q3[i] + body[4].sij_q3[i];
        body[5].ri_q4[i] = body[4].sij_q4[i];
    }

    mat(body[5].Ai_q1, body[5].sijp, 3, 3, 3, body[5].sij_q1);
    mat(body[5].Ai_q2, body[5].sijp, 3, 3, 3, body[5].sij_q2);
    mat(body[5].Ai_q3, body[5].sijp, 3, 3, 3, body[5].sij_q3);
    mat(body[5].Ai_q4, body[5].sijp, 3, 3, 3, body[5].sij_q4);
    mat(body[5].Ai_q5, body[5].sijp, 3, 3, 3, body[5].sij_q5);
    for(int i = 0; i < 3; i++){
        body[6].ri_q1[i] = body[5].ri_q1[i] + body[5].sij_q1[i];
        body[6].ri_q2[i] = body[5].ri_q2[i] + body[5].sij_q2[i];
        body[6].ri_q3[i] = body[5].ri_q3[i] + body[5].sij_q3[i];
        body[6].ri_q4[i] = body[5].ri_q4[i] + body[5].sij_q4[i];
        body[6].ri_q5[i] = body[5].sij_q5[i];
    }

    mat(body[6].Ai_q1, body[6].sijp, 3, 3, 3, body[6].Ai_q1_sijp);
    mat(body[6].Ai_q2, body[6].sijp, 3, 3, 3, body[6].Ai_q2_sijp);
    mat(body[6].Ai_q3, body[6].sijp, 3, 3, 3, body[6].Ai_q3_sijp);
    mat(body[6].Ai_q4, body[6].sijp, 3, 3, 3, body[6].Ai_q4_sijp);
    mat(body[6].Ai_q5, body[6].sijp, 3, 3, 3, body[6].Ai_q5_sijp);
    mat(body[6].Ai_q6, body[6].sijp, 3, 3, 3, body[6].Ai_q6_sijp);

    for(int i = 0; i < 3; i++){
        re_q1[i] = body[6].ri_q1[i] + body[6].Ai_q1_sijp[i];
        re_q2[i] = body[6].ri_q2[i] + body[6].Ai_q2_sijp[i];
        re_q3[i] = body[6].ri_q3[i] + body[6].Ai_q3_sijp[i];
        re_q4[i] = body[6].ri_q4[i] + body[6].Ai_q4_sijp[i];
        re_q5[i] = body[6].ri_q5[i] + body[6].Ai_q5_sijp[i];
        re_q6[i] = body[6].Ai_q6_sijp[i];
    }

    for(int i = 0; i < 3; i++){
        Jv[i*6+0] = re_q1[i];
        Jv[i*6+1] = re_q2[i];
        Jv[i*6+2] = re_q3[i];
        Jv[i*6+3] = re_q4[i];
        Jv[i*6+4] = re_q5[i];
        Jv[i*6+5] = re_q6[i];
    }

    mat(body[6].Ai_q1, body[6].Cij, 3, 3, 3, 3, Ae_q1);
    mat(body[6].Ai_q2, body[6].Cij, 3, 3, 3, 3, Ae_q2);
    mat(body[6].Ai_q3, body[6].Cij, 3, 3, 3, 3, Ae_q3);
    mat(body[6].Ai_q4, body[6].Cij, 3, 3, 3, 3, Ae_q4);
    mat(body[6].Ai_q5, body[6].Cij, 3, 3, 3, 3, Ae_q5);
    mat(body[6].Ai_q6, body[6].Cij, 3, 3, 3, 3, Ae_q6);

    memset(x_qi, 0, sizeof(double)*6);
    memset(y_qi, 0, sizeof(double)*6);
    memset(z_qi, 0, sizeof(double)*6);

    if(body[6].Ae[2*3+0] < 1){
        if(body[6].Ae[2*3+0] > -1){
            // y = asin(-m02)
            y_qi[0] = -(1/sqrt(1 - pow(body[6].Ae[2*3+0], 2)))*Ae_q1[2*3+0];
            y_qi[1] = -(1/sqrt(1 - pow(body[6].Ae[2*3+0], 2)))*Ae_q2[2*3+0];
            y_qi[2] = -(1/sqrt(1 - pow(body[6].Ae[2*3+0], 2)))*Ae_q3[2*3+0];
            y_qi[3] = -(1/sqrt(1 - pow(body[6].Ae[2*3+0], 2)))*Ae_q4[2*3+0];
            y_qi[4] = -(1/sqrt(1 - pow(body[6].Ae[2*3+0], 2)))*Ae_q5[2*3+0];
            y_qi[5] = -(1/sqrt(1 - pow(body[6].Ae[2*3+0], 2)))*Ae_q6[2*3+0];

            // z = atan2(m10, m00)
            z_qi[0] = (1/(1 + pow(body[6].Ae[1*3+0]/body[6].Ae[0*3+0],2)))*(Ae_q1[1*3+0]*body[6].Ae[0*3+0] - body[6].Ae[1*3+0]*Ae_q1[0*3+0])/pow(body[6].Ae[0*3+0],2);
            z_qi[1] = (1/(1 + pow(body[6].Ae[1*3+0]/body[6].Ae[0*3+0],2)))*(Ae_q2[1*3+0]*body[6].Ae[0*3+0] - body[6].Ae[1*3+0]*Ae_q2[0*3+0])/pow(body[6].Ae[0*3+0],2);
            z_qi[2] = (1/(1 + pow(body[6].Ae[1*3+0]/body[6].Ae[0*3+0],2)))*(Ae_q3[1*3+0]*body[6].Ae[0*3+0] - body[6].Ae[1*3+0]*Ae_q3[0*3+0])/pow(body[6].Ae[0*3+0],2);
            z_qi[3] = (1/(1 + pow(body[6].Ae[1*3+0]/body[6].Ae[0*3+0],2)))*(Ae_q4[1*3+0]*body[6].Ae[0*3+0] - body[6].Ae[1*3+0]*Ae_q4[0*3+0])/pow(body[6].Ae[0*3+0],2);
            z_qi[4] = (1/(1 + pow(body[6].Ae[1*3+0]/body[6].Ae[0*3+0],2)))*(Ae_q5[1*3+0]*body[6].Ae[0*3+0] - body[6].Ae[1*3+0]*Ae_q5[0*3+0])/pow(body[6].Ae[0*3+0],2);
            z_qi[5] = (1/(1 + pow(body[6].Ae[1*3+0]/body[6].Ae[0*3+0],2)))*(Ae_q6[1*3+0]*body[6].Ae[0*3+0] - body[6].Ae[1*3+0]*Ae_q6[0*3+0])/pow(body[6].Ae[0*3+0],2);

            // x = atan2(m21, m22)
            x_qi[0] = (1/(1 + pow(body[6].Ae[2*3+1]/body[6].Ae[2*3+2],2)))*(Ae_q1[2*3+1]*body[6].Ae[2*3+2] - body[6].Ae[2*3+1]*Ae_q1[2*3+2])/pow(body[6].Ae[2*3+2],2);
            x_qi[1] = (1/(1 + pow(body[6].Ae[2*3+1]/body[6].Ae[2*3+2],2)))*(Ae_q2[2*3+1]*body[6].Ae[2*3+2] - body[6].Ae[2*3+1]*Ae_q2[2*3+2])/pow(body[6].Ae[2*3+2],2);
            x_qi[2] = (1/(1 + pow(body[6].Ae[2*3+1]/body[6].Ae[2*3+2],2)))*(Ae_q3[2*3+1]*body[6].Ae[2*3+2] - body[6].Ae[2*3+1]*Ae_q3[2*3+2])/pow(body[6].Ae[2*3+2],2);
            x_qi[3] = (1/(1 + pow(body[6].Ae[2*3+1]/body[6].Ae[2*3+2],2)))*(Ae_q4[2*3+1]*body[6].Ae[2*3+2] - body[6].Ae[2*3+1]*Ae_q4[2*3+2])/pow(body[6].Ae[2*3+2],2);
            x_qi[4] = (1/(1 + pow(body[6].Ae[2*3+1]/body[6].Ae[2*3+2],2)))*(Ae_q5[2*3+1]*body[6].Ae[2*3+2] - body[6].Ae[2*3+1]*Ae_q5[2*3+2])/pow(body[6].Ae[2*3+2],2);
            x_qi[5] = (1/(1 + pow(body[6].Ae[2*3+1]/body[6].Ae[2*3+2],2)))*(Ae_q6[2*3+1]*body[6].Ae[2*3+2] - body[6].Ae[2*3+1]*Ae_q6[2*3+2])/pow(body[6].Ae[2*3+2],2);
        }
        else{
            // m20 = -1
            // y = pi/2
            memset(y_qi, 0, sizeof(double)*6);

            // z = -atan2(-m12, m11)
            z_qi[0] = -(1/(1 + pow(body[6].Ae[1*3+2]/body[6].Ae[1*3+1],2)))*(body[6].Ae[1*3+2]*Ae_q1[1*3+1] - Ae_q1[1*3+2]*body[6].Ae[1*3+1])/pow(body[6].Ae[1*3+1],2);
            z_qi[1] = -(1/(1 + pow(body[6].Ae[1*3+2]/body[6].Ae[1*3+1],2)))*(body[6].Ae[1*3+2]*Ae_q2[1*3+1] - Ae_q2[1*3+2]*body[6].Ae[1*3+1])/pow(body[6].Ae[1*3+1],2);
            z_qi[2] = -(1/(1 + pow(body[6].Ae[1*3+2]/body[6].Ae[1*3+1],2)))*(body[6].Ae[1*3+2]*Ae_q3[1*3+1] - Ae_q3[1*3+2]*body[6].Ae[1*3+1])/pow(body[6].Ae[1*3+1],2);
            z_qi[3] = -(1/(1 + pow(body[6].Ae[1*3+2]/body[6].Ae[1*3+1],2)))*(body[6].Ae[1*3+2]*Ae_q4[1*3+1] - Ae_q4[1*3+2]*body[6].Ae[1*3+1])/pow(body[6].Ae[1*3+1],2);
            z_qi[4] = -(1/(1 + pow(body[6].Ae[1*3+2]/body[6].Ae[1*3+1],2)))*(body[6].Ae[1*3+2]*Ae_q5[1*3+1] - Ae_q5[1*3+2]*body[6].Ae[1*3+1])/pow(body[6].Ae[1*3+1],2);
            z_qi[5] = -(1/(1 + pow(body[6].Ae[1*3+2]/body[6].Ae[1*3+1],2)))*(body[6].Ae[1*3+2]*Ae_q6[1*3+1] - Ae_q6[1*3+2]*body[6].Ae[1*3+1])/pow(body[6].Ae[1*3+1],2);

            // x = 0
            memset(x_qi, 0, sizeof(double)*6);
        }
    }
    else{
        // m20 = 1
        // y = -pi/2
        memset(y_qi, 0, sizeof(double)*6);

        // z = atan2(-m12, m11);
        z_qi[0] = (1/(1 + pow(body[6].Ae[1*3+2]/body[6].Ae[1*3+1],2)))*(body[6].Ae[1*3+2]*Ae_q1[1*3+1] - Ae_q1[1*3+2]*body[6].Ae[1*3+1])/pow(body[6].Ae[1*3+1],2);
        z_qi[1] = (1/(1 + pow(body[6].Ae[1*3+2]/body[6].Ae[1*3+1],2)))*(body[6].Ae[1*3+2]*Ae_q2[1*3+1] - Ae_q2[1*3+2]*body[6].Ae[1*3+1])/pow(body[6].Ae[1*3+1],2);
        z_qi[2] = (1/(1 + pow(body[6].Ae[1*3+2]/body[6].Ae[1*3+1],2)))*(body[6].Ae[1*3+2]*Ae_q3[1*3+1] - Ae_q3[1*3+2]*body[6].Ae[1*3+1])/pow(body[6].Ae[1*3+1],2);
        z_qi[3] = (1/(1 + pow(body[6].Ae[1*3+2]/body[6].Ae[1*3+1],2)))*(body[6].Ae[1*3+2]*Ae_q4[1*3+1] - Ae_q4[1*3+2]*body[6].Ae[1*3+1])/pow(body[6].Ae[1*3+1],2);
        z_qi[4] = (1/(1 + pow(body[6].Ae[1*3+2]/body[6].Ae[1*3+1],2)))*(body[6].Ae[1*3+2]*Ae_q5[1*3+1] - Ae_q5[1*3+2]*body[6].Ae[1*3+1])/pow(body[6].Ae[1*3+1],2);
        z_qi[5] = (1/(1 + pow(body[6].Ae[1*3+2]/body[6].Ae[1*3+1],2)))*(body[6].Ae[1*3+2]*Ae_q6[1*3+1] - Ae_q6[1*3+2]*body[6].Ae[1*3+1])/pow(body[6].Ae[1*3+1],2);

        // x = 0
        memset(x_qi, 0, sizeof(double)*3);
    }

    for(int i = 0; i < 6; i++){
        Jw[0*6+i] = x_qi[i];
        Jw[1*6+i] = y_qi[i];
        Jw[2*6+i] = z_qi[i];
    }

//    for (uint indx = 1; indx <= num_body; indx++){
//        for(uint i = 0; i < 3; i++){
//            body[indx].oi[i] = body[num_body].re[i] - body[indx].ri[i];
//        }
//        tilde(body[indx].zi, body[indx].zit);
//        mat(body[indx].zit, body[indx].oi, 3, 3, 3, body[indx].Jvi);
//        memcpy(body[indx].Jwi, body[indx].zi, sizeof(double)*3);

//        for (uint i = 0; i < 3; i++){
//            Jv[i*num_body + (indx - 1)] = body[indx].Jvi[i];
//            Jw[i*num_body + (indx - 1)] = body[indx].Jwi[i];
//        }
//    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 6; j++){
            J[i*6+j] = Jv[i*6+j];
            J[(i+3)*6+j] = Jw[i*6+j];
        }
    }


    //    Body *body0, *body1;
    //    Body *body_end = &body[num_body];

    //    for (uint indx = 1; indx <= num_body; indx++) {
    //        body0 = &body[indx - 1];
    //        body1 = &body[indx];
    //        mat(body0->Cij, body1->Aijpp, 3, 3, 3, 3, body0->Cij_Aijpp);

    //        double *Aijpp_qi_ptr = body1->Aijpp_qi;
    //        *(Aijpp_qi_ptr++) = -sin(body1->qi);	*(Aijpp_qi_ptr++) = -cos(body1->qi);	*(Aijpp_qi_ptr++) = 0;
    //        *(Aijpp_qi_ptr++) =  cos(body1->qi);	*(Aijpp_qi_ptr++) = -sin(body1->qi);	*(Aijpp_qi_ptr++) = 0;
    //        *(Aijpp_qi_ptr++) = 0;                  *(Aijpp_qi_ptr++) = 0;                  *(Aijpp_qi_ptr) = 0;

    //        mat(body1->Ai_Cij, body1->Aijpp_qi, 3, 3, 3, 3, body1->Ai_Cij_Aijpp_qi);
    //    }

    //    double temp1[9] = { 0, }, temp2[3] = {0,};
    //    for (uint indx = 1; indx <= num_body; indx++) {
    //        body0 = &body[indx - 1];
    //        body1 = &body[indx];
    //        memset(body1->A6_qi, 0, sizeof(double) * 9);
    //        memset(body1->r6_qi, 0, sizeof(double) * 3);
    //        for (uint indx2 = indx; indx2 <= num_body; indx2++) {
    //            if (indx2 == indx) {
    //                for(uint i = 0; i < 9;i++){
    //                    body1->A6_qi[i] += body1->Ai_Cij_Aijpp_qi[i];
    //                }
    //            }
    //            else {
    //                mat(body1->A6_qi, body[indx2 - 1].Cij_Aijpp, 3, 3, 3, 3, temp1);
    //                memcpy(body1->A6_qi, temp1, sizeof(double) * 9);
    //            }
    //            if (indx2 < num_body) {
    //                mat(body1->A6_qi, body[indx2].sijp, 3, 3, 3, temp2);
    //                for(uint i = 0; i < 3; i++){
    //                    body1->r6_qi[i] += temp2[i];
    //                }
    //            }
    //        }

    //        mat(body1->A6_qi, body_end->Cij, 3, 3, 3, 3, body1->Ae_qi);
    //        mat(body1->A6_qi, body_end->sijp, 3, 3, 3, body1->re_qi);
    //        for (uint i = 0; i < 3; i++){
    //            body1->re_qi[i] += body1->r6_qi[i];
    //        }

    //        for (uint i = 0; i < 3; i++){
    //            Jv[i*num_body + indx - 1] = body1->re_qi[i];
    //        }
    //    }

    //    m00 = body_end->Ae[0];
    //    m02 = body_end->Ae[2];
    //    m10 = body_end->Ae[3];
    //    m11 = body_end->Ae[4];
    //    m12 = body_end->Ae[5];
    //    m20 = body_end->Ae[6];
    //    m21 = body_end->Ae[7];
    //    m22 = body_end->Ae[8];

    //    memset(x_qi, 0, sizeof(double)*6);
    //    memset(y_qi, 0, sizeof(double)*6);
    //    memset(z_qi, 0, sizeof(double)*6);

    //    if(m20 < 1){
    //        if(m20 > -1){
    //            // y = asin(-m02);
    //            y_qi[0] = -(1/sqrt(1 - pow(body_end->Ae[0*3+2],2)))*body[1].Ae_qi[0*3+2];
    //            y_qi[1] = -(1/sqrt(1 - pow(body_end->Ae[0*3+2],2)))*body[2].Ae_qi[0*3+2];
    //            y_qi[2] = -(1/sqrt(1 - pow(body_end->Ae[0*3+2],2)))*body[3].Ae_qi[0*3+2];
    //            y_qi[3] = -(1/sqrt(1 - pow(body_end->Ae[0*3+2],2)))*body[4].Ae_qi[0*3+2];
    //            y_qi[4] = -(1/sqrt(1 - pow(body_end->Ae[0*3+2],2)))*body[5].Ae_qi[0*3+2];
    //            y_qi[5] = -(1/sqrt(1 - pow(body_end->Ae[0*3+2],2)))*body[6].Ae_qi[0*3+2];

    //            // z = atan2(m10, m00);
    //            z_qi[0] = (1/(1 + pow(body_end->Ae[1*3+0]/body_end->Ae[0*3+0],2)))*(body[1].Ae_qi[1*3+0]*body_end->Ae[0*3+0] - body_end->Ae[1*3+0]*body[1].Ae_qi[0*3+0])/pow(body_end->Ae[0*3+0],2);
    //            z_qi[1] = (1/(1 + pow(body_end->Ae[1*3+0]/body_end->Ae[0*3+0],2)))*(body[2].Ae_qi[1*3+0]*body_end->Ae[0*3+0] - body_end->Ae[1*3+0]*body[2].Ae_qi[0*3+0])/pow(body_end->Ae[0*3+0],2);
    //            z_qi[2] = (1/(1 + pow(body_end->Ae[1*3+0]/body_end->Ae[0*3+0],2)))*(body[3].Ae_qi[1*3+0]*body_end->Ae[0*3+0] - body_end->Ae[1*3+0]*body[3].Ae_qi[0*3+0])/pow(body_end->Ae[0*3+0],2);
    //            z_qi[3] = (1/(1 + pow(body_end->Ae[1*3+0]/body_end->Ae[0*3+0],2)))*(body[4].Ae_qi[1*3+0]*body_end->Ae[0*3+0] - body_end->Ae[1*3+0]*body[4].Ae_qi[0*3+0])/pow(body_end->Ae[0*3+0],2);
    //            z_qi[4] = (1/(1 + pow(body_end->Ae[1*3+0]/body_end->Ae[0*3+0],2)))*(body[5].Ae_qi[1*3+0]*body_end->Ae[0*3+0] - body_end->Ae[1*3+0]*body[5].Ae_qi[0*3+0])/pow(body_end->Ae[0*3+0],2);
    //            z_qi[5] = (1/(1 + pow(body_end->Ae[1*3+0]/body_end->Ae[0*3+0],2)))*(body[6].Ae_qi[1*3+0]*body_end->Ae[0*3+0] - body_end->Ae[1*3+0]*body[6].Ae_qi[0*3+0])/pow(body_end->Ae[0*3+0],2);

    //            // x = atan2(m21, m22);
    //            x_qi[0] = (1/(1 + pow(body_end->Ae[2*3+1]/body_end->Ae[2*3+2],2)))*(body[1].Ae_qi[2*3+1]*body_end->Ae[2*3+2] - body_end->Ae[2*3+1]*body[1].Ae_qi[2*3+2])/pow(body_end->Ae[2*3+2],2);
    //            x_qi[1] = (1/(1 + pow(body_end->Ae[2*3+1]/body_end->Ae[2*3+2],2)))*(body[2].Ae_qi[2*3+1]*body_end->Ae[2*3+2] - body_end->Ae[2*3+1]*body[2].Ae_qi[2*3+2])/pow(body_end->Ae[2*3+2],2);
    //            x_qi[2] = (1/(1 + pow(body_end->Ae[2*3+1]/body_end->Ae[2*3+2],2)))*(body[3].Ae_qi[2*3+1]*body_end->Ae[2*3+2] - body_end->Ae[2*3+1]*body[3].Ae_qi[2*3+2])/pow(body_end->Ae[2*3+2],2);
    //            x_qi[3] = (1/(1 + pow(body_end->Ae[2*3+1]/body_end->Ae[2*3+2],2)))*(body[4].Ae_qi[2*3+1]*body_end->Ae[2*3+2] - body_end->Ae[2*3+1]*body[4].Ae_qi[2*3+2])/pow(body_end->Ae[2*3+2],2);
    //            x_qi[4] = (1/(1 + pow(body_end->Ae[2*3+1]/body_end->Ae[2*3+2],2)))*(body[5].Ae_qi[2*3+1]*body_end->Ae[2*3+2] - body_end->Ae[2*3+1]*body[5].Ae_qi[2*3+2])/pow(body_end->Ae[2*3+2],2);
    //            x_qi[5] = (1/(1 + pow(body_end->Ae[2*3+1]/body_end->Ae[2*3+2],2)))*(body[6].Ae_qi[2*3+1]*body_end->Ae[2*3+2] - body_end->Ae[2*3+1]*body[6].Ae_qi[2*3+2])/pow(body_end->Ae[2*3+2],2);
    //        }
    //        else{
    //            // m20 = -1
    //             printf("Not a unique solution : thetaX - thetaZ = atan2(-m12, m11)\n");

    //            // y = M_PI_2;
    //            memset(y_qi, 0, sizeof(double)*6);

    //            // z = -atan2(-m12, m11);
    //            z_qi[0] = -(1/(1 + pow(body_end->Ae[1*3+2]/body_end->Ae[1*3+1], 2)))*(body_end->Ae[1*3+2]*body[1].Ae_qi[1*3+1] - body[1].Ae_qi[1*3+2]*body_end->Ae[1*3+1])/pow(body_end->Ae[1*3+1],2);
    //            z_qi[1] = -(1/(1 + pow(body_end->Ae[1*3+2]/body_end->Ae[1*3+1], 2)))*(body_end->Ae[1*3+2]*body[2].Ae_qi[1*3+1] - body[2].Ae_qi[1*3+2]*body_end->Ae[1*3+1])/pow(body_end->Ae[1*3+1],2);
    //            z_qi[2] = -(1/(1 + pow(body_end->Ae[1*3+2]/body_end->Ae[1*3+1], 2)))*(body_end->Ae[1*3+2]*body[3].Ae_qi[1*3+1] - body[3].Ae_qi[1*3+2]*body_end->Ae[1*3+1])/pow(body_end->Ae[1*3+1],2);
    //            z_qi[3] = -(1/(1 + pow(body_end->Ae[1*3+2]/body_end->Ae[1*3+1], 2)))*(body_end->Ae[1*3+2]*body[4].Ae_qi[1*3+1] - body[4].Ae_qi[1*3+2]*body_end->Ae[1*3+1])/pow(body_end->Ae[1*3+1],2);
    //            z_qi[4] = -(1/(1 + pow(body_end->Ae[1*3+2]/body_end->Ae[1*3+1], 2)))*(body_end->Ae[1*3+2]*body[5].Ae_qi[1*3+1] - body[5].Ae_qi[1*3+2]*body_end->Ae[1*3+1])/pow(body_end->Ae[1*3+1],2);
    //            z_qi[5] = -(1/(1 + pow(body_end->Ae[1*3+2]/body_end->Ae[1*3+1], 2)))*(body_end->Ae[1*3+2]*body[6].Ae_qi[1*3+1] - body[6].Ae_qi[1*3+2]*body_end->Ae[1*3+1])/pow(body_end->Ae[1*3+1],2);

    //            // x = 0;
    //            memset(x_qi, 0, sizeof(double)*6);
    //        }
    //    }
    //    else{
    //        // m20 = 1;
    //         printf("Not a unique solution : thetaX + thetaZ = atan2(-m12, m11)\n");

    //        // y = -M_PI_2;
    //        memset(y_qi, 0, sizeof(double)*6);

    //        // z = atan2(-m12, m11);
    //        z_qi[0] = (1/(1 + pow(body_end->Ae[1*3+2]/body_end->Ae[1*3+1], 2)))*(body_end->Ae[1*3+2]*body[1].Ae_qi[1*3+1] - body[1].Ae_qi[1*3+2]*body_end->Ae[1*3+1])/pow(body_end->Ae[1*3+1],2);
    //        z_qi[1] = (1/(1 + pow(body_end->Ae[1*3+2]/body_end->Ae[1*3+1], 2)))*(body_end->Ae[1*3+2]*body[2].Ae_qi[1*3+1] - body[2].Ae_qi[1*3+2]*body_end->Ae[1*3+1])/pow(body_end->Ae[1*3+1],2);
    //        z_qi[2] = (1/(1 + pow(body_end->Ae[1*3+2]/body_end->Ae[1*3+1], 2)))*(body_end->Ae[1*3+2]*body[3].Ae_qi[1*3+1] - body[3].Ae_qi[1*3+2]*body_end->Ae[1*3+1])/pow(body_end->Ae[1*3+1],2);
    //        z_qi[3] = (1/(1 + pow(body_end->Ae[1*3+2]/body_end->Ae[1*3+1], 2)))*(body_end->Ae[1*3+2]*body[4].Ae_qi[1*3+1] - body[4].Ae_qi[1*3+2]*body_end->Ae[1*3+1])/pow(body_end->Ae[1*3+1],2);
    //        z_qi[4] = (1/(1 + pow(body_end->Ae[1*3+2]/body_end->Ae[1*3+1], 2)))*(body_end->Ae[1*3+2]*body[5].Ae_qi[1*3+1] - body[5].Ae_qi[1*3+2]*body_end->Ae[1*3+1])/pow(body_end->Ae[1*3+1],2);
    //        z_qi[5] = (1/(1 + pow(body_end->Ae[1*3+2]/body_end->Ae[1*3+1], 2)))*(body_end->Ae[1*3+2]*body[6].Ae_qi[1*3+1] - body[6].Ae_qi[1*3+2]*body_end->Ae[1*3+1])/pow(body_end->Ae[1*3+1],2);

    //        // x = 0;
    //        memset(x_qi, 0, sizeof(double)*6);
    //    }

    //    Jw[0*6+0] = x_qi[0];
    //    Jw[0*6+1] = x_qi[1];
    //    Jw[0*6+2] = x_qi[2];
    //    Jw[0*6+3] = x_qi[3];
    //    Jw[0*6+4] = x_qi[4];
    //    Jw[0*6+5] = x_qi[5];

    //    Jw[1*6+0] = y_qi[0];
    //    Jw[1*6+1] = y_qi[1];
    //    Jw[1*6+2] = y_qi[2];
    //    Jw[1*6+3] = y_qi[3];
    //    Jw[1*6+4] = y_qi[4];
    //    Jw[1*6+5] = y_qi[5];

    //    Jw[2*6+0] = z_qi[0];
    //    Jw[2*6+1] = z_qi[1];
    //    Jw[2*6+2] = z_qi[2];
    //    Jw[2*6+3] = z_qi[3];
    //    Jw[2*6+4] = z_qi[4];
    //    Jw[2*6+5] = z_qi[5];

    //    memcpy(J, Jv, sizeof(double) * 3 * num_body);
    //    memcpy(J + 3 * num_body, Jw, sizeof(double) * 3 * num_body);

    //    delete[] Jv;
    //    delete[] Jw;
}

void RobotArm::dynamics()
{
//    Body *body0, *body1, *body2;
    for(uint indx = 1; indx <= num_body; indx++){
//        body1 = &body[indx];
//        body0 = &body[indx - 1];
        // velocity state
        mat(body[indx].Ai_Cij, body[indx].u_vec, 3, 3, 3, body[indx].Hi);
        tilde(body[indx].ri, body[indx].rit);
        mat(body[indx].rit, body[indx].Hi, 3, 3, 3, body[indx].Bi);
        memcpy(body[indx].Bi + 3, body[indx].Hi, sizeof(double)*3);
        for (uint i = 0; i < 6; i++){
            body[indx].Yih[i] = body[indx - 1].Yih[i] + body[indx].Bi[i]*body[indx].qi_dot;
        }

        // cartesian velocity
        for (uint i = 0; i < 3; i++){
            for(uint j = 0; j < 3; j++){
                body[indx].Ti[i*6 + j] = i == j ? 1 : 0;
                body[indx].Ti[(i + 3)*6 + (j + 3)] = i == j ? 1 : 0;
                body[indx].Ti[(i + 3)*6 + j] = 0;
                body[indx].Ti[i*6 + (j + 3)] = -body[indx].rit[i*3 + j];
            }
        }

        mat(body[indx].Ti, body[indx].Yih, 6, 6, 6, body[indx].Yib);
        memcpy(body[indx].ri_dot, body[indx].Yib, sizeof(double)*3);
        memcpy(body[indx].wi, body[indx].Yib + 3, sizeof(double)*3);
        tilde(body[indx].wi, body[indx].wit);
        mat(body[indx].Ai, body[indx].rhoip, 3, 3, 3, body[indx].rhoi);
        for(uint i = 0; i < 3; i++){
            body[indx].ric[i] = body[indx].ri[i] + body[indx].rhoi[i];
        }
        mat(body[indx].wit, body[indx].rhoi, 3, 3, 3, body[indx].ric_dot);
        for (uint i = 0; i < 3; i++){
            body[indx].ric_dot[i] += body[indx].ri_dot[i];
        }

        // mass & force
        mat(body[indx].Ai, body[indx].Cii, 3, 3, 3, 3, body[indx].Ai_Cii);
        double temp[9] = {0,}, temp2 = 0;
        mat(body[indx].Ai_Cii, body[indx].Jip, 3, 3, 3, 3, temp);
        for(uint i = 0; i < 3; i++){
            for(uint j = 0; j < 3; j++){
                temp2 = 0;
                for(uint k = 0; k < 3; k++){
                    temp2 += temp[i*3 + k]*body[indx].Ai_Cii[j*3 + k];
                }
                body[indx].Jic[i*3 + j] = temp2;
            }
        }
        tilde(body[indx].ri_dot, body[indx].rit_dot);
        tilde(body[indx].ric_dot, body[indx].rict_dot);
        tilde(body[indx].ric, body[indx].rict);
//        double temp3[9] = {0,};
        mat(body[indx].rict, body[indx].rict, 3, 3, 3, 3, temp3);
        for(uint i = 0; i < 3; i++){
            for(uint j = 0; j < 3; j++){
                body[indx].Mih[i*6 + j] = i == j ? body[indx].mi : 0;
                body[indx].Mih[(i + 3)*6 + j] = body[indx].mi*body[indx].rict[i*3 + j];
                body[indx].Mih[i*6 + (j + 3)] = -body[indx].mi*body[indx].rict[i*3 + j];
                body[indx].Mih[(i + 3)*6 + (j + 3)] = body[indx].Jic[i*3 + j] - body[indx].mi*temp3[i*3 + j];
            }
        }
        body[indx].fic[0] = 0;
        body[indx].fic[1] = 0;
        body[indx].fic[2] = body[indx].mi*g;
        body[indx].tic[0] = 0;
        body[indx].tic[1] = 0;
        body[indx].tic[2] = 0;
//        double rict_dot_wi[3] = {0,};
        mat(body[indx].rict_dot, body[indx].wi, 3, 3, 3, body[indx].rict_dot_wi);
        for (uint i = 0; i < 3; i++){
            body[indx].Qih_g[i] = body[indx].fic[i];
            body[indx].Qih_c[i] = body[indx].mi*body[indx].rict_dot_wi[i];
            body[indx].Qih[i] = body[indx].Qih_g[i] + body[indx].Qih_c[i];
        }
//        double rict_fic[3] = {0,}, rict_rict_dot_wi[3] = {0,}, Jic_wi[3] = {0,}, wit_Jic_wi[3] = {0,};
        mat(body[indx].rict, body[indx].fic, 3, 3, 3, body[indx].rict_fic);
        mat(body[indx].rict, body[indx].rict_dot_wi, 3, 3, 3, body[indx].rict_rict_dot_wi);
        mat(body[indx].Jic, body[indx].wi, 3, 3, 3, body[indx].Jic_wi);
        mat(body[indx].wit, body[indx].Jic_wi, 3, 3, 3, body[indx].wit_Jic_wi);
        for (uint i = 0; i < 3; i++){
            body[indx].Qih_g[i + 3] = body[indx].rict_fic[i];
            body[indx].Qih_c[i + 3] = body[indx].mi*body[indx].rict_rict_dot_wi[i] - body[indx].wit_Jic_wi[i];
            body[indx].Qih[i + 3] = body[indx].tic[i] + body[indx].Qih_g[i + 3] + body[indx].Qih_c[i + 3];
        }

        // velocity coupling
        mat(body[indx - 1].wit, body[indx].Hi, 3, 3, 3, body[indx].Hi_dot);
//        double rit_dot_Hi[3] = {0,}, rit_Hi_dot[3] = {0,};
        mat(body[indx].rit_dot, body[indx].Hi, 3, 3, 3, body[indx].rit_dot_Hi);
        mat(body[indx].rit, body[indx].Hi_dot, 3, 3, 3, body[indx].rit_Hi_dot);
        for(uint i = 0; i < 3; i++){
            body[indx].Di[i] = (body[indx].rit_dot_Hi[i] + body[indx].rit_Hi_dot[i])*body[indx].qi_dot;
            body[indx].Di[i+3] = body[indx].Hi_dot[i]*body[indx].qi_dot;
        }

        memcpy(body[indx].Di_sum, body[indx].Di, sizeof(double)*6);
        for(uint indx2 = indx - 1; indx2 >= 1; indx2--){
            for(uint i = 0; i < 6; i++){
                body[indx].Di_sum[i] += body[indx2].Di[i];
            }
        }
    }

    // system EQM
    for(uint indx = num_body; indx >= 1;  indx--){
//        body1 = &body[i];
        if (indx == num_body){
            memcpy(body[indx].Ki, body[indx].Mih, sizeof(double)*36);
            memcpy(body[indx].Li, body[indx].Qih, sizeof(double)*6);
            memcpy(body[indx].Li_g, body[indx].Qih_g, sizeof(double)*6);
            memcpy(body[indx].Li_c, body[indx].Qih_c, sizeof(double)*6);
        }
        else{
//            body2 = &body[i + 1];
            for(uint i = 0; i < 36; i++){
                body[indx].Ki[i] = body[indx].Mih[i] + body[indx + 1].Ki[i];
            }
            mat(body[indx + 1].Ki, body[indx + 1].Di, 6, 6, 6, body[indx + 1].Ki_Di);
            for(uint i = 0; i < 6; i++){
                body[indx].Li[i] = body[indx].Qih[i] + body[indx + 1].Li[i] - body[indx + 1].Ki_Di[i];
                body[indx].Li_g[i] = body[indx].Qih_g[i] + body[indx + 1].Li_g[i] - body[indx + 1].Ki_Di[i];
                body[indx].Li_c[i] = body[indx].Qih_c[i] + body[indx + 1].Li_c[i] - body[indx + 1].Ki_Di[i];
            }
        }
    }

    memset(Q, 0, sizeof(double)*num_body);
    memset(Q_g, 0, sizeof(float)*num_body);
    memset(Q_c, 0, sizeof(float)*num_body);
    for(uint indx = 1; indx <= num_body; indx++){
//        body1 = &body[indx];
        mat(body[indx].Ki, body[indx].Di_sum, 6, 6, 6, body[indx].Ki_Di_sum);
        for(uint i = 0; i < 6; i++){
            Q[indx - 1] += body[indx].Bi[i]*(body[indx].Li[i] - body[indx].Ki_Di_sum[i]);
            Q_g[indx - 1] += body[indx].Bi[i]*(body[indx].Li_g[i] - body[indx].Ki_Di_sum[i]);
            Q_c[indx - 1] += body[indx].Bi[i]*(body[indx].Li_c[i] - body[indx].Ki_Di_sum[i]);
        }
    }

    memset(M, 0, sizeof(double)*num_body*num_body);
    for(uint indx = 1; indx <= num_body; indx++){
        for(uint indx2 = 1; indx2 <= num_body; indx2++){
            if (indx == indx2){
                mat(body[indx].Ki, body[indx].Bi, 6, 6, 6, body[indx].Ki_Bi);
            }
            else if(indx < indx2){
                mat(body[indx2].Ki, body[indx2].Bi, 6, 6, 6, body[indx].Ki_Bi);
            }
            else if(indx > indx2){
                mat(body[indx].Ki, body[indx2].Bi, 6, 6, 6, body[indx].Ki_Bi);
            }
            body[indx].temp_Bi_Ki_Bi = 0;
            for(uint i = 0; i < 6; i++){
                body[indx].temp_Bi_Ki_Bi += body[indx].Bi[i]*body[indx].Ki_Bi[i];
            }
            M[(indx - 1)*num_body + (indx2 - 1)] = body[indx].temp_Bi_Ki_Bi;
        }
    }

#if 1
    indx_array = new int[num_body];
    fac_mat = new double[num_body*num_body];
    qddot = new double[num_body];

    numeric->ludcmp(M, static_cast<int>(num_body), indx_array, 0.0, fac_mat);
    numeric->lubksb(fac_mat, static_cast<int>(num_body), indx_array, Q, qddot);

    for(uint indx = 1; indx <= num_body; indx++){
        body[indx].qi_ddot = qddot[indx - 1];
    }

    delete[] indx_array;
    delete[] fac_mat;
    delete[] qddot;
#endif
}

void RobotArm::save_data() {
    fprintf(fp, "%.7f\t", t_current);
    for (uint i = 1; i <= num_body; i++) {
        fprintf(fp, "%.7f\t", body[i].qi);
    }

    kinematics();

    fprintf(fp, "%.7f\t%.7f\t%.7f\t", body[num_body].re[0], body[num_body].re[1], body[num_body].re[2]);
    fprintf(fp, "%.7f\t%.7f\t%.7f\t", body[num_body].ori[0], body[num_body].ori[1], body[num_body].ori[2]);

    for (uint i = 1; i <= num_body; i++) {
        fprintf(fp, "%.7f\t", body[i].qi_dot);
    }

    for (uint i = 1; i <= num_body; i++) {
        fprintf(fp, "%.7f\t", body[i].qi_ddot);
    }
    fprintf(fp, "\n");
}

void RobotArm::residual(){
    for(uint indx = 1; indx <= num_body; indx++){
        body[indx].alpha = Q_g[indx] - Q_c[indx];
        body[indx].yp = body[indx].Ta + body[indx].alpha - body[indx].r_hat;

        body[indx].y = body[indx].y_old + body[indx].yp*h + 0.5*h*h*(body[indx].yp - body[indx].yp_old);
    }

    for(uint indx = 1; indx <= num_body; indx++){
        body[indx].p_linear = 0;
        for (uint j = 0; j < 3; j++) {
            body[indx].p_linear += powf(body[indx].ric_dot[j], 2);
        }
        body[indx].p_linear *= 0.5*body[indx].mi;
        body[indx].p_rotate = 0;
        for (uint j = 0; j < 3; j++) {
            body[indx].p_rotate += body[indx].wi[j] * body[indx].Jic_wi[j];
        }
        body[indx].p_rotate *= 0.5;
        body[indx].p = body[indx].p_linear + body[indx].p_rotate;
    }

    for(uint indx = 1; indx <= num_body; indx++){
        body[indx].r_hat = body[indx].K*(body[indx].y - body[indx].p);

        body[indx].y_old = body[indx].y;
        body[indx].yp_old = body[indx].yp;
    }

    t_current += h;
}

void RobotArm::high_pass_filter(double cur, double *timeZone, double *cur_filter, double ts, double f_cut){
    w_cut = 2*M_PI*f_cut;
    tau = 1 / w_cut;
    tau_ts = 1/(tau + ts);
    a1 = -tau*tau_ts;
    b0 = tau*tau_ts;
    b1 = -tau*tau_ts;
    a2 = 0;
    b2 = 0;
    sum0 = 0;

    sum0 = -a1*timeZone[1] - a2*timeZone[0];
    timeZone[2] = cur + sum0;
    *cur_filter = b0*timeZone[2] + b1*timeZone[1] + b2*timeZone[0];

    timeZone[0] = timeZone[1];
    timeZone[1] = timeZone[2];
}
