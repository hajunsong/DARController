#ifndef DATACONTROL_H
#define DATACONTROL_H

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <vector>

#include <native/timer.h>

#include "fileio.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
using namespace rapidjson;

#define PRINT_ON 0

const uint8_t SOCKET_TOKEN_SIZE = 2;

const uint8_t NUM_JOINT = 7;
const uint8_t NUM_DOF = 6;
// const uint8_t MODULE_TYPE = 2; // 1: FAR v1, 2: FAR v2, 3: FAR v3
const uint8_t DATA_INDEX_LEN = 1;
const uint8_t MOTION_DATA_LEN = 8;
const uint8_t WORD_DATA_LEN = 4;
const uint8_t TIME_LEN = 8;

const uint8_t OP_MODE_LEN = 1;
const uint8_t SUB_MODE_LEN = 1;
const uint8_t SECTION_LEN = 1;
const uint8_t DESIRED_JOINT_LEN = 8;
const uint8_t DESIRED_CARTESIAN_LEN = 8;

const uint8_t CMD_TYPE_LEN = 1;
const uint8_t CYCLE_COUNT_LEN = 1;
const uint8_t ROW_SIZE_LEN = 1;
const uint8_t COL_SIZE_LEN = 1;
const uint8_t MOVE_TIME_LEN = 8;
const uint8_t ACC_TIME_LEN = 8;
const uint8_t PATH_DATA_LEN = 8;

const uint8_t MASS_LEN = 8;
const uint8_t TORQUE_CONST_LEN = 8;

const char KEY_Q = 113;
const char KEY_Z = 122;
const char KEY_X = 120;
const char KEY_C = 99;
const char KEY_V = 118;
const char KEY_A = 97;
const char KEY_S = 115;
const char KEY_D = 100;
const char KEY_F = 102;
const char KEY_W = 119;
const char KEY_R = 114;
const char KEY_ESC = 27;

const double ENC2DEG = 0.021972656;
const double DEG2ENC = 45.5111;
const double DEG2RAD = 0.017453293; // 3.14159265358979323846/180.0;
const double RAD2DEG = 57.295779513; // 180.0/3.14159265358979323846;
const double ENC2RPM = 	0.229;
const double RPM2DEG = 6;
//const double RAW2mA = 2.69;
//const double mA2RAW = 0.371747212;
const double RAW2mA[7] = {4, 4, 4, 3, 3, 3, 3};
const double mA2RAW[7] = {0.25, 0.25, 0.25, 0.333, 0.333, 0.333, 0.333};

const double initJoint1Deg = 40;
const double initJoint3Deg = -175;
const double finishJointDeg[7] = {0, 9.93, -149.94, -129.98, 0, 0, 0};

//const double operateFeedingReadyPose[6] = {-0.222, 0.058758770483143,  0.0989879236551952, -1.57080736369899,  1.48352986419518, 1.5708665530537};
const double feedingOrientation[3] = {1.5707963267949, 0, -1.5707963267949}; // 90 0 -90
//const double feedingSection1TopPosition[3] = {-0.299594, 0.184727, -0.039 + 0.17};
const double calOrientation[3] = {1.57079632584418, -0.272542874123738, -1.56458522472617};

const double waitJointDeg1[7] = {0, 23.73, -91.582, -68.752, 0, 0, 0};
const double waitJointDeg2[7] = {0, 5.625, -147.173, -142.163, 0, 0, 0};

class DataControl
{
public:
    typedef struct{
        char opMode, subMode;
        double desiredJoint[NUM_JOINT], desiredPose[NUM_DOF];
        double move_time, acc_time;
    }StructClientToServer;

    typedef struct{
        char data_index;
        double presentJointPosition[NUM_JOINT], presentCartesianPose[NUM_DOF];
        double desiredJointPosition[NUM_JOINT], desiredCartesianPose[NUM_DOF];
        double presentJointVelocity[NUM_JOINT], presentJointCurrent[NUM_JOINT];
        double presentCartesianVelocity[NUM_DOF];
        double time, module_time, ik_time;
        double presentJointTorque[NUM_JOINT], presentJointResidual[NUM_JOINT];
        char opMode;
        // raw data
        int indx, cur_status_word[NUM_JOINT];
        long tar_position[NUM_JOINT], cur_position[NUM_JOINT];
        long /*tar_velocity[NUM_JOINT],*/ cur_velocity[NUM_JOINT];
        int tar_torque[NUM_JOINT], cur_torque[NUM_JOINT];
    }StructServerToClient;

    typedef struct{
        double t;
        double present_end_pose[NUM_DOF];
        double desired_end_pose[NUM_DOF];
        double present_end_pose_calc[NUM_JOINT];
        double present_end_vel[NUM_DOF];
        double present_q[NUM_JOINT];
        double present_q_dot[NUM_JOINT];
        double desired_q[NUM_JOINT];
        double desired_beta, desired_d7, desired_q7, desired_q6;

        int indx, cur_status_word[NUM_JOINT];
        long tar_position[NUM_JOINT], cur_position[NUM_JOINT];
        long tar_position_prev[NUM_JOINT];
        long cur_velocity[NUM_JOINT];
        int tar_torque[NUM_JOINT], cur_torque[NUM_JOINT];
        int tar_control_word[NUM_JOINT], cur_modes_of_operation[NUM_JOINT];
        uint32_t err_state[NUM_JOINT];
        int data_index;
        char op_mode;

        unsigned long time1, time2, module_time1, module_time2, ik_time1, ik_time2;

        double Td[NUM_JOINT], Tg[NUM_JOINT], T[NUM_JOINT], T_limit[NUM_JOINT];
        bool T_err;
        double Kp, Dp, Kr, Dr;
        double F[6];
        double present_joint_torque[NUM_JOINT];
        double present_joint_residual[NUM_JOINT];
        double residual_limit_p[NUM_JOINT], residual_limit_n[NUM_JOINT];
        bool module_init;
        char joint_op_mode;
        char run_mode;
        double desiredForce;
    }StructRobotData;

    typedef struct{
        std::vector<double> path_x, path_y, path_z, path_theta, path_beta, path_d7, path_q6, path_q7;
        double r[3], R_init[9];
        unsigned int data_size;
    }StructPathGenerateData;

    typedef struct{
        unsigned char cmd_type, cycle_count;
        signed char cycle_count_max;
        uint8_t row, col;
        std::vector<double> total_time, point_theta, acc_time;
        std::vector<double> point_px, point_py, point_pz, point_rx, point_ry, point_rz;
        std::vector<double> point_px_home, point_py_home, point_pz_home, point_rx_home, point_ry_home, point_rz_home;
        std::vector<int> point_option;
        std::vector<double> point_q6, point_q7;
        std::vector<double> comm_data;
        double teaching_pose[NUM_DOF];
        double teaching_d7, teaching_beta;
        std::vector<StructPathGenerateData> movePath;
        StructPathGenerateData readyPath;
        uint path_data_indx;
        uint8_t path_struct_indx;
        double offset[2];
    }StructPathData;

    typedef struct{
        int mode;
        unsigned int section;
    }StructOperateMode;

    typedef struct{
        char interface_cmd, interface_sub;
        signed char food_pixel[20];
        int food_pos[10];
        bool tablet_connect, tablet_check, tablet_check_old;
        bool camera_request, camera_request_old;
        double sp_food[3];
        unsigned char dining_speed, dining_time;
        unsigned char dining_tool;
        bool dining_interrupt;
        bool receive_point;
        bool wait_feeding;
    }StructKITECHData;

    typedef struct{
        unsigned int section1, section2, section3, section4, section5;
        unsigned int section1_cnt, section2_cnt, section3_cnt, section4_cnt, section5_cnt;
    }StructTrayInfor;

    typedef struct{
        vector<double> wp;
        unsigned int size[2];
    }waypoints;

    typedef struct{
        bool dining_time_send, dining_time_send_old;
        char dining_time_infor[12];
        bool dining_time_check_start;
        bool dining_start;
        int robot_id;
        int dining_step;
    }StructDiningInfor;

    enum OpMode{ServoOnOff = 0, Initialize, Wait, JointMove, CartesianMove, PathGenerateMode, ReadyMode, RunMode, TorqueID, OperateMode, ObiMode, TestMode};
    enum Motion{JogMotion = 0, JointMotion, CartesianJogMotion, CartesianMotion};
    enum Module{FAR_V1=1, FAR_V2, FAR_V3, FAR_V4, FAR_V5};
    enum Comm{RS485=1, RS232, EtherCAT};
    enum CmdType{PathCmd=1, ReadyCmd, RunCmd, StopCmd, FileReady, FileRun, CustomRun, CalMode, PathMode, ForceMode};
    enum Operate{Start=1, Stop, StartTeaching, StopTeaching, StartFeeding, StopFeeding, Feeding, FeedingSwitch, ReadyFeeding, WaitFeeding1, WaitFeeding2};
    enum Section{Side1=1, Side2, Side3, Soup, Rice, Mouse, Home, Test};

    bool cartesian_goal_reach;
    bool joint_goal_reach;
    bool config_check;
    bool feeding;
    bool tablet_mode;
    unsigned char section_indx, select_indx;
    bool time_check_start;
    bool joystick_mode;
    bool collision_detect_enable, safe_mode_enable, collision;

    DataControl();
    ~DataControl();
    void DataReset();

    StructClientToServer ClientToServer;
    std::vector<StructServerToClient> ServerToClient;
    StructRobotData RobotData;
    std::string err_msg[NUM_JOINT];
    std::vector<StructRobotData> loggingRobotData;
    std::vector<std::string> loggingErrorMessage[NUM_JOINT];
    StructPathData PathData;
    StructOperateMode operateMode;
    StructKITECHData KITECHData;
    StructTrayInfor trayInfor;
    StructDiningInfor diningInfor;
    double sectionOffset[3*5];

    void jointPositionENC2DEG(int32_t pos_enc[], double pos_deg[]);
    void jointPositionENC2RAD(int32_t pos_enc[], double pos_rad[]);
    void jointPositionENC2DEG(long pos_enc[], double pos_deg[]);
    void jointPositionENC2RAD(long pos_enc[], double pos_rad[]);
    void cartesianPoseScaleUp(double pose_small[], double pose_big[]);
    void cartesianPoseScaleDown(double pose_big[], double pose_small[]);
    void jointPositionRAD2ENC(double pos_rad[], int32_t pos_enc[]);
    void jointPositionDEG2ENC(double pos_deg[], int32_t pos_end[]);
    void jointPositionRAD2ENC(double pos_rad[], long pos_enc[]);
    void jointPositionDEG2ENC(double pos_deg[], long pos_end[]);
    void jointPositionDEG2ENC(const double pos_deg[], int32_t pos_end[]);
    void jointPositionDEG2ENC(const int32_t pos_deg[], int32_t pos_end[]);
    void jointPositionDEG2ENC(int32_t pos_deg[], int32_t pos_end[]);
    void jointVelocityENC2RPM(int32_t vel_enc[], double vel_rpm[]);
    void jointVelocityENC2RAD(int32_t vel_enc[], double vel_rad[]);
    void jointVelocityENC2RAD(long vel_enc[], double vel_rad[]);
    void jointCurrentRAW2mA(int16_t cur_raw[], double cur_mA[]);
    void jointCurrentmA2RAW(double cur_mA[], int16_t cur_raw[]);
    void jointCurrentRAW2A(int cur_raw[], double cur_A[]);
    void jointCurrentA2RAW(double cur_A[], int cur_raw[]);
    void jointPositionRAD2DEG(double pos_rad[], double pos_deg[]);
    void jointCurrentRAW2Torque(int16_t cur_raw[], double cur_torque[]);

    int32_t joint_offset[NUM_JOINT];
    double tool_offset[3];
    double operateCameraReadyJoint[NUM_JOINT];
    double feedingReadyJoint[NUM_JOINT];
    uint8_t MODULE_TYPE, ROBOT_ID;
    int8_t module_dir[NUM_JOINT];
    int key_value;
    double teachingJoint[NUM_JOINT];
    bool logging_enable;
    bool current_state_print;
    int dining_delay, dining_delay_cnt;
    double dining_speed, dining_speed_default;
    double select_speed;
    bool button_mode;
    bool camera_wait;
    bool robotStart;

    waypoints wp_rice1, wp_rice2, wp_rice3, wp_rice4, wp_rice5, wp_rice6, wp_rice7, wp_rice8, wp_rice9;
    waypoints wp_side1_1, wp_side1_2, wp_side2_1, wp_side2_2, wp_side3_1, wp_side3_2, wp_soup1;
    vector<waypoints> wp_rice, wp_side1, wp_side2, wp_side3, wp_soup;
    waypoints readyJoints;
    std::vector<double> joint0, joint1, joint2, joint3, joint4, joint5, joint6;
    std::vector<double> joint_path[NUM_JOINT];
    unsigned int joint_path_index, joint_path_size, joint_wp_index;
    std::vector<double> beta_path, d7_path;
    double cur_beta, des_beta, cur_d7, des_d7;
    double des_q6, des_q7;
    std::vector<double> q6_path, q7_path;
    double offset[3];
    std::vector<double> joint_wp, cartesian_wp;
    double torque_const1, torque_const2;
    int torque_offset[NUM_JOINT];
    bool err_flag;
};


#include <cmath>
#include <stdlib.h>

typedef struct{
    vector<double> x, y, z, q6, q7, file_data, time;
}StructDiningWaypoint;

class DiningWaypoints{
public:

    DiningWaypoints();
    void reset();

    StructDiningWaypoint diningSection[5], diningSection1[6], diningSection2[6], diningSection3, diningSectionSoup, diningSectionRice[12];
    vector<StructDiningWaypoint*> diningPath;
    double get_z_offset(int arg){return z_offset[arg];}
    double get_side1_offset(unsigned int arg1, unsigned int arg2){return side1_offset[arg1*2 + arg2];}
    double get_side2_offset(unsigned int arg1, unsigned int arg2){return side2_offset[arg1*2 + arg2];}
    double get_side3_offset(unsigned int arg1, unsigned int arg2){return side3_offset[arg1*2 + arg2];}
    //    double get_side5_x_offset(unsigned int arg1){return side5_offset_x[arg1%4];}
    //    double get_side5_y_offset(unsigned int arg1){return side5_offset_y[arg1/4];}
    double get_side5_x_offset(unsigned int arg1){return side5_offset_x[arg1%4];}
    double get_side5_y_offset(unsigned int arg1){return side5_offset_y[arg1/4];}
    int get_spoon_wp_size(){return side2_spoon_wp_size;}

private:
    double y_offset[2], z_offset_ref, z_offset[2];
    double spoon, chop;
    double ref_s1[3], ref_s2[3], ref_s3[3], ref_s4[3], ref_s5[9];
    double ref_s5_x[4], ref_s5_y[3], ref_s5_z;
    double y_offset_circle;
    double offset[4][2];
    double side1_offset[4*2], side2_offset[2*2], side3_offset[4*2], side5_offset[9*4], side5_offset_x[4], side5_offset_y[3];
    int side2_spoon_wp_size;
};

#endif // DATACONTROL_H
