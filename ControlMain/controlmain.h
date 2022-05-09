#ifndef CONTROLMAIN_H
#define CONTROLMAIN_H

#include <iostream>
#include <vector>

using namespace std;

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>

#include <pthread.h>

#include "DataControl/datacontrol.h"
#include "TcpServer/tcpserver.h"
#include "RobotArm/robotarm.h"
#include "EtherCAT/userinterface.h"

#include <ctime>
#define _CRT_SECURE_NO_WARNINGS // 혹은 localtime_s를 사용

class ControlMain
{
public:
    ControlMain();
    ~ControlMain();
    void start();
    bool robot_task_run;
    RT_TASK robot_task;
    DataControl *dataControl;

    void robotServoOn(char enable);
    void robotKinematics();
    void robotDynamics();
    void robotDOB();
    void robotWait();
    void robotJointMove(char mode, double desJoint[NUM_JOINT]);
    void robotPathGenerate(std::vector<double> px, std::vector<double> py, std::vector<double> pz, std::vector<double> q6, std::vector<double> q7);
    void robotRun();
    void robotOperate();
    void robotVSD();

    char data_indx;

    TcpServer *tcpServer;
    TcpServer *tcpServerLatte;
    TcpServer *tcpServerDIO;
    TcpServer *tcpServerTemp;
    TcpServer *tcpServerPath;
    TcpServer *tcpServerDiningInfor;
    TcpServer *tcpServerKey;
    TcpServer *tcpServerOffset;

    static void* init_func(void* arg);
    bool init_thread_run;

    userinterface *ecatInterface;

private:
    static void robot_RT(void* arg);
    static void *key_input_func(void* arg);
    static void *logging_func(void* arg);
    void robot_rt_start();
    void robot_rt_stop();

    RobotArm *robotArm;

    void moduleInit();

    void goalReachCart(double desired_pose[NUM_DOF], double present_pose[NUM_DOF], bool *goal_reach);
    void goalReachJoint(long[NUM_JOINT], long[NUM_JOINT], bool *goal_reach);
    void path_generator(double x0, double xf, double tf, double ta, double h, std::vector<double> *path);
//    void path_generator_circle(double p1, double p2, double pcx, double pcy, std::vector<double> path_x, std::vector<double> *path_y);
//    void func_q6d7_to_q7(double q6, double d7, double* q7);
//    void func_q6q7_to_d7(double q6, double q7, double* d7);

    uint8_t module_indx;

    int delay_cnt, delay_cnt_max;
    double step_size;

    double goal_current[NUM_JOINT];

    pthread_t init_thread, key_input_thread, logging_thread;
    bool key_input_thread_run;
    bool logging_thread_run;

    FILE *fp_logging;

    DiningWaypoints diningWaypoints;

    int print_cnt;
    bool servo;

    double goal_torque[7];
    bool jog_command;

    int time_check_delay_cnt, time_check_delay_max;

    int cycle_count;
};

#endif // CONTROLMAIN_H

inline static int getch()
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}
