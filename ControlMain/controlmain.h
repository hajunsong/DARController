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
    void robotInitialize();
    void robotKinematics();
    void robotDynamics();
    void robotWait();
    void robotJointMove(char mode, double desJoint[NUM_JOINT]);
    void robotCartesianMove(char mode, double desCartesian[NUM_DOF]);
    void robotPathGenerate(std::vector<double> px, std::vector<double> py, std::vector<double> pz, std::vector<double> beta, std::vector<double> d7, std::vector<int> option, double offset);
    void robotPathGenerate(std::vector<double> px, std::vector<double> py, std::vector<double> pz, std::vector<double> rx, std::vector<double> ry, std::vector<double> rz);
    void robotPathGenerate(std::vector<double> px, std::vector<double> py, std::vector<double> pz, std::vector<double> beta, std::vector<double> d7);
    void robotRun();
    void robotReady();
//    void robotSPGC();
    void robotOperate();
//    void robotVSD();
    void robotTest();

    char data_indx;

    TcpServer *tcpServer;
    TcpServer *tcpServerLatte;
    TcpServer *tcpServerDIO;

    static void* init_func(void* arg);
    bool init_thread_run;

    userinterface *ecatInterface;

//    int key_value;

//    void setTabletMode();
//    void unsetTabletMode();

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
    void path_generator(double x0, double xf, double tf, double ta, double h, std::vector<double> *path, int path_index=0);
    void path_generator_circle(double p1, double p2, double pcx, double pcy, std::vector<double> path_x, std::vector<double> *path_y);
    void func_q6d7_to_q7(double q6, double d7, double* q7);
    void func_q6q7_to_d7(double q6, double q7, double* d7);

    uint8_t module_indx;

    int delay_cnt, delay_cnt_max;
    double step_size;

    double goal_current[NUM_JOINT];

    pthread_t init_thread, key_input_thread, logging_thread;
    bool key_input_thread_run;
    bool logging_thread_run;

    FILE *fp_logging;

    DiningWaypoints diningWaypoints;
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
