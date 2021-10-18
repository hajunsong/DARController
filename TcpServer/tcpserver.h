#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sstream>

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <rtdk.h>

#include "DataControl/datacontrol.h"

#include <string>
#include <QString>
using namespace std;

const int MAXCONNECTIONS = 5;
const int MAXWAITBUFSIZE = 4096;
const int MAXSENDBUFSIZE = 1024;
const int MAXRECEIVEBUFSIZE = 1024;
const int SENDBUFSIZE = 156;

const uint8_t SOP_RX = 0xCA;
const uint8_t SOP_TX = 0xCB;
const uint8_t EOP = 0xCE;
const uint8_t CMD_DATA = 0xD0;
const uint8_t CMD_SECTION = 0xD1;
const uint8_t CMD_FEEDING = 0xD2;
const uint8_t CMD_TABLET_CHECK = 0xD3;
const uint8_t CMD_DINING_SPEED = 0XD5;
const uint8_t CMD_DINING_TIME = 0XD6;

class TcpServer
{
public:
    TcpServer(DataControl *dataControl);
    ~TcpServer();

    DataControl* dataControl;

    void start();
    void initSocket();
    void connectSocket();
    void stop();
    void sendDataLatte();
    void recvDataLatte();
    void sendData();
    void recvData();
    void sendKey(const char* key);
    bool isConnected(){return connected;}
    void setting(uint16_t _port);
    uint16_t getPort(){return port;}
    void setCommPeriod(unsigned int _time);
    int clientSockFD;
    bool recv_comm_run, send_comm_run, comm_manager_run, dio_comm_run, recv_latte_comm_run, send_latte_comm_run;
    bool temp_comm_run;

private:
    long sendByteLen;
    long byteLen, len;
    unsigned int curLen;
    int listenSockFD;
    char *ptrRecvBufIndx;
    unsigned char bufRecv[MAXRECEIVEBUFSIZE];
    char bufWait[MAXWAITBUFSIZE];
    char bufSend[MAXSENDBUFSIZE];
    int dataLen;

    unsigned int comm_period;

    sockaddr_in server_addr, client_addr;
    uint16_t port;

    bool connected;

    pthread_t comm_manager_thread;
    pthread_t dio_comm_thread;
    pthread_t recv_comm_thread, send_comm_thread;
    pthread_t recv_latte_comm_thread, send_latte_comm_thread;
    pthread_t temp_comm_thread;

    static void* comm_manager_func(void *arg);
    static void* recv_comm_func(void *arg);
    static void* send_comm_func(void *arg);
    static void* dio_comm_func(void *arg);
    static void* recv_latte_comm_func(void *arg);
    static void* send_latte_comm_func(void *arg);
    static void* temp_comm_func(void *arg);
};

#endif // TCPSERVER_H
