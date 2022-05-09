#include "tcpserver.h"

TcpServer::TcpServer(DataControl* _dataControl)
{
    dataControl = _dataControl;

    connected = false;

    comm_period = 10e3;

    recv_comm_run = false;
    send_comm_run = false;

    recv_latte_comm_run = false;
    send_latte_comm_run = false;

    comm_manager_run = false;
    dio_comm_run = false;

    recv_path_comm_run = false;
    send_path_comm_run = false;

    send_dining_infor_comm_run = false;

    offset_comm_run = false;

    port = 0;
}

TcpServer::~TcpServer(){
    stop();
}

void TcpServer::start(){
    pthread_create(&comm_manager_thread, NULL, comm_manager_func, this);
}

void TcpServer::setting(uint16_t _port)
{
    port = _port;
}

void TcpServer::setCommPeriod(unsigned int _time)
{
    comm_period = _time;
}

void TcpServer::stop(){
    switch(port){
        case 5050:
        {
            if(recv_comm_run){
                pthread_cancel(recv_comm_thread);
                usleep(10000);
                printf("Finished Comm Rx Task\n");
            }
            else{
                printf("Comm Rx Thread not running...\n");
            }

            if(send_comm_run){
                pthread_cancel(send_comm_thread);
                usleep(10000);
                printf("Finished Comm Tx Task\n");
            }
            else{
                printf("Comm Tx Thread not running...\n");
            }

            break;
        }
        case 5053:
        {
            if(recv_latte_comm_run){
                pthread_cancel(recv_latte_comm_thread);
                usleep(10000);
                printf("Finished LATTE Comm Rx Task\n");
            }
            else{
                printf("Comm Latte Rx Thread not running...\n");
            }

            if(send_latte_comm_run){
                pthread_cancel(send_latte_comm_thread);
                usleep(10000);
                printf("Finished LATTE Comm Tx Task\n");
            }
            else{
                printf("Comm Latte Tx Thread not running...\n");
            }

            break;
        }
        case 5059:
        {
            if(dio_comm_run){
                pthread_cancel(dio_comm_thread);
                usleep(10000);
                printf("Finished DIO Comm Thread\n");
            }
            else{
                printf("Comm DIO Thread not running...\n");
            }

            break;
        }
        case 5051:
        {
            if(temp_comm_run){
                pthread_cancel(temp_comm_thread);
                usleep(10000);
                printf("Finished Temp Comm Thread\n");
            }
            else{
                printf("Comm Temp Thread not running...\n");
            }

            break;
        }
        case 5052:
        {
            if(recv_path_comm_run){
                pthread_cancel(recv_path_comm_thread);
                usleep(10000);
                printf("Finished Comm Path Rx Task\n");
            }
            else{
                printf("Comm Path Rx Thread not running...\n");
            }

            if(send_path_comm_run){
                pthread_cancel(send_path_comm_thread);
                usleep(10000);
                printf("Finished Path Comm Tx Task\n");
            }
            else{
                printf("Comm Path Tx Thread not running...\n");
            }
            break;
        }
        case 5054:
        {
            if(send_dining_infor_comm_run){
                pthread_cancel(send_dining_infor_comm_thread);
                usleep(10000);
                printf("Finished Comm Dining infor Tx Task\n");
            }
            else{
                printf("Comm Dining Infor Tx Thread not running...\n");
            }
            break;
        }
        case 5058:
        {
            if(key_comm_run){
                pthread_cancel(key_comm_thread);
                usleep(100);
                printf("Finished Comm Key Rx Task\n");
            }
            else{
                printf("Comm Key Rx Thread not running...\n");
            }
            break;
        }
        case 5056:
        {
            if(offset_comm_run){
                pthread_cancel(offset_comm_thread);
                usleep(100);
                printf("Finished Comm Offset Rx Task\n");
            }
            else{
                printf("Comm Offset Rx Thread not running...\n");
            }
            break;
        }
        default:
            break;
    }

    if(comm_manager_run){
        pthread_cancel(comm_manager_thread);
        usleep(10000);
        printf("Finished Comm Manager Thread\n");
    }
}

void *TcpServer::comm_manager_func(void *arg)
{
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->comm_manager_run = true;

    pTcpServer->initSocket();

    while(pTcpServer->comm_manager_run){
        switch(pTcpServer->port)
        {
            case 5050:
            {
                pTcpServer->connectSocket();

                pthread_create(&pTcpServer->recv_comm_thread, NULL, pTcpServer->recv_comm_func, pTcpServer);
                pthread_create(&pTcpServer->send_comm_thread, NULL, pTcpServer->send_comm_func, pTcpServer);

                pthread_join(pTcpServer->recv_comm_thread, NULL);
                usleep(10000);

                pthread_join(pTcpServer->send_comm_thread, NULL);
                usleep(10000);

                pthread_cancel(pTcpServer->recv_comm_thread);
                usleep(10000);

                pthread_cancel(pTcpServer->send_comm_thread);
                usleep(10000);

                break;
            }
            case 5053:
            {
                pTcpServer->connectSocket();

                pthread_create(&pTcpServer->recv_latte_comm_thread, NULL, pTcpServer->recv_latte_comm_func, pTcpServer);
                pthread_create(&pTcpServer->send_latte_comm_thread, NULL, pTcpServer->send_latte_comm_func, pTcpServer);

                pthread_join(pTcpServer->recv_latte_comm_thread, NULL);
                usleep(10000);

                pthread_join(pTcpServer->send_latte_comm_thread, NULL);
                usleep(10000);

                pthread_cancel(pTcpServer->recv_latte_comm_thread);
                usleep(10000);

                pthread_cancel(pTcpServer->send_latte_comm_thread);
                usleep(10000);

                break;
            }
            case 5059:
            {
                pTcpServer->connectSocket();

                pthread_create(&pTcpServer->dio_comm_thread, NULL, pTcpServer->dio_comm_func, pTcpServer);

                pthread_join(pTcpServer->dio_comm_thread, NULL);
                usleep(10000);

                pthread_cancel(pTcpServer->dio_comm_thread);
                usleep(10000);

                break;
            }
            case 5051:
            {
                pTcpServer->connectSocket();

                pthread_create(&pTcpServer->temp_comm_thread, NULL, pTcpServer->temp_comm_func, pTcpServer);

                pthread_join(pTcpServer->temp_comm_thread, NULL);
                usleep(10000);

                pthread_cancel(pTcpServer->temp_comm_thread);
                usleep(10000);

                break;
            }
            case 5052:
            {
                pTcpServer->connectSocket();

                pthread_create(&pTcpServer->recv_path_comm_thread, NULL, pTcpServer->recv_path_comm_func, pTcpServer);
                pthread_create(&pTcpServer->send_path_comm_thread, NULL, pTcpServer->send_path_comm_func, pTcpServer);

                pthread_join(pTcpServer->recv_path_comm_thread, NULL);
                usleep(10000);

                pthread_join(pTcpServer->send_path_comm_thread, NULL);
                usleep(10000);

                pthread_cancel(pTcpServer->recv_path_comm_thread);
                usleep(10000);

                pthread_cancel(pTcpServer->send_path_comm_thread);
                usleep(10000);

                break;
            }
            case 5054:
            {
                pTcpServer->connectSocket();

                pthread_create(&pTcpServer->send_dining_infor_comm_thread, NULL, pTcpServer->send_dining_infor_comm_func, pTcpServer);

                pthread_join(pTcpServer->send_dining_infor_comm_thread, NULL);
                usleep(10000);

                pthread_cancel(pTcpServer->send_dining_infor_comm_thread);
                usleep(10000);

                break;
            }
            case 5058:
            {
                pTcpServer->connectSocket();

                pthread_create(&pTcpServer->key_comm_thread, NULL, pTcpServer->key_comm_func, pTcpServer);

                pthread_join(pTcpServer->key_comm_thread, NULL);
                usleep(100);

                pthread_cancel(pTcpServer->key_comm_thread);
                usleep(100);

                break;
            }
            case 5056:
            {
                pTcpServer->connectSocket();

                pthread_create(&pTcpServer->offset_comm_thread, NULL, pTcpServer->offset_comm_func, pTcpServer);

                pthread_join(pTcpServer->offset_comm_thread, NULL);
                usleep(100);

                pthread_cancel(pTcpServer->offset_comm_thread);
                usleep(100);

                break;
            }
            default:
            {
                break;
            }
        }
    }

    return NULL;
}

void *TcpServer::recv_comm_func(void *arg){
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->recv_comm_run = true;

    while(pTcpServer->comm_manager_run && pTcpServer->recv_comm_run){
        if(pTcpServer->isConnected()){
            memset(pTcpServer->bufRecv, 0, MAXRECEIVEBUFSIZE);
            pTcpServer->byteLen = recv(pTcpServer->clientSockFD, pTcpServer->bufRecv, MAXRECEIVEBUFSIZE, 0);

            if(pTcpServer->byteLen > 0){
                rt_printf("[CLIENT] Received byteLen : %ld\n", pTcpServer->byteLen);
                rt_printf("[CLIENT] Received data : ");
                for(uint8_t i = 0; i < 23; i++){
                    rt_printf("%d ", pTcpServer->bufRecv[i]);
                }
                rt_printf("\n\n");
            }

            if(pTcpServer->byteLen == 0){
                rt_printf("[CLIENT] Recv Disconnected\n");
                pTcpServer->connected = false;
            }
            else if(pTcpServer->byteLen > 0){
                pTcpServer->recvData();
            }
        }
        else{
            pTcpServer->recv_comm_run = false;
            break;
        }

        usleep(pTcpServer->comm_period);
    }

    rt_printf("Finished Client Comm RX Thread(%d)\n", pTcpServer->port);

    return NULL;
}

void *TcpServer::send_comm_func(void *arg){
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->sendByteLen = 0;
    pTcpServer->send_comm_run = true;

    while(pTcpServer->comm_manager_run && pTcpServer->send_comm_run){
        if(pTcpServer->connected){
            pTcpServer->sendData();

            if(pTcpServer->sendByteLen < 0){
                rt_printf("[CLIENT] Send Disconnected(%d)\n", pTcpServer->port);
                pTcpServer->connected = false;
            }
        }
        else{
            pTcpServer->send_comm_run = false;
            break;
        }

        usleep(pTcpServer->comm_period);
    }

    rt_printf("Finished Client Comm TX Thread(%d)\n", pTcpServer->port);

    return NULL;
}

void *TcpServer::recv_latte_comm_func(void *arg)
{
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->recv_latte_comm_run = true;

    while(pTcpServer->comm_manager_run && pTcpServer->recv_latte_comm_run){
        if(pTcpServer->isConnected()){
            memset(pTcpServer->bufRecv, 0, MAXRECEIVEBUFSIZE);
            pTcpServer->byteLen = recv(pTcpServer->clientSockFD, pTcpServer->bufRecv, MAXRECEIVEBUFSIZE, 0);

            if(pTcpServer->byteLen > 0){
                rt_printf("[LATTE] Received byteLen : %ld\n", pTcpServer->byteLen);
                rt_printf("[LATTE] Received data : ");
                for(uint8_t i = 0; i < 23; i++){
                    rt_printf("%d ", pTcpServer->bufRecv[i]);
                }
                rt_printf("\n\n");
            }

            if(pTcpServer->byteLen == 0){
                rt_printf("[LATTE] Recv Disconnected\n");
                pTcpServer->connected = false;
            }
            else if(pTcpServer->byteLen > 0){
                pTcpServer->recvDataLatte();
            }
        }
        else{
            pTcpServer->recv_latte_comm_run = false;
            break;
        }

        usleep(pTcpServer->comm_period);
    }

    rt_printf("Finished LATTE Comm RX Thread(%d)\n", pTcpServer->port);

    return NULL;
}

void *TcpServer::send_latte_comm_func(void *arg)
{
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->sendByteLen = 0;
    pTcpServer->send_latte_comm_run = true;

    while(pTcpServer->comm_manager_run && pTcpServer->send_latte_comm_run){
        if(pTcpServer->connected){
            pTcpServer->sendDataLatte();

            if(pTcpServer->sendByteLen < 0){
                rt_printf("[LATTE] Send Disconnected(%d)\n", pTcpServer->port);
                pTcpServer->connected = false;
            }
        }
        else{
            pTcpServer->send_latte_comm_run = false;
            break;
        }

        usleep(pTcpServer->comm_period);
    }

    rt_printf("Finished LATTE Comm TX Thread(%d)\n", pTcpServer->port);

    return NULL;
}

void *TcpServer::dio_comm_func(void *arg){
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->dio_comm_run = true;

    while(pTcpServer->comm_manager_run && pTcpServer->dio_comm_run){
        memset(pTcpServer->bufRecv, 0, MAXRECEIVEBUFSIZE);
        pTcpServer->byteLen = recv(pTcpServer->clientSockFD, pTcpServer->bufRecv, MAXRECEIVEBUFSIZE, 0);

        if(pTcpServer->byteLen > 0){
            //            rt_printf("[DIO] Received byteLen : %ld\n", pTcpServer->byteLen);
            //            rt_printf("[DIO] Received data : ");
            //            for(uint8_t i = 0; i < pTcpServer->byteLen; i++){
            //                rt_printf("%d ", pTcpServer->bufRecv[i]);
            //            }
            //            rt_printf("\n\n");

            pTcpServer->dataControl->key_value = 0;
            if(pTcpServer->dataControl->ClientToServer.opMode == DataControl::Wait){
                if(pTcpServer->bufRecv[1] == 49){
                    pTcpServer->dataControl->key_value = KEY_Z;
                    rt_printf("Pressed key z\n");
                    while(pTcpServer->dataControl->key_value == KEY_Z){
                        usleep(10000);
                    }
                }

                if(pTcpServer->bufRecv[4] == 49){
                    pTcpServer->dataControl->key_value = KEY_X;
                    rt_printf("Pressed key x\n");
                    while(pTcpServer->dataControl->key_value == KEY_X){
                        usleep(10000);
                    }
                }

                if(pTcpServer->bufRecv[7] == 49){

                }
            }

            memset(pTcpServer->bufSend, 0, MAXSENDBUFSIZE);
            pTcpServer->bufSend[0] = static_cast<char>(pTcpServer->dataControl->key_value);
            pTcpServer->sendByteLen = send(pTcpServer->clientSockFD, pTcpServer->bufSend, 1, 0);

        }
        else if(pTcpServer->byteLen == 0){
            rt_printf("[DIO] Recv Disconnected\n");
            pTcpServer->dio_comm_run = false;
        }

        usleep(100000);
    }

    rt_printf("Finished Comm RX Thread(%d)\n", pTcpServer->port);

    return NULL;
}

void *TcpServer::temp_comm_func(void *arg)
{
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->temp_comm_run = true;

    while(pTcpServer->comm_manager_run && pTcpServer->temp_comm_run){
        memset(pTcpServer->bufRecv, 0, MAXRECEIVEBUFSIZE);
        pTcpServer->byteLen = recv(pTcpServer->clientSockFD, pTcpServer->bufRecv, MAXRECEIVEBUFSIZE, 0);

        if(pTcpServer->byteLen > 0){
            printf("[TEMP] Received byteLen : %ld\n", pTcpServer->byteLen);
            printf("[TEMP] Received data : ");
            for(uint8_t i = 0; i < pTcpServer->byteLen; i++){
                printf("%d ", pTcpServer->bufRecv[i]);
            }
            printf("\n\n");

            char buf[8];
            double temp_data1, temp_data2, temp_data3 = 200.0;
            int indx = 0;

            memcpy(buf, pTcpServer->bufRecv + indx, 8);
            indx += 8;
            temp_data1 = atof(buf);

            memcpy(buf, pTcpServer->bufRecv + indx, 8);
            indx += 8;
            temp_data2 = atof(buf);

            printf("receive data : %f, %f\n", temp_data1, temp_data2);

            memcpy(pTcpServer->dataControl->ClientToServer.desiredJoint, pTcpServer->dataControl->feedingReadyJoint, sizeof(double)*NUM_JOINT);

            pTcpServer->dataControl->ClientToServer.opMode = DataControl::JointMove;
            pTcpServer->dataControl->ClientToServer.subMode = DataControl::JointMotion;
            pTcpServer->dataControl->joint_path_index = 0;

            while(!(pTcpServer->dataControl->ClientToServer.opMode == DataControl::Wait)){
                usleep(1000);
            }

            pTcpServer->dataControl->ClientToServer.desiredPose[0] = temp_data1;
            pTcpServer->dataControl->ClientToServer.desiredPose[1] = temp_data2;
            pTcpServer->dataControl->ClientToServer.desiredPose[2] = temp_data3;
            pTcpServer->dataControl->ClientToServer.desiredPose[3] = feedingOrientation[0];
            pTcpServer->dataControl->ClientToServer.desiredPose[4] = feedingOrientation[1];
            pTcpServer->dataControl->ClientToServer.desiredPose[5] = feedingOrientation[2];

            pTcpServer->dataControl->ClientToServer.move_time = 2.0;
            pTcpServer->dataControl->ClientToServer.acc_time = 0.8;

            for(int i = 0; i < 6; i++){
                rt_printf("des pose %d : %f\n", i, pTcpServer->dataControl->ClientToServer.desiredPose[i]);
            }
            rt_printf("move_time : %f\n", pTcpServer->dataControl->ClientToServer.move_time);
            rt_printf("acc_time  : %f\n", pTcpServer->dataControl->ClientToServer.acc_time);

            pTcpServer->dataControl->ClientToServer.opMode = DataControl::CartesianMove;
            pTcpServer->dataControl->ClientToServer.subMode = DataControl::CartesianMotion;

            while(!(pTcpServer->dataControl->ClientToServer.opMode == DataControl::Wait)){
                usleep(1000);
            }
        }
        else if(pTcpServer->byteLen == 0){
            printf("[TEMP] Recv Disconnected\n");
            pTcpServer->temp_comm_run = false;
        }

        memset(pTcpServer->bufSend, 0, SENDBUFSIZE);

        pTcpServer->bufSend[0] = 1;
        pTcpServer->sendByteLen = send(pTcpServer->clientSockFD, pTcpServer->bufSend, 1, 0);

        usleep(10000);
    }

    rt_printf("Finished Comm RX Thread(%d)\n", pTcpServer->port);

    return NULL;
}

void *TcpServer::recv_path_comm_func(void *arg)
{
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->recv_path_comm_run = true;

    while(pTcpServer->comm_manager_run && pTcpServer->recv_path_comm_run){
        if(pTcpServer->isConnected()){
            memset(pTcpServer->bufRecv, 0, MAXRECEIVEBUFSIZE);
            pTcpServer->byteLen = recv(pTcpServer->clientSockFD, pTcpServer->bufRecv, MAXRECEIVEBUFSIZE, 0);

            if(pTcpServer->byteLen > 0){
                rt_printf("[PATH] Received byteLen : %ld\n", pTcpServer->byteLen);
                //                rt_printf("[PATH] Received data : ");
                //                for(uint8_t i = 0; i < pTcpServer->byteLen; i++){
                //                    rt_printf("%d ", pTcpServer->bufRecv[i]);
                //                }
                //                rt_printf("\n\n");

                pTcpServer->recvDataPath();
            }
            else if(pTcpServer->byteLen <= 0){
                rt_printf("[PATH] Recv Disconnected\n");
                pTcpServer->connected = false;
            }
        }
        else{
            pTcpServer->recv_path_comm_run = false;
            break;
        }

        usleep(pTcpServer->comm_period);
    }

    rt_printf("Finished Comm RX Thread(%d)\n", pTcpServer->port);

    return NULL;
}

void *TcpServer::send_path_comm_func(void *arg)
{
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->sendByteLen = 0;
    pTcpServer->send_path_comm_run = true;

    while(pTcpServer->comm_manager_run && pTcpServer->send_path_comm_run){
        if(pTcpServer->connected){
            pTcpServer->sendDataPath();

            if(pTcpServer->sendByteLen < 0){
                printf("[PATH] Send Disconnected(%d)\n", pTcpServer->port);
                pTcpServer->connected = false;
            }
        }
        else{
            pTcpServer->send_path_comm_run = false;
            break;
        }

        usleep(pTcpServer->comm_period);
    }

    rt_printf("Finished Comm TX Thread(%d)\n", pTcpServer->port);

    return NULL;
}

void *TcpServer::send_dining_infor_comm_func(void *arg)
{
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->sendByteLen = 0;
    pTcpServer->send_dining_infor_comm_run = true;

    while(pTcpServer->comm_manager_run && pTcpServer->send_dining_infor_comm_run){
        if(pTcpServer->connected){

            pTcpServer->sendDataDiningInfor();

            if(pTcpServer->sendByteLen < 0){
                printf("[DINING INFOR] Send Disconnected(%d)\n", pTcpServer->port);
                pTcpServer->connected = false;
            }
        }
        else{
            pTcpServer->send_dining_infor_comm_run = false;
            break;
        }

        usleep(pTcpServer->comm_period);
    }

    rt_printf("Finished Comm TX Thread(%d)\n", pTcpServer->port);

    return NULL;
}

void *TcpServer::key_comm_func(void *arg)
{
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->key_comm_run = true;

    while(pTcpServer->comm_manager_run && pTcpServer->key_comm_run){
        memset(pTcpServer->bufRecv, 0, MAXRECEIVEBUFSIZE);
        pTcpServer->byteLen = recv(pTcpServer->clientSockFD, pTcpServer->bufRecv, MAXRECEIVEBUFSIZE, 0);

        if(pTcpServer->byteLen > 0){
            //            printf("[KEY] Received byteLen : %ld\n", pTcpServer->byteLen);
            //            printf("[KEY] Received data : ");
            //            for(uint8_t i = 0; i < pTcpServer->byteLen; i++){
            //                printf("%d ", pTcpServer->bufRecv[i]);
            //            }
            //            printf("\n\n");

            if(pTcpServer->bufRecv[2] > 0){
                pTcpServer->recvDataKey();
            }

        }
        else if(pTcpServer->byteLen == 0){
            rt_printf("[KEY] Recv Disconnected\n");
            pTcpServer->key_comm_run = false;
        }

        usleep(100);
    }

    rt_printf("Finished Comm RX Thread(%d)\n", pTcpServer->port);

    return NULL;
}

void *TcpServer::offset_comm_func(void *arg)
{
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->offset_comm_run = true;

    while(pTcpServer->comm_manager_run && pTcpServer->offset_comm_run){
        memset(pTcpServer->bufRecv, 0, MAXRECEIVEBUFSIZE);
        pTcpServer->byteLen = recv(pTcpServer->clientSockFD, pTcpServer->bufRecv, MAXRECEIVEBUFSIZE, 0);

        if(pTcpServer->byteLen > 0){
            //            printf("[KEY] Received byteLen : %ld\n", pTcpServer->byteLen);
            //            printf("[KEY] Received data : ");
            //            for(uint8_t i = 0; i < pTcpServer->byteLen; i++){
            //                printf("%d ", pTcpServer->bufRecv[i]);
            //            }
            //            printf("\n\n");

            pTcpServer->recvDataOffset();
        }
        else if(pTcpServer->byteLen == 0){
            rt_printf("[OFFSET] Recv Disconnected\n");
            pTcpServer->offset_comm_run = false;
        }

        usleep(100);
    }

    rt_printf("Finished Comm RX Thread(%d)\n", pTcpServer->port);

    return NULL;
}

void TcpServer::recvDataLatte(){
    if(bufRecv[0] == SOP_RX){
        dataControl->KITECHData.camera_request = false;
        dataControl->KITECHData.interface_cmd = bufRecv[1];
        dataControl->KITECHData.interface_sub = bufRecv[2];

        switch(dataControl->KITECHData.interface_cmd){
            case CMD_DATA:
            {
                rt_printf("Received Data Packet\n");

                for(int i = 0; i < 20; i++){
                    dataControl->KITECHData.food_pixel[i] = (signed char)bufRecv[i + 2];
                }

                for(int i = 0, j = 0; i < 10; i++, j+=2){
                    dataControl->KITECHData.food_pos[i] = (int)dataControl->KITECHData.food_pixel[j]*100 + (int)dataControl->KITECHData.food_pixel[j+1];
                }

                rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[0], dataControl->KITECHData.food_pos[1]);
                rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[2], dataControl->KITECHData.food_pos[3]);
                rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[4], dataControl->KITECHData.food_pos[5]);
                rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[6], dataControl->KITECHData.food_pos[7]);
                rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[8], dataControl->KITECHData.food_pos[9]);

                dataControl->KITECHData.food_pos[0] = 0;
                dataControl->KITECHData.food_pos[1] = 0;
                dataControl->KITECHData.food_pos[2] = 0;
                dataControl->KITECHData.food_pos[3] = 0;

                //                dataControl->KITECHData.food_pos[4] = 0;
                //                dataControl->KITECHData.food_pos[5] = 0;
                if(dataControl->KITECHData.food_pos[4] < -33){
                    dataControl->KITECHData.food_pos[4] = -33;
                }
                if(dataControl->KITECHData.food_pos[4] > 33){
                    dataControl->KITECHData.food_pos[4] = 33;
                }

                dataControl->KITECHData.food_pos[6] = 0;
                dataControl->KITECHData.food_pos[7] = 0;
                dataControl->KITECHData.food_pos[8] = 0;
                dataControl->KITECHData.food_pos[9] = 0;

                dataControl->KITECHData.camera_request = false;
                dataControl->KITECHData.receive_point = true;
                dataControl->camera_wait = false;

                break;
            }
            default:break;
        }
    }
    if(bufRecv[0] == SOP_RX && dataControl->ClientToServer.opMode == DataControl::Wait && !dataControl->feeding && !dataControl->camera_wait){
        dataControl->KITECHData.camera_request = false;
        dataControl->KITECHData.interface_cmd = bufRecv[1];
        dataControl->KITECHData.interface_sub = bufRecv[2];

        if(!dataControl->robotStart){
            if(dataControl->KITECHData.interface_cmd != CMD_TABLET_CHECK){
                dataControl->key_value = KEY_Z;
            }
            else{
                if(dataControl->KITECHData.interface_sub == 1){
                    dataControl->KITECHData.tablet_connect = true;
                    rt_printf("tabelt connected\n");
                }
                else{
                    dataControl->KITECHData.tablet_connect = false;
                    rt_printf("tabelt disconnected\n");
                }
            }
        }
        else{
            switch(dataControl->KITECHData.interface_cmd){
                case CMD_FEEDING:
                {
                    rt_printf("Received Feeding Packet\n");
                    rt_printf("Section : %d\n", dataControl->KITECHData.interface_sub);
                    dataControl->section_indx = dataControl->KITECHData.interface_sub - 1;
                    dataControl->button_mode = false;

                    if(dataControl->KITECHData.wait_feeding){
                        dataControl->ClientToServer.opMode = DataControl::OperateMode;
                        dataControl->operateMode.mode = DataControl::WaitFeeding1;

                        while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
                            usleep(1000);
                        }
                    }

                    //                if(dataControl->section_indx != 2)
                    {
                        dataControl->ClientToServer.opMode = DataControl::OperateMode;
                        dataControl->operateMode.mode = DataControl::ReadyFeeding;

                        while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
                            usleep(1000);
                        }

                        dataControl->ClientToServer.opMode = DataControl::OperateMode;
                        dataControl->operateMode.mode = DataControl::Feeding;
                        switch(dataControl->section_indx){
                            case 0: // side 1
                            {
                                dataControl->trayInfor.section1 = dataControl->trayInfor.section1_cnt%6;
                                rt_printf("section1 count : %d, %d\n", dataControl->trayInfor.section1_cnt, dataControl->trayInfor.section1);
                                dataControl->operateMode.section = DataControl::Side1;
                                break;
                            }
                            case 1: // side 2
                            {
                                dataControl->trayInfor.section2 = dataControl->trayInfor.section2_cnt%6;
                                rt_printf("section2 count : %d, %d\n", dataControl->trayInfor.section2_cnt, dataControl->trayInfor.section2);
                                dataControl->operateMode.section = DataControl::Side2;
                                break;
                            }
                            case 2: // side 3
                            {
                                dataControl->trayInfor.section3 = dataControl->trayInfor.section3_cnt%4;
                                rt_printf("section3 count : %d, %d\n", dataControl->trayInfor.section3_cnt, dataControl->trayInfor.section3);
                                dataControl->operateMode.section = DataControl::Side3;
                                break;
                            }
                            case 3: // soup
                            {
                                rt_printf("section4 count : %d\n", dataControl->trayInfor.section4);
                                dataControl->operateMode.section = DataControl::Soup;
                                break;
                            }
                            case 4: // rice
                            {
                                dataControl->trayInfor.section5 = dataControl->trayInfor.section5_cnt%12;
                                rt_printf("section5 count : %d, %d\n", dataControl->trayInfor.section5_cnt, dataControl->trayInfor.section5);
                                dataControl->operateMode.section = DataControl::Rice;
                                break;
                            }
                        }
                        dataControl->select_indx = dataControl->section_indx;
                    }
                    break;
                }
                case CMD_TABLET_CHECK:
                {
                    if(dataControl->KITECHData.interface_sub == 1){
                        dataControl->KITECHData.tablet_connect = true;
                        rt_printf("tabelt connected\n");
                    }
                    else{
                        dataControl->KITECHData.tablet_connect = false;
                        rt_printf("tabelt disconnected\n");
                    }
                    break;
                }
                case CMD_TOOL:
                {
                    //                dataControl->KITECHData.dining_tool = bufRecv[3];
                    //                if(dataControl->KITECHData.dining_tool == 1){
                    //                    rt_printf("selected chopsticks\n");
                    //                }
                    //                else if(dataControl->KITECHData.dining_tool == 0){
                    //                    rt_printf("selected spoon\n");
                    //                }
                    if(bufRecv[3] == 0){ // chopsticks
                        dataControl->KITECHData.dining_tool = 1;
                        rt_printf("selected chopsticks\n");
                    }
                    else if(bufRecv[3] == 1){ // spoon
                        dataControl->KITECHData.dining_tool = 0;
                        rt_printf("selected spoon\n");
                    }
                    break;
                }
                case CMD_DINING_SPEED:
                {
                    dataControl->KITECHData.dining_speed = bufRecv[2];
                    switch(dataControl->KITECHData.dining_speed){
                        case 1:
                            dataControl->dining_speed = 1.3;
                            break;
                        case 2:
                            dataControl->dining_speed = 1.0;
                            break;
                        case 3:
                            dataControl->dining_speed = 0.7;
                            break;
                        default:
                            break;
                    }

                    rt_printf("dining speed : %d\n", dataControl->KITECHData.dining_speed);
                    break;
                }
                case CMD_DINING_TIME:
                {
                    dataControl->KITECHData.dining_time = bufRecv[2];
                    rt_printf("dining time : %d\n", dataControl->KITECHData.dining_time);
                    dataControl->dining_delay = dataControl->KITECHData.dining_time*1000;
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
    }
    else if(bufRecv[0] == SOP_RX && dataControl->ClientToServer.opMode == DataControl::OperateMode && dataControl->operateMode.section == DataControl::Home && dataControl->feeding){
        dataControl->KITECHData.camera_request = false;
        dataControl->KITECHData.interface_cmd = bufRecv[1];
        dataControl->KITECHData.interface_sub = bufRecv[2];

        switch(dataControl->KITECHData.interface_cmd){
            case CMD_SECTION:
            case CMD_FEEDING:
                if(dataControl->dining_delay_cnt > 0){
                    rt_printf("interrupt!!!\n");
                    dataControl->KITECHData.dining_interrupt = true;
                }
                else{
                    dataControl->KITECHData.dining_interrupt = false;
                }
                break;
            default:
                break;
        }
    }
}

void TcpServer::sendDataLatte(){
    if(dataControl->KITECHData.camera_request && (dataControl->KITECHData.camera_request_old != dataControl->KITECHData.camera_request)){
        memset(bufSend, 0, SENDBUFSIZE);

        bufSend[0] = SOP_TX;
        bufSend[1] = 1;
        bufSend[2] = EOP;
        dataControl->camera_wait = true;
        usleep(5000000);
        sendByteLen = send(clientSockFD, bufSend, 3, 0);
        if(sendByteLen > 0){
            rt_printf("Send to KITECH request camera\n");
        }

        usleep(5000000);
        dataControl->camera_wait = false;

        if(!dataControl->button_mode){
            dataControl->ClientToServer.opMode = DataControl::OperateMode;
            dataControl->operateMode.mode = DataControl::WaitFeeding1;

            while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
                usleep(1000);
            }

            dataControl->ClientToServer.opMode = DataControl::OperateMode;
            dataControl->operateMode.mode = DataControl::WaitFeeding2;

            while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
                usleep(1000);
            }

            dataControl->ClientToServer.opMode = DataControl::ServoOnOff;
            dataControl->ClientToServer.subMode = 0;

            if(sendByteLen < 0){
                rt_printf("Send Error\n");
                comm_manager_run = false;
                connected = false;

                return;
            }
        }
        else if(dataControl->button_mode){
            while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
                usleep(1000);
            }

            if(sendByteLen < 0){
                rt_printf("Send Error\n");
                comm_manager_run = false;
                connected = false;

                return;
            }

            dataControl->ClientToServer.opMode = DataControl::OperateMode;
            dataControl->operateMode.mode = DataControl::ReadyFeeding;

            while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
                usleep(1000);
            }

            dataControl->select_indx++;
            if(dataControl->select_indx >= 6) {
                dataControl->select_indx = 1;
            }
            rt_printf("Section Index : %d\n", dataControl->select_indx);
            dataControl->operateMode.section = dataControl->select_indx;
            dataControl->section_indx = dataControl->select_indx - 1;

            switch(dataControl->section_indx){
                case 0: // side 1
                {
                    dataControl->trayInfor.section1 = dataControl->trayInfor.section1_cnt%6;
                    rt_printf("section1 count : %d, %d\n", dataControl->trayInfor.section1_cnt, dataControl->trayInfor.section1);
                    dataControl->operateMode.section = DataControl::Side1;
                    break;
                }
                case 1: // side 2
                {
                    dataControl->trayInfor.section2 = dataControl->trayInfor.section2_cnt%6;
                    rt_printf("section2 count : %d, %d\n", dataControl->trayInfor.section2_cnt, dataControl->trayInfor.section2);
                    dataControl->operateMode.section = DataControl::Side2;
                    break;
                }
                case 2: // side 3
                {
                    dataControl->trayInfor.section3 = dataControl->trayInfor.section3_cnt%4;
                    rt_printf("section3 count : %d, %d\n", dataControl->trayInfor.section3_cnt, dataControl->trayInfor.section3);
                    dataControl->operateMode.section = DataControl::Side3;
                    break;
                }
                case 3: // soup
                {
                    rt_printf("section4 count : %d\n", dataControl->trayInfor.section4);
                    dataControl->operateMode.section = DataControl::Soup;
                    break;
                }
                case 4: // rice
                {
                    dataControl->trayInfor.section5 = dataControl->trayInfor.section5_cnt%12;
                    rt_printf("section5 count : %d, %d\n", dataControl->trayInfor.section5_cnt, dataControl->trayInfor.section5);
                    dataControl->operateMode.section = DataControl::Rice;
                    break;
                }
            }

            dataControl->ClientToServer.opMode = DataControl::OperateMode;
            dataControl->operateMode.mode = DataControl::FeedingSwitch;
        }
    }
    dataControl->KITECHData.camera_request_old = dataControl->KITECHData.camera_request;

    if(dataControl->KITECHData.tablet_check && (dataControl->KITECHData.tablet_check_old != dataControl->KITECHData.tablet_check)){
        memset(bufSend, 0, SENDBUFSIZE);

        bufSend[0] = SOP_TX;
        bufSend[1] = 2;
        bufSend[2] = EOP;
        sendByteLen = send(clientSockFD, bufSend, 3, 0);
        if(sendByteLen > 0){
            rt_printf("Send to KITECH tablet connect check\n");
        }

        if(sendByteLen < 0){
            rt_printf("Send Error\n");
            comm_manager_run = false;
            connected = false;

            return;
        }
    }
    dataControl->KITECHData.tablet_check_old = dataControl->KITECHData.tablet_check;
}

void TcpServer::sendDataDiningInfor(){
    if(dataControl->diningInfor.dining_time_send && (dataControl->diningInfor.dining_time_send != dataControl->diningInfor.dining_time_send_old)){
        memset(bufSend, 0, SENDBUFSIZE);

        bufSend[0] = SOP_TX;
        bufSend[1] = CMD_DINING_INFO;
        int indx = 2;

        bufSend[indx++] = dataControl->diningInfor.robot_id;
        bufSend[indx++] = dataControl->diningInfor.dining_step;

        for(int i = 0; i < 12; i++){
            bufSend[indx++] = dataControl->diningInfor.dining_time_infor[i];
        }
        bufSend[indx++] = DataControl::Side1;
        bufSend[indx++] = dataControl->trayInfor.section1_cnt;
        bufSend[indx++] = DataControl::Side2;
        bufSend[indx++] = dataControl->trayInfor.section2_cnt;
        bufSend[indx++] = DataControl::Side3;
        bufSend[indx++] = dataControl->trayInfor.section3_cnt;
        bufSend[indx++] = DataControl::Soup;
        bufSend[indx++] = dataControl->trayInfor.section4_cnt;
        bufSend[indx++] = DataControl::Rice;
        bufSend[indx++] = dataControl->trayInfor.section5_cnt;

        bufSend[indx++] = EOP;

        sendByteLen = send(clientSockFD, bufSend, indx, 0);
        rt_printf("Send to GIST dining information\n");

        for(int i = 0; i < indx; i++){
            rt_printf("%d ", bufSend[i]);
        }
        rt_printf("\n");

        //        if(sendByteLen < 0){
        //            rt_printf("Send Error\n");
        //            comm_manager_run = false;
        //            connected = false;

        //            return;
        //        }
        dataControl->diningInfor.dining_time_send = false;
    }
    dataControl->diningInfor.dining_time_send_old = dataControl->diningInfor.dining_time_send;
}

void TcpServer::recvDataKey()
{
    dataControl->KITECHData.interface_cmd = bufRecv[1];
    dataControl->KITECHData.interface_sub = bufRecv[2];

    if(!dataControl->robotStart){
        if(dataControl->KITECHData.interface_cmd != CMD_TABLET_CHECK){
            dataControl->key_value = KEY_Z;
        }
        else{
            if(dataControl->KITECHData.interface_sub == 1){
                dataControl->KITECHData.tablet_connect = true;
                rt_printf("tabelt connected\n");
            }
            else{
                dataControl->KITECHData.tablet_connect = false;
                rt_printf("tabelt disconnected\n");
            }
        }
    }

    else{

        if(dataControl->KITECHData.interface_sub == 3){
            rt_printf("Received safe mode packet\n");

            dataControl->safe_mode_enable ^= true;

            rt_printf("Safe mode : %s\n", dataControl->safe_mode_enable ? "ON" : "OFF");

            if(!dataControl->safe_mode_enable){
                dataControl->collision_detect_enable = false;
            }
            ////
            //// TODO : LED display
            ////
        }

        //    rt_printf("bufRecv[0] : %s\n", bufRecv[0] == dataControl->ClientToServer.opMode ? "true" : "false");
        //    rt_printf("feeding : %s\n", dataControl->feeding ? "true" : "false");
        //    rt_printf("camera_wait : %s\n", dataControl->camera_wait ? "true" : "false");

        else {
            if(bufRecv[0] == SOP_RX && dataControl->ClientToServer.opMode == DataControl::Wait && !dataControl->feeding && !dataControl->camera_wait){
                dataControl->KITECHData.camera_request = false;
                //        dataControl->KITECHData.interface_cmd = bufRecv[1];
                //        dataControl->KITECHData.interface_sub = bufRecv[2];

                //        printf("[KEY] Received byteLen : %ld\n", byteLen);
                //        printf("[KEY] Received data : ");
                //        for(uint8_t i = 0; i < byteLen; i++){
                //            printf("%d ", bufRecv[i]);
                //        }
                //        printf("\n\n");

                switch(dataControl->KITECHData.interface_cmd){
                    case CMD_SECTION:
                    {
                        dataControl->button_mode = true;
                        if(dataControl->KITECHData.wait_feeding){
                            dataControl->ClientToServer.opMode = DataControl::OperateMode;
                            dataControl->operateMode.mode = DataControl::WaitFeeding1;

                            while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
                                usleep(100);
                            }

                            dataControl->ClientToServer.opMode = DataControl::OperateMode;
                            dataControl->operateMode.mode = DataControl::ReadyFeeding;

                            while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
                                usleep(100);
                            }
                        }

                        if(!dataControl->button_mode){
                            dataControl->ClientToServer.opMode = DataControl::OperateMode;
                            dataControl->operateMode.mode = DataControl::ReadyFeeding;
                        }

                        while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
                            usleep(100);
                        }

                        // choose section
                        if(dataControl->KITECHData.interface_sub == 1){
                            rt_printf("Received Section Packet\n");
                            dataControl->select_indx++;
                            if(dataControl->select_indx >= 6) {
                                dataControl->select_indx = 1;
                            }
                            rt_printf("Section Index : %d\n", dataControl->select_indx);
                            dataControl->operateMode.section = dataControl->select_indx;
                            dataControl->section_indx = dataControl->select_indx - 1;

                            dataControl->ClientToServer.opMode = DataControl::OperateMode;
                            dataControl->operateMode.mode = DataControl::FeedingSwitch;

                            switch(dataControl->section_indx){
                                case 0: // side 1
                                {
                                    dataControl->trayInfor.section1 = dataControl->trayInfor.section1_cnt%6;
                                    rt_printf("section1 count : %d, %d\n", dataControl->trayInfor.section1_cnt, dataControl->trayInfor.section1);
                                    dataControl->operateMode.section = DataControl::Side1;
                                    break;
                                }
                                case 1: // side 2
                                {
                                    dataControl->trayInfor.section2 = dataControl->trayInfor.section2_cnt%6;
                                    rt_printf("section2 count : %d, %d\n", dataControl->trayInfor.section2_cnt, dataControl->trayInfor.section2);
                                    dataControl->operateMode.section = DataControl::Side2;
                                    break;
                                }
                                case 2: // side 3
                                {
                                    dataControl->trayInfor.section3 = dataControl->trayInfor.section3_cnt%4;
                                    rt_printf("section3 count : %d, %d\n", dataControl->trayInfor.section3_cnt, dataControl->trayInfor.section3);
                                    dataControl->operateMode.section = DataControl::Side3;
                                    break;
                                }
                                case 3: // soup
                                {
                                    rt_printf("section4 count : %d\n", dataControl->trayInfor.section4);
                                    dataControl->operateMode.section = DataControl::Soup;
                                    break;
                                }
                                case 4: // rice
                                {
                                    dataControl->trayInfor.section5 = dataControl->trayInfor.section5_cnt%12;
                                    rt_printf("section5 count : %d, %d\n", dataControl->trayInfor.section5_cnt, dataControl->trayInfor.section5);
                                    dataControl->operateMode.section = DataControl::Rice;
                                    break;
                                }
                            }
                        }
                        // feeding
                        else if(dataControl->KITECHData.interface_sub == 2){
                            rt_printf("Received Feeding Packet\n");
                            rt_printf("Section : %d\n", dataControl->section_indx);
                            //                    dataControl->operateMode.section = dataControl->section_indx + 1;

                            dataControl->ClientToServer.opMode = DataControl::OperateMode;
                            dataControl->operateMode.mode = DataControl::Feeding;

                            switch(dataControl->section_indx){
                                case 0: // side 1
                                {
                                    dataControl->trayInfor.section1 = dataControl->trayInfor.section1_cnt%6;
                                    rt_printf("section1 count : %d, %d\n", dataControl->trayInfor.section1_cnt, dataControl->trayInfor.section1);
                                    dataControl->operateMode.section = DataControl::Side1;
                                    break;
                                }
                                case 1: // side 2
                                {
                                    dataControl->trayInfor.section2 = dataControl->trayInfor.section2_cnt%6;
                                    rt_printf("section2 count : %d, %d\n", dataControl->trayInfor.section2_cnt, dataControl->trayInfor.section2);
                                    dataControl->operateMode.section = DataControl::Side2;
                                    break;
                                }
                                case 2: // side 3
                                {
                                    dataControl->trayInfor.section3 = dataControl->trayInfor.section3_cnt;
                                    rt_printf("section3 count : %d, %d\n", dataControl->trayInfor.section3_cnt, dataControl->trayInfor.section3);
                                    dataControl->operateMode.section = DataControl::Side3;
                                    break;
                                }
                                case 3: // soup
                                {
                                    rt_printf("section4 count : %d\n", dataControl->trayInfor.section4);
                                    dataControl->operateMode.section = DataControl::Soup;
                                    break;
                                }
                                case 4: // rice
                                {
                                    dataControl->trayInfor.section5 = dataControl->trayInfor.section5_cnt%12;
                                    rt_printf("section5 count : %d, %d\n", dataControl->trayInfor.section5_cnt, dataControl->trayInfor.section5);
                                    dataControl->operateMode.section = DataControl::Rice;
                                    break;
                                }
                            }
                            dataControl->select_indx = dataControl->section_indx;
                        }
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
        }
    }
}

void TcpServer::recvDataOffset()
{
    if(bufRecv[1] == CMD_OFFSET){
        switch(bufRecv[2]){
            case 0x01:
            {
                int indx = 3;
                char buf[8] = {0,};

                printf("receive section offset data : ");
                for(int i = 0; i < 15; i++){
                    memcpy(buf, bufRecv + indx, sizeof(double));
                    dataControl->sectionOffset[i] = atof(buf);
                    indx += sizeof(double);
                    printf("%f, ", dataControl->sectionOffset[i]);
                }
                printf("\n");
                break;
            }
            case 0x02:
            {
                int indx = 3;
                printf("receive torque offset data : ");
                for(int i = 0; i < NUM_JOINT; i++, indx++){
                    //                    memcpy(buf, bufRecv + indx, sizeof(int));
                    dataControl->torque_offset[i] = static_cast<int>(bufRecv[indx]);
                    if(dataControl->torque_offset[i] >= 127){
                        dataControl->torque_offset[i] -= 256;
                    }
                    printf("%d, ", dataControl->torque_offset[i]);
                }
                printf("\n");
                break;
            }
            case 0x03:
            {
                int indx = 3;
                printf("receive recognition data : ");
                for(int i = 0; i < 2; i++, indx++){
                    dataControl->KITECHData.food_pos[i+4] = static_cast<int>(bufRecv[indx]);
                    if(dataControl->KITECHData.food_pos[i+4] >= 127){
                        dataControl->KITECHData.food_pos[i+4] -= 256;
                    }
                    printf("%d, ", dataControl->KITECHData.food_pos[i+4]);
                }
                printf("\n");
                break;
            }
            default:
                break;
        }
    }
}

void TcpServer::recvData(){
    if(bufRecv[0] == 'N' && bufRecv[1] == 'D'){
        dataControl->config_check = false;
        dataControl->RobotData.module_init = false;

        if(bufRecv[2] == NUM_JOINT && bufRecv[3] == NUM_DOF && bufRecv[4] == dataControl->MODULE_TYPE){
            printf("Client & Server configuration check complete\n");
            dataControl->RobotData.joint_op_mode = bufRecv[5];
            dataControl->config_check = true;
            dataControl->tablet_mode = false;

            printf("joint op mode : %d\n", dataControl->RobotData.joint_op_mode);
        }
        else{
            if(bufRecv[2] != NUM_JOINT){
                printf("Client & Server does not matched number of joints\n");
            }
            if(bufRecv[3] != NUM_DOF){
                printf("Client & Server does not matched degree of freedom\n");
            }
            if(bufRecv[4] != dataControl->MODULE_TYPE){
                printf("Client & Server does not matched module type\n");
            }
        }

        if(!dataControl->config_check){
            sendKey("X");
        }
    }
    else if(bufRecv[0] == 'N' && bufRecv[1] == 'S'){
        uint16_t indx = SOCKET_TOKEN_SIZE;

        dataControl->ClientToServer.opMode = bufRecv[indx];
        indx += OP_MODE_LEN;
        dataControl->ClientToServer.subMode = bufRecv[indx];
        indx += SUB_MODE_LEN;

        if(dataControl->ClientToServer.opMode == DataControl::JointMove){
            char buf[8];
            for(int i = 0; i < NUM_JOINT; i++, indx += DESIRED_JOINT_LEN){
                memcpy(buf, bufRecv + indx, DESIRED_JOINT_LEN);
                dataControl->ClientToServer.desiredJoint[i] = atof(buf);
            }
            for(int i = 0; i < NUM_DOF; i++){
                dataControl->ClientToServer.desiredPose[i] = 0;
            }

            rt_printf("%f %f %f %f %f %f %f\n",
                      dataControl->ClientToServer.desiredJoint[0],
                    dataControl->ClientToServer.desiredJoint[1],
                    dataControl->ClientToServer.desiredJoint[2],
                    dataControl->ClientToServer.desiredJoint[3],
                    dataControl->ClientToServer.desiredJoint[4],
                    dataControl->ClientToServer.desiredJoint[5],
                    dataControl->ClientToServer.desiredJoint[6]);
            dataControl->joint_path_index = 0;
        }
        else if(dataControl->ClientToServer.opMode == DataControl::CartesianMove){
            char buf[8];
            for(int i = 0; i < NUM_JOINT; i++){
                dataControl->ClientToServer.desiredJoint[i] = 0;
            }
            for(int i = 0; i < NUM_DOF; i++, indx += DESIRED_CARTESIAN_LEN){
                memcpy(buf, bufRecv + indx, DESIRED_CARTESIAN_LEN);
                dataControl->ClientToServer.desiredPose[i] = atof(buf);
            }
            memcpy(buf, bufRecv + indx, MOVE_TIME_LEN);
            dataControl->ClientToServer.move_time = atof(buf);
            indx += MOVE_TIME_LEN;
            memcpy(buf, bufRecv + indx, ACC_TIME_LEN);
            dataControl->ClientToServer.acc_time = atof(buf);
            indx += ACC_TIME_LEN;

            rt_printf("op mode : %d\n", dataControl->ClientToServer.opMode);
            rt_printf("sub mode : %d\n", dataControl->ClientToServer.subMode);

            for(int i = 0; i < 6; i++){
                rt_printf("des pose %d : %f\n", i, dataControl->ClientToServer.desiredPose[i]);
            }
            rt_printf("move_time : %f\n", dataControl->ClientToServer.move_time);
            rt_printf("acc_time  : %f\n", dataControl->ClientToServer.acc_time);
        }
    }
    else if(bufRecv[0] == 'N' && bufRecv[1] == 'U'){
        uint16_t indx = SOCKET_TOKEN_SIZE;

        dataControl->ClientToServer.opMode = static_cast<char>(bufRecv[indx]);
        indx += OP_MODE_LEN;
        dataControl->PathData.cmd_type = static_cast<char>(bufRecv[indx]);
        indx += CMD_TYPE_LEN;

        switch(dataControl->PathData.cmd_type){
            case DataControl::PathCmd:
            {
                dataControl->PathData.row = static_cast<char>(bufRecv[indx]);
                indx += ROW_SIZE_LEN;
                dataControl->PathData.col = static_cast<char>(bufRecv[indx]);
                indx += COL_SIZE_LEN;
                dataControl->PathData.total_time.clear();
                dataControl->PathData.point_px.clear();
                dataControl->PathData.point_py.clear();
                dataControl->PathData.point_pz.clear();
                dataControl->PathData.acc_time.clear();
                dataControl->PathData.point_rx.clear();
                dataControl->PathData.point_ry.clear();
                dataControl->PathData.point_rz.clear();
                dataControl->PathData.point_theta.clear();
                dataControl->PathData.movePath.resize(dataControl->PathData.row);

                char buf[8];
                double bufData = 0;
                for(int i = 0; i < dataControl->PathData.row; i++){
                    memcpy(buf, bufRecv + indx, MOVE_TIME_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.total_time.push_back(bufData);
                    indx += MOVE_TIME_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.point_px.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.point_py.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.point_pz.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.point_rx.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.point_ry.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.point_rz.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, ACC_TIME_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.acc_time.push_back(bufData);
                    indx += ACC_TIME_LEN;
                }
                rt_printf("received path data : \n");
                for(unsigned int i = 0; i < dataControl->PathData.row; i++){
                    rt_printf("%f %f %f %f %f %f %f %f\n",
                              dataControl->PathData.total_time[i], dataControl->PathData.point_px[i],
                              dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                              dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i],
                              dataControl->PathData.point_rz[i], dataControl->PathData.acc_time[i]);
                }

                break;
            }
            case DataControl::ReadyCmd:
            {
                dataControl->RobotData.run_mode = DataControl::ReadyCmd;
                dataControl->PathData.path_data_indx = 0;
                dataControl->PathData.path_struct_indx = 0;

                for(uint i = 0; i < dataControl->PathData.row; i++){
                    dataControl->PathData.movePath[i].path_x.clear();
                    dataControl->PathData.movePath[i].path_y.clear();
                    dataControl->PathData.movePath[i].path_z.clear();
                    dataControl->PathData.movePath[i].path_theta.clear();
                }
                dataControl->PathData.readyPath.path_x.clear();
                dataControl->PathData.readyPath.path_y.clear();
                dataControl->PathData.readyPath.path_z.clear();
                dataControl->PathData.readyPath.path_theta.clear();
                rt_printf("Ready Feeding Assitant Robot\n");
                break;
            }
            case DataControl::RunCmd:
            {
                dataControl->PathData.cycle_count_max = static_cast<signed char>(bufRecv[indx]);
                indx += CYCLE_COUNT_LEN;
                dataControl->RobotData.run_mode = DataControl::RunCmd;
                dataControl->PathData.path_data_indx = 0;
                dataControl->PathData.path_struct_indx = 0;
                dataControl->PathData.cycle_count = 1;
                rt_printf("Start Feeding Assitant Robot\n");
                break;
            }
            case DataControl::StopCmd:
            {
                dataControl->PathData.path_data_indx = 0;
                dataControl->PathData.path_struct_indx = 0;
                dataControl->ClientToServer.opMode = DataControl::Wait;
                break;
            }
        }
    }
    else if(bufRecv[0] == 'N' && bufRecv[1] == 'O'){
        rt_printf("%d\n", dataControl->feeding);
        if (!dataControl->feeding){
            rt_printf("Mode : %d, Sub Mode : %d, Section : %d\n", bufRecv[2], bufRecv[3], bufRecv[4]);

            if(bufRecv[2] == DataControl::OperateMode){
                switch(bufRecv[3]){
                    case DataControl::Start:
                    {
                        dataControl->ClientToServer.opMode = DataControl::OperateMode;
                        dataControl->operateMode.mode = DataControl::Start;

                        while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
                            usleep(1000);
                        }

                        dataControl->KITECHData.camera_request = true;

                        break;
                    }
                    case DataControl::StartTeaching:
                    {
                        dataControl->ClientToServer.opMode = DataControl::ServoOnOff;
                        dataControl->ClientToServer.subMode = 0;

                        usleep(1500000);
                        rt_printf("Servo off\n");

                        dataControl->ClientToServer.opMode = DataControl::OperateMode;
                        dataControl->operateMode.mode = DataControl::StartTeaching;

                        break;
                    }
                    case DataControl::StopTeaching:
                    {
                        dataControl->ClientToServer.opMode = DataControl::ServoOnOff;
                        dataControl->ClientToServer.subMode = 1;

                        usleep(1500000);
                        rt_printf("Servo on\n");

                        dataControl->ClientToServer.opMode = DataControl::OperateMode;
                        dataControl->operateMode.mode = DataControl::StopTeaching;
                        usleep(10000);
                        while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
                            usleep(1000);
                        }

                        dataControl->ClientToServer.opMode = DataControl::OperateMode;
                        dataControl->operateMode.mode = DataControl::Start;

                        rt_printf("Feeding ready complete\n");

                        while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
                            usleep(1000);
                        }

                        dataControl->KITECHData.camera_request = true;

                        break;
                    }
                    case DataControl::StartFeeding:
                    {
                        dataControl->ClientToServer.opMode = DataControl::OperateMode;
                        dataControl->operateMode.mode = DataControl::ReadyFeeding;

                        break;
                    }
                    case DataControl::Feeding:
                    {
                        rt_printf("Received Feeding Packet\n");
                        rt_printf("Section : %d\n", dataControl->KITECHData.interface_sub);
                        dataControl->section_indx = dataControl->KITECHData.interface_sub - 1;

                        dataControl->ClientToServer.opMode = DataControl::OperateMode;
                        dataControl->operateMode.mode = DataControl::ReadyFeeding;

                        while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
                            usleep(1000);
                        }

                        dataControl->ClientToServer.opMode = DataControl::OperateMode;
                        dataControl->operateMode.mode = DataControl::Feeding;
                        switch(dataControl->section_indx){
                            case 0: // side 1
                            {
                                dataControl->trayInfor.section1 = dataControl->trayInfor.section1_cnt%6;
                                rt_printf("section1 count : %d, %d\n", dataControl->trayInfor.section1_cnt, dataControl->trayInfor.section1);
                                dataControl->operateMode.section = DataControl::Side1;
                                break;
                            }
                            case 1: // side 2
                            {
                                dataControl->trayInfor.section2 = dataControl->trayInfor.section2_cnt%6;
                                rt_printf("section2 count : %d, %d\n", dataControl->trayInfor.section2_cnt, dataControl->trayInfor.section2);
                                dataControl->operateMode.section = DataControl::Side2;
                                break;
                            }
                            case 2: // side 3
                            {
                                dataControl->trayInfor.section3 = dataControl->trayInfor.section3_cnt%4;
                                rt_printf("section3 count : %d, %d\n", dataControl->trayInfor.section3_cnt, dataControl->trayInfor.section3);
                                dataControl->operateMode.section = DataControl::Side3;
                                break;
                            }
                            case 3: // soup
                            {
                                rt_printf("section4 count : %d\n", dataControl->trayInfor.section4);
                                dataControl->operateMode.section = DataControl::Soup;
                                break;
                            }
                            case 4: // rice
                            {
                                rt_printf("section5 count : %d, %d\n", dataControl->trayInfor.section5, dataControl->trayInfor.section5%9);
                                dataControl->operateMode.section = DataControl::Rice;
                                break;
                            }
                        }
                        break;
                    }
                }
            }
        }
    }
    else if(bufRecv[0] == 'N' && bufRecv[1] == 'T'){
        uint16_t indx = SOCKET_TOKEN_SIZE;

        char buf[8];
        memcpy(buf, bufRecv + indx, 8);
        dataControl->torque_const1 = atof(buf);
        indx += 8;
        memcpy(buf, bufRecv + indx, 8);
        dataControl->torque_const2 = atof(buf);
        indx += 8;

        rt_printf("receive torque const : %f, %f\n", dataControl->torque_const1, dataControl->torque_const2);
    }
    else if(bufRecv[0] == 'N' && bufRecv[1] == 'V'){
        uint16_t indx = SOCKET_TOKEN_SIZE;

        char buf[8];
        memcpy(buf, bufRecv + indx, 8);
        dataControl->RobotData.Kp = atof(buf);
        indx += 8;
        memcpy(buf, bufRecv + indx, 8);
        dataControl->RobotData.Dp = atof(buf);
        indx += 8;
        memcpy(buf, bufRecv + indx, 8);
        dataControl->RobotData.Kr = atof(buf);
        indx += 8;
        memcpy(buf, bufRecv + indx, 8);
        dataControl->RobotData.Dr = atof(buf);
        indx += 8;

        rt_printf("receive spring/damper const : %f, %f, %f, %f\n",
                  dataControl->RobotData.Kp, dataControl->RobotData.Dp, dataControl->RobotData.Kr, dataControl->RobotData.Dr);
    }    else if(bufRecv[0] == 'N' && bufRecv[1] == 'F'){
        uint16_t indx = SOCKET_TOKEN_SIZE;

        char buf[8];
        memcpy(buf, bufRecv + indx, 8);
        dataControl->RobotData.desiredForce = atof(buf);
        indx += 8;

        rt_printf("receive desired force : %f\n", dataControl->RobotData.desiredForce);
    }
}

void TcpServer::sendData(){
    //    if(port == 5050){
    //        rt_printf("server to client size : %d\n", dataControl->ServerToClient.size());
    if(dataControl->ServerToClient.size() > 0){
        unsigned int size = dataControl->ServerToClient.size();
        for(unsigned int i = 0; i < size; i++){
            uint16_t indx = 0;
            memset(bufSend, 0, SENDBUFSIZE);
            strcpy(bufSend, "NC");
            indx = SOCKET_TOKEN_SIZE;

            bufSend[indx] = dataControl->ServerToClient.front().data_index;
            indx += DATA_INDEX_LEN;
            //                rt_printf("Indx : %d\n", dataControl->ServerToClient.front().data_index);
            memcpy(bufSend + indx, QString::number(dataControl->ServerToClient.front().time).toStdString().c_str(), TIME_LEN);
            indx += TIME_LEN;
            memcpy(bufSend + indx, QString::number(dataControl->ServerToClient.front().module_time).toStdString().c_str(), TIME_LEN);
            indx += TIME_LEN;
            memcpy(bufSend + indx, QString::number(dataControl->ServerToClient.front().ik_time).toStdString().c_str(), TIME_LEN);
            indx += TIME_LEN;

            for(int i = 0; i < NUM_JOINT; i++){
                memcpy(bufSend + indx, QString::number(dataControl->ServerToClient.front().cur_status_word[i]).toStdString().c_str(), WORD_DATA_LEN);
                indx += WORD_DATA_LEN;
            }

            for(int i = 0; i < NUM_JOINT; i++){
                memcpy(bufSend + indx, QString::number((dataControl->ServerToClient.front().cur_position[i])*ENC2DEG).toStdString().c_str(), MOTION_DATA_LEN);
                indx += MOTION_DATA_LEN;
            }

            for(int i = 0; i < NUM_JOINT; i++){
                memcpy(bufSend + indx, QString::number(dataControl->ServerToClient.front().cur_velocity[i]).toStdString().c_str(), MOTION_DATA_LEN);
                indx += MOTION_DATA_LEN;
            }

            for(int i = 0; i < NUM_JOINT; i++){
                memcpy(bufSend + indx, QString::number(dataControl->ServerToClient.front().cur_torque[i]).toStdString().c_str(), MOTION_DATA_LEN);
                indx += MOTION_DATA_LEN;
            }

            for(int i = 0; i < NUM_JOINT; i++){
                memcpy(bufSend + indx, QString::number(dataControl->ServerToClient.front().tar_position[i]).toStdString().c_str(), MOTION_DATA_LEN);
                indx += MOTION_DATA_LEN;
            }

            for(int i = 0; i < NUM_JOINT; i++){
                memcpy(bufSend + indx, QString::number(dataControl->ServerToClient.front().tar_torque[i]).toStdString().c_str(), MOTION_DATA_LEN);
                indx += MOTION_DATA_LEN;
            }

            for(int i = 0; i < NUM_DOF; i++){
                memcpy(bufSend + indx, QString::number(dataControl->ServerToClient.front().presentCartesianPose[i]).toStdString().c_str(), MOTION_DATA_LEN);
                indx += MOTION_DATA_LEN;
            }

            //                for(int i = 0; i < NUM_DOF; i++){
            //                    memcpy(bufSend + indx, QString::number(dataControl->ServerToClient.front().presentCartesianVelocity[i]).toStdString().c_str(), MOTION_DATA_LEN);
            //                    indx += MOTION_DATA_LEN;
            //                }

            //                for(int j = 0; j < NUM_JOINT; j++){
            //                    memcpy(bufSend + indx, to_string(dataControl->ServerToClient.front().presentJointTorque[j]).c_str(), MOTION_DATA_LEN);
            //                    indx += MOTION_DATA_LEN;
            //                }
            //                for(int j = 0; j < NUM_JOINT; j++){
            //                    memcpy(bufSend + indx, to_string(dataControl->ServerToClient.front().presentJointResidual[j]).c_str(), MOTION_DATA_LEN);
            //                    indx += MOTION_DATA_LEN;
            //                }
            bufSend[indx] = dataControl->ServerToClient.front().opMode;
            indx++;

            strcpy(bufSend + indx, "NE");
            indx += SOCKET_TOKEN_SIZE;

            sendByteLen = send(clientSockFD, bufSend, static_cast<size_t>(indx), 0);
            //                rt_printf("sendByteLen : %d\n", sendByteLen);
            //                rt_printf("Send data : ");
            //                for(int i = 0; i < sendByteLen; i++){
            //                    rt_printf("%d ", bufSend[i]);
            //                }
            //                rt_printf("\n");

            dataControl->ServerToClient.erase(dataControl->ServerToClient.begin());
            //            dataControl->ServerToClient.clear();

            if(sendByteLen < 0){
                rt_printf("Send Error\n");
                comm_manager_run = false;
                connected = false;
                dataControl->config_check = false;
                dataControl->ServerToClient.clear();

                return;
            }
        }
    }
}

void TcpServer::recvDataPath()
{
    if(bufRecv[0] == SOP_PATH_RX){
        switch(bufRecv[1]){
            case Init:
            {
                rt_printf("Client & Server configuration check complete\n");
                dataControl->config_check = true;
                dataControl->tablet_mode = true;

                break;
            }
            case Start:
            {
                dataControl->key_value = KEY_Z;

                break;
            }
            case Apply_size:
            {
                uint16_t indx = 2;

                dataControl->PathData.row = static_cast<char>(bufRecv[indx]);
                indx += ROW_SIZE_LEN;
                dataControl->PathData.col = static_cast<char>(bufRecv[indx]);
                indx += COL_SIZE_LEN;

                dataControl->PathData.comm_data.clear();

                rt_printf("received path data size : %d, %d\n", dataControl->PathData.row, dataControl->PathData.col);

                dataControl->PathData.row++;
                break;
            }
            case Apply_path:
            {
                uint16_t indx = 2;

                uint16_t row = static_cast<char>(bufRecv[indx]);
                indx += ROW_SIZE_LEN;

                char buf[8];
                double bufData = 0;
                for(int i = 0; i < row; i++){
                    memcpy(buf, bufRecv + indx, MOVE_TIME_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.comm_data.push_back(bufData);
                    indx += MOVE_TIME_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.comm_data.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.comm_data.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.comm_data.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.comm_data.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.comm_data.push_back(bufData);
                    indx += PATH_DATA_LEN;
                }

                rt_printf("received path data size : %d\n", row);
                if(row == 8){
                    rt_printf("received path data : \n");
                    for(unsigned int i = 0; i < dataControl->PathData.row-1; i++){
                        rt_printf("%d %f %f %f %f %f %f\n", i,
                                  dataControl->PathData.comm_data[i*dataControl->PathData.col + 0],
                                dataControl->PathData.comm_data[i*dataControl->PathData.col + 1],
                                dataControl->PathData.comm_data[i*dataControl->PathData.col + 2],
                                dataControl->PathData.comm_data[i*dataControl->PathData.col + 3],
                                dataControl->PathData.comm_data[i*dataControl->PathData.col + 4],
                                dataControl->PathData.comm_data[i*dataControl->PathData.col + 5]);
                    }
                }
                break;
            }
            case Run:
            {
                if(dataControl->PathData.comm_data.size() > 0){
                    dataControl->key_value = KEY_C;
                }
                break;
            }
            case Stop:
            {
                dataControl->PathData.path_data_indx = 0;
                dataControl->PathData.path_struct_indx = 0;
                dataControl->ClientToServer.opMode = DataControl::Wait;
                break;
            }
            case Offset:
            {
                uint16_t indx = 2;
                char buf[8];
                double bufData = 0;

                memcpy(buf, bufRecv + indx, MOVE_TIME_LEN);
                bufData = atof(buf);
                //                dataControl->PathData.comm_data.push_back(bufData);
                dataControl->PathData.offset[0] = bufData;
                indx += MOVE_TIME_LEN;

                memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                bufData = atof(buf);
                //                dataControl->PathData.comm_data.push_back(bufData);
                dataControl->PathData.offset[1] = bufData;
                indx += PATH_DATA_LEN;

                rt_printf("received offset data : %f, %f\n", dataControl->PathData.offset[0], dataControl->PathData.offset[1]);

                break;
            }
            default:
            {
                break;
            }
        }
    }
}

void TcpServer::sendDataPath()
{
    //    rt_printf("%d\n", dataControl->ServerToClient.size());
    if(dataControl->ServerToClient.size() > 0){
        unsigned int size = dataControl->ServerToClient.size();
        for(unsigned int i = 0; i < size; i++){
            uint16_t indx = 0;
            memset(bufSend, 0, SENDBUFSIZE);

            bufSend[0] = SOP_PATH_TX;
            indx += 1;

            bufSend[indx] = dataControl->ServerToClient.front().indx;
            indx += DATA_INDEX_LEN;

            for(int i = 0; i < NUM_JOINT; i++){
                memcpy(bufSend + indx, QString::number((dataControl->ServerToClient.front().cur_position[i])*ENC2DEG).toStdString().c_str(), MOTION_DATA_LEN);
                indx += MOTION_DATA_LEN;
            }

            for(int i = 0; i < NUM_JOINT; i++){
                memcpy(bufSend + indx, QString::number((dataControl->ServerToClient.front().tar_position[i])*ENC2DEG).toStdString().c_str(), MOTION_DATA_LEN);
                indx += MOTION_DATA_LEN;
            }

            for(int i = 0; i < 3; i++){
                memcpy(bufSend + indx, QString::number(dataControl->ServerToClient.front().presentCartesianPose[i]).toStdString().c_str(), MOTION_DATA_LEN);
                indx += MOTION_DATA_LEN;
            }

            bufSend[indx] = dataControl->ServerToClient.front().opMode;
            indx++;

            bufSend[indx] = EOP_PATH;
            indx += 1;

            sendByteLen = send(clientSockFD, bufSend, static_cast<size_t>(indx), 0);

            dataControl->ServerToClient.erase(dataControl->ServerToClient.begin());
            dataControl->ServerToClient.clear();
        }

        //        rt_printf("sendByteLen : %d\n", sendByteLen);
        //        rt_printf("Send data : ");
        //        for(int i = 0; i < sendByteLen; i++){
        //            rt_printf("%d ", bufSend[i]);
        //        }
        //        rt_printf("\n");

        if(sendByteLen < 0){
            rt_printf("Send Error\n");
            comm_manager_run = false;
            connected = false;
            dataControl->config_check = false;
        }
    }
}

void TcpServer::initSocket()
{
    listenSockFD = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(listenSockFD < 0){
        cout << endl << "socket create error" << endl;
        return;
    }

    int on = 1;
    if(setsockopt(listenSockFD, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&on), sizeof(on)) < 0){
        cout << endl << "set option curLen = 0; error!!" << endl;
        return;
    }
    if(setsockopt(listenSockFD, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&on), sizeof(on)) < 0){
        cout << endl << "set option curLen = 0; error!!" << endl;
        return;
    }

    // server_addr.sin_addr.s_addr = inet_addr("192.168.0.100");
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);

    cout << "server binded" << endl;
    cout << endl << "address : " << inet_ntoa(server_addr.sin_addr) << endl;
    cout << "port : " << ntohs(server_addr.sin_port) << endl << endl;

    if(bind(listenSockFD, reinterpret_cast<struct sockaddr*>(&server_addr), sizeof(server_addr)) < 0){
        cout << endl << "bind error" << endl;
        return;
    }
}

void TcpServer::connectSocket()
{
    cout << "server running waiting. waiting client..." << endl;

    if(listen(listenSockFD, MAXCONNECTIONS) < 0){
        cout << endl << "listen error" << endl;
    }
    int clientAddrSize = sizeof(client_addr);

    curLen = 0;
    memset(bufWait, 0, MAXWAITBUFSIZE);
    ptrRecvBufIndx = bufWait;
    clientSockFD = accept(listenSockFD, reinterpret_cast<struct sockaddr*>(&client_addr), reinterpret_cast<socklen_t*>(&clientAddrSize));

    if(clientSockFD < 0){
        cout << endl << "accept error" << endl;
    }

    cout << "(" << port << ") " << "client accepted" << endl;
    cout << "(" << port << ") " << "address : " << inet_ntoa(client_addr.sin_addr) << endl;
    cout << "(" << port << ") " << "port : " << ntohs(client_addr.sin_port) << endl;

    connected = true;
}

void TcpServer::sendKey(const char* key){
    memset(bufSend, 0, MAXSENDBUFSIZE);
    strcpy(bufSend, key);
    len = 1;
    sendByteLen = send(clientSockFD, bufSend, static_cast<size_t>(len), 0);
    rt_printf("sendByteLen : %ld\n", sendByteLen);
    if(sendByteLen < 0){
        rt_printf("Send Error\n");
        comm_manager_run = false;
        connected = false;
    }
}
