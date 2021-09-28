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
    if(port == 5050){
        if(recv_comm_run){
            pthread_cancel(recv_comm_thread);
            usleep(10000);
            rt_printf("Finished Comm Rx Task\n");
        }
        else{
            rt_printf("Comm Rx Thread not running...\n");
        }

        if(send_comm_run){
            pthread_cancel(send_comm_thread);
            usleep(10000);
            rt_printf("Finished Comm Tx Task\n");
        }
        else{
            rt_printf("Comm Tx Thread not running...\n");
        }
    }
    else if(port == 5053){
        if(recv_latte_comm_run){
            pthread_cancel(recv_latte_comm_thread);
            usleep(10000);
            rt_printf("Finished LATTE Comm Rx Task\n");
        }
        else{
            rt_printf("Comm Latte Rx Thread not running...\n");
        }

        if(send_latte_comm_run){
            pthread_cancel(send_latte_comm_thread);
            usleep(10000);
            rt_printf("Finished LATTE Comm Tx Task\n");
        }
        else{
            rt_printf("Comm Latte Tx Thread not running...\n");
        }
    }
    else if(port == 5059){
        if(dio_comm_run){
            pthread_cancel(dio_comm_thread);
            usleep(10000);
            rt_printf("Finished DIO Comm Thread\n");
        }
        else{
            rt_printf("Comm DIO Thread not running...\n");
        }
    }

    pthread_cancel(comm_manager_thread);
    usleep(10000);
    rt_printf("Finished Comm Manager Thread\n");
}

void *TcpServer::comm_manager_func(void *arg)
{
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->comm_manager_run = true;

    pTcpServer->initSocket();

    if(pTcpServer->port == 5050){
        while(pTcpServer->comm_manager_run){
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
        }
    }
    else if(pTcpServer->port == 5053){
        while(pTcpServer->comm_manager_run){
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
        }
    }
    else if(pTcpServer->port == 5059){
        while(pTcpServer->comm_manager_run){
            pTcpServer->connectSocket();

            pthread_create(&pTcpServer->dio_comm_thread, NULL, pTcpServer->dio_comm_func, pTcpServer);

            pthread_join(pTcpServer->dio_comm_thread, NULL);
            usleep(10000);

            pthread_cancel(pTcpServer->dio_comm_thread);
            usleep(10000);
        }
    }
    return NULL;
}

void *TcpServer::dio_comm_func(void *arg){
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->dio_comm_run = true;

    while(pTcpServer->dio_comm_run){
        memset(pTcpServer->bufRecv, 0, MAXRECEIVEBUFSIZE);
        pTcpServer->byteLen = recv(pTcpServer->clientSockFD, pTcpServer->bufRecv, MAXRECEIVEBUFSIZE, 0);

        if(pTcpServer->byteLen > 0){
//            rt_printf("[DIO] Received byteLen : %ld\n", pTcpServer->byteLen);
//            rt_printf("[DIO] Received data : ");
//            for(uint8_t i = 0; i < pTcpServer->byteLen; i++){
//                rt_printf("%d ", pTcpServer->bufRecv[i]);
//            }
//            rt_printf("\n\n");

            pTcpServer->dataControl->key_value = KEY_ESC;
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
                    while(pTcpServer->dataControl->key_value == KEY_Z){
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

void TcpServer::recvDataLatte(){
    if(bufRecv[0] == SOP_RX && dataControl->ClientToServer.opMode == DataControl::Wait/* && !dataControl->feeding*//* && (buf[3] == EOP || buf[22] == EOP)*/){
        dataControl->KITECHData.camera_request = false;
        dataControl->KITECHData.interface_cmd = bufRecv[1];
        dataControl->KITECHData.interface_sub = bufRecv[2];

        switch(dataControl->KITECHData.interface_cmd){
            case CMD_DATA:
            {
                rt_printf("Received Data Packet\n");

//                for(int i = 0; i < 20; i++){
//                    dataControl->KITECHData.food_pixel[i] = (signed char)bufRecv[i + 2];
//                }

//                for(int i = 0, j = 0; i < 10; i++, j+=2){
//                    dataControl->KITECHData.food_pos[i] = (int)dataControl->KITECHData.food_pixel[j]*100 + (int)dataControl->KITECHData.food_pixel[j+1];
//                }

//                rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[0], dataControl->KITECHData.food_pos[1]);
//                rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[2], dataControl->KITECHData.food_pos[3]);
//                rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[4], dataControl->KITECHData.food_pos[5]);
//                rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[6], dataControl->KITECHData.food_pos[7]);
//                rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[8], dataControl->KITECHData.food_pos[9]);

//                for(int i = 0; i < 10; i++){
//                    if(abs(dataControl->KITECHData.food_pos[i]) < 10 || abs(dataControl->KITECHData.food_pos[i]) > 200){
//                        rt_printf("Re-Send to KITECH request camera\n");
//                        dataControl->KITECHData.camera_request = true;
//                        usleep(3000000);
//                        dataControl->KITECHData.camera_request = false;
//                        break;
//                    }
//                }
                break;
            }
            case CMD_SECTION:
            {

                dataControl->ClientToServer.opMode = DataControl::OperateMode;
                dataControl->operateMode.mode = DataControl::ReadyFeeding;


                while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
                    usleep(1000);
                }

                // choose section
                if(dataControl->KITECHData.interface_sub == 1){
                    rt_printf("Received Section Packet\n");
                    dataControl->select_indx++;
                    if(dataControl->select_indx >= 6) {
                        dataControl->select_indx = 1;
                    }
                    if(dataControl->select_indx == 3){
                        dataControl->select_indx = 4;
                    }
                    rt_printf("Section Index : %d\n", dataControl->select_indx);
                    dataControl->operateMode.section = dataControl->select_indx;
                    dataControl->section_indx = dataControl->select_indx - 1;

                    dataControl->ClientToServer.opMode = DataControl::OperateMode;
                    dataControl->operateMode.mode = DataControl::FeedingSwitch;
                }
                else if(dataControl->KITECHData.interface_sub == 2){
                    rt_printf("Received Feeding Packet\n");
                    rt_printf("Section : %d\n", dataControl->section_indx);
//                    dataControl->operateMode.section = dataControl->section_indx + 1;

                    dataControl->ClientToServer.opMode = DataControl::OperateMode;
                    dataControl->operateMode.mode = DataControl::Feeding;

                    switch(dataControl->section_indx){
                        case 0: // side 1
                        {
                            dataControl->trayInfor.section1 = dataControl->trayInfor.section1_cnt%4;
                            rt_printf("section1 count : %d, %d\n", dataControl->trayInfor.section1_cnt, dataControl->trayInfor.section1);
                            dataControl->operateMode.section = DataControl::Side1;
                            break;
                        }
                        case 1: // side 2
                        {
                            dataControl->trayInfor.section2 = dataControl->trayInfor.section2_cnt%2;
                            rt_printf("section2 count : %d, %d\n", dataControl->trayInfor.section2_cnt, dataControl->trayInfor.section2);
                            dataControl->operateMode.section = DataControl::Side2;
                            break;
                        }
                        case 2: // side 3
                        {
//                            dataControl->trayInfor.section3 = dataControl->trayInfor.section3_cnt%4;
//                            rt_printf("section3 count : %d, %d\n", dataControl->trayInfor.section3_cnt, dataControl->trayInfor.section3);
//                            dataControl->operateMode.section = DataControl::Side3;
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
            case CMD_FEEDING:
            {
                rt_printf("Received Feeding Packet\n");
                rt_printf("Section : %d\n", dataControl->KITECHData.interface_sub);
                dataControl->section_indx = dataControl->KITECHData.interface_sub - 1;

                if(dataControl->section_indx != 2){
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
                            dataControl->trayInfor.section1 = dataControl->trayInfor.section1_cnt%4;
                            rt_printf("section1 count : %d, %d\n", dataControl->trayInfor.section1_cnt, dataControl->trayInfor.section1);
                            dataControl->operateMode.section = DataControl::Side1;
                            break;
                        }
                        case 1: // side 2
                        {
                            dataControl->trayInfor.section2 = dataControl->trayInfor.section2_cnt%2;
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
                }
                else{
                    dataControl->KITECHData.tablet_connect = false;
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

void TcpServer::sendDataLatte(){
//    if(dataControl->KITECHData.camera_request && (dataControl->KITECHData.camera_request_old != dataControl->KITECHData.camera_request)){
//        memset(bufSend, 0, SENDBUFSIZE);

//        bufSend[0] = SOP_TX;
//        bufSend[1] = 1;
//        bufSend[2] = EOP;
//        sendByteLen = send(clientSockFD, bufSend, 3, 0);
//        rt_printf("Send to KITECH request camera\n");

//        if(sendByteLen < 0){
//            rt_printf("Send Error\n");
//            comm_manager_run = false;
//            connected = false;

//            return;
//        }
//    }
//    dataControl->KITECHData.camera_request_old = dataControl->KITECHData.camera_request;

    if(dataControl->KITECHData.tablet_check && (dataControl->KITECHData.tablet_check_old != dataControl->KITECHData.tablet_check)){
        memset(bufSend, 0, SENDBUFSIZE);

        bufSend[0] = SOP_TX;
        bufSend[1] = 2;
        bufSend[2] = EOP;
        sendByteLen = send(clientSockFD, bufSend, 3, 0);
        rt_printf("Send to KITECH tablet connect check\n");

        if(sendByteLen < 0){
            rt_printf("Send Error\n");
            comm_manager_run = false;
            connected = false;

            return;
        }
    }
    dataControl->KITECHData.tablet_check_old = dataControl->KITECHData.tablet_check;
}

void TcpServer::recvData(){
    if(bufRecv[0] == 'N' && bufRecv[1] == 'D'){
        dataControl->config_check = false;
        dataControl->RobotData.module_init = false;

        if(bufRecv[2] == NUM_JOINT && bufRecv[3] == NUM_DOF && bufRecv[4] == dataControl->MODULE_TYPE){
            rt_printf("Client & Server configuration check complete\n");
            dataControl->RobotData.joint_op_mode = bufRecv[5];
            dataControl->config_check = true;
            dataControl->tablet_mode = false;
        }
        else{
            if(bufRecv[2] != NUM_JOINT){
                rt_printf("Client & Server does not matched number of joints\n");
            }
            if(bufRecv[3] != NUM_DOF){
                rt_printf("Client & Server does not matched degree of freedom\n");
            }
            if(bufRecv[4] != dataControl->MODULE_TYPE){
                rt_printf("Client & Server does not matched module type\n");
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
                dataControl->PathData.cycle_count_max = static_cast<char>(bufRecv[indx]);
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

//                        while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
//                            usleep(1000);
//                        }

//                        dataControl->KITECHData.camera_request = true;

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

//                        while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
//                            usleep(1000);
//                        }

//                        dataControl->KITECHData.camera_request = true;

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
                                dataControl->trayInfor.section1 = dataControl->trayInfor.section1_cnt%4;
                                rt_printf("section1 count : %d, %d\n", dataControl->trayInfor.section1_cnt, dataControl->trayInfor.section1);
                                dataControl->operateMode.section = DataControl::Side1;
                                break;
                            }
                            case 1: // side 2
                            {
                                dataControl->trayInfor.section2 = dataControl->trayInfor.section2_cnt%4;
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

    cout << "client accepted" << endl;
    cout << "address : " << inet_ntoa(client_addr.sin_addr) << endl;
    cout << "port : " << ntohs(client_addr.sin_port) << endl;

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
