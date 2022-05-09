#include "controlmain.h"

ControlMain::ControlMain()
{
    dataControl = new DataControl();

    tcpServer = new TcpServer(dataControl);
    tcpServerLatte = new TcpServer(dataControl);
    tcpServerDIO = new TcpServer(dataControl);
    tcpServerTemp = new TcpServer(dataControl);
    tcpServerPath = new TcpServer(dataControl);
    tcpServerDiningInfor = new TcpServer(dataControl);
    tcpServerKey = new TcpServer(dataControl);
    tcpServerOffset = new TcpServer(dataControl);

    dataControl->RobotData.module_init = false;

    data_indx = 0;
    module_indx = 0;
    step_size = 0.001;

    robotArm = new RobotArm(6, NUM_DOF, step_size, dataControl->MODULE_TYPE);
    robotArm->set_tool_offset(dataControl->tool_offset);

    delay_cnt_max = 100;
    delay_cnt = 0;
    dataControl->feeding = false;

    print_cnt = 0;

    init_thread_run = false;
    key_input_thread_run = false;
    logging_thread_run = false;

    robot_task_run = false;

    ecatInterface = new userinterface();

    servo = false;
    jog_command = false;

    time_check_delay_cnt = 0;
    time_check_delay_max = 10000;

    cycle_count = 0;
}

ControlMain::~ControlMain()
{
    printf("finishing...\n");

    if(init_thread_run){
        init_thread_run = false;
        pthread_cancel(init_thread);
        printf("Finished Module Init Thread\n");
        usleep(10000);
    }

    if(key_input_thread_run){
        key_input_thread_run = false;
        pthread_cancel(key_input_thread);
        printf("Finished Key Input Task\n");
        usleep(10000);
    }

    robot_rt_stop();
    usleep(10000);

    if(logging_thread_run){
        logging_thread_run = false;
        usleep(10000);
        pthread_cancel(logging_thread);
        usleep(10000);
    }

    delete tcpServer;
    usleep(10000);
    printf("Complete destructure TcpServer\n");

    delete tcpServerLatte;
    usleep(10000);
    printf("Complete destructure TcpServerLatte\n");

    delete tcpServerDIO;
    usleep(10000);
    printf("Complete destructure TcpServerDIO\n");

    delete tcpServerTemp;
    usleep(10000);
    printf("Complete destructure TcpServerTemp\n");

    delete tcpServerPath;
    usleep(10000);
    printf("Complete destructure TcpServerPath\n");

    delete tcpServerDiningInfor;
    usleep(10000);
    printf("Complete destructure TcpServerDiningInfor\n");

    delete tcpServerKey;
    usleep(10000);
    printf("Complete destructure TcpServerKey\n");

    delete tcpServerOffset;
    usleep(10000);
    printf("Complete destructure TcpServerOffset\n");

    delete robotArm;
    printf("Complete destructure RobotArm\n");

    usleep(10000);
    if (dataControl->RobotData.module_init){
        ecatInterface->stop();
        dataControl->RobotData.module_init = false;
    }

    //    delete dataControl;
    //    printf("Complete destructure DataControl\n");

    delete ecatInterface;
    printf("Complete destructure ECAT Interface\n");

    printf("Finished\n");
}

void ControlMain::start(){
    printf("Start RobotController\n");

    pthread_create(&key_input_thread, NULL, key_input_func, this);
    usleep(10000);

    if(dataControl->logging_enable){
        pthread_create(&logging_thread, NULL, logging_func, this);
        usleep(10000);
    }

    tcpServer->setting(5050);
    tcpServer->start();
    usleep(10000);

    tcpServerLatte->setting(5053);
    tcpServerLatte->start();
    usleep(10000);

    tcpServerDIO->setting(5059);
    tcpServerDIO->start();
    usleep(1000);

    //    tcpServerTemp->setting(5051);
    //    tcpServerTemp->start();
    //    usleep(1000);

    //    tcpServerPath->setting(5052);
    //    tcpServerPath->start();
    //    usleep(1000);

    tcpServerDiningInfor->setting(5054);
    tcpServerDiningInfor->start();
    usleep(1000);

    tcpServerKey->setting(5058);
    tcpServerKey->start();
    usleep(1000);

    tcpServerOffset->setting(5056);
    tcpServerOffset->start();
    usleep(1000);

    init_thread_run = true;
    pthread_create(&init_thread, NULL, init_func, this);

    //    usleep(1000000);
    //    dataControl->tablet_mode = true;
    //    dataControl->config_check = true;
    //    while(!dataControl->ClientToServer.opMode){
    //        usleep(1000);
    //    }

    //    while(!robot_task_run){
    //        usleep(1000);
    //    }
    //    printf("Tablet mode init finished\n");

    //    dataControl->ClientToServer.opMode = DataControl::ServoOnOff;
    //    dataControl->ClientToServer.subMode = 1;
    //    while(!(dataControl->ClientToServer.opMode == DataControl::Wait)){
    //        usleep(1000);
    //    }
    //    printf("Servo on\n");
}

void* ControlMain::init_func(void *arg){
    ControlMain *pThis = static_cast<ControlMain*>(arg);

    while(!pThis->dataControl->config_check){
        usleep(10000);
    }

    printf("Start ecat module initialize\n");
    pThis->ecatInterface->init();
    printf("End ecat module initialize\n");

    while(pThis->init_thread_run){
        if(!pThis->dataControl->RobotData.module_init){
            pThis->moduleInit();
        }
        else{
            pThis->init_thread_run = false;
            break;
        }
        usleep(1000);
    }

    pThis->ecatInterface->servo_mode(pThis->dataControl->RobotData.joint_op_mode);

    printf("finished init thread\n");

    return NULL;
}

void ControlMain::robot_rt_start(){
    rt_print_auto_init(1);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    int ret = rt_task_create(&robot_task, "Robot Controll Task", 0, 90, 0);
    if (ret < 0){
        printf("Failed to create Robot Control Task %s\n", strerror(-ret));
    }
    else{
        printf("Robot Control RT Task Start\n");

        dataControl->ClientToServer.opMode = DataControl::Wait;

        rt_task_start(&robot_task, &robot_RT, this);
    }
}

void ControlMain::robot_rt_stop(){
    if (robot_task_run){
        robot_task_run = false;

        rt_task_suspend(&robot_task);
        printf("Robot RT Task Stop\n");
        rt_task_delete(&robot_task);
        printf("Robot RT Task Delete\n");
    }
    else{
        printf("Robot Control RT Thread not running...\n");
    }
}

void *ControlMain::key_input_func(void *arg){
    ControlMain *pControlMain = static_cast<ControlMain*>(arg);

    pControlMain->key_input_thread_run = true;

    unsigned int led_cnt = 0;

    while(pControlMain->key_input_thread_run){

        led_cnt++;
        usleep(1000);

        if(pControlMain->tcpServerDIO->dio_comm_run && pControlMain->tcpServerLatte->recv_latte_comm_run && pControlMain->tcpServerLatte->send_latte_comm_run && !pControlMain->dataControl->config_check){
//        if(pControlMain->tcpServerDIO->dio_comm_run && !pControlMain->dataControl->config_check){
            //        if(!pControlMain->dataControl->config_check){
            pControlMain->dataControl->tablet_mode = true;
            pControlMain->dataControl->config_check = true;
            pControlMain->dataControl->RobotData.joint_op_mode = 3;

            while(pControlMain->init_thread_run){
                usleep(1000);
            }

            while(!pControlMain->robot_task_run){
                usleep(1000);
            }

            printf("Tablet mode init finished\n");

//            pControlMain->dataControl->key_value = KEY_Z; // delete
        }

        if(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait && !pControlMain->dataControl->KITECHData.tablet_connect){
            if(led_cnt <= 10000){
                pControlMain->dataControl->KITECHData.tablet_check = true;
            }
            else if(led_cnt > 10000 && led_cnt <= 20000){
                pControlMain->dataControl->KITECHData.tablet_check = false;
            }
            else{
                led_cnt = 0;
            }
        }

        if(pControlMain->dataControl->key_value == KEY_V){
            pControlMain->dataControl->tablet_mode = true;
            pControlMain->dataControl->config_check = true;
            pControlMain->dataControl->RobotData.joint_op_mode = 3;

            while(pControlMain->init_thread_run){
                usleep(1000);
            }

            while(!pControlMain->robot_task_run){
                usleep(1000);
            }

            printf("Custom mode init finished\n");
        }
        else if(pControlMain->dataControl->key_value == KEY_Z){
            printf("Pressed key Z\n");

            pControlMain->ecatInterface->set_led(3);

            if(!pControlMain->dataControl->robotStart){
                rt_printf("Start\n");

                pControlMain->dataControl->ClientToServer.opMode = DataControl::ServoOnOff;
                pControlMain->dataControl->ClientToServer.subMode = 1;

                while(!((pControlMain->ecatInterface->v_global_ethercat_control.f_get_slave()[0].v_global_cur_status_word == 4151) &&
                        (pControlMain->ecatInterface->v_global_ethercat_control.f_get_slave()[1].v_global_cur_status_word == 4151) &&
                        (pControlMain->ecatInterface->v_global_ethercat_control.f_get_slave()[2].v_global_cur_status_word == 4151) &&
                        (pControlMain->ecatInterface->v_global_ethercat_control.f_get_slave()[3].v_global_cur_status_word == 4151) &&
                        (pControlMain->ecatInterface->v_global_ethercat_control.f_get_slave()[4].v_global_cur_status_word == 4151) &&
                        (pControlMain->ecatInterface->v_global_ethercat_control.f_get_slave()[5].v_global_cur_status_word == 4151) &&
                        (pControlMain->ecatInterface->v_global_ethercat_control.f_get_slave()[6].v_global_cur_status_word == 4151)))
                {
                    usleep(1000);
                }

                while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                    usleep(1000);
                }

                rt_printf("Servo on\n");

                for(int i = 0; i < NUM_JOINT; i++){
                    pControlMain->dataControl->ClientToServer.desiredJoint[i] = pControlMain->dataControl->RobotData.cur_position[i]*ENC2DEG;
                }
                pControlMain->dataControl->ClientToServer.desiredJoint[3] = initJoint3Deg;

                pControlMain->dataControl->ClientToServer.opMode = DataControl::JointMove;
                pControlMain->dataControl->ClientToServer.subMode = DataControl::JointMotion;
                pControlMain->dataControl->joint_path_index = 0;

                while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                    usleep(1000);
                }

                for(int i = 0; i < NUM_JOINT; i++){
                    pControlMain->dataControl->ClientToServer.desiredJoint[i] = pControlMain->dataControl->RobotData.cur_position[i]*ENC2DEG;
                }
                pControlMain->dataControl->ClientToServer.desiredJoint[1] = initJoint1Deg;

                pControlMain->dataControl->ClientToServer.opMode = DataControl::JointMove;
                pControlMain->dataControl->ClientToServer.subMode = DataControl::JointMotion;
                pControlMain->dataControl->joint_path_index = 0;

                while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                    usleep(1000);
                }

                pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
                pControlMain->dataControl->operateMode.mode = DataControl::Start;

                while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                    usleep(1000);
                }
                printf("Feeding start\n");
                pControlMain->dataControl->section_indx = 0;
                pControlMain->dataControl->select_indx = 0;

                pControlMain->dataControl->key_value = 0;
                pControlMain->dataControl->robotStart = true;

                ///////////////////////
                //            pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
                //            pControlMain->dataControl->operateMode.mode = DataControl::ReadyFeeding;

                //            while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                //                usleep(1000);
                //            }

                //            pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
                //            pControlMain->dataControl->operateMode.mode = DataControl::Feeding;
                //            pControlMain->dataControl->operateMode.section = DataControl::Test;
                ///////////////////////
            }
            else{
                rt_printf("Finish\n");

                time_t timer = time(NULL);
                struct tm* t = localtime(&timer);
                int year = t->tm_year + 1900;
                pControlMain->dataControl->diningInfor.dining_time_infor[0] = static_cast<char>(year/1000);
                pControlMain->dataControl->diningInfor.dining_time_infor[1] = (year%1000)/100;
                pControlMain->dataControl->diningInfor.dining_time_infor[2] = (year%100)/10;
                pControlMain->dataControl->diningInfor.dining_time_infor[3] = (year%100)%10;
                pControlMain->dataControl->diningInfor.dining_time_infor[4] = static_cast<char>((t->tm_mon + 1)/10);
                pControlMain->dataControl->diningInfor.dining_time_infor[5] = (t->tm_mon + 1)%10;
                pControlMain->dataControl->diningInfor.dining_time_infor[6] = static_cast<char>(t->tm_mday/10);
                pControlMain->dataControl->diningInfor.dining_time_infor[7] = t->tm_mday%10;
                pControlMain->dataControl->diningInfor.dining_time_infor[8] = static_cast<char>(t->tm_hour/10);
                pControlMain->dataControl->diningInfor.dining_time_infor[9] = t->tm_hour%10;
                pControlMain->dataControl->diningInfor.dining_time_infor[10] = static_cast<char>(t->tm_min/10);
                pControlMain->dataControl->diningInfor.dining_time_infor[11] = t->tm_min%10;
                pControlMain->dataControl->diningInfor.dining_step = 3;

                pControlMain->dataControl->diningInfor.dining_time_send = true;

                if(!pControlMain->dataControl->KITECHData.wait_feeding){
                    pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
                    pControlMain->dataControl->operateMode.mode = DataControl::WaitFeeding1;

                    while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                        usleep(1000);
                    }

                    pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
                    pControlMain->dataControl->operateMode.mode = DataControl::WaitFeeding2;

                    while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                        usleep(1000);
                    }
                }

                pControlMain->dataControl->ClientToServer.opMode = DataControl::ServoOnOff;
                pControlMain->dataControl->ClientToServer.subMode = 0;

                while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                    usleep(1000);
                }

                usleep(100000);
                pControlMain->dataControl->DataReset();

                pControlMain->dataControl->key_value = 0;
                pControlMain->dataControl->robotStart = false;
                pControlMain->ecatInterface->set_led(3);

                usleep(5000000);
            }
        }
        else if(pControlMain->dataControl->key_value == KEY_X){
            printf("Pressed key X\n");
            if(pControlMain->dataControl->operateMode.mode == DataControl::WaitFeeding2){
                pControlMain->dataControl->operateMode.mode = DataControl::StartTeaching;
                pControlMain->ecatInterface->set_led(1);
                usleep(100);

                pControlMain->dataControl->ClientToServer.opMode = DataControl::ServoOnOff;
                pControlMain->dataControl->ClientToServer.subMode = 1;

                while(!((pControlMain->ecatInterface->v_global_ethercat_control.f_get_slave()[0].v_global_cur_status_word == 4151) &&
                      (pControlMain->ecatInterface->v_global_ethercat_control.f_get_slave()[1].v_global_cur_status_word == 4151) &&
                      (pControlMain->ecatInterface->v_global_ethercat_control.f_get_slave()[2].v_global_cur_status_word == 4151) &&
                      (pControlMain->ecatInterface->v_global_ethercat_control.f_get_slave()[3].v_global_cur_status_word == 4151) &&
                      (pControlMain->ecatInterface->v_global_ethercat_control.f_get_slave()[4].v_global_cur_status_word == 4151) &&
                      (pControlMain->ecatInterface->v_global_ethercat_control.f_get_slave()[5].v_global_cur_status_word == 4151) &&
                      (pControlMain->ecatInterface->v_global_ethercat_control.f_get_slave()[6].v_global_cur_status_word == 4151)))
                {
                    usleep(100);
                }

                while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                    usleep(100);
                }

                printf("Servo on\n");

                pControlMain->dataControl->RobotData.joint_op_mode = 0;
                pControlMain->ecatInterface->servo_mode(pControlMain->dataControl->RobotData.joint_op_mode);

                while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                    usleep(100);
                }
                usleep(1500000);
                printf("Start teaching\n");

                pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
            }
            else if(pControlMain->dataControl->operateMode.mode == DataControl::StartTeaching){
                pControlMain->ecatInterface->set_led(0);
                usleep(100);

//                pControlMain->dataControl->ClientToServer.opMode = DataControl::ServoOnOff;
//                pControlMain->dataControl->ClientToServer.subMode = 1;

//                usleep(1500000);
//                printf("Servo on\n");

//                while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
//                    usleep(1000);
//                }
                usleep(1500000);

                pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
                pControlMain->dataControl->operateMode.mode = DataControl::StopTeaching;
                usleep(100);

                pControlMain->dataControl->RobotData.joint_op_mode = 3;
                pControlMain->ecatInterface->servo_mode(pControlMain->dataControl->RobotData.joint_op_mode);

                while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                    usleep(100);
                }

                pControlMain->dataControl->select_speed = 3.0;

                pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
                pControlMain->dataControl->operateMode.mode = DataControl::WaitFeeding1;

                while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                    usleep(100);
                }

                pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
                pControlMain->dataControl->operateMode.mode = DataControl::WaitFeeding2;

                while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                    usleep(100);
                }

                pControlMain->dataControl->ClientToServer.opMode = DataControl::ServoOnOff;
                pControlMain->dataControl->ClientToServer.subMode = 0;

                pControlMain->dataControl->select_speed = 1.8;

//                pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
//                pControlMain->dataControl->operateMode.mode = DataControl::Start;

                printf("Feeding ready complete\n");
            }
            pControlMain->dataControl->key_value = 0;
        }
        else if(pControlMain->dataControl->key_value == KEY_ESC){
            pControlMain->dataControl->PathData.path_data_indx = 0;
            pControlMain->dataControl->PathData.path_struct_indx = 0;
            pControlMain->dataControl->PathData.cycle_count_max = -1;

//            pControlMain->ecatInterface->set_led(2);
            pControlMain->dataControl->collision = true;
            pControlMain->dataControl->ClientToServer.opMode = DataControl::Wait;
        }
        else if(pControlMain->dataControl->key_value == KEY_Q){
            pControlMain->dataControl->button_mode = true;
            if(pControlMain->dataControl->KITECHData.wait_feeding){
                pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
                pControlMain->dataControl->operateMode.mode = DataControl::WaitFeeding1;

                while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                    usleep(100);
                }

            }
            pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
            pControlMain->dataControl->operateMode.mode = DataControl::ReadyFeeding;

            while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                usleep(100);
            }

            if(!pControlMain->dataControl->button_mode){
                pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
                pControlMain->dataControl->operateMode.mode = DataControl::ReadyFeeding;
            }

            while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                usleep(100);
            }

            // choose section
            rt_printf("Received Section Packet\n");
            pControlMain->dataControl->select_indx++;
            if(pControlMain->dataControl->select_indx >= 6) {
                pControlMain->dataControl->select_indx = 1;
            }
            rt_printf("Section Index : %d\n", pControlMain->dataControl->select_indx);
            pControlMain->dataControl->operateMode.section = pControlMain->dataControl->select_indx;
            pControlMain->dataControl->section_indx = pControlMain->dataControl->select_indx - 1;

            pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
            pControlMain->dataControl->operateMode.mode = DataControl::FeedingSwitch;
        }
        else if(pControlMain->dataControl->key_value == KEY_W){
            rt_printf("Received Feeding Packet\n");
            rt_printf("Section : %d\n", pControlMain->dataControl->KITECHData.interface_sub);
            pControlMain->dataControl->section_indx = pControlMain->dataControl->KITECHData.interface_sub - 1;

            if(pControlMain->dataControl->KITECHData.wait_feeding){
                pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
                pControlMain->dataControl->operateMode.mode = DataControl::WaitFeeding1;

                while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                    usleep(1000);
                }
            }

            pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
            pControlMain->dataControl->operateMode.mode = DataControl::ReadyFeeding;

            while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::Wait)){
                usleep(1000);
            }

            pControlMain->dataControl->ClientToServer.opMode = DataControl::OperateMode;
            pControlMain->dataControl->operateMode.mode = DataControl::Feeding;
            switch(pControlMain->dataControl->section_indx){
                case 0: // side 1
                {
                    pControlMain->dataControl->trayInfor.section1 = pControlMain->dataControl->trayInfor.section1_cnt%6;
                    rt_printf("section1 count : %d, %d\n", pControlMain->dataControl->trayInfor.section1_cnt, pControlMain->dataControl->trayInfor.section1);
                    pControlMain->dataControl->operateMode.section = DataControl::Side1;
                    break;
                }
                case 1: // side 2
                {
                    pControlMain->dataControl->trayInfor.section2 = pControlMain->dataControl->trayInfor.section2_cnt%6;
                    rt_printf("section2 count : %d, %d\n", pControlMain->dataControl->trayInfor.section2_cnt, pControlMain->dataControl->trayInfor.section2);
                    pControlMain->dataControl->operateMode.section = DataControl::Side2;
                    break;
                }
                case 2: // side 3
                {
                    pControlMain->dataControl->trayInfor.section3 = pControlMain->dataControl->trayInfor.section3_cnt%4;
                    rt_printf("section3 count : %d, %d\n", pControlMain->dataControl->trayInfor.section3_cnt, pControlMain->dataControl->trayInfor.section3);
                    pControlMain->dataControl->operateMode.section = DataControl::Side3;
                    break;
                }
                case 3: // soup
                {
                    rt_printf("section4 count : %d\n", pControlMain->dataControl->trayInfor.section4);
                    pControlMain->dataControl->operateMode.section = DataControl::Soup;
                    break;
                }
                case 4: // rice
                {
                    pControlMain->dataControl->trayInfor.section5 = pControlMain->dataControl->trayInfor.section5_cnt%12;
                    rt_printf("section5 count : %d, %d\n", pControlMain->dataControl->trayInfor.section5_cnt, pControlMain->dataControl->trayInfor.section5);
                    pControlMain->dataControl->operateMode.section = DataControl::Rice;
                    break;
                }
            }
            pControlMain->dataControl->select_indx = pControlMain->dataControl->section_indx;
        }

//        pControlMain->dataControl->key_value = getch();
//        printf("key value : %d\n", pControlMain->dataControl->key_value);
    }
    printf("Finished Key Input Thread\n");

    return NULL;
}

void *ControlMain::logging_func(void *arg)
{
    ControlMain *pControlMain = static_cast<ControlMain*>(arg);

    time_t timer = time(NULL);
    struct tm* t = localtime(&timer);

    QString file_name = "/mnt/mtd5/daincube/KETI/logging/";
    file_name += QString::number(t->tm_year + 1900)
            + (t->tm_mon + 1 < 10 ? "0" + QString::number(t->tm_mon + 1) : QString::number(t->tm_mon + 1))
            + (t->tm_mday < 10 ? "0" + QString::number(t->tm_mday) : QString::number(t->tm_mday))
            + "_"
            + (t->tm_hour < 10 ? "0" + QString::number(t->tm_hour) : QString::number(t->tm_hour))
            + (t->tm_min < 10 ? "0" + QString::number(t->tm_min) : QString::number(t->tm_min))
            + (t->tm_sec < 10 ? "0" + QString::number(t->tm_sec) : QString::number(t->tm_sec))
            + ".csv";

    pControlMain->fp_logging = fopen(file_name.toStdString().c_str(), "w+");

    // data header
    fprintf(pControlMain->fp_logging, "data_index,");
    fprintf(pControlMain->fp_logging, "robot_state,");
    fprintf(pControlMain->fp_logging, "robot_sub_state,");
    for(int j = 0; j < NUM_JOINT; j++){
        fprintf(pControlMain->fp_logging, "status_word_%d,", j);
    }
    for(int j = 0; j < NUM_JOINT; j++){
        fprintf(pControlMain->fp_logging, "control_word_%d,", j);
    }
    for(int j = 0; j < NUM_JOINT; j++){
        fprintf(pControlMain->fp_logging, "mode_of_operation_%d,", j);
    }
    for(int j = 0; j < NUM_JOINT; j++){
        fprintf(pControlMain->fp_logging, "cur_position_%d,", j);
    }
    for(int j = 0; j < NUM_JOINT; j++){
        fprintf(pControlMain->fp_logging, "cur_velocity_%d,", j);
    }
    for(int j = 0; j < NUM_JOINT; j++){
        fprintf(pControlMain->fp_logging, "cur_torque_%d,", j);
    }
    for(int j = 0; j < NUM_JOINT; j++){
        fprintf(pControlMain->fp_logging, "tar_position_%d,", j);
    }
    for(int j = 0; j < NUM_JOINT; j++){
        fprintf(pControlMain->fp_logging, "tar_torque_%d,", j);
    }

    fprintf(pControlMain->fp_logging, "present_end px,present_end py,present_end pz,present_end rx,present_end ry,present_end rz,");
    fprintf(pControlMain->fp_logging, "present_end vx,present_end vy,present_end vz,present_end wx,present_end wy,present_end wz,");
    fprintf(pControlMain->fp_logging, "desired_end px,desired_end py,desired_end pz,desired_end rx,desired_end ry,desired_end rz,");
    for(int j = 0; j < 6; j++){
        fprintf(pControlMain->fp_logging, "present_q_%d,", j);
    }
    for(int j = 0; j < 6; j++){
        fprintf(pControlMain->fp_logging, "desired_q_%d,", j);
    }

    for(int j = 0; j < NUM_JOINT; j++){
        fprintf(pControlMain->fp_logging, "err_state_%d,", j);
    }
    for(int j = 0; j < NUM_JOINT; j++){
        fprintf(pControlMain->fp_logging, "joint_residual_%d,", j);
    }
    //    for(int j = 0; j < NUM_JOINT; j++){
    //        fprintf(pControlMain->fp_logging, "err_msg_%d,", j);
    //    }
    fprintf(pControlMain->fp_logging, "\n");

    pControlMain->logging_thread_run = true;

    while(pControlMain->logging_thread_run){
        if(pControlMain->dataControl->loggingRobotData.size() > 0){
            //            printf("stack size : %d\n", pControlMain->dataControl->loggingRobotData.size());
            for(unsigned int i = 0; i < pControlMain->dataControl->loggingRobotData.size(); i++){
                // data index(check for the data continuous)
                fprintf(pControlMain->fp_logging, "%d,", pControlMain->dataControl->loggingRobotData.front().data_index);

                fprintf(pControlMain->fp_logging, "%d,", pControlMain->dataControl->loggingRobotData.front().op_mode);
                fprintf(pControlMain->fp_logging, "%d,", pControlMain->dataControl->loggingRobotData.front().run_mode);

                // joint space information
                for(int j = 0; j < NUM_JOINT; j++){
                    fprintf(pControlMain->fp_logging, "%d,", pControlMain->dataControl->loggingRobotData.front().cur_status_word[j]);
                }
                for(int j = 0; j < NUM_JOINT; j++){
                    fprintf(pControlMain->fp_logging, "%d,", pControlMain->dataControl->loggingRobotData.front().tar_control_word[j]);
                }
                for(int j = 0; j < NUM_JOINT; j++){
                    fprintf(pControlMain->fp_logging, "%d,", pControlMain->dataControl->loggingRobotData.front().cur_modes_of_operation[j]);
                }
                for(int j = 0; j < NUM_JOINT; j++){
                    fprintf(pControlMain->fp_logging, "%ld,", pControlMain->dataControl->loggingRobotData.front().cur_position[j]);
                }
                for(int j = 0; j < NUM_JOINT; j++){
                    fprintf(pControlMain->fp_logging, "%ld,", pControlMain->dataControl->loggingRobotData.front().cur_velocity[j]);
                }
                for(int j = 0; j < NUM_JOINT; j++){
                    fprintf(pControlMain->fp_logging, "%d,", pControlMain->dataControl->loggingRobotData.front().cur_torque[j]);
                }
                for(int j = 0; j < NUM_JOINT; j++){
                    fprintf(pControlMain->fp_logging, "%ld,", pControlMain->dataControl->loggingRobotData.front().tar_position[j]);
                }
                for(int j = 0; j < NUM_JOINT; j++){
                    fprintf(pControlMain->fp_logging, "%d,", pControlMain->dataControl->loggingRobotData.front().tar_torque[j]);
                }

                // cartesian space information
                for(int j = 0; j < NUM_DOF; j++){
                    fprintf(pControlMain->fp_logging, "%f,", pControlMain->dataControl->loggingRobotData.front().present_end_pose[j]);
                }
                for(int j = 0; j < NUM_DOF; j++){
                    fprintf(pControlMain->fp_logging, "%f,", pControlMain->dataControl->loggingRobotData.front().present_end_vel[j]);
                }
                for(int j = 0; j < NUM_DOF; j++){
                    fprintf(pControlMain->fp_logging, "%f,", pControlMain->dataControl->loggingRobotData.front().desired_end_pose[j]);
                }
                for(int j = 0; j < 6; j++){
                    fprintf(pControlMain->fp_logging, "%f,", pControlMain->dataControl->loggingRobotData.front().present_q[j]);
                }
                for(int j = 0; j < 6; j++){
                    fprintf(pControlMain->fp_logging, "%f,", pControlMain->dataControl->loggingRobotData.front().desired_q[j]);
                }

                // err information
                for(int j = 0; j < NUM_JOINT; j++){
                    fprintf(pControlMain->fp_logging, "%d,", pControlMain->dataControl->loggingRobotData.front().err_state[j]);
                }

                for(int j = 0; j < 6; j++){
                    fprintf(pControlMain->fp_logging, "%f,", pControlMain->dataControl->loggingRobotData.front().present_joint_residual[j]);
                }
                //                for(int j = 0; j < NUM_JOINT; j++){
                //                    fprintf(pControlMain->fp_logging, "%s,", pControlMain->dataControl->loggingErrorMessage[j].front().c_str());
                //                    pControlMain->dataControl->loggingErrorMessage[j].erase(pControlMain->dataControl->loggingErrorMessage[j].begin());
                //                }
                fprintf(pControlMain->fp_logging, "\n");

                pControlMain->dataControl->loggingRobotData.erase(pControlMain->dataControl->loggingRobotData.begin());
            }
        }
        usleep(1000);
    }

    printf("Logging finished\n");
    fclose(pControlMain->fp_logging);

    return NULL;
}

void ControlMain::moduleInit(){
#ifdef DYNAMIXELLIB_H
#endif
#ifdef USERINTERFACE_H
    while(!dataControl->RobotData.module_init){
        ecatInterface->ecat_start();
        dataControl->RobotData.module_init = true;
    }

    if(!dataControl->tablet_mode){
        if(key_input_thread_run){
            key_input_thread_run = false;
            pthread_cancel(key_input_thread);
            printf("Finished Key Input Task\n");
            usleep(100);
        }
    }

    while(!(REF(ecatInterface->v_global_ethercat_control.f_get_master_state_ptr()).al_states >= EC_AL_STATES_OP &&
            REF(ecatInterface->v_global_ethercat_control.f_get_domain_state_ptr()).state.wc_state == EC_WC_COMPLETE)){
        usleep(10000);
    }
    rt_printf("Operating start\n");

    //    if(!dataControl->tablet_mode){
    //        tcpServer->sendKey("S");
    //    }

    rt_printf("Present Joint ENC : %f, %f, %f, %f, %f, %f, %f\n",
              dataControl->module_dir[0]*(ecatInterface->v_global_ethercat_control.f_get_slave()[0].v_global_cur_position - dataControl->joint_offset[0])*ENC2DEG,
            dataControl->module_dir[1]*(ecatInterface->v_global_ethercat_control.f_get_slave()[1].v_global_cur_position - dataControl->joint_offset[1])*ENC2DEG,
            dataControl->module_dir[2]*(ecatInterface->v_global_ethercat_control.f_get_slave()[2].v_global_cur_position - dataControl->joint_offset[2])*ENC2DEG,
            dataControl->module_dir[3]*(ecatInterface->v_global_ethercat_control.f_get_slave()[3].v_global_cur_position - dataControl->joint_offset[3])*ENC2DEG,
            dataControl->module_dir[4]*(ecatInterface->v_global_ethercat_control.f_get_slave()[4].v_global_cur_position - dataControl->joint_offset[4])*ENC2DEG,
            dataControl->module_dir[5]*(ecatInterface->v_global_ethercat_control.f_get_slave()[5].v_global_cur_position - dataControl->joint_offset[5])*ENC2DEG,
            dataControl->module_dir[6]*(ecatInterface->v_global_ethercat_control.f_get_slave()[6].v_global_cur_position - dataControl->joint_offset[6])*ENC2DEG);

    if(dataControl->module_dir[2]*(ecatInterface->v_global_ethercat_control.f_get_slave()[2].v_global_cur_position - dataControl->joint_offset[2])*ENC2DEG > 100){
        dataControl->joint_offset[2] += 360*DEG2ENC;

        rt_printf("Present Joint ENC : %f, %f, %f, %f, %f, %f, %f\n",
                  dataControl->module_dir[0]*(ecatInterface->v_global_ethercat_control.f_get_slave()[0].v_global_cur_position - dataControl->joint_offset[0])*ENC2DEG,
                dataControl->module_dir[1]*(ecatInterface->v_global_ethercat_control.f_get_slave()[1].v_global_cur_position - dataControl->joint_offset[1])*ENC2DEG,
                dataControl->module_dir[2]*(ecatInterface->v_global_ethercat_control.f_get_slave()[2].v_global_cur_position - dataControl->joint_offset[2])*ENC2DEG,
                dataControl->module_dir[3]*(ecatInterface->v_global_ethercat_control.f_get_slave()[3].v_global_cur_position - dataControl->joint_offset[3])*ENC2DEG,
                dataControl->module_dir[4]*(ecatInterface->v_global_ethercat_control.f_get_slave()[4].v_global_cur_position - dataControl->joint_offset[4])*ENC2DEG,
                dataControl->module_dir[5]*(ecatInterface->v_global_ethercat_control.f_get_slave()[5].v_global_cur_position - dataControl->joint_offset[5])*ENC2DEG,
                dataControl->module_dir[6]*(ecatInterface->v_global_ethercat_control.f_get_slave()[6].v_global_cur_position - dataControl->joint_offset[6])*ENC2DEG);
    }
    //    dataControl->joint_offset[5] = ecatInterface->v_global_ethercat_control.f_get_slave()[5].v_global_cur_position;
    //    dataControl->joint_offset[6] = ecatInterface->v_global_ethercat_control.f_get_slave()[6].v_global_cur_position;

    //    dataControl->joint_offset[1] = dataControl->joint_offset[1] - static_cast<int32_t>(9.93*DEG2ENC);
    //    dataControl->joint_offset[2] = dataControl->joint_offset[2] + static_cast<int32_t>((180.0 - 30.06)*DEG2ENC);
    //    dataControl->joint_offset[3] = dataControl->joint_offset[3] - static_cast<int32_t>(((180.0 - 129.98) + 90.0)*DEG2ENC);

    //    dataControl->joint_offset[3] = dataControl->joint_offset[3] - static_cast<int32_t>(90*DEG2ENC);

    //    dataControl->joint_offset[2] += static_cast<int32_t>(360*DEG2ENC);

    if(dataControl->module_dir[4]*(ecatInterface->v_global_ethercat_control.f_get_slave()[4].v_global_cur_position - dataControl->joint_offset[4])*ENC2DEG < -100){
        dataControl->joint_offset[4] += 360*DEG2ENC;

        rt_printf("Present Joint ENC : %f, %f, %f, %f, %f, %f, %f\n",
                  dataControl->module_dir[0]*(ecatInterface->v_global_ethercat_control.f_get_slave()[0].v_global_cur_position - dataControl->joint_offset[0])*ENC2DEG,
                dataControl->module_dir[1]*(ecatInterface->v_global_ethercat_control.f_get_slave()[1].v_global_cur_position - dataControl->joint_offset[1])*ENC2DEG,
                dataControl->module_dir[2]*(ecatInterface->v_global_ethercat_control.f_get_slave()[2].v_global_cur_position - dataControl->joint_offset[2])*ENC2DEG,
                dataControl->module_dir[3]*(ecatInterface->v_global_ethercat_control.f_get_slave()[3].v_global_cur_position - dataControl->joint_offset[3])*ENC2DEG,
                dataControl->module_dir[4]*(ecatInterface->v_global_ethercat_control.f_get_slave()[4].v_global_cur_position - dataControl->joint_offset[4])*ENC2DEG,
                dataControl->module_dir[5]*(ecatInterface->v_global_ethercat_control.f_get_slave()[5].v_global_cur_position - dataControl->joint_offset[5])*ENC2DEG,
                dataControl->module_dir[6]*(ecatInterface->v_global_ethercat_control.f_get_slave()[6].v_global_cur_position - dataControl->joint_offset[6])*ENC2DEG);
    }

    if(dataControl->module_dir[3]*(ecatInterface->v_global_ethercat_control.f_get_slave()[3].v_global_cur_position - dataControl->joint_offset[3])*ENC2DEG > 100){
        dataControl->joint_offset[3] -= 360*DEG2ENC;

        rt_printf("Present Joint ENC : %f, %f, %f, %f, %f, %f, %f\n",
                  dataControl->module_dir[0]*(ecatInterface->v_global_ethercat_control.f_get_slave()[0].v_global_cur_position - dataControl->joint_offset[0])*ENC2DEG,
                dataControl->module_dir[1]*(ecatInterface->v_global_ethercat_control.f_get_slave()[1].v_global_cur_position - dataControl->joint_offset[1])*ENC2DEG,
                dataControl->module_dir[2]*(ecatInterface->v_global_ethercat_control.f_get_slave()[2].v_global_cur_position - dataControl->joint_offset[2])*ENC2DEG,
                dataControl->module_dir[3]*(ecatInterface->v_global_ethercat_control.f_get_slave()[3].v_global_cur_position - dataControl->joint_offset[3])*ENC2DEG,
                dataControl->module_dir[4]*(ecatInterface->v_global_ethercat_control.f_get_slave()[4].v_global_cur_position - dataControl->joint_offset[4])*ENC2DEG,
                dataControl->module_dir[5]*(ecatInterface->v_global_ethercat_control.f_get_slave()[5].v_global_cur_position - dataControl->joint_offset[5])*ENC2DEG,
                dataControl->module_dir[6]*(ecatInterface->v_global_ethercat_control.f_get_slave()[6].v_global_cur_position - dataControl->joint_offset[6])*ENC2DEG);
    }

    //    if(dataControl->module_dir[5]*(ecatInterface->v_global_ethercat_control.f_get_slave()[5].v_global_cur_position - dataControl->joint_offset[5])*ENC2DEG < -100){
    //        dataControl->joint_offset[5] -= 360*DEG2ENC;

    //        rt_printf("Present Joint ENC : %f, %f, %f, %f, %f, %f, %f\n",
    //               dataControl->module_dir[0]*(ecatInterface->v_global_ethercat_control.f_get_slave()[0].v_global_cur_position - dataControl->joint_offset[0])*ENC2DEG,
    //               dataControl->module_dir[1]*(ecatInterface->v_global_ethercat_control.f_get_slave()[1].v_global_cur_position - dataControl->joint_offset[1])*ENC2DEG,
    //               dataControl->module_dir[2]*(ecatInterface->v_global_ethercat_control.f_get_slave()[2].v_global_cur_position - dataControl->joint_offset[2])*ENC2DEG,
    //               dataControl->module_dir[3]*(ecatInterface->v_global_ethercat_control.f_get_slave()[3].v_global_cur_position - dataControl->joint_offset[3])*ENC2DEG,
    //               dataControl->module_dir[4]*(ecatInterface->v_global_ethercat_control.f_get_slave()[4].v_global_cur_position - dataControl->joint_offset[4])*ENC2DEG,
    //               dataControl->module_dir[5]*(ecatInterface->v_global_ethercat_control.f_get_slave()[5].v_global_cur_position - dataControl->joint_offset[5])*ENC2DEG,
    //               dataControl->module_dir[6]*(ecatInterface->v_global_ethercat_control.f_get_slave()[6].v_global_cur_position - dataControl->joint_offset[6])*ENC2DEG);
    //    }

    robot_rt_start();
#endif
}
