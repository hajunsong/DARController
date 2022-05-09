#include "ControlMain/controlmain.h"

void ControlMain::robot_RT(void *arg){
    ControlMain* pThis = static_cast<ControlMain*>(arg);
    RTIME now, previous;
    previous = rt_timer_read();
    pThis->dataControl->RobotData.time1 = static_cast<unsigned long>(rt_timer_read());

    pThis->robot_task_run = false;

    rt_task_set_periodic(&pThis->robot_task, TM_NOW, 1e6);

    pThis->robot_task_run = true;

    while(pThis->robot_task_run){
        rt_task_wait_period(NULL); //wait for next cycle

        pThis->dataControl->RobotData.time2 = static_cast<unsigned long>(rt_timer_read());

        switch(pThis->dataControl->ClientToServer.opMode){
            case DataControl::ServoOnOff:
                pThis->robotServoOn(pThis->dataControl->ClientToServer.subMode);
                break;
            case DataControl::Initialize:
                //                pThis->robotInitialize();
                break;
            case DataControl::Wait:
                pThis->robotWait();
                break;
            case DataControl::JointMove:
                pThis->robotJointMove(pThis->dataControl->ClientToServer.subMode, pThis->dataControl->ClientToServer.desiredJoint);
                break;
            case DataControl::RunMode:
                pThis->robotRun();
                break;
            case DataControl::OperateMode:
                pThis->robotOperate();
                break;
            default : break;
        }

        pThis->robotKinematics();

        pThis->dataControl->RobotData.data_index++;
        pThis->dataControl->RobotData.op_mode = pThis->dataControl->ClientToServer.opMode;
        for(int i = 0; i < NUM_JOINT; i++){
            pThis->dataControl->RobotData.cur_status_word[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_cur_status_word;
            pThis->dataControl->RobotData.tar_control_word[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_tar_control_word;
            pThis->dataControl->RobotData.cur_modes_of_operation[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_cur_modes_of_operation;
            pThis->dataControl->RobotData.cur_position[i] = pThis->dataControl->module_dir[i]*(pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_cur_position - pThis->dataControl->joint_offset[i]);
            pThis->dataControl->RobotData.cur_velocity[i] = pThis->dataControl->module_dir[i]*pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_cur_velocity;
            pThis->dataControl->RobotData.cur_torque[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_cur_torque_value1;
//            pThis->dataControl->RobotData.tar_position[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_tar_position;
//            pThis->dataControl->RobotData.tar_torque[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_tar_torque;
            pThis->dataControl->RobotData.err_state[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_cur_err_state_num;
            //                pThis->dataControl->err_msg[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_cur_err_state_msg;
        }


        if(pThis->dataControl->collision_detect_enable){
//            pThis->robotDOB();
            if(abs(pThis->dataControl->RobotData.cur_position[0] - pThis->dataControl->RobotData.tar_position[0]) > 30){
                pThis->dataControl->PathData.path_data_indx = 0;
                pThis->dataControl->PathData.path_struct_indx = 0;
                pThis->dataControl->PathData.cycle_count_max = -1;

                pThis->dataControl->collision = true;
                pThis->dataControl->ClientToServer.opMode = DataControl::Wait;
            }
            else if(abs(pThis->dataControl->RobotData.cur_position[4] - pThis->dataControl->RobotData.tar_position[4]) > 30){
                pThis->dataControl->PathData.path_data_indx = 0;
                pThis->dataControl->PathData.path_struct_indx = 0;
                pThis->dataControl->PathData.cycle_count_max = -1;

                pThis->dataControl->collision = true;
                pThis->dataControl->ClientToServer.opMode = DataControl::Wait;
            }

//            else if(abs(pThis->dataControl->RobotData.cur_position[5] - pThis->dataControl->RobotData.tar_position[5]) > 10){
//                pThis->dataControl->PathData.path_data_indx = 0;
//                pThis->dataControl->PathData.path_struct_indx = 0;
//                pThis->dataControl->PathData.cycle_count_max = -1;

//                pThis->dataControl->collision = true;
//                pThis->dataControl->ClientToServer.opMode = DataControl::Wait;
//            }

//            else if(abs(pThis->dataControl->RobotData.cur_position[6] - pThis->dataControl->RobotData.tar_position[6]) > 10){
//                pThis->dataControl->PathData.path_data_indx = 0;
//                pThis->dataControl->PathData.path_struct_indx = 0;
//                pThis->dataControl->PathData.cycle_count_max = -1;

//                pThis->dataControl->collision = true;
//                pThis->dataControl->ClientToServer.opMode = DataControl::Wait;
//            }

            if(pThis->dataControl->collision)
            {
                rt_printf("collision : %d, %d, %d, %d, %d, %d, %d\n",
                          abs(pThis->dataControl->RobotData.cur_position[0] - pThis->dataControl->RobotData.tar_position[0]),
                        abs(pThis->dataControl->RobotData.cur_position[1] - pThis->dataControl->RobotData.tar_position[1]),
                        abs(pThis->dataControl->RobotData.cur_position[2] - pThis->dataControl->RobotData.tar_position[2]),
                        abs(pThis->dataControl->RobotData.cur_position[3] - pThis->dataControl->RobotData.tar_position[3]),
                        abs(pThis->dataControl->RobotData.cur_position[4] - pThis->dataControl->RobotData.tar_position[4]),
                        abs(pThis->dataControl->RobotData.cur_position[5] - pThis->dataControl->RobotData.tar_position[5]),
                        abs(pThis->dataControl->RobotData.cur_position[6] - pThis->dataControl->RobotData.tar_position[6]));
            }
        }

        if(pThis->dataControl->RobotData.joint_op_mode == 0){
            if(!pThis->jog_command){
                pThis->robotDynamics();
                if(pThis->servo)
                {
                    for(int i = 0; i < NUM_JOINT; i++){
                        pThis->dataControl->RobotData.tar_torque[i] += pThis->dataControl->torque_offset[i];
                    }
                    pThis->ecatInterface->servo_run(pThis->dataControl->RobotData.tar_torque, pThis->dataControl->module_dir);
                }
            }
        }

        if(pThis->dataControl->current_state_print){
            if(pThis->print_cnt >= 1000){
                rt_printf("present q(enc) : %d, %d, %d, %d, %d, %d, %d\n",
                          (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[0].v_global_cur_position),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[1].v_global_cur_position),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[2].v_global_cur_position),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[3].v_global_cur_position),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[4].v_global_cur_position),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[5].v_global_cur_position),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[6].v_global_cur_position));
                rt_printf("present q(deg) : %f, %f, %f, %f, %f, %f, %f\n",
                          (pThis->dataControl->RobotData.cur_position[0]) * ENC2DEG,
                        (pThis->dataControl->RobotData.cur_position[1]) * ENC2DEG,
                        (pThis->dataControl->RobotData.cur_position[2]) * ENC2DEG,
                        (pThis->dataControl->RobotData.cur_position[3]) * ENC2DEG,
                        (pThis->dataControl->RobotData.cur_position[4]) * ENC2DEG,
                        (pThis->dataControl->RobotData.cur_position[5]) * ENC2DEG,
                        (pThis->dataControl->RobotData.cur_position[6]) * ENC2DEG);
                rt_printf("status word : %d, %d, %d, %d, %d, %d, %d\n",
                          (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[0].v_global_cur_status_word),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[1].v_global_cur_status_word),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[2].v_global_cur_status_word),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[3].v_global_cur_status_word),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[4].v_global_cur_status_word),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[5].v_global_cur_status_word),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[6].v_global_cur_status_word));
                rt_printf("present current : %d, %d, %d, %d, %d, %d, %d\n",
                          (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[0].v_global_cur_torque_value1),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[1].v_global_cur_torque_value1),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[2].v_global_cur_torque_value1),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[3].v_global_cur_torque_value1),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[4].v_global_cur_torque_value1),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[5].v_global_cur_torque_value1),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[6].v_global_cur_torque_value1));
                rt_printf("v_global_cur_modes_of_operation : %d, %d, %d, %d, %d, %d, %d\n",
                          (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[0].v_global_cur_modes_of_operation),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[1].v_global_cur_modes_of_operation),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[2].v_global_cur_modes_of_operation),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[3].v_global_cur_modes_of_operation),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[4].v_global_cur_modes_of_operation),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[5].v_global_cur_modes_of_operation),
                        (pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[6].v_global_cur_modes_of_operation));
                rt_printf("current pose : %f, %f, %f, %f, %f, %f\n",
                          pThis->dataControl->RobotData.present_end_pose[0], pThis->dataControl->RobotData.present_end_pose[1], pThis->dataControl->RobotData.present_end_pose[2],
                          pThis->dataControl->RobotData.present_end_pose[3], pThis->dataControl->RobotData.present_end_pose[4], pThis->dataControl->RobotData.present_end_pose[5]);
                pThis->print_cnt = 0;
            }
            pThis->print_cnt++;
        }

//        rt_printf("%d %d\n", pThis->dataControl->ClientToServer.opMode, pThis->dataControl->feeding);

        if(pThis->dataControl->logging_enable && pThis->logging_thread_run){
            pThis->dataControl->loggingRobotData.resize(pThis->dataControl->loggingRobotData.size() + 1);
            memcpy(&pThis->dataControl->loggingRobotData.back(), &pThis->dataControl->RobotData, sizeof(DataControl::StructRobotData));
            //            for(unsigned int i = 0; i < NUM_JOINT; i++){
            //                pThis->dataControl->loggingErrorMessage[i].resize(pThis->dataControl->loggingErrorMessage[i].size() + 1);
            //                pThis->dataControl->loggingErrorMessage[i].push_back(pThis->dataControl->err_msg[i]);
            //            }
            //            rt_printf("data index : %d\n", pThis->dataControl->RobotData.data_index);
        }

        if(pThis->tcpServer->isConnected() || pThis->tcpServerPath->isConnected()){
            pThis->dataControl->ServerToClient.resize(pThis->dataControl->ServerToClient.size() + 1);
            pThis->dataControl->ServerToClient.back().indx = pThis->data_indx++;
            pThis->dataControl->ServerToClient.back().time = 0;
            pThis->dataControl->ServerToClient.back().module_time = 0l;
            pThis->dataControl->ServerToClient.back().ik_time = 0;
            for(int i = 0; i < NUM_JOINT; i++){
                pThis->dataControl->ServerToClient.back().cur_status_word[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_cur_status_word;
                pThis->dataControl->ServerToClient.back().cur_position[i] = pThis->dataControl->RobotData.cur_position[i];
                pThis->dataControl->ServerToClient.back().cur_velocity[i] = pThis->dataControl->RobotData.cur_velocity[i];
                pThis->dataControl->ServerToClient.back().cur_torque[i] = pThis->dataControl->RobotData.cur_torque[i];
                pThis->dataControl->ServerToClient.back().tar_position[i] = pThis->dataControl->module_dir[i]*(pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_tar_position - pThis->dataControl->joint_offset[i]);
//                pThis->dataControl->ServerToClient.back().tar_position[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_tar_position;
                pThis->dataControl->ServerToClient.back().tar_torque[i] = pThis->dataControl->RobotData.tar_torque[i];
            }

            pThis->dataControl->cartesianPoseScaleUp(pThis->dataControl->RobotData.present_end_pose, pThis->dataControl->ServerToClient.back().presentCartesianPose);
            pThis->dataControl->cartesianPoseScaleUp(pThis->dataControl->RobotData.present_end_vel, pThis->dataControl->ServerToClient.back().presentCartesianVelocity);

            pThis->dataControl->ServerToClient.back().opMode = pThis->dataControl->RobotData.op_mode;
        }
        else{
            if(!pThis->dataControl->tablet_mode){
                pThis->robot_task_run = false;
            }
        }

        now = rt_timer_read();
    }
}

void ControlMain::robotServoOn(char arg){
    if(arg){
        ecatInterface->servo_on();
        servo = true;
    }
    else{
        ecatInterface->servo_off();
        servo = false;
    }

    dataControl->ClientToServer.opMode = DataControl::Wait;
}

void ControlMain::robotWait()
{
    if(dataControl->robotStart){
        if(dataControl->collision){
    //        usleep(100000);
            dataControl->operateMode.section = DataControl::Home;
            dataControl->ClientToServer.opMode = DataControl::OperateMode;

            dataControl->dining_delay_cnt = dataControl->dining_delay + 1;

            dataControl->collision = false;
            dataControl->collision_detect_enable = false;
        }
    //                    pThis->ecatInterface->servo_stop();
        if(dataControl->KITECHData.tablet_connect){
            if(dataControl->operateMode.mode == DataControl::StartTeaching){
                ecatInterface->set_led(1);
            }
            else{
                ecatInterface->set_led(3);
            }
        }
        else{
            ecatInterface->set_led(2);
        }

        if(tcpServerPath->getPort() != 0){
            ecatInterface->set_led(3);
        }


        if(dataControl->operateMode.mode == DataControl::Start){
            dataControl->KITECHData.camera_request = true;
        }
    }
    else{
        ecatInterface->set_led(3);
    }
}

void ControlMain::robotJointMove(char mode, double desJoint[])
{
    ecatInterface->set_led(2);
    switch(mode){
        case DataControl::JogMotion:
            if(dataControl->RobotData.joint_op_mode == 0){
                jog_command = true;
                for(int i = 0; i < NUM_JOINT; i++){
                    if(desJoint[i] > 0){
                        ++dataControl->RobotData.tar_torque[i];
                    }
                    else if(desJoint[i] < 0){
                        --dataControl->RobotData.tar_torque[i];
                    }
                    else{
                        dataControl->RobotData.tar_torque[i] = dataControl->RobotData.tar_torque[i];
                    }
                }

                ecatInterface->servo_run(dataControl->RobotData.tar_torque, dataControl->module_dir);
            }
            else{
                for(unsigned char i = 0; i < NUM_JOINT; i++){
                    dataControl->RobotData.tar_position[i] = dataControl->RobotData.cur_position[i] + static_cast<long>(desJoint[i]*DEG2ENC);
                }

                ecatInterface->servo_run(dataControl->RobotData.tar_position, dataControl->module_dir, dataControl->joint_offset);
            }

            ecatInterface->set_led(2);
            dataControl->ClientToServer.opMode = DataControl::Wait;
            ecatInterface->set_led(0);
            break;
        case DataControl::JointMotion:
            //            for(unsigned char i = 0; i < NUM_JOINT; i++){
            //                dataControl->RobotData.tar_position[i] = static_cast<long>(desJoint[i]*DEG2ENC);
            //            }

            if(dataControl->joint_path_index == 0){
                for(int i = 0; i < NUM_JOINT; i++){
                    dataControl->joint_path[i].clear();
                    path_generator(dataControl->RobotData.cur_position[i]*ENC2DEG, desJoint[i], dataControl->select_speed, 0.5, step_size, &dataControl->joint_path[i]);
                }
            }

            for(int i = 0; i < NUM_JOINT; i++){
                dataControl->RobotData.tar_position[i] = static_cast<long>(dataControl->joint_path[i][dataControl->joint_path_index]*DEG2ENC);
            }
            ecatInterface->servo_run(dataControl->RobotData.tar_position, dataControl->module_dir, dataControl->joint_offset);

            //            rt_printf("joint path index : %d\n", dataControl->joint_path_index++);
            dataControl->joint_path_index++;
            if(dataControl->joint_path_index >= dataControl->joint_path[0].size()){
                ecatInterface->set_led(0);
                dataControl->ClientToServer.opMode = DataControl::Wait;

                ecatInterface->set_led(3);
                dataControl->ClientToServer.subMode = 99;
                dataControl->joint_path_index = 0;
            }
            break;
        default :
            break;
    }
}

void ControlMain::robotKinematics(){
    for(int i = 0; i < 6; i++){
        dataControl->RobotData.present_q[i] = dataControl->RobotData.cur_position[i]*ENC2DEG*DEG2RAD;
    }

    robotArm->run_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.present_end_pose);
}

void ControlMain::robotDynamics()
{
    for(int i = 0; i < 6; i++){
        dataControl->RobotData.present_q[i] = dataControl->RobotData.cur_position[i]*ENC2DEG*DEG2RAD;
        dataControl->RobotData.present_q_dot[i] = dataControl->RobotData.cur_velocity[i]*RPM2DEG*DEG2RAD;
    }

    double goal_torque[7] = {0,};
    robotArm->gravity_compensation(dataControl->RobotData.present_q, dataControl->RobotData.present_q_dot, goal_torque);

    double goal_current[7] = {0,};
    double alpha = 1;
    for(uint i = 0; i < 6; i++){
        if(dataControl->MODULE_TYPE == DataControl::FAR_V1){
//            if (i < 3){
//                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W270)*alpha;
//            }
//            else{
//                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W150)*alpha;
//            }
        }
        else if(dataControl->MODULE_TYPE == DataControl::FAR_V2){
//            if (i < 3){
//                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_V270)*alpha;
//            }
//            else{
//                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_V350)*alpha;
//            }
        }
        else if(dataControl->MODULE_TYPE == DataControl::FAR_V3){
//            if (i < 3){
//                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W270)*alpha;
//            }
//            else{
//                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W350)*alpha;
//            }
        }
        else if(dataControl->MODULE_TYPE == DataControl::FAR_V4){
            if (i < 3){
                goal_current[i] = 1000*(goal_torque[i] / dataControl->torque_const1)*alpha;
            }
            else{
                goal_current[i] = 1000*(goal_torque[i] / dataControl->torque_const2)*alpha;
            }
        }
        else if(dataControl->MODULE_TYPE == DataControl::FAR_V5){
            if (i < 3){
                goal_current[i] = 1000*(goal_torque[i] / dataControl->torque_const1)*alpha;
            }
            else{
                goal_current[i] = 1000*(goal_torque[i] / dataControl->torque_const2)*alpha;
            }
        }
    }
    dataControl->jointCurrentA2RAW(goal_current, dataControl->RobotData.tar_torque);
}

void ControlMain::robotDOB()
{
//    for(int i = 0; i < 6; i++){
//        dataControl->RobotData.present_q[i] = dataControl->RobotData.cur_position[i]*ENC2DEG*DEG2RAD;
//        dataControl->RobotData.present_q_dot[i] = dataControl->RobotData.cur_velocity[i]*RPM2DEG*DEG2RAD;
//        if(i < 3){
//            dataControl->RobotData.present_joint_torque[i] = dataControl->RobotData.cur_torque[i]*RAW2mA[i]*0.001;
//        }
//        else{
//            dataControl->RobotData.present_joint_torque[i] = dataControl->RobotData.cur_torque[i]*RAW2mA[i]*0.001;
//        }
//    }

//    double sum_current = 0;
//    for(int i = 0; i < NUM_JOINT; i++){
//        sum_current += (dataControl->RobotData.cur_torque[i])*RAW2mA[i];
//    }
//    rt_printf("%.3f\t\t", sum_current);

//    memset(dataControl->RobotData.present_joint_residual, 0, sizeof(double)*6);
//    robotArm->disturbance_observer(dataControl->RobotData.present_q, dataControl->RobotData.present_q_dot, dataControl->RobotData.present_joint_torque, dataControl->RobotData.present_joint_residual);
//    dataControl->RobotData.present_joint_residual[5] = sum_current;
//    rt_printf("%f, %f, %f, %f, %f, %f\n",
//              dataControl->RobotData.present_joint_residual[0], dataControl->RobotData.present_joint_residual[1], dataControl->RobotData.present_joint_residual[2],
//            dataControl->RobotData.present_joint_residual[3], dataControl->RobotData.present_joint_residual[4], dataControl->RobotData.present_joint_residual[5]);

//    double alpha = 1.1;
//    dataControl->RobotData.residual_limit_p[0] = 0.181052*alpha;
//    dataControl->RobotData.residual_limit_p[1] = 0.298015*alpha;
//    dataControl->RobotData.residual_limit_p[2] = 0.070687*alpha;
//    dataControl->RobotData.residual_limit_p[3] = 0.046528*alpha;
//    dataControl->RobotData.residual_limit_p[4] = 0.047994*alpha;
//    dataControl->RobotData.residual_limit_p[5] = 0*alpha;

//    dataControl->RobotData.residual_limit_n[0] = -0.186456*alpha;
//    dataControl->RobotData.residual_limit_n[1] = -0.199547*alpha;
//    dataControl->RobotData.residual_limit_n[2] = -0.059228*alpha;
//    dataControl->RobotData.residual_limit_n[3] = -0.045333*alpha;
//    dataControl->RobotData.residual_limit_n[4] = -0.045201*alpha;
//    dataControl->RobotData.residual_limit_n[5] = 0*alpha;

//    for(int i = 0; i < 6; i++){
//        if(dataControl->RobotData.present_joint_residual[i] > dataControl->RobotData.residual_limit_p[i]){
//            dataControl->PathData.path_data_indx = 0;
//            dataControl->PathData.path_struct_indx = 0;
//            dataControl->operateMode.section = DataControl::Home;
//            dataControl->ClientToServer.opMode = DataControl::OperateMode;
//            dataControl->collision_detect_enable = false;


//            rt_printf("Present Joint Residual : %f, %f, %f, %f, %f, %f, %d\n",
//                      dataControl->RobotData.present_joint_residual[0], dataControl->RobotData.present_joint_residual[1], dataControl->RobotData.present_joint_residual[2],
//                    dataControl->RobotData.present_joint_residual[3], dataControl->RobotData.present_joint_residual[4], dataControl->RobotData.present_joint_residual[5], i+1);
//        }
//        else if(dataControl->RobotData.present_joint_residual[i] < dataControl->RobotData.residual_limit_n[i]){
//            dataControl->PathData.path_data_indx = 0;
//            dataControl->PathData.path_struct_indx = 0;
//            dataControl->operateMode.section = DataControl::Home;
//            dataControl->ClientToServer.opMode = DataControl::OperateMode;
//            dataControl->collision_detect_enable = false;

//            rt_printf("Present Joint Residual : %f, %f, %f, %f, %f, %f, %d\n",
//                      dataControl->RobotData.present_joint_residual[0], dataControl->RobotData.present_joint_residual[1], dataControl->RobotData.present_joint_residual[2],
//                    dataControl->RobotData.present_joint_residual[3], dataControl->RobotData.present_joint_residual[4], dataControl->RobotData.present_joint_residual[5], i+1);
//        }
//    }
}

void ControlMain::robotRun()
{
    switch(dataControl->RobotData.run_mode){
        case DataControl::RunCmd:
        {
            dataControl->RobotData.desired_end_pose[0] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_x[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[1] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_y[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[2] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_z[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[3] = feedingOrientation[0];
            dataControl->RobotData.desired_end_pose[4] = feedingOrientation[1];
            dataControl->RobotData.desired_end_pose[5] = feedingOrientation[2];

//            if(dataControl->RobotData.desired_end_pose[0] > -0.1){
//                dataControl->RobotData.desired_end_pose[0] = -0.1;
//            }

            for(int i = 0; i < 6; i++){
                dataControl->RobotData.present_q[i] = dataControl->RobotData.cur_position[i]*ENC2DEG*DEG2RAD;
                dataControl->RobotData.present_q[5] = 0;
            }

            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose, dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);

//            rt_printf("present_q : %f, %f, %f, %f, %f, %f\n",
//                      dataControl->RobotData.present_q[0], dataControl->RobotData.present_q[1], dataControl->RobotData.present_q[2],
//                    dataControl->RobotData.present_q[3], dataControl->RobotData.present_q[4], dataControl->RobotData.present_q[5]);
//            rt_printf("desired_q : %f, %f, %f, %f, %f, %f\n",
//                      dataControl->RobotData.desired_q[0], dataControl->RobotData.desired_q[1], dataControl->RobotData.desired_q[2],
//                    dataControl->RobotData.desired_q[3], dataControl->RobotData.desired_q[4], dataControl->RobotData.desired_q[5]);
//            rt_printf("delta_q : %f, %f, %f, %f, %f, %f\n",
//                    abs(abs(dataControl->RobotData.desired_q[0]) - abs(dataControl->RobotData.present_q[0])),
//                    abs(abs(dataControl->RobotData.desired_q[1]) - abs(dataControl->RobotData.present_q[1])),
//                    abs(abs(dataControl->RobotData.desired_q[2]) - abs(dataControl->RobotData.present_q[2])),
//                    abs(abs(dataControl->RobotData.desired_q[3]) - abs(dataControl->RobotData.present_q[3])),
//                    abs(abs(dataControl->RobotData.desired_q[4]) - abs(dataControl->RobotData.present_q[4])),
//                    abs(abs(dataControl->RobotData.desired_q[5]) - abs(dataControl->RobotData.present_q[5])));

//            for(int i = 0; i < 6; i++){
//                if(abs(dataControl->RobotData.desired_q[i] - dataControl->RobotData.present_q[i]) > 0.05){
////                    dataControl->PathData.path_data_indx -= 5;

////                    dataControl->RobotData.present_end_pose[0] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_x[dataControl->PathData.path_data_indx];
////                    dataControl->RobotData.present_end_pose[1] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_y[dataControl->PathData.path_data_indx];
////                    dataControl->RobotData.present_end_pose[2] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_z[dataControl->PathData.path_data_indx];
////                    dataControl->RobotData.present_end_pose[3] = feedingOrientation[0];
////                    dataControl->RobotData.present_end_pose[4] = feedingOrientation[1];
////                    dataControl->RobotData.present_end_pose[5] = feedingOrientation[2];

////                    dataControl->PathData.path_data_indx += 10;
//                    rt_printf("Error!!!!!!\n");
//                    return;
//                }
//            }

            if (delay_cnt == 0){
//                rt_printf("path data indx : %d\n", dataControl->PathData.path_data_indx);
                dataControl->RobotData.tar_position[0] = static_cast<long>(dataControl->RobotData.desired_q[0]*RAD2DEG*DEG2ENC);
                dataControl->RobotData.tar_position[1] = static_cast<long>(dataControl->RobotData.desired_q[1]*RAD2DEG*DEG2ENC);
                dataControl->RobotData.tar_position[2] = static_cast<long>(dataControl->RobotData.desired_q[2]*RAD2DEG*DEG2ENC);
                dataControl->RobotData.tar_position[3] = static_cast<long>(dataControl->RobotData.desired_q[3]*RAD2DEG*DEG2ENC);
                dataControl->RobotData.tar_position[4] = static_cast<long>(dataControl->RobotData.desired_q[4]*RAD2DEG*DEG2ENC);
                dataControl->RobotData.tar_position[5] = static_cast<long>(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_q6[dataControl->PathData.path_data_indx]*DEG2ENC);
                dataControl->RobotData.tar_position[6] = static_cast<long>(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_q7[dataControl->PathData.path_data_indx]*DEG2ENC);

                ecatInterface->servo_run(dataControl->RobotData.tar_position, dataControl->module_dir, dataControl->joint_offset);
                dataControl->PathData.path_data_indx += 1;
            }
            ecatInterface->set_led(2);

            if (dataControl->PathData.path_data_indx >= dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].data_size - 1){
                //                    rt_printf("delay cnt : %d\n", delay_cnt);
                //                    delay_cnt++;
                //                    if(delay_cnt >= delay_cnt_max){
                dataControl->PathData.path_data_indx = 0;
                dataControl->PathData.path_struct_indx++;
                //                        delay_cnt = 0;
                //                    }
                //                    else{
                //                        ecatInterface->set_led(3);
                //                        return;
                //                    }

                if (dataControl->PathData.path_struct_indx >= dataControl->PathData.row - 1){
                    if(dataControl->PathData.cycle_count_max == -1)
                    {
                        dataControl->PathData.path_data_indx = 0;
                        dataControl->PathData.path_struct_indx = 0;
                    }
                    else
                    {
                        if(dataControl->feeding){
                            switch(dataControl->operateMode.section){
                                case DataControl::Side1:
                                case DataControl::Side2:
                                case DataControl::Side3:
                                case DataControl::Soup:
                                case DataControl::Rice:
                                    dataControl->operateMode.section = DataControl::Mouse;
                                    dataControl->ClientToServer.opMode = DataControl::OperateMode;
                                    if(dataControl->safe_mode_enable)
                                        dataControl->collision_detect_enable = true;
                                    break;
                                case DataControl::Mouse:
                                    dataControl->operateMode.section = DataControl::Home;
                                    dataControl->ClientToServer.opMode = DataControl::OperateMode;
                                    dataControl->collision_detect_enable = false;
                                    break;
                                case DataControl::Home:
                                    dataControl->ClientToServer.opMode = DataControl::OperateMode;
                                    dataControl->operateMode.mode = DataControl::Start;
                                    dataControl->feeding = false;
                                    break;
                                default:
                                    break;
                            }
                        }
                        else{
                            dataControl->ClientToServer.opMode = DataControl::Wait;
                        }

                        dataControl->PathData.path_data_indx = 0;
                        dataControl->PathData.path_struct_indx = 0;

                        rt_printf("robot motion exit\n");
                        ecatInterface->set_led(1);
                    }
                }
            }
            break;
        }
        default:
            break;
    }
}

void ControlMain::robotOperate(){
    switch(dataControl->operateMode.mode){
        case DataControl::Start:
        {
            rt_printf("Start\n");
            memcpy(dataControl->ClientToServer.desiredJoint, dataControl->operateCameraReadyJoint, sizeof(double)*NUM_JOINT);
            dataControl->ClientToServer.opMode = DataControl::JointMove;
            dataControl->ClientToServer.subMode = DataControl::JointMotion;
            dataControl->joint_path_index = 0;

            break;
        }
        case DataControl::Stop:
        {
            break;
        }
        case DataControl::StartTeaching:
        {
            ecatInterface->set_led(1);

            dataControl->PathData.total_time.clear();
            dataControl->PathData.point_px.clear();
            dataControl->PathData.point_py.clear();
            dataControl->PathData.point_pz.clear();
            dataControl->PathData.point_rx.clear();
            dataControl->PathData.point_ry.clear();
            dataControl->PathData.point_rz.clear();
            dataControl->PathData.point_theta.clear();
            dataControl->PathData.acc_time.clear();

            dataControl->PathData.row = 2;
            dataControl->PathData.movePath.resize(dataControl->PathData.row);
            for(uint i = 0; i < dataControl->PathData.row; i++){
                dataControl->PathData.movePath[i].path_x.clear();
                dataControl->PathData.movePath[i].path_y.clear();
                dataControl->PathData.movePath[i].path_z.clear();
                dataControl->PathData.movePath[i].path_theta.clear();
            }

            dataControl->PathData.total_time.push_back(0);
            dataControl->PathData.point_px.push_back(dataControl->RobotData.present_end_pose[0]);
            dataControl->PathData.point_py.push_back(dataControl->RobotData.present_end_pose[1]);
            dataControl->PathData.point_pz.push_back(dataControl->RobotData.present_end_pose[2]);
            dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
            dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
            dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
            dataControl->PathData.acc_time.push_back(1);

            dataControl->ClientToServer.opMode = DataControl::Wait;
            break;
        }
        case DataControl::StopTeaching:
        {
            ecatInterface->set_led(0);

            //            for(int i = 0; i < NUM_JOINT; i++){
            //                dataControl->teachingJoint[i] = dataControl->RobotData.cur_position[i]*ENC2DEG;
            //            }

            memcpy(dataControl->PathData.teaching_pose, dataControl->RobotData.present_end_pose, sizeof(double)*NUM_DOF);

            dataControl->PathData.total_time.push_back(dataControl->dining_speed_default*dataControl->dining_speed);
            dataControl->PathData.point_px.push_back(dataControl->PathData.teaching_pose[0]);
            dataControl->PathData.point_py.push_back(dataControl->PathData.teaching_pose[1]);
            dataControl->PathData.point_pz.push_back(dataControl->PathData.teaching_pose[2]);
            dataControl->PathData.point_rx.push_back(dataControl->PathData.teaching_pose[3]);
            dataControl->PathData.point_ry.push_back(dataControl->PathData.teaching_pose[4]);
            dataControl->PathData.point_rz.push_back(dataControl->PathData.teaching_pose[5]);
            dataControl->PathData.acc_time.push_back(1);

            for(unsigned char i = 0; i < dataControl->PathData.row; i ++){
                rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                          dataControl->PathData.total_time[i],
                          dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                          dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                          dataControl->PathData.acc_time[i]);
            }

            for(int i = 0; i < NUM_JOINT; i++){
                dataControl->RobotData.tar_position[i] = dataControl->RobotData.cur_position[i];
            }
            ecatInterface->servo_run(dataControl->RobotData.tar_position, dataControl->module_dir, dataControl->joint_offset);

            dataControl->ClientToServer.opMode = DataControl::Wait;
            break;
        }
        case DataControl::ReadyFeeding:
        {
            memcpy(dataControl->ClientToServer.desiredJoint, dataControl->feedingReadyJoint, sizeof(double)*NUM_JOINT);
            dataControl->ClientToServer.opMode = DataControl::JointMove;
            dataControl->ClientToServer.subMode = DataControl::JointMotion;
            dataControl->joint_path_index = 0;

            delay_cnt_max = 2000;
            delay_cnt = 0;

            rt_printf("Feeding ready finish\n");

            break;
        }

        case DataControl::FeedingSwitch:
        {
            ecatInterface->set_led(1);
            dataControl->KITECHData.camera_request = false;

            dataControl->PathData.total_time.clear();
            dataControl->PathData.point_px.clear();
            dataControl->PathData.point_py.clear();
            dataControl->PathData.point_pz.clear();
            dataControl->PathData.point_rx.clear();
            dataControl->PathData.point_ry.clear();
            dataControl->PathData.point_rz.clear();
            dataControl->PathData.point_theta.clear();
            dataControl->PathData.acc_time.clear();
            dataControl->PathData.point_q6.clear();
            dataControl->PathData.point_q7.clear();

            dataControl->PathData.row = 2;
            dataControl->PathData.movePath.clear();
            dataControl->PathData.movePath.resize(dataControl->PathData.row);

            for(uint i = 0; i < dataControl->PathData.row; i++){
                dataControl->PathData.movePath[i].path_x.clear();
                dataControl->PathData.movePath[i].path_y.clear();
                dataControl->PathData.movePath[i].path_z.clear();
                dataControl->PathData.movePath[i].path_theta.clear();
                dataControl->PathData.movePath[i].path_q6.clear();
                dataControl->PathData.movePath[i].path_q7.clear();

                dataControl->PathData.total_time.push_back(dataControl->select_speed);
                dataControl->PathData.acc_time.push_back(dataControl->select_speed/3.0);
            }

            dataControl->PathData.point_px.push_back(dataControl->RobotData.present_end_pose[0]);
            dataControl->PathData.point_py.push_back(dataControl->RobotData.present_end_pose[1]);
            dataControl->PathData.point_pz.push_back(dataControl->RobotData.present_end_pose[2]);
            dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
            dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
            dataControl->PathData.point_rz.push_back(feedingOrientation[2]);

            dataControl->PathData.point_q6.push_back(dataControl->RobotData.cur_position[5]*ENC2DEG);
            dataControl->PathData.point_q7.push_back(dataControl->RobotData.cur_position[6]*ENC2DEG);

            switch(dataControl->operateMode.section){
                case DataControl::Side1:
                    dataControl->PathData.point_px.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section1].x[0]);
                    dataControl->PathData.point_py.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section1].y[0]);
                    dataControl->PathData.point_pz.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section1].z[0]);
                    dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                    dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                    dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                    dataControl->PathData.point_q6.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section1].q6[0]);
                    dataControl->PathData.point_q7.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section1].q7[0]);
                    break;
                case DataControl::Side3:
                    dataControl->PathData.point_px.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->x[0] + dataControl->KITECHData.food_pos[2*(dataControl->operateMode.section - 1)]*0.001);
                    dataControl->PathData.point_py.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->y[0] - dataControl->KITECHData.food_pos[2*(dataControl->operateMode.section - 1) + 1]*0.001);
                    dataControl->PathData.point_pz.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->z[0]);
                    dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                    dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                    dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                    dataControl->PathData.point_q6.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->q6[0]);
                    dataControl->PathData.point_q7.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->q7[0]);
                    break;
                case DataControl::Soup:
                    dataControl->PathData.point_px.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->x[0]);
                    dataControl->PathData.point_py.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->y[0]);
                    dataControl->PathData.point_pz.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->z[0]);
                    dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                    dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                    dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                    dataControl->PathData.point_q6.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->q6[0]);
                    dataControl->PathData.point_q7.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->q7[0]);
                    break;
                case DataControl::Side2:
                    dataControl->PathData.point_px.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section2].x[0]);
                    dataControl->PathData.point_py.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section2].y[0]);
                    dataControl->PathData.point_pz.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section2].z[0]);
                    dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                    dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                    dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                    dataControl->PathData.point_q6.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section2].q6[0]);
                    dataControl->PathData.point_q7.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section2].q7[0]);
                    break;
                case DataControl::Rice:
                    dataControl->PathData.point_px.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section5].x[0]);
                    dataControl->PathData.point_py.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section5].y[0]);
                    dataControl->PathData.point_pz.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section5].z[0]);
                    dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                    dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                    dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                    dataControl->PathData.point_q6.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section5].q6[0]);
                    dataControl->PathData.point_q7.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section5].q7[0]);
                    break;
                default:
                    break;
            }

            rt_printf("path data : \n");
            for(uint8_t i = 0; i < dataControl->PathData.row; i ++){
                rt_printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                          dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i],
                          dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                          dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                          dataControl->PathData.point_q6[i], dataControl->PathData.point_q7[i]);
            }

            robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz, dataControl->PathData.point_q6, dataControl->PathData.point_q7);

            dataControl->PathData.path_data_indx = 0;
            dataControl->PathData.path_struct_indx = 0;
            dataControl->PathData.cycle_count_max = 1;
            delay_cnt = 0;
            delay_cnt_max = 200;

            dataControl->ClientToServer.opMode = DataControl::RunMode;
            dataControl->RobotData.run_mode = DataControl::RunCmd;

            break;
        }
        case DataControl::Feeding:
        {
            if(!dataControl->diningInfor.dining_time_check_start){
                time_t timer = time(NULL);
                struct tm* t = localtime(&timer);
                int year = t->tm_year + 1900;
                dataControl->diningInfor.dining_time_infor[0] = static_cast<char>(year/1000);
                dataControl->diningInfor.dining_time_infor[1] = (year%1000)/100;
                dataControl->diningInfor.dining_time_infor[2] = (year%100)/10;
                dataControl->diningInfor.dining_time_infor[3] = (year%100)%10;
                dataControl->diningInfor.dining_time_infor[4] = static_cast<char>((t->tm_mon + 1)/10);
                dataControl->diningInfor.dining_time_infor[5] = (t->tm_mon + 1)%10;
                dataControl->diningInfor.dining_time_infor[6] = static_cast<char>(t->tm_mday/10);
                dataControl->diningInfor.dining_time_infor[7] = t->tm_mday%10;
                dataControl->diningInfor.dining_time_infor[8] = static_cast<char>(t->tm_hour/10);
                dataControl->diningInfor.dining_time_infor[9] = t->tm_hour%10;
                dataControl->diningInfor.dining_time_infor[10] = static_cast<char>(t->tm_min/10);
                dataControl->diningInfor.dining_time_infor[11] = t->tm_min%10;
                dataControl->diningInfor.dining_step = dataControl->diningInfor.dining_start ? 2 : 1;
                dataControl->diningInfor.dining_time_check_start = true;

                rt_printf("[DEBUG] Dining Time Information : ");
                for(int i = 0; i < 12; i++){
                    rt_printf("%d ", dataControl->diningInfor.dining_time_infor[i]);
                }
                rt_printf("\n");
            }

            dataControl->feeding = true;
            dataControl->KITECHData.camera_request = false;

            dataControl->PathData.total_time.clear();
            dataControl->PathData.point_px.clear();
            dataControl->PathData.point_py.clear();
            dataControl->PathData.point_pz.clear();
            dataControl->PathData.point_rx.clear();
            dataControl->PathData.point_ry.clear();
            dataControl->PathData.point_rz.clear();
            dataControl->PathData.point_theta.clear();
            dataControl->PathData.point_option.clear();
            dataControl->PathData.acc_time.clear();
            dataControl->PathData.point_q6.clear();
            dataControl->PathData.point_q7.clear();

            switch(dataControl->operateMode.section){
                case DataControl::Side1:
                    dataControl->PathData.row = static_cast<unsigned char>(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section1].x.size());
                    if(!dataControl->button_mode){
                        dataControl->PathData.row += 1;
                    }
                    break;
                case DataControl::Side3:
                    dataControl->PathData.row = static_cast<unsigned char>(diningWaypoints.diningPath[dataControl->operateMode.section-1]->x.size());
                    if(!dataControl->button_mode){
                        dataControl->PathData.row += 1;
                    }
                    break;
                case DataControl::Soup:
                    dataControl->PathData.row = static_cast<unsigned char>(diningWaypoints.diningPath[dataControl->operateMode.section-1]->x.size());
                    if(!dataControl->button_mode){
                        dataControl->PathData.row += 1;
                    }
                    break;
                case DataControl::Side2:
                    dataControl->PathData.row = static_cast<unsigned char>(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section2].x.size());
                    if(!dataControl->button_mode){
                        dataControl->PathData.row += 1;
                    }
                    break;
                case DataControl::Rice:
                    dataControl->PathData.row = static_cast<unsigned char>(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section5].x.size());
                    if(!dataControl->button_mode){
                        dataControl->PathData.row += 1;
                    }
                    break;
                case DataControl::Mouse:
                    dataControl->PathData.row = 2;
                    break;
                case DataControl::Home:
                    dataControl->PathData.row = 2;
                    break;
            }
            dataControl->PathData.movePath.clear();
            dataControl->PathData.movePath.resize(dataControl->PathData.row);

//            if(dataControl->operateMode.section == DataControl::Side3){
//                dataControl->PathData.total_time.push_back(diningWaypoints.diningPath[dataControl->operateMode.section - 1]->time[1]);
//                dataControl->PathData.acc_time.push_back(dataControl->PathData.total_time.back()/3.0);
//            }

            for(uint i = 0; i < dataControl->PathData.row -1; i++){
                dataControl->PathData.movePath[i].path_x.clear();
                dataControl->PathData.movePath[i].path_y.clear();
                dataControl->PathData.movePath[i].path_z.clear();
                dataControl->PathData.movePath[i].path_theta.clear();
                dataControl->PathData.movePath[i].path_q6.clear();
                dataControl->PathData.movePath[i].path_q7.clear();

                if(dataControl->operateMode.section <= DataControl::Rice){
                    switch(dataControl->operateMode.section)
                    {
                        case DataControl::Side1:
                            dataControl->PathData.total_time.push_back(diningWaypoints.diningPath[dataControl->operateMode.section - 1][dataControl->trayInfor.section1].time[i + (dataControl->button_mode ? 1 : 0)]);
                            dataControl->PathData.acc_time.push_back(dataControl->PathData.total_time.back()/3.0);
                            break;
                        case DataControl::Side3:
                            dataControl->PathData.total_time.push_back(diningWaypoints.diningPath[dataControl->operateMode.section - 1]->time[i + (dataControl->button_mode ? 1 : 0)]);
                            dataControl->PathData.acc_time.push_back(dataControl->PathData.total_time.back()/3.0);
                            break;
                        case DataControl::Soup:
                            dataControl->PathData.total_time.push_back(diningWaypoints.diningPath[dataControl->operateMode.section - 1]->time[i + (dataControl->button_mode ? 1 : 0)]);
                            dataControl->PathData.acc_time.push_back(dataControl->PathData.total_time.back()/3.0);
                            break;
                        case DataControl::Side2:
                            dataControl->PathData.total_time.push_back(diningWaypoints.diningPath[dataControl->operateMode.section - 1][dataControl->trayInfor.section2].time[i + (dataControl->button_mode ? 1 : 0)]);
                            dataControl->PathData.acc_time.push_back(dataControl->PathData.total_time.back()/3.0);
                            break;
                        case DataControl::Rice:
                            dataControl->PathData.total_time.push_back(diningWaypoints.diningPath[dataControl->operateMode.section - 1][dataControl->trayInfor.section5].time[i + (dataControl->button_mode ? 1 : 0)]);
                            dataControl->PathData.acc_time.push_back(dataControl->PathData.total_time.back()/3.0);
                            break;
                        default:
                            break;
                    }
                }
                else{
                    dataControl->PathData.total_time.push_back(dataControl->dining_speed_default*dataControl->dining_speed);
                    dataControl->PathData.acc_time.push_back(1);
                }
            }

            dataControl->PathData.total_time.push_back(0);
            dataControl->PathData.acc_time.push_back(dataControl->PathData.total_time.back()/3.0);

            dataControl->PathData.point_px.push_back(dataControl->RobotData.present_end_pose[0]);
            dataControl->PathData.point_py.push_back(dataControl->RobotData.present_end_pose[1]);
            dataControl->PathData.point_pz.push_back(dataControl->RobotData.present_end_pose[2]);
            dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
            dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
            dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
            dataControl->PathData.point_q6.push_back(dataControl->RobotData.cur_position[5]*ENC2DEG);
            dataControl->PathData.point_q7.push_back(dataControl->RobotData.cur_position[6]*ENC2DEG);

//            if(dataControl->operateMode.section == DataControl::Side3){
//                dataControl->PathData.point_px.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->x[1] + dataControl->KITECHData.food_pos[2*(dataControl->operateMode.section - 1)]*0.001 + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1)]);
//                dataControl->PathData.point_py.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->y[1] - dataControl->KITECHData.food_pos[2*(dataControl->operateMode.section - 1) + 1]*0.001 + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1) + 1]);
//                dataControl->PathData.point_pz.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->z[1] + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1) + 2] + 0.005);
//                dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
//                dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
//                dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
//                dataControl->PathData.point_q6.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->q6[1]);
//                dataControl->PathData.point_q7.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->q7[1]);
//            }

            switch(dataControl->operateMode.section){
                case DataControl::Side1:
                    for(uint8_t i = dataControl->button_mode ? 1 : 0; i < dataControl->PathData.row + (dataControl->button_mode ? 0 : -1); i++){
                        dataControl->PathData.point_px.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section1].x[i] + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1)]);
                        dataControl->PathData.point_py.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section1].y[i] + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1) + 1]);
                        dataControl->PathData.point_pz.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section1].z[i] + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1) + 2]);
                        dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                        dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                        dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                        dataControl->PathData.point_q6.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section1].q6[i]);
                        dataControl->PathData.point_q7.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section1].q7[i]);
                    }
                    dataControl->trayInfor.section1_cnt++;
                    break;
                case DataControl::Side3:
                    for(uint8_t i = dataControl->button_mode ? 1 : 0; i < dataControl->PathData.row + (dataControl->button_mode ? 0 : -1); i++){
                        dataControl->PathData.point_px.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->x[i] + dataControl->KITECHData.food_pos[2*(dataControl->operateMode.section - 1)]*0.001 + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1)]);
                        dataControl->PathData.point_py.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->y[i] - dataControl->KITECHData.food_pos[2*(dataControl->operateMode.section - 1) + 1]*0.001 + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1) + 1]);
                        dataControl->PathData.point_pz.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->z[i] + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1) + 2]);
                        dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                        dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                        dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                        dataControl->PathData.point_q6.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->q6[i]);
                        dataControl->PathData.point_q7.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->q7[i]);
                    }
                    dataControl->trayInfor.section3_cnt++;
                    break;
                case DataControl::Soup:
                    for(uint8_t i = dataControl->button_mode ? 1 : 0; i < dataControl->PathData.row + (dataControl->button_mode ? 0 : -1); i++){
                        dataControl->PathData.point_px.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->x[i] + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1)]);
                        dataControl->PathData.point_py.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->y[i] + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1) + 1]);
                        dataControl->PathData.point_pz.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->z[i] + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1) + 2]);
                        dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                        dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                        dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                        dataControl->PathData.point_q6.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->q6[i]);
                        dataControl->PathData.point_q7.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1]->q7[i]);
                    }
                    dataControl->trayInfor.section4_cnt++;
                    break;
                case DataControl::Side2:
                    for(uint8_t i = dataControl->button_mode ? 1 : 0; i < dataControl->PathData.row + (dataControl->button_mode ? 0 : -1); i++){
                        dataControl->PathData.point_px.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section2].x[i] + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1)]);
                        dataControl->PathData.point_py.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section2].y[i] + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1) + 1]);
                        dataControl->PathData.point_pz.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section2].z[i] + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1) + 2]);
                        dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                        dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                        dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                        dataControl->PathData.point_q6.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section2].q6[i]);
                        dataControl->PathData.point_q7.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section2].q7[i]);
                    }
                    dataControl->trayInfor.section2_cnt++;
                    break;
                case DataControl::Rice:
                    for(uint8_t i = dataControl->button_mode ? 1 : 0; i < dataControl->PathData.row + (dataControl->button_mode ? 0 : -1); i++){
                        dataControl->PathData.point_px.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section5].x[i]
                                + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1)]);
                        dataControl->PathData.point_py.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section5].y[i]
                                + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1) + 1]);
                        dataControl->PathData.point_pz.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section5].z[i]
                                + dataControl->sectionOffset[3*(dataControl->operateMode.section - 1) + 2]);
                        dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                        dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                        dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                        dataControl->PathData.point_q6.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section5].q6[i]);
                        dataControl->PathData.point_q7.push_back(diningWaypoints.diningPath[dataControl->operateMode.section-1][dataControl->trayInfor.section5].q7[i]);
                    }
                    dataControl->trayInfor.section5_cnt++;
                    break;

                case DataControl::Mouse:
                {
                    rt_printf("go mouse\n");

                    dataControl->PathData.point_px.push_back(dataControl->PathData.teaching_pose[0]);
                    dataControl->PathData.point_py.push_back(dataControl->PathData.teaching_pose[1]);
                    dataControl->PathData.point_pz.push_back(dataControl->PathData.teaching_pose[2]);
                    dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                    dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                    dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                    dataControl->PathData.point_q6.push_back(dataControl->RobotData.cur_position[5]*ENC2DEG);
                    dataControl->PathData.point_q7.push_back(dataControl->RobotData.cur_position[6]*ENC2DEG);

                    break;
                }
                case DataControl::Home:
                {
                    if(dataControl->dining_delay_cnt >= dataControl->dining_delay){
                        dataControl->dining_delay_cnt = 0;
                        rt_printf("go home\n");
                    }
                    else{
                        dataControl->dining_delay_cnt++;
                        //                        rt_printf("dining delay cnt : %d\n", dataControl->dining_delay_cnt);
                        if(dataControl->KITECHData.dining_interrupt){
                            dataControl->dining_delay_cnt = dataControl->dining_delay;
                            dataControl->KITECHData.dining_interrupt = false;
                        }
                        return;
                    }

                    dataControl->PathData.point_px.push_back(diningWaypoints.diningPath[0]->x[0]);
                    dataControl->PathData.point_py.push_back(diningWaypoints.diningPath[0]->y[0]);
                    dataControl->PathData.point_pz.push_back(diningWaypoints.diningPath[0]->z[0]);
                    dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                    dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                    dataControl->PathData.point_rz.push_back(feedingOrientation[2]);

                    dataControl->PathData.point_q6.push_back(0);
                    dataControl->PathData.point_q7.push_back(0);

                    dataControl->diningInfor.dining_time_send = true;
                    dataControl->diningInfor.dining_start = true;
                    dataControl->diningInfor.dining_time_check_start = false;

                    rt_printf("Dining Time Information : ");
                    for(int i = 0; i < 12; i++){
                        rt_printf("%d ", dataControl->diningInfor.dining_time_infor[i]);
                    }
                    rt_printf("\n");

                    break;
                }
            }

            rt_printf("path data size : %d\n", dataControl->PathData.row);
            rt_printf("path data : \n");

            for(uint8_t i = 0; i < dataControl->PathData.row; i ++){
                rt_printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                          dataControl->PathData.total_time[i],
                          dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i], dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                          dataControl->PathData.point_q6[i], dataControl->PathData.point_q7[i], dataControl->PathData.acc_time[i]);
            }

            robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz, dataControl->PathData.point_q6, dataControl->PathData.point_q7);

            dataControl->ClientToServer.opMode = DataControl::RunMode;
            dataControl->RobotData.run_mode = DataControl::RunCmd;
            dataControl->PathData.path_data_indx = 0;
            dataControl->PathData.path_struct_indx = 0;
            dataControl->PathData.cycle_count_max = -1*0;
            delay_cnt = 0;
            delay_cnt_max = 200;

            break;
        }
        case DataControl::WaitFeeding1:
        {
            memcpy(dataControl->ClientToServer.desiredJoint, waitJointDeg1, sizeof(double)*NUM_JOINT);
            dataControl->ClientToServer.opMode = DataControl::JointMove;
            dataControl->ClientToServer.subMode = DataControl::JointMotion;
            dataControl->joint_path_index = 0;

            dataControl->KITECHData.wait_feeding = false;
            dataControl->feeding = false;
            dataControl->camera_wait = false;

            break;
        }
        case DataControl::WaitFeeding2:
        {
            memcpy(dataControl->ClientToServer.desiredJoint, waitJointDeg2, sizeof(double)*NUM_JOINT);
            dataControl->ClientToServer.opMode = DataControl::JointMove;
            dataControl->ClientToServer.subMode = DataControl::JointMotion;
            dataControl->joint_path_index = 0;

            dataControl->KITECHData.wait_feeding = true;
            dataControl->feeding = false;
            dataControl->camera_wait = false;

            break;
        }
        default:
        {
            break;
        }
    }
}

void ControlMain::robotPathGenerate(std::vector<double> px, std::vector<double> py, std::vector<double> pz, std::vector<double> q6, std::vector<double> q7)
{
    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        dataControl->PathData.movePath[i].path_x.clear();
        dataControl->PathData.movePath[i].path_y.clear();
        dataControl->PathData.movePath[i].path_z.clear();
        dataControl->PathData.movePath[i].path_q6.clear();
        dataControl->PathData.movePath[i].path_q7.clear();
        path_generator(px[i], px[i + 1], dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_x);
        path_generator(py[i], py[i + 1], dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_y);
        path_generator(pz[i], pz[i + 1], dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_z);
        path_generator(q6[i], q6[i + 1], dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_q6);
        path_generator(q7[i], q7[i + 1], dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_q7);
    }

    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        dataControl->PathData.movePath[i].data_size = dataControl->PathData.movePath[i].path_x.size();
        rt_printf("section %d path size : %d\n", i, dataControl->PathData.movePath[i].data_size);
    }

//    int indx = 1;
//    for (int i = 0; i < dataControl->PathData.row - 1; i++){
//        for (uint j = 0; j < dataControl->PathData.movePath[i].path_x.size(); j++){
//            printf("%d, %f, %f, %f, %f, %f\n",
//                   indx++, dataControl->PathData.movePath[i].path_x[j], dataControl->PathData.movePath[i].path_y[j], dataControl->PathData.movePath[i].path_z[j],
//                   dataControl->PathData.movePath[i].path_q6[j], dataControl->PathData.movePath[i].path_q7[j]);
//        }
//    }
}

void ControlMain::robotVSD()
{
    robotArm->run_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.present_end_pose);

    robotArm->gravity_compensation(dataControl->RobotData.present_q, dataControl->RobotData.present_q_dot, dataControl->RobotData.Tg);

    robotArm->jacobian_zyx();

    for(uint i = 0; i < 6; i++)
    {
        dataControl->RobotData.present_end_vel[i] = 0;
        for(uint j = 0; j < 6; j++)
        {
            dataControl->RobotData.present_end_vel[i] += robotArm->J[i*6 + j]*dataControl->RobotData.present_q_dot[j];
        }
    }

//    dataControl->RobotData.desired_end_pose[0] = dataControl->RobotData.desiredForce/dataControl->RobotData.Kp + dataControl->RobotData.present_end_pose[0];

    rt_printf("desired end pose : %f, %f, %f, %f, %f, %f\n", dataControl->RobotData.desired_end_pose[0], dataControl->RobotData.desired_end_pose[1],
            dataControl->RobotData.desired_end_pose[2], dataControl->RobotData.desired_end_pose[3],
            dataControl->RobotData.desired_end_pose[4], dataControl->RobotData.desired_end_pose[5]);
    rt_printf("present end pose : %f, %f, %f, %f, %f, %f\n\n", dataControl->RobotData.present_end_pose[0], dataControl->RobotData.present_end_pose[1],
            dataControl->RobotData.present_end_pose[2], dataControl->RobotData.present_end_pose[3],
            dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[5]);

//    dataControl->RobotData.Kp = 30;
//    dataControl->RobotData.Dp = 0;
//    dataControl->RobotData.Kr = 30;
//    dataControl->RobotData.Dr = 0;

    for(int i = 0; i < 3; i++){
        dataControl->RobotData.F[i] =
                dataControl->RobotData.Kp*(dataControl->RobotData.desired_end_pose[i] - dataControl->RobotData.present_end_pose[i]) -
                dataControl->RobotData.Dp*(dataControl->RobotData.present_end_vel[i]);
        dataControl->RobotData.F[i+3] =
                dataControl->RobotData.Kr*(dataControl->RobotData.desired_end_pose[i + 3] - dataControl->RobotData.present_end_pose[i + 3]) -
                dataControl->RobotData.Dr*(dataControl->RobotData.present_end_vel[i + 3]);
    }
//    dataControl->RobotData.F[0] = dataControl->RobotData.desiredForce;

    rt_printf("Force vsd : %f\t %f\t %f\t %f\t %f\t %f\n\n",
              dataControl->RobotData.F[0], dataControl->RobotData.F[1], dataControl->RobotData.F[2],
            dataControl->RobotData.F[3], dataControl->RobotData.F[4], dataControl->RobotData.F[5]);


    for(uint i = 0; i < 6; i++){
        dataControl->RobotData.Td[i] = 0;
        for(uint j = 0; j < 6; j++){
            dataControl->RobotData.Td[i] += robotArm->J[j*6 + i]*dataControl->RobotData.F[j];
        }
    }

//    rt_printf("\tTorque vsd : %f\t %f\t %f\t %f\t %f\t %f\n\n",
//              dataControl->RobotData.Td[0], dataControl->RobotData.Td[1], dataControl->RobotData.Td[2],
//            dataControl->RobotData.Td[3], dataControl->RobotData.Td[4], dataControl->RobotData.Td[5]);

//    dataControl->RobotData.T_limit[0] = 30.0;
//    dataControl->RobotData.T_limit[1] = 30.0;
//    dataControl->RobotData.T_limit[2] = 30.0;
//    dataControl->RobotData.T_limit[3] = 30.0;
//    dataControl->RobotData.T_limit[4] = 30.0;
//    dataControl->RobotData.T_limit[5] = 30.0;

//    dataControl->RobotData.T_err = false;
//    for(int i = 0; i < 6; i++){
//        if(abs(dataControl->RobotData.Td[i]) > dataControl->RobotData.T_limit[i]){
//            dataControl->RobotData.T_err = true;
//            rt_printf("\n\tVSD Torque exceed torque limit\n");
//            rt_printf("\tTorque vsd : %f\t %f\t %f\t %f\t %f\t %f\n\n",
//                      dataControl->RobotData.Td[0], dataControl->RobotData.Td[1], dataControl->RobotData.Td[2],
//                    dataControl->RobotData.Td[3], dataControl->RobotData.Td[4], dataControl->RobotData.Td[5]);
//            break;
//        }
//    }
//    if(dataControl->RobotData.T_err)
//        memset(dataControl->RobotData.Td, 0, sizeof(double)*6);

    for(int i = 0; i < 6; i++){
        dataControl->RobotData.T[i] = dataControl->RobotData.Td[i] + dataControl->RobotData.Tg[i];
    }
}

void ControlMain::goalReachCart(double desired_pose[], double present_pose[], bool *goal_reach)
{
    double epsilon_pos = 0.05;
    //    double epsilon_ang = 0.5;

    double pos = sqrt(pow(desired_pose[0] - present_pose[0], 2) + pow(desired_pose[1] - present_pose[1], 2) + pow(desired_pose[2] - present_pose[2], 2));
    //    double ang_x = abs(desired_pose[3] - present_pose[3]);
    //    double ang_y = abs(desired_pose[4] - present_pose[4]);
    //    double ang_z = abs(desired_pose[5] - present_pose[5]);

    //    rt_printf("pos : %f\n", pos);
    //    rt_printf("ang_r : %f\t ang_p : %f\t ang_y : %f\n", ang_r, ang_p, ang_y);

    if (pos < epsilon_pos/* && ang_x < epsilon_ang && ang_y < epsilon_ang && ang_z < epsilon_ang*/){
        *goal_reach = true;
    }
    else{
        *goal_reach = false;
    }
}

void ControlMain::goalReachJoint(long tar_pos[], long cur_pos[], bool *goal_reach)
{
    double epsilon_pos = 10;
    bool v_goal_reach[NUM_JOINT] = {false,};

    for(int i = 0; i < NUM_JOINT; i++){
        if(abs(tar_pos[i] - cur_pos[i]) > epsilon_pos){
            v_goal_reach[i] = false;
        }
        else{
            v_goal_reach[i] = true;
        }
    }
    for(int i = 0; i < NUM_JOINT; i++){
        *goal_reach &= v_goal_reach[i];
    }
}

void ControlMain::path_generator(double x0, double xf, double tf, double ta, double h, std::vector<double> *path)
{
    // if(path_index == 1){
    //     double x_step = (xf - x0)/(tf/h);
    //     double x = x0;
    //     for(double t = 0; t < tf; t += h){
    //         path->push_back(x);
    //         x += x_step;
    //     }
    // }
    // else{
    double td = tf - ta;
    double vd = (xf - x0)/td;
    double xa = x0 + 0.5*ta*vd;
    double xd = xf - 0.5*ta*vd;

    double pos0, posf, vel0, velf, acc0, accf, ts;
    double a0, a1, a2, a3, a4, a5;

    // section of acceleration
    pos0 = x0; posf = xa; vel0 = 0; velf = vd; acc0 = 0; accf = 0; ts = ta;
    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
    a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
    a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

    for(double t = 0; t < ts; t += h){
        path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
    }

    // section of constant velocity
    pos0 = xa; posf = xd; vel0 = vd; velf = vd; acc0 = 0; accf = 0; ts = td - ta;
    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
    a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
    a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

    for(double t = 0; t < ts; t += h){
        path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
    }

    // section of deceleration
    pos0 = xd; posf = xf; vel0 = vd; velf = 0; acc0 = 0; accf = 0; ts = tf - td;
    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
    a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
    a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

    for(double t = 0; t < ts; t += h){
        path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
    }
    // }
}

//void ControlMain::path_generator_circle(double p1, double p2, double pcx, double pcy, std::vector<double> path_x, std::vector<double> *path_y)
//{
//    double xxc2 = pow((p1 - pcx), 2);
//    double yyc2 = pow((p2 - pcy), 2);
//    double r2 = xxc2 + yyc2;

//    for(unsigned int i = 0; i < path_x.size(); i++){
//        path_y->push_back(pcy + sqrt(r2 - pow((path_x[i] - pcx), 2)));
//    }
//}

//void ControlMain::func_q6d7_to_q7(double q6, double d7, double *q7)
//{
//    //    if(d7 >= 25){
//    //        d7 = 25;
//    //    }
//    //    else
//    if(d7 <= -25){
//        d7 = -25;
//    }
//    *q7 = (d7/8.0 + q6*DEG2RAD);
//}

//void ControlMain::func_q6q7_to_d7(double q6, double q7, double *d7)
//{
//    *d7 = 8.0*(q7 + (25.0/32.0)*q6);
//}

// void ControlMain::path_generator(double x0, double xf, double tf, double ta, double h, std::vector<double> *path, int path_index)
// {
// //    rt_printf("Start path generator, x0 : %f, xf : %f, tf : %f, ta : %f, h : %f\n", x0, xf, tf, ta, h);
//     double td = tf - ta;
//     double vd = (xf - x0)/td;
//     double xa = x0 + 0.5*ta*vd;
//     double xd = xf - 0.5*ta*vd;

//     double pos0, posf, vel0, velf, acc0, accf, ts;
//     double a0, a1, a2, a3, a4, a5;

//     // section of acceleration
//     pos0 = x0; posf = xa; vel0 = 0; velf = vd; acc0 = 0; accf = 0; ts = ta;
//     a0 = pos0;
//     a1 = vel0;
//     a2 = acc0/2;
//     a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
//     a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
//     a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

//     for(double t = 0; t < ts; t += h){
//         path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
//     }

//     // section of constant velocity
//     pos0 = xa; posf = xd; vel0 = vd; velf = vd; acc0 = 0; accf = 0; ts = td - ta;
//     a0 = pos0;
//     a1 = vel0;
//     a2 = acc0/2;
//     a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
//     a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
//     a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

//     for(double t = 0; t < ts; t += h){
//         path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
//     }

//     // section of deceleration
//     pos0 = xd; posf = xf; vel0 = vd; velf = 0; acc0 = 0; accf = 0; ts = tf - td;
//     a0 = pos0;
//     a1 = vel0;
//     a2 = acc0/2;
//     a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
//     a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
//     a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

//     for(double t = 0; t < ts; t += h){
//         path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
//     }
// }
