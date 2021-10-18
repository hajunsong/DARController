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
            case DataControl::CartesianMove:
                pThis->robotCartesianMove(pThis->dataControl->ClientToServer.subMode, pThis->dataControl->ClientToServer.desiredPose);
                break;
            case DataControl::PathGenerateMode:
                break;
            case DataControl::ReadyMode:
                pThis->robotReady();
                break;
            case DataControl::RunMode:
                pThis->robotRun();
                break;
            case DataControl::TorqueID:
                //                pThis->robotSPGC();
                break;
            case DataControl::OperateMode:
                pThis->robotOperate();
                break;
            case DataControl::ObiMode:
                break;
            case DataControl::TestMode:
                //                pThis->robotTest();
                break;
            default : break;
        }

        pThis->robotKinematics();
        //        pThis->robotDynamics();

        pThis->dataControl->RobotData.data_index++;
        pThis->dataControl->RobotData.op_mode = pThis->dataControl->ClientToServer.opMode;
        for(int i = 0; i < NUM_JOINT; i++){
            pThis->dataControl->RobotData.cur_status_word[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_cur_status_word;
            pThis->dataControl->RobotData.tar_control_word[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_tar_control_word;
            pThis->dataControl->RobotData.cur_modes_of_operation[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_cur_modes_of_operation;
            pThis->dataControl->RobotData.cur_position[i] = pThis->dataControl->module_dir[i]*(pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_cur_position - pThis->dataControl->joint_offset[i]);
            pThis->dataControl->RobotData.cur_velocity[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_cur_velocity;
            pThis->dataControl->RobotData.cur_torque[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_cur_torque_value1;
            pThis->dataControl->RobotData.tar_position[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_tar_position;
            pThis->dataControl->RobotData.tar_torque[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_tar_torque;
            pThis->dataControl->RobotData.err_state[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_cur_err_state_num;
            //                pThis->dataControl->err_msg[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_cur_err_state_msg;
        }

        if(pThis->dataControl->current_state_print){
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
        }

        if(pThis->dataControl->logging_enable && pThis->logging_thread_run){
            pThis->dataControl->loggingRobotData.resize(pThis->dataControl->loggingRobotData.size() + 1);
            memcpy(&pThis->dataControl->loggingRobotData.back(), &pThis->dataControl->RobotData, sizeof(DataControl::StructRobotData));
            //            for(unsigned int i = 0; i < NUM_JOINT; i++){
            //                pThis->dataControl->loggingErrorMessage[i].resize(pThis->dataControl->loggingErrorMessage[i].size() + 1);
            //                pThis->dataControl->loggingErrorMessage[i].push_back(pThis->dataControl->err_msg[i]);
            //            }
            //            rt_printf("data index : %d\n", pThis->dataControl->RobotData.data_index);
        }

        if(pThis->tcpServer->isConnected()){
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
                pThis->dataControl->ServerToClient.back().tar_position[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_tar_position;
                pThis->dataControl->ServerToClient.back().tar_torque[i] = pThis->ecatInterface->v_global_ethercat_control.f_get_slave()[i].v_global_tar_torque;
                pThis->dataControl->cartesianPoseScaleUp(pThis->dataControl->RobotData.present_end_pose, pThis->dataControl->ServerToClient.back().presentCartesianPose);
                pThis->dataControl->cartesianPoseScaleUp(pThis->dataControl->RobotData.present_end_vel, pThis->dataControl->ServerToClient.back().presentCartesianVelocity);
            }
        }
        else{
            if(!pThis->dataControl->tablet_mode){
                pThis->robot_task_run = false;
            }
        }

        now = rt_timer_read();
    }
}

void ControlMain::robotServoOn(char servo){
    if(servo){
        ecatInterface->servo_on();
    }
    else{
        ecatInterface->servo_off();
    }

    dataControl->ClientToServer.opMode = DataControl::Wait;
}

void ControlMain::robotInitialize()
{
    //    dataControl->DataReset();
    //    init_thread_run = true;
    //    pthread_create(&init_thread, NULL, init_func, this);
}

void ControlMain::robotWait()
{
    //                pThis->ecatInterface->servo_stop();
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


    if(dataControl->operateMode.mode == DataControl::Start){
        dataControl->KITECHData.camera_request = true;
    }
}

void ControlMain::robotJointMove(char mode, double desJoint[])
{
    ecatInterface->set_led(2);
    switch(mode){
        case DataControl::JogMotion:
            for(unsigned char i = 0; i < NUM_JOINT; i++){
                dataControl->RobotData.tar_position[i] = dataControl->RobotData.cur_position[i] + static_cast<long>(desJoint[i]*DEG2ENC);
            }

            ecatInterface->servo_run(dataControl->RobotData.tar_position, dataControl->module_dir, dataControl->joint_offset);

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
                dataControl->RobotData.tar_position[i] = dataControl->joint_path[i][dataControl->joint_path_index]*DEG2ENC;
            }
            ecatInterface->servo_run(dataControl->RobotData.tar_position, dataControl->module_dir, dataControl->joint_offset);

            //            rt_printf("joint path index : %d\n", dataControl->joint_path_index++);
            dataControl->joint_path_index++;
            if(dataControl->joint_path_index >= dataControl->joint_path[0].size()){
                ecatInterface->set_led(0);
                dataControl->ClientToServer.opMode = DataControl::Wait;

                ecatInterface->set_led(3);
                dataControl->ClientToServer.subMode = -1;
                dataControl->joint_path_index = 0;
            }
            break;
        default :
            break;
    }
}

void ControlMain::robotCartesianMove(char mode, double desCartesian[NUM_DOF])
{
    dataControl->cartesianPoseScaleDown(desCartesian, dataControl->RobotData.desired_end_pose);

//    switch(mode){
//        case DataControl::CartesianJogMotion:
//            for(unsigned char i = 0; i < NUM_DOF; i++){
//                dataControl->RobotData.desired_end_pose[i] += dataControl->RobotData.present_end_pose[i];
//            }
//            break;
//        case DataControl::CartesianMotion:
//            break;
//        default:
//            break;
//    }

    if(dataControl->ClientToServer.move_time > 0 && dataControl->ClientToServer.acc_time > 0
            && dataControl->ClientToServer.move_time >= dataControl->ClientToServer.acc_time){
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
        double time = 0;
        for(uint i = 0; i < dataControl->PathData.row; i++){
            dataControl->PathData.movePath[i].path_x.clear();
            dataControl->PathData.movePath[i].path_y.clear();
            dataControl->PathData.movePath[i].path_z.clear();
            dataControl->PathData.movePath[i].path_theta.clear();

            dataControl->PathData.total_time.push_back(time);
            dataControl->PathData.acc_time.push_back(dataControl->ClientToServer.acc_time);
            time += dataControl->ClientToServer.move_time;
        }

        dataControl->PathData.point_px.push_back(dataControl->RobotData.present_end_pose[0]);
        dataControl->PathData.point_py.push_back(dataControl->RobotData.present_end_pose[1]);
        dataControl->PathData.point_pz.push_back(dataControl->RobotData.present_end_pose[2]);
        dataControl->PathData.point_rx.push_back(dataControl->RobotData.present_end_pose[3]);
        dataControl->PathData.point_ry.push_back(dataControl->RobotData.present_end_pose[4]);
        dataControl->PathData.point_rz.push_back(dataControl->RobotData.present_end_pose[5]);

        dataControl->PathData.point_px.push_back(dataControl->RobotData.desired_end_pose[0]);
        dataControl->PathData.point_py.push_back(dataControl->RobotData.desired_end_pose[1]);
        dataControl->PathData.point_pz.push_back(dataControl->RobotData.desired_end_pose[2]);
        dataControl->PathData.point_rx.push_back(dataControl->RobotData.present_end_pose[3]);
        dataControl->PathData.point_ry.push_back(dataControl->RobotData.present_end_pose[4]);
        dataControl->PathData.point_rz.push_back(dataControl->RobotData.present_end_pose[5]);

        rt_printf("path data : \n");
        for(uint8_t i = 0; i < dataControl->PathData.row; i ++){
            rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                      dataControl->PathData.total_time[i],
                      dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                      dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                      dataControl->PathData.acc_time[i]);
        }

        robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz,
                          dataControl->PathData.point_rx, dataControl->PathData.point_ry, dataControl->PathData.point_rz);

        //        RobotArm::zyx2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], dataControl->PathData.movePath[0].R_init);
        //        memcpy(dataControl->PathData.movePath[0].R_init, robotArm->body[robotArm->num_body].Ae, sizeof(double)*9);

        dataControl->ClientToServer.opMode = DataControl::RunMode;
        dataControl->RobotData.run_mode = DataControl::CalMode;
        dataControl->PathData.path_data_indx = 0;
        dataControl->PathData.path_struct_indx = 0;
        dataControl->PathData.cycle_count_max = 1;
        delay_cnt = 0;
        delay_cnt_max = 200;
    }
}

void ControlMain::robotKinematics(){
    for(int i = 0; i < 6; i++){
        dataControl->RobotData.present_q[i] = dataControl->RobotData.cur_position[i]*ENC2DEG*DEG2RAD;
    }
    //    dataControl->RobotData.present_q[5] = 0;

    //    func_q6q7_to_d7(dataControl->RobotData.cur_position[6]*ENC2DEG*DEG2RAD, dataControl->RobotData.cur_position[5]*ENC2DEG*DEG2RAD, &dataControl->RobotData.d7);
    //    robotArm->set_tool_offset(0.148 + abs(dataControl->RobotData.d7)*0.001, dataControl->RobotData.d7 >= 0 ? -0.003 : 0.007, 0);
    //    dataControl->RobotData.present_q[5] = dataControl->RobotData.cur_position[6]*ENC2DEG*DEG2RAD*(-25.0/32.0)*0;

    robotArm->run_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.present_end_pose);


    //    dataControl->jointVelocityENC2RAD(dataControl->RobotData.cur_velocity, dataControl->RobotData.present_q_dot);
    //    robotArm->jacobian_zyx();

    //    double velocity;
    //    for(uint i = 0; i < 6; i++)
    //    {
    //        velocity = 0;
    //        for(uint j = 0; j < 6; j++)
    //        {
    //            velocity += robotArm->J[i*6 + j]*dataControl->RobotData.present_q_dot[j];
    //        }
    //        dataControl->RobotData.present_end_vel[i] = velocity;
    //    }
}

void ControlMain::robotReady()
{
    path_generator(dataControl->RobotData.present_end_pose[0], dataControl->PathData.point_px[0], 5.0, 1.0, step_size, &dataControl->PathData.readyPath.path_x, 0);
    path_generator(dataControl->RobotData.present_end_pose[1], dataControl->PathData.point_py[0], 5.0, 1.0, step_size, &dataControl->PathData.readyPath.path_y, 0);
    path_generator(dataControl->RobotData.present_end_pose[2], dataControl->PathData.point_pz[0], 5.0, 1.0, step_size, &dataControl->PathData.readyPath.path_z, 0);

    //    double R_init[9], R_final[9], r[3], theta;
    //    RobotArm::zyx2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], R_init);
    //    RobotArm::zyx2mat(dataControl->PathData.point_rz[0], dataControl->PathData.point_ry[0], dataControl->PathData.point_rx[0], R_final);
    //    RobotArm::mat_to_axis_angle(R_init, R_final, r, &theta);
    //    memcpy(dataControl->PathData.readyPath.r, r, sizeof(double)*3);

    //    path_generator(0, theta, 2.0, 0.4, step_size, &dataControl->PathData.readyPath.path_theta, 0);
    //    memcpy(dataControl->PathData.readyPath.R_init, robotArm->body[robotArm->num_body].Ae, sizeof(double)*9);

    dataControl->RobotData.run_mode = DataControl::ReadyCmd;

    dataControl->PathData.readyPath.data_size = dataControl->PathData.readyPath.path_x.size();
    rt_printf("ready path size : %d\n", dataControl->PathData.readyPath.data_size);
    dataControl->ClientToServer.opMode = DataControl::RunMode;

    dataControl->PathData.path_data_indx = 0;
    delay_cnt = 0;
}

void ControlMain::robotRun()
{
    switch(dataControl->RobotData.run_mode){
        case DataControl::ReadyCmd:
        {
            dataControl->RobotData.desired_end_pose[0] = dataControl->PathData.readyPath.path_x[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[1] = dataControl->PathData.readyPath.path_y[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[2] = dataControl->PathData.readyPath.path_z[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[3] = feedingOrientation[0];
            dataControl->RobotData.desired_end_pose[4] = feedingOrientation[1];
            dataControl->RobotData.desired_end_pose[5] = feedingOrientation[2];

            for(int i = 0; i < 6; i++){
                dataControl->RobotData.present_q[i] = dataControl->RobotData.cur_position[i]*ENC2DEG*DEG2RAD;
            }
            dataControl->RobotData.present_q[5] = 0;

            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);

            dataControl->RobotData.tar_position[0] = dataControl->RobotData.desired_q[0]*RAD2DEG*DEG2ENC;
            dataControl->RobotData.tar_position[1] = dataControl->RobotData.desired_q[1]*RAD2DEG*DEG2ENC;
            dataControl->RobotData.tar_position[2] = dataControl->RobotData.desired_q[2]*RAD2DEG*DEG2ENC;
            dataControl->RobotData.tar_position[3] = dataControl->RobotData.desired_q[3]*RAD2DEG*DEG2ENC;
            dataControl->RobotData.tar_position[4] = dataControl->RobotData.desired_q[4]*RAD2DEG*DEG2ENC;
            dataControl->RobotData.tar_position[5] = dataControl->RobotData.desired_q7*RAD2DEG*DEG2ENC*0;
            dataControl->RobotData.tar_position[6] = dataControl->RobotData.desired_q6*RAD2DEG*DEG2ENC*0;

            ecatInterface->servo_run(dataControl->RobotData.tar_position, dataControl->module_dir, dataControl->joint_offset);

#if PRINT_ON
            rt_printf("path_data_indx : %d\n", dataControl->PathData.path_data_indx);
#endif

            dataControl->PathData.path_data_indx += 1;
            ecatInterface->set_led(2);

            if (dataControl->PathData.path_data_indx >= dataControl->PathData.readyPath.data_size - 1){
                dataControl->PathData.path_data_indx = 0;
                dataControl->ClientToServer.opMode = DataControl::Wait;
                ecatInterface->set_led(1);

                robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz, dataControl->PathData.point_beta, dataControl->PathData.point_d7);
            }

            break;
        }
        case DataControl::RunCmd:
        {
            dataControl->RobotData.desired_end_pose[0] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_x[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[1] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_y[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[2] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_z[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[3] = feedingOrientation[0];
            dataControl->RobotData.desired_end_pose[4] = feedingOrientation[1];
            dataControl->RobotData.desired_end_pose[5] = feedingOrientation[2];
            dataControl->RobotData.desired_beta = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_beta[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_d7 = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_d7[dataControl->PathData.path_data_indx];

            //                rt_printf("index : %d, des_beta : %f, des_d7 : %f\n", dataControl->PathData.path_data_indx, dataControl->RobotData.desired_beta, dataControl->RobotData.desired_d7);

            for(int i = 0; i < 6; i++){
                dataControl->RobotData.present_q[i] = dataControl->RobotData.cur_position[i]*ENC2DEG*DEG2RAD;
                dataControl->RobotData.present_q[5] = 0;
            }
            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);

            if (delay_cnt == 0){
                dataControl->RobotData.desired_q6 = dataControl->RobotData.desired_beta*(-32.0/25.0);
                func_q6d7_to_q7(dataControl->RobotData.desired_q6, dataControl->RobotData.desired_d7, &dataControl->RobotData.desired_q7);

                dataControl->RobotData.tar_position[0] = dataControl->RobotData.desired_q[0]*RAD2DEG*DEG2ENC;
                dataControl->RobotData.tar_position[1] = dataControl->RobotData.desired_q[1]*RAD2DEG*DEG2ENC;
                dataControl->RobotData.tar_position[2] = dataControl->RobotData.desired_q[2]*RAD2DEG*DEG2ENC;
                dataControl->RobotData.tar_position[3] = dataControl->RobotData.desired_q[3]*RAD2DEG*DEG2ENC;
                dataControl->RobotData.tar_position[4] = dataControl->RobotData.desired_q[4]*RAD2DEG*DEG2ENC;
                dataControl->RobotData.tar_position[5] = dataControl->RobotData.desired_q7*RAD2DEG*DEG2ENC;
                dataControl->RobotData.tar_position[6] = dataControl->RobotData.desired_q6*DEG2ENC;

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
                                    break;
                                case DataControl::Mouse:
                                    dataControl->operateMode.section = DataControl::Home;
                                    dataControl->ClientToServer.opMode = DataControl::OperateMode;
                                    break;
                                case DataControl::Home:
                                    dataControl->ClientToServer.opMode = DataControl::OperateMode;
                                    dataControl->operateMode.mode = DataControl::Start;
                                    //                                    dataControl->ClientToServer.opMode = DataControl::Wait;
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
        case DataControl::CalMode:
        {
            dataControl->RobotData.desired_end_pose[0] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_x[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[1] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_y[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[2] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_z[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[3] = dataControl->PathData.point_rx[0];
            dataControl->RobotData.desired_end_pose[4] = dataControl->PathData.point_ry[0];
            dataControl->RobotData.desired_end_pose[5] = dataControl->PathData.point_rz[0];

            for(int i = 0; i < 6; i++){
                dataControl->RobotData.present_q[i] = dataControl->RobotData.cur_position[i]*ENC2DEG*DEG2RAD;
            }
            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);

            if(delay_cnt == 0){
                dataControl->RobotData.tar_position[0] = dataControl->RobotData.desired_q[0]*RAD2DEG*DEG2ENC;
                dataControl->RobotData.tar_position[1] = dataControl->RobotData.desired_q[1]*RAD2DEG*DEG2ENC;
                dataControl->RobotData.tar_position[2] = dataControl->RobotData.desired_q[2]*RAD2DEG*DEG2ENC;
                dataControl->RobotData.tar_position[3] = dataControl->RobotData.desired_q[3]*RAD2DEG*DEG2ENC;
                dataControl->RobotData.tar_position[4] = dataControl->RobotData.desired_q[4]*RAD2DEG*DEG2ENC;
                dataControl->RobotData.tar_position[5] = dataControl->RobotData.desired_q[5]*RAD2DEG*DEG2ENC;
                dataControl->RobotData.tar_position[6] = dataControl->RobotData.tar_position[5];

                ecatInterface->servo_run(dataControl->RobotData.tar_position, dataControl->module_dir, dataControl->joint_offset);
                dataControl->PathData.path_data_indx += 1;
            }

            ecatInterface->set_led(2);

            if (dataControl->PathData.path_data_indx >= dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].data_size - 1){
                //                rt_printf("delay cnt : %d\n", delay_cnt);
                delay_cnt++;

                if(delay_cnt >= delay_cnt_max){
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx++;
                    delay_cnt = 0;
                }

                if (dataControl->PathData.path_struct_indx >= dataControl->PathData.row - 1){
                    if(dataControl->PathData.cycle_count_max == -1)
                    {
                        dataControl->PathData.path_data_indx = 0;
                        dataControl->PathData.path_struct_indx = 0;
                    }
                    else
                    {
                        dataControl->ClientToServer.opMode = DataControl::Wait;

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

            for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                          dataControl->PathData.total_time[i],
                          dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                          dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                          dataControl->PathData.acc_time[i]);
            }

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
            dataControl->PathData.point_beta.clear();
            dataControl->PathData.point_d7.clear();
            dataControl->PathData.point_option.clear();
            dataControl->PathData.acc_time.clear();

            dataControl->PathData.row = 2;
            dataControl->PathData.movePath.clear();
            dataControl->PathData.movePath.resize(dataControl->PathData.row);

            double time = 0;
            for(uint i = 0; i < dataControl->PathData.row; i++){
                dataControl->PathData.movePath[i].path_x.clear();
                dataControl->PathData.movePath[i].path_y.clear();
                dataControl->PathData.movePath[i].path_z.clear();
                dataControl->PathData.movePath[i].path_theta.clear();
                dataControl->PathData.movePath[i].path_beta.clear();
                dataControl->PathData.movePath[i].path_d7.clear();

                dataControl->PathData.total_time.push_back(time);
                dataControl->PathData.acc_time.push_back(1);
                time += dataControl->select_speed;
            }

            dataControl->PathData.point_px.push_back(dataControl->RobotData.present_end_pose[0]);
            dataControl->PathData.point_py.push_back(dataControl->RobotData.present_end_pose[1]);
            dataControl->PathData.point_pz.push_back(dataControl->RobotData.present_end_pose[2]);
            dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
            dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
            dataControl->PathData.point_rz.push_back(feedingOrientation[2]);

            dataControl->cur_beta = dataControl->RobotData.cur_position[6]*ENC2DEG*(-32.0/25.0);
            func_q6q7_to_d7(dataControl->RobotData.cur_position[6]*ENC2DEG, dataControl->RobotData.cur_position[5]*ENC2DEG, &dataControl->cur_d7);

            dataControl->PathData.point_beta.push_back(0);
            dataControl->PathData.point_d7.push_back(0);
            dataControl->PathData.point_option.push_back(0);

            switch(dataControl->operateMode.section){
                case DataControl::Side1:
                {
                    dataControl->PathData.point_px.push_back(diningWaypoints.diningSection[0].x[0]);
                    dataControl->PathData.point_py.push_back(diningWaypoints.diningSection[0].y[0]);
                    dataControl->PathData.point_pz.push_back(diningWaypoints.diningSection[0].z[0]);
                    dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                    dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                    dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                    dataControl->PathData.point_beta.push_back(diningWaypoints.diningSection[0].beta[0]);
                    dataControl->PathData.point_d7.push_back(diningWaypoints.diningSection[0].d7[0]);
                    dataControl->PathData.point_option.push_back(diningWaypoints.diningSection[0].option[0]);

                    break;
                }
                case DataControl::Side2:
                {
                    dataControl->PathData.point_px.push_back(diningWaypoints.diningSection[1].x[0]);
                    dataControl->PathData.point_py.push_back(diningWaypoints.diningSection[1].y[0]);
                    dataControl->PathData.point_pz.push_back(diningWaypoints.diningSection[1].z[0]);
                    dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                    dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                    dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                    dataControl->PathData.point_beta.push_back(diningWaypoints.diningSection[1].beta[0]);
                    dataControl->PathData.point_d7.push_back(diningWaypoints.diningSection[1].d7[0]);
                    dataControl->PathData.point_option.push_back(diningWaypoints.diningSection[1].option[0]);

                    break;
                }
                case DataControl::Side3:
                {
                    dataControl->PathData.point_px.push_back(diningWaypoints.diningSection[2].x[0]);
                    dataControl->PathData.point_py.push_back(diningWaypoints.diningSection[2].y[0]);
                    dataControl->PathData.point_pz.push_back(diningWaypoints.diningSection[2].z[0]);
                    dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                    dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                    dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                    dataControl->PathData.point_beta.push_back(diningWaypoints.diningSection[2].beta[0]);
                    dataControl->PathData.point_d7.push_back(diningWaypoints.diningSection[2].d7[0]);
                    dataControl->PathData.point_option.push_back(diningWaypoints.diningSection[2].option[0]);

                    break;
                }
                case DataControl::Soup:
                {
                    dataControl->PathData.point_px.push_back(diningWaypoints.diningSection[3].x[0]);
                    dataControl->PathData.point_py.push_back(diningWaypoints.diningSection[3].y[0]);
                    dataControl->PathData.point_pz.push_back(diningWaypoints.diningSection[3].z[0]);
                    dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                    dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                    dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                    dataControl->PathData.point_beta.push_back(diningWaypoints.diningSection[3].beta[0]);
                    dataControl->PathData.point_d7.push_back(diningWaypoints.diningSection[3].d7[0]);
                    dataControl->PathData.point_option.push_back(diningWaypoints.diningSection[3].option[0]);

                    break;
                }
                case DataControl::Rice:
                {
                    dataControl->PathData.point_px.push_back(diningWaypoints.diningSection[4].x[0]);
                    dataControl->PathData.point_py.push_back(diningWaypoints.diningSection[4].y[0]);
                    dataControl->PathData.point_pz.push_back(diningWaypoints.diningSection[4].z[0]);
                    dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                    dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                    dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                    dataControl->PathData.point_beta.push_back(diningWaypoints.diningSection[4].beta[0]);
                    dataControl->PathData.point_d7.push_back(diningWaypoints.diningSection[4].d7[0]);
                    dataControl->PathData.point_option.push_back(diningWaypoints.diningSection[4].option[0]);

                    break;
                }
            }

            rt_printf("path data : \n");
            for(uint8_t i = 0; i < dataControl->PathData.row; i ++){
                rt_printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d\n",
                          dataControl->PathData.total_time[i],
                          dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i], dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                          dataControl->PathData.point_beta[i], dataControl->PathData.point_d7[i],
                          dataControl->PathData.acc_time[i], dataControl->PathData.point_option[i]);
            }

            robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz, dataControl->PathData.point_beta, dataControl->PathData.point_d7, dataControl->PathData.point_option, diningWaypoints.get_z_offset(0) + 0.1);
//            robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz, dataControl->PathData.point_rx, dataControl->PathData.point_ry, dataControl->PathData.point_rz);

            dataControl->ClientToServer.opMode = DataControl::RunMode;
            dataControl->RobotData.run_mode = DataControl::RunCmd;
            dataControl->PathData.path_data_indx = 0;
            dataControl->PathData.path_struct_indx = 0;
            dataControl->PathData.cycle_count_max = 1;
            delay_cnt = 0;
            delay_cnt_max = 200;

            break;
        }
        case DataControl::Feeding:
        {
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
            dataControl->PathData.point_beta.clear();
            dataControl->PathData.point_d7.clear();
            dataControl->PathData.point_option.clear();
            dataControl->PathData.acc_time.clear();

            switch(dataControl->operateMode.section){
                case DataControl::Side1:
                    dataControl->PathData.row = diningWaypoints.diningSection[0].x.size() + 2;
                    break;
                case DataControl::Side2:
                    dataControl->PathData.row = diningWaypoints.diningSection[1].x.size() + 2;
                    break;
                case DataControl::Side3:
                    dataControl->PathData.row = diningWaypoints.diningSection[2].x.size() + 2;
                    break;
                case DataControl::Soup:
                    dataControl->PathData.row = diningWaypoints.diningSection[3].x.size() + 2;
                    break;
                case DataControl::Rice:
                    dataControl->PathData.row = diningWaypoints.diningSection[4].x.size() + 2;
                    break;
                case DataControl::Mouse:
                case DataControl::Home:
                    dataControl->PathData.row = 2;
                    break;
                case DataControl::Test:
                    dataControl->PathData.row = 4;
                    break;

            }
            dataControl->PathData.movePath.clear();
            dataControl->PathData.movePath.resize(dataControl->PathData.row);

            double time = 0;
            if(dataControl->operateMode.section <= DataControl::Rice){
                for(uint i = 0; i < dataControl->PathData.row; i++){
                    dataControl->PathData.movePath[i].path_x.clear();
                    dataControl->PathData.movePath[i].path_y.clear();
                    dataControl->PathData.movePath[i].path_z.clear();
                    dataControl->PathData.movePath[i].path_theta.clear();
                    dataControl->PathData.movePath[i].path_beta.clear();
                    dataControl->PathData.movePath[i].path_d7.clear();

                    dataControl->PathData.total_time.push_back(time);
                    dataControl->PathData.acc_time.push_back(1);
                    time += dataControl->select_speed;
                }
            }
            else{
                for(uint i = 0; i < dataControl->PathData.row; i++){
                    dataControl->PathData.movePath[i].path_x.clear();
                    dataControl->PathData.movePath[i].path_y.clear();
                    dataControl->PathData.movePath[i].path_z.clear();
                    dataControl->PathData.movePath[i].path_theta.clear();
                    dataControl->PathData.movePath[i].path_beta.clear();
                    dataControl->PathData.movePath[i].path_d7.clear();

                    dataControl->PathData.total_time.push_back(time);
                    dataControl->PathData.acc_time.push_back(1);
                    time += dataControl->dining_speed_default*dataControl->dining_speed;
                }
            }

            dataControl->PathData.point_px.push_back(dataControl->RobotData.present_end_pose[0]);
            dataControl->PathData.point_py.push_back(dataControl->RobotData.present_end_pose[1]);
            dataControl->PathData.point_pz.push_back(dataControl->RobotData.present_end_pose[2]);
            dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
            dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
            dataControl->PathData.point_rz.push_back(feedingOrientation[2]);

            if(dataControl->operateMode.section <= DataControl::Rice){

                dataControl->cur_beta = dataControl->RobotData.cur_position[6]*ENC2DEG*(-32.0/25.0);
                func_q6q7_to_d7(dataControl->RobotData.cur_position[6]*ENC2DEG, dataControl->RobotData.cur_position[5]*ENC2DEG, &dataControl->cur_d7);

                dataControl->PathData.point_beta.push_back(0);
                dataControl->PathData.point_d7.push_back(0);
                rt_printf("cur_beta : %f, cur_d7 : %f\n", dataControl->cur_beta, dataControl->cur_d7);
            }
            else{
                dataControl->PathData.point_beta.push_back(dataControl->PathData.teaching_beta);
                dataControl->PathData.point_d7.push_back(dataControl->PathData.teaching_d7);
            }
            dataControl->PathData.point_option.push_back(0);

            switch(dataControl->operateMode.section){
                case DataControl::Side1:
                {
                    for(unsigned int i = 0; i < dataControl->PathData.row - 2; i++){
                        dataControl->PathData.point_px.push_back(diningWaypoints.diningSection[0].x[i] + dataControl->KITECHData.food_pos[0]*0.001);
                        dataControl->PathData.point_py.push_back(diningWaypoints.diningSection[0].y[i] - dataControl->KITECHData.food_pos[1]*0.001);
//                        dataControl->PathData.point_px.push_back(diningWaypoints.diningSection[0].x[i] + diningWaypoints.get_side1_offset(dataControl->trayInfor.section1, 0));
//                        dataControl->PathData.point_py.push_back(diningWaypoints.diningSection[0].y[i] + diningWaypoints.get_side1_offset(dataControl->trayInfor.section1, 1));
                        dataControl->PathData.point_pz.push_back(diningWaypoints.diningSection[0].z[i]);
                        dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                        dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                        dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                        dataControl->PathData.point_beta.push_back(diningWaypoints.diningSection[0].beta[i]);
                        dataControl->PathData.point_d7.push_back(diningWaypoints.diningSection[0].d7[i]);
                        dataControl->PathData.point_option.push_back(diningWaypoints.diningSection[0].option[i]);
                    }
                    dataControl->trayInfor.section1_cnt++;
                    break;
                }
                case DataControl::Side2:
                {
                    for(unsigned int i = 0; i < dataControl->PathData.row - 2; i++){
                        dataControl->PathData.point_px.push_back(diningWaypoints.diningSection[1].x[i] + dataControl->KITECHData.food_pos[2]*0.001);
                        dataControl->PathData.point_py.push_back(diningWaypoints.diningSection[1].y[i] - dataControl->KITECHData.food_pos[3]*0.001);
//                        dataControl->PathData.point_px.push_back(diningWaypoints.diningSection[1].x[i] + diningWaypoints.get_side2_offset(dataControl->trayInfor.section2, 0));
//                        dataControl->PathData.point_py.push_back(diningWaypoints.diningSection[1].y[i] + diningWaypoints.get_side2_offset(dataControl->trayInfor.section2, 1));
                        dataControl->PathData.point_pz.push_back(diningWaypoints.diningSection[1].z[i]);
                        dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                        dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                        dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                        dataControl->PathData.point_beta.push_back(diningWaypoints.diningSection[1].beta[i]);
                        dataControl->PathData.point_d7.push_back(diningWaypoints.diningSection[1].d7[i]);
                        dataControl->PathData.point_option.push_back(diningWaypoints.diningSection[1].option[i]);
                    }
                    dataControl->trayInfor.section2_cnt++;
                    break;
                }
                case DataControl::Side3:
                {
                    for(unsigned int i = 0; i < dataControl->PathData.row - 2; i++){
                        dataControl->PathData.point_px.push_back(diningWaypoints.diningSection[2].x[i] + dataControl->KITECHData.food_pos[4]*0.001);
                        dataControl->PathData.point_py.push_back(diningWaypoints.diningSection[2].y[i] - dataControl->KITECHData.food_pos[5]*0.001);
//                        dataControl->PathData.point_px.push_back(diningWaypoints.diningSection[2].x[i] + diningWaypoints.get_side3_offset(dataControl->trayInfor.section3, 0));
//                        dataControl->PathData.point_py.push_back(diningWaypoints.diningSection[2].y[i] + diningWaypoints.get_side3_offset(dataControl->trayInfor.section3, 1));
                        dataControl->PathData.point_pz.push_back(diningWaypoints.diningSection[2].z[i]);
                        dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                        dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                        dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                        dataControl->PathData.point_beta.push_back(diningWaypoints.diningSection[2].beta[i]);
                        dataControl->PathData.point_d7.push_back(diningWaypoints.diningSection[2].d7[i]);
                        dataControl->PathData.point_option.push_back(diningWaypoints.diningSection[2].option[i]);
                    }
                    dataControl->trayInfor.section3_cnt++;
                    break;
                }
                case DataControl::Soup:
                {
                    for(unsigned int i = 0; i < dataControl->PathData.row - 2; i++){
                        dataControl->PathData.point_px.push_back(diningWaypoints.diningSection[3].x[i]);
                        dataControl->PathData.point_py.push_back(diningWaypoints.diningSection[3].y[i]);
                        dataControl->PathData.point_pz.push_back(diningWaypoints.diningSection[3].z[i]);
                        dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                        dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                        dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                        dataControl->PathData.point_beta.push_back(diningWaypoints.diningSection[3].beta[i]);
                        dataControl->PathData.point_d7.push_back(diningWaypoints.diningSection[3].d7[i]);
                        dataControl->PathData.point_option.push_back(diningWaypoints.diningSection[3].option[i]);
                    }
                    break;
                }
                case DataControl::Rice:
                {
                    for(unsigned int i = 0; i < dataControl->PathData.row - 2; i++){
                        dataControl->PathData.point_px.push_back(diningWaypoints.diningSection[4].x[i] + diningWaypoints.get_side5_x_offset(dataControl->trayInfor.section5));
                        dataControl->PathData.point_py.push_back(diningWaypoints.diningSection[4].y[i] + diningWaypoints.get_side5_y_offset(dataControl->trayInfor.section5));
                        dataControl->PathData.point_pz.push_back(diningWaypoints.diningSection[4].z[i]);
                        dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                        dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                        dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                        dataControl->PathData.point_beta.push_back(diningWaypoints.diningSection[4].beta[i]);
                        dataControl->PathData.point_d7.push_back(diningWaypoints.diningSection[4].d7[i]);
                        dataControl->PathData.point_option.push_back(diningWaypoints.diningSection[4].option[i]);
                    }

                    dataControl->trayInfor.section5_cnt++;

                    if(dataControl->trayInfor.section5 == 3){
                        dataControl->trayInfor.section5_cnt = 0;
                    }
                    break;
                }
                case DataControl::Mouse:
                {
                    rt_printf("go mouse\n");

                    dataControl->PathData.point_px.push_back(dataControl->PathData.teaching_pose[0]);
                    dataControl->PathData.point_py.push_back(dataControl->PathData.teaching_pose[1]);
                    //                    if(dataControl->PathData.teaching_beta != 0){
                    //                        dataControl->PathData.point_pz.push_back(dataControl->PathData.teaching_pose[2] + (0.168 - 0.019)*sin(abs(dataControl->PathData.teaching_beta)*DEG2RAD) + 0.015);
                    //                    }
                    //                    else{
                    dataControl->PathData.point_pz.push_back(dataControl->PathData.teaching_pose[2]);
                    //                    }
                    dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                    dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                    dataControl->PathData.point_rz.push_back(feedingOrientation[2]);

                    dataControl->cur_beta = dataControl->RobotData.cur_position[6]*ENC2DEG*(-32.0/25.0);
                    func_q6q7_to_d7(dataControl->RobotData.cur_position[6]*ENC2DEG, dataControl->RobotData.cur_position[5]*ENC2DEG, &dataControl->cur_d7);

                    dataControl->PathData.point_beta.push_back(dataControl->PathData.teaching_beta);
                    dataControl->PathData.point_d7.push_back(dataControl->PathData.teaching_d7);
                    rt_printf("cur_beta : %f, cur_d7 : %f\n", dataControl->cur_beta, dataControl->cur_d7);

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
                        return;
                    }

                    dataControl->PathData.point_px.push_back(diningWaypoints.diningSection[1].x[0]);
                    dataControl->PathData.point_py.push_back(diningWaypoints.diningSection[1].y[0]);
                    dataControl->PathData.point_pz.push_back(diningWaypoints.diningSection[1].z[0]);
                    dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                    dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                    dataControl->PathData.point_rz.push_back(feedingOrientation[2]);
                    dataControl->PathData.point_beta.push_back(0);
                    dataControl->PathData.point_d7.push_back(0);

                    break;
                }
                case DataControl::Test:
                {
                    rt_printf("Test motion\n");
                    dataControl->PathData.point_px.push_back(diningWaypoints.diningSection[0].x[2] + diningWaypoints.get_side1_offset(dataControl->trayInfor.section1, 0));
                    dataControl->PathData.point_py.push_back(diningWaypoints.diningSection[0].y[2] + diningWaypoints.get_side1_offset(dataControl->trayInfor.section1, 1));
                    dataControl->PathData.point_pz.push_back(diningWaypoints.diningSection[0].z[2]);
                    dataControl->PathData.point_rx.push_back(dataControl->RobotData.present_end_pose[3]);
                    dataControl->PathData.point_ry.push_back(dataControl->RobotData.present_end_pose[4]);
                    dataControl->PathData.point_rz.push_back(dataControl->RobotData.present_end_pose[5]);
                    dataControl->PathData.point_beta.push_back(0);
                    dataControl->PathData.point_d7.push_back(0);
                    dataControl->PathData.point_option.push_back(0);

                    dataControl->PathData.point_px.push_back(dataControl->PathData.teaching_pose[0]);
                    dataControl->PathData.point_py.push_back(dataControl->PathData.teaching_pose[1]);
                    dataControl->PathData.point_pz.push_back(dataControl->PathData.teaching_pose[2]);
                    dataControl->PathData.point_rx.push_back(dataControl->RobotData.present_end_pose[3]);
                    dataControl->PathData.point_ry.push_back(dataControl->RobotData.present_end_pose[4]);
                    dataControl->PathData.point_rz.push_back(dataControl->RobotData.present_end_pose[5]);
                    dataControl->PathData.point_beta.push_back(0);
                    dataControl->PathData.point_d7.push_back(0);
                    dataControl->PathData.point_option.push_back(0);

                    dataControl->PathData.point_px.push_back(dataControl->RobotData.present_end_pose[0]);
                    dataControl->PathData.point_py.push_back(dataControl->RobotData.present_end_pose[1]);
                    dataControl->PathData.point_pz.push_back(dataControl->RobotData.present_end_pose[2]);
                    dataControl->PathData.point_rx.push_back(dataControl->RobotData.present_end_pose[3]);
                    dataControl->PathData.point_ry.push_back(dataControl->RobotData.present_end_pose[4]);
                    dataControl->PathData.point_rz.push_back(dataControl->RobotData.present_end_pose[5]);
                    dataControl->PathData.point_beta.push_back(0);
                    dataControl->PathData.point_d7.push_back(0);
                    dataControl->PathData.point_option.push_back(0);

                    break;
                }
            }

            if(dataControl->operateMode.section <= DataControl::Rice){
                dataControl->PathData.point_px.push_back(dataControl->RobotData.present_end_pose[0]);
                dataControl->PathData.point_py.push_back(dataControl->RobotData.present_end_pose[1]);
                dataControl->PathData.point_pz.push_back(dataControl->RobotData.present_end_pose[2]);
                dataControl->PathData.point_rx.push_back(feedingOrientation[0]);
                dataControl->PathData.point_ry.push_back(feedingOrientation[1]);
                dataControl->PathData.point_rz.push_back(feedingOrientation[2]);

                dataControl->cur_beta = dataControl->RobotData.cur_position[6]*ENC2DEG*(-32.0/25.0);
                func_q6q7_to_d7(dataControl->RobotData.cur_position[6]*ENC2DEG, dataControl->RobotData.cur_position[5]*ENC2DEG, &dataControl->cur_d7);
                rt_printf("cur_beta : %f, cur_d7 : %f\n", dataControl->cur_beta, dataControl->cur_d7);

                dataControl->PathData.teaching_d7 = dataControl->PathData.point_d7.back();
                dataControl->PathData.point_d7.push_back(dataControl->PathData.point_d7.back());

                if(dataControl->operateMode.section == DataControl::Side1 || dataControl->operateMode.section == DataControl::Side3){
                    dataControl->PathData.point_beta.push_back(0);
                    dataControl->PathData.teaching_beta = 0;
                }
                else{
                    dataControl->PathData.point_beta.push_back(dataControl->PathData.point_beta.back());
                    dataControl->PathData.teaching_beta = dataControl->PathData.point_beta.back();
                }
                dataControl->PathData.point_option.push_back(0);
            }

            rt_printf("path data : \n");
            for(uint8_t i = 0; i < dataControl->PathData.row; i ++){
                rt_printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d\n",
                          dataControl->PathData.total_time[i],
                          dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i], dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                          dataControl->PathData.point_beta[i], dataControl->PathData.point_d7[i],
                          dataControl->PathData.acc_time[i], dataControl->PathData.point_option[i]);
            }

            if(dataControl->operateMode.section <= DataControl::Rice){
                robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz, dataControl->PathData.point_beta, dataControl->PathData.point_d7, dataControl->PathData.point_option, diningWaypoints.get_z_offset(0) + 0.05);
            }
            else{
                robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz, dataControl->PathData.point_beta, dataControl->PathData.point_d7, dataControl->PathData.point_option, 0);
            }
//            robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz, dataControl->PathData.point_rx, dataControl->PathData.point_ry, dataControl->PathData.point_rz);

            dataControl->ClientToServer.opMode = DataControl::RunMode;
            dataControl->RobotData.run_mode = DataControl::RunCmd;
            dataControl->PathData.path_data_indx = 0;
            dataControl->PathData.path_struct_indx = 0;
            dataControl->PathData.cycle_count_max = -1*0;
            delay_cnt = 0;
            delay_cnt_max = 200;

            break;
        }
    }
}

void ControlMain::robotTest()
{
    if(dataControl->joint_path_index == 0){
        rt_printf("Start Test Mode\n");

        dataControl->cur_beta = dataControl->RobotData.cur_position[6]*ENC2DEG*(-32.0/25.0);
        func_q6q7_to_d7(dataControl->RobotData.cur_position[6]*ENC2DEG, dataControl->RobotData.cur_position[5]*ENC2DEG, &dataControl->cur_d7);
        rt_printf("cur_beta : %f, cur_d7 : %f\n", dataControl->cur_beta, dataControl->cur_d7);

        dataControl->des_beta = -90;
        dataControl->des_d7 = dataControl->cur_d7;

        path_generator(dataControl->cur_beta, dataControl->des_beta, 5.0, 1.0, 0.001, &dataControl->beta_path);
        path_generator(dataControl->cur_d7, dataControl->des_d7, 5.0, 1.0, 0.001, &dataControl->d7_path);

        path_generator(dataControl->des_beta, dataControl->des_beta, 5.0, 1.0, 0.001, &dataControl->beta_path);
        path_generator(dataControl->des_d7, -30, 5.0, 1.0, 0.001, &dataControl->d7_path);

        path_generator(dataControl->des_beta, dataControl->des_beta, 5.0, 1.0, 0.001, &dataControl->beta_path);
        path_generator(-30, 30, 5.0, 1.0, 0.001, &dataControl->d7_path);

        path_generator(dataControl->des_beta, dataControl->des_beta, 5.0, 1.0, 0.001, &dataControl->beta_path);
        path_generator(30, dataControl->des_d7, 5.0, 1.0, 0.001, &dataControl->d7_path);

        path_generator(dataControl->des_beta, dataControl->cur_beta, 5.0, 1.0, 0.001, &dataControl->beta_path);
        path_generator(dataControl->des_d7, dataControl->cur_d7, 5.0, 1.0, 0.001, &dataControl->d7_path);
    }

    dataControl->des_q6 = dataControl->beta_path[dataControl->joint_path_index]*(-32.0/25.0);
    func_q6d7_to_q7(dataControl->des_q6, dataControl->d7_path[dataControl->joint_path_index], &dataControl->des_q7);

    dataControl->RobotData.tar_position[0] =  45*DEG2ENC;
    dataControl->RobotData.tar_position[1] = -20*DEG2ENC;
    dataControl->RobotData.tar_position[2] = -55*DEG2ENC;
    dataControl->RobotData.tar_position[3] = -75*DEG2ENC;
    dataControl->RobotData.tar_position[4] =  45*DEG2ENC;
    dataControl->RobotData.tar_position[5] = dataControl->des_q7*RAD2DEG*DEG2ENC;
    dataControl->RobotData.tar_position[6] = dataControl->des_q6*DEG2ENC;

    ecatInterface->servo_run(dataControl->RobotData.tar_position, dataControl->module_dir, dataControl->joint_offset);

    //    rt_printf("des_q6 : %f, des_q7 : %f\n", dataControl->des_q6, dataControl->des_q7);
    //    rt_printf("joint path index : %d, beta : %f, d7 : %f\n", dataControl->joint_path_index, dataControl->beta_path[dataControl->joint_path_index], dataControl->d7_path[dataControl->joint_path_index]);
    dataControl->joint_path_index++;

    if(dataControl->joint_path_index >= dataControl->beta_path.size()){
        //        dataControl->ClientToServer.opMode = DataControl::Wait;
        dataControl->joint_path_index = 0;
    }
}

void ControlMain::robotPathGenerate(std::vector<double> px, std::vector<double> py, std::vector<double> pz, std::vector<double> rx, std::vector<double> ry, std::vector<double> rz)
{
    dataControl->PathData.point_theta.clear();

    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        dataControl->PathData.movePath[i].path_x.clear();
        dataControl->PathData.movePath[i].path_y.clear();
        dataControl->PathData.movePath[i].path_z.clear();
        path_generator(px[i], px[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_x, i);
        path_generator(py[i], py[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_y, i);
        path_generator(pz[i], pz[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_z, i);
    }

    dataControl->PathData.point_theta.push_back(0);
    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        double R_init[9], R_final[9], r[3], theta;
        RobotArm::zyx2mat(rz[i], ry[i], rx[i], R_init);
        RobotArm::zyx2mat(rz[i + 1], ry[i + 1], rx[i + 1], R_final);
        RobotArm::mat_to_axis_angle(R_init, R_final, r, &theta);
        memcpy(dataControl->PathData.movePath[i].r, r, sizeof(double)*3);
        dataControl->PathData.point_theta.push_back(theta);

        //        rt_printf("r : %f, %f, %f, %f\n", r[0], r[1], r[2], theta);
    }

    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        path_generator(0, dataControl->PathData.point_theta[i + 1],
                dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_theta, i);

        dataControl->PathData.movePath[i].data_size = dataControl->PathData.movePath[i].path_x.size();
        rt_printf("section %d path size : %d\n", i, dataControl->PathData.movePath[i].data_size);
    }

    //     int indx = 1;
    //     for (int i = 0; i < dataControl->PathData.row - 1; i++){
    //         for (uint j = 0; j < dataControl->PathData.movePath[i].path_x.size(); j++){
    //             printf("%d, %f, %f, %f\n",
    //                    indx++, dataControl->PathData.movePath[i].path_x[j], dataControl->PathData.movePath[i].path_y[j], dataControl->PathData.movePath[i].path_z[j]);
    //         }
    //     }

    //     dataControl->ClientToServer.opMode = DataControl::Wait;
}

void ControlMain::robotPathGenerate(std::vector<double> px, std::vector<double> py, std::vector<double> pz, std::vector<double> beta, std::vector<double> d7, std::vector<int> option, double offset)
{
    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        dataControl->PathData.movePath[i].path_x.clear();
        dataControl->PathData.movePath[i].path_y.clear();
        dataControl->PathData.movePath[i].path_z.clear();
        path_generator(px[i], px[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_x);
        path_generator(py[i], py[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_y);
        if(option[i+1] == 0 || true){
            path_generator(pz[i], pz[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_z);
        }
        else{
            path_generator_circle(pz[i], pz[i + 1], py[i] + offset, pz[1] - offset, dataControl->PathData.movePath[i].path_y, &dataControl->PathData.movePath[i].path_z);
        }
        path_generator(beta[i], beta[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_beta);
        path_generator(d7[i], d7[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_d7);
    }

    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        dataControl->PathData.movePath[i].data_size = dataControl->PathData.movePath[i].path_x.size();
        rt_printf("section %d path size : %d\n", i, dataControl->PathData.movePath[i].data_size);
    }

    //     int indx = 1;
    //     for (int i = 0; i < dataControl->PathData.row - 1; i++){
    //         for (uint j = 0; j < dataControl->PathData.movePath[i].path_x.size(); j++){
    //             rt_printf("%d, %f, %f, %f\n", indx++, dataControl->PathData.movePath[i].path_x[j], dataControl->PathData.movePath[i].path_y[j], dataControl->PathData.movePath[i].path_z[j]);
    //         }
    //     }

    //     dataControl->ClientToServer.opMode = DataControl::Wait;
}

void ControlMain::robotPathGenerate(std::vector<double> px, std::vector<double> py, std::vector<double> pz, std::vector<double> beta, std::vector<double> d7)
{
    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        dataControl->PathData.movePath[i].path_x.clear();
        dataControl->PathData.movePath[i].path_y.clear();
        dataControl->PathData.movePath[i].path_z.clear();
        path_generator(px[i], px[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_x);
        path_generator(py[i], py[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_y);
        path_generator(pz[i], pz[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_z);
        path_generator(beta[i], beta[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_beta);
        path_generator(d7[i], d7[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_d7);
    }

    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        dataControl->PathData.movePath[i].data_size = dataControl->PathData.movePath[i].path_x.size();
        rt_printf("section %d path size : %d\n", i, dataControl->PathData.movePath[i].data_size);
    }

    // int indx = 1;
    // for (int i = 0; i < dataControl->PathData.row - 1; i++){
    //     for (uint j = 0; j < dataControl->PathData.movePath[i].path_x.size(); j++){
    //         printf("%d, %f, %f, %f\n",
    //                indx++, dataControl->PathData.movePath[i].path_x[j], dataControl->PathData.movePath[i].path_y[j], dataControl->PathData.movePath[i].path_z[j]);
    //     }
    // }

    // dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
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

void ControlMain::path_generator(double x0, double xf, double tf, double ta, double h, std::vector<double> *path, int path_index)
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

void ControlMain::path_generator_circle(double p1, double p2, double pcx, double pcy, std::vector<double> path_x, std::vector<double> *path_y)
{
    double xxc2 = pow((p1 - pcx), 2);
    double yyc2 = pow((p2 - pcy), 2);
    double r2 = xxc2 + yyc2;

    for(unsigned int i = 0; i < path_x.size(); i++){
        path_y->push_back(pcy + sqrt(r2 - pow((path_x[i] - pcx), 2)));
    }
}

void ControlMain::func_q6d7_to_q7(double q6, double d7, double *q7)
{
    //    if(d7 >= 25){
    //        d7 = 25;
    //    }
    //    else
    if(d7 <= -25){
        d7 = -25;
    }
    *q7 = (d7/8.0 + q6*DEG2RAD);
}

void ControlMain::func_q6q7_to_d7(double q6, double q7, double *d7)
{
    *d7 = 8.0*(q7 + (25.0/32.0)*q6);
}

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
