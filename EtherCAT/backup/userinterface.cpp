#include "userinterface.h"
#include "esi.h"

userinterface::userinterface()
{
    v_global_cur_all_state   = REQ_SERVO_OFF;  // current all slave state
    v_global_prev_all_state  = -1;  			// preview all slave state
    v_global_recheck_flag = 0;

    v_global_ethercat_control.f_oper_slave_setting_initialize(0, // slave number
                                                            "LSMPO Servo", // slave device name
                                                            "LSMPO", // slave vendor name
                                                            0x0000ffff, // slave vendor id
                                                            0x00000000,
                                                            true); // slave product code

    v_global_ethercat_control.f_oper_slave_setting_initialize(1, // slave number
                                                            "LSMPO Servo", // slave device name
                                                            "LSMPO", // slave vendor name
                                                            0x0000ffff, // slave vendor id
                                                            0x00000000,
                                                            true); // slave product code

    pthread_create(&f_oper_stateUpdateTimer, NULL, f_oper_stateUpdateTimer_Callback, this);
}

userinterface::~userinterface()
{
    usleep(1000000);
    on_v_action_pb_servo_STOP_clicked();

    usleep(1000000);
    on_v_action_pb_SERVO_BT_clicked();

    usleep(1000000);
    on_v_action_pb_ECAT_start_clicked();
}

void userinterface::start()
{
    usleep(1000000);
    on_v_action_pb_ECAT_start_clicked();

    usleep(1000000);
    on_v_action_pb_SERVO_BT_clicked();

//    usleep(1000000);
//    ui->rbRunType1->setChecked(true);

    usleep(10000);
    on_v_action_pb_servo_RUN_clicked();
}


void userinterface::on_v_action_pb_ECAT_start_clicked()
{
    int *v_area_req_ecat_state = v_global_ethercat_control.f_get_req_ecat_state_ptr();
    int v_area_slave_count = 0;
    ecatslave *v_area_slaves = v_global_ethercat_control.f_get_slave();



    if(REF(v_area_req_ecat_state) == 0){
        d_print("EtherCAT start \n");
    }
    else{
        d_print("EtherCAT stop \n");
    }


    if(!REF(v_area_req_ecat_state))
    {

        REF(v_area_req_ecat_state) = 1;
        for(v_area_slave_count= 0; v_area_slave_count < 1; ++v_area_slave_count )
        {
            v_area_slaves[v_area_slave_count].v_global_req_servo_state = REQ_SERVO_OFF;
        }

        v_global_ethercat_control.f_oper_master_initialize(2, // slave total count
                                                           1, // domain total count
                                                           1000000, // free run sycle time, 1ms
                                                           500000, // sync run sycle time
                                                           0, // ref count
                                                           1, // dcmod flag
                                                           true); // sorvo driver check
//        ui->v_action_pb_ECAT_start->setText("STOP");
    }
    else
    {
        REF(v_area_req_ecat_state) = 0;
        for(v_area_slave_count= 0; v_area_slave_count < 2; ++v_area_slave_count )
        {
            v_area_slaves[v_area_slave_count].v_global_req_servo_state = REQ_SERVO_OFF;
        }

//        ui->v_action_pb_ECAT_start->setText("START");
    }
}

void userinterface::on_v_action_pb_SERVO_BT_clicked()
{
    ecatslave *v_area_slaves = v_global_ethercat_control.f_get_slave();


    int v_area_slave_count = 0;
    for(v_area_slave_count = 0; v_area_slave_count < v_global_ethercat_control.f_get_slave_count(); ++v_area_slave_count)
    {
        if(v_global_cur_all_state >= STATE_SERVO_ON)
        {
            v_area_slaves[v_area_slave_count].v_global_req_servo_state = REQ_SERVO_OFF;
            d_print("Slave[%d] Set SERVO_OFF \n",v_area_slave_count);
        }
        else
        {
            v_area_slaves[v_area_slave_count].v_global_req_servo_state = REQ_SERVO_ON;
            d_print("Slave[%d] Set SERVO_ON \n",v_area_slave_count);
        }
    }



}




/*  Function name : stateUpdateTimer_Callback
 *  Description   : This function
 *  Return value  : None.
 */
void* userinterface::f_oper_stateUpdateTimer_Callback(void* arg)
{
    userinterface* pThis = static_cast<userinterface*>(arg);

    QString str;
    while(true){
#if 1


        if(pThis->v_global_recheck_flag == 0){
            for (int i = 0; i<MAX_SLAVES; i++)
            {
                pThis->v_global_prev_online_slave_state[i].state.online = 1;
                pThis->v_global_recheck_flag = 1;
            }
        }

        str = pThis->v_global_ethercat_control.f_oper_get_ecat_state();
        str += " , WC " + QString::number(REF(pThis->v_global_ethercat_control.f_get_cur_ecat_state_ptr()));
//        ui->v_action_labal_ECAT_state->setText(str);
        d_print("%s\n", str.toStdString().c_str());



#endif


        pThis->v_global_cur_all_state = pThis->v_global_ethercat_control.f_oper_get_servo_valid_state();

        if(pThis->v_global_prev_all_state != pThis->v_global_cur_all_state)
        {
            pThis->v_global_prev_all_state = pThis->v_global_cur_all_state;
            // servo state update
            switch(pThis->v_global_cur_all_state)
            {
            case STATE_SERVO_OFF:
    //            ui->v_action_labal_servo_state->setText("Servo Off");
                break;
            case STATE_SERVO_SW_ON:
    //            ui->v_action_labal_servo_state->setText("Servo Sw");
                break;
            case STATE_SERVO_ON:
    //            ui->v_action_labal_servo_state->setText("Servo On");
                for(int slv_inx=0; slv_inx < pThis->v_global_ethercat_control.f_get_slave_count(); slv_inx++)
                {
                    if(pThis->v_global_ethercat_control.f_get_slave()[slv_inx].v_global_req_servo_state == REQ_SERVO_ON)
                        pThis->v_global_ethercat_control.f_get_slave()[slv_inx].v_global_req_servo_state = REQ_SERVO_STOP;
                }
                break;
            case STATE_SERVO_ERROR:
    //            ui->v_action_labal_servo_state->setText("Servo Error");
                break;
            }
        }

        usleep(500000);

    }


//    if(v_global_cur_all_state == STATE_SERVO_ON)
//    {
//        ui->v_action_pb_SERVO_BT->setText("Off");
//    }
//    else
//    {
//        ui->v_action_pb_SERVO_BT->setText("On");
//    }^

//    ui->txtTargetPos->setText(QString::number(v_global_ethercat_control.f_get_slave()[0].v_global_tar_position));
//    ui->txtTargetVel->setText(QString::number(v_global_ethercat_control.f_get_slave()[0].v_global_tar_velocity));
    //    ui->txtTargetTor->setText(QString::number(v_global_ethercat_control.f_get_slave()[0].v_global_cur_status_word));
}

void userinterface::on_v_action_pb_servo_RUN_clicked()
{
    d_print("ServoRun .. \n");


    for(int slv_inx=0; slv_inx < v_global_ethercat_control.f_get_slave_count(); slv_inx++)
    {
       v_global_ethercat_control.f_get_slave()[slv_inx].v_global_run_speed = 10;
       v_global_ethercat_control.f_get_slave()[slv_inx].v_global_run_type = RUN_FORWARD;
//               ui->rbRunType1->isChecked ()? RUN_FORWARD : (ui->rbRunType2->isChecked ()?  RUN_REVERSE : RUN_TASK);
       v_global_ethercat_control.f_get_slave()[slv_inx].v_global_req_servo_state = REQ_SERVO_RUN;
    }
}

void userinterface::on_v_action_pb_servo_STOP_clicked()
{
    d_print("ServoSTOP .. \n");
    for(int slv_inx=0; slv_inx < v_global_ethercat_control.f_get_slave_count(); slv_inx++)
    {
       v_global_ethercat_control.f_get_slave()[slv_inx].v_global_req_servo_state = REQ_SERVO_STOP;
    }
}
