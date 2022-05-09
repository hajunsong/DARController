#include "userinterface.h"
#include "esi.h"
#include "QDebug"
userinterface::userinterface()
{
    qDebug() << ("USER INTERFACE!!!");
    v_global_cur_all_state   = REQ_SERVO_OFF;  // current all slave state
    v_global_prev_all_state  = -1;  			// preview all slave state
    v_global_recheck_flag = 0;
/*
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
    v_global_ethercat_control.f_oper_slave_setting_initialize(2, // slave number
                                                            "LSMPO Servo", // slave device name
                                                            "LSMPO", // slave vendor name
                                                            0x0000ffff, // slave vendor id
                                                            0x00000000,
                                                            true); // slave product code
    v_global_ethercat_control.f_oper_slave_setting_initialize(3, // slave number
                                                            "LMPO Servo", // slave device name
                                                            "LSMPO", // slave vendor name
                                                            0x0000ffff, // slave vendor id
                                                            0x00000000,
                                                            true); // slave product code
    v_global_ethercat_control.f_oper_slave_setting_initialize(4, // slave number
                                                            "LSMPO Servo", // slave device name
                                                            "LSMPO", // slave vendor name
                                                            0x0000ffff, // slave vendor id
                                                            0x00000000,
                                                            true); // slave product code
    v_global_ethercat_control.f_oper_slave_setting_initialize(5, // slave number
                                                            "LSMPO Servo", // slave device name
                                                            "LSMPO", // slave vendor name
                                                            0x0000ffff, // slave vendor id
                                                            0x00000000,
                                                            true); // slave product code
    v_global_ethercat_control.f_oper_slave_setting_initialize(6, // slave number
                                                            "LSMPO Servo", // slave device name
                                                            "LSMPO", // slave vendor name
                                                            0x0000ffff, // slave vendor id
                                                            0x00000000,
                                                            true); // slave product code

    pthread_create(&f_oper_stateUpdateTimer, NULL, f_oper_stateUpdateTimer_Callback, this);

    usleep(1000000);
    on_v_action_pb_ECAT_start_clicked();

    usleep(1000000);
    on_v_action_pb_SERVO_BT_clicked();

    usleep(10000);
    on_v_action_pb_servo_RUN_clicked();
    */
}

userinterface::~userinterface()
{

}

void userinterface::init(){
    d_print("Start ecat initialize\n");
    std::string slave_model_str = "LSMPO Servo";
    std::string slave_vendor_str = "LSMPO";
    char slave_model_name[255], slave_vendor_name[255];
    memcpy(slave_model_name, slave_model_str.c_str(), sizeof(char)*slave_model_str.length());
    memcpy(slave_vendor_name, slave_vendor_str.c_str(), sizeof(char)*slave_vendor_str.length());

//    v_global_ethercat_control.f_oper_slave_setting_initialize(0, slave_model_name, slave_vendor_name, 0x0000ffff, 0x00000000, true);
    v_global_ethercat_control.f_oper_slave_setting_initialize(0, slave_model_name, slave_vendor_name, 0x00000ba9, 0x00000001, true);
    v_global_ethercat_control.f_oper_slave_setting_initialize(1, slave_model_name, slave_vendor_name, 0x00000ba9, 0x00000001, true);
    v_global_ethercat_control.f_oper_slave_setting_initialize(2, slave_model_name, slave_vendor_name, 0x00000ba9, 0x00000001, true);
    v_global_ethercat_control.f_oper_slave_setting_initialize(3, slave_model_name, slave_vendor_name, 0x00000ba9, 0x00000001, true);
    v_global_ethercat_control.f_oper_slave_setting_initialize(4, slave_model_name, slave_vendor_name, 0x00000ba9, 0x00000001, true);
    v_global_ethercat_control.f_oper_slave_setting_initialize(5, slave_model_name, slave_vendor_name, 0x00000ba9, 0x00000001, true);
    v_global_ethercat_control.f_oper_slave_setting_initialize(6, slave_model_name, slave_vendor_name, 0x00000ba9, 0x00000001, true);

    /* MODIFIED */
    v_global_ethercat_control.f_oper_master_initialize(7, 1, 1000000, 500000, 0, 1, true);
}

void userinterface::ecat_start()
{
    int *v_area_req_ecat_state = v_global_ethercat_control.f_get_req_ecat_state_ptr();
    ecatslave *v_area_slaves = v_global_ethercat_control.f_get_slave();

    if(REF(v_area_req_ecat_state) == 0){
        d_print("EtherCAT start \n");

        REF(v_area_req_ecat_state) = 1;
        for(int v_area_slave_count= 0; v_area_slave_count < v_global_ethercat_control.f_get_slave_count(); ++v_area_slave_count )
        {
            v_area_slaves[v_area_slave_count].v_global_req_servo_state = REQ_SERVO_OFF;
        }
    }
    else{
        d_print("EtherCAT already started \n");

    }

    pthread_create(&f_oper_stateUpdateTimer, NULL, f_oper_stateUpdateTimer_Callback, this);
}

void userinterface::servo_on()
{
    ecatslave *v_area_slaves = v_global_ethercat_control.f_get_slave();

    for(int v_area_slave_count = 0; v_area_slave_count < v_global_ethercat_control.f_get_slave_count(); ++v_area_slave_count)
    {
        v_area_slaves[v_area_slave_count].v_global_req_servo_state = REQ_SERVO_ON;
        d_print("Slave[%d] Set SERVO_ON \n",v_area_slave_count);
    }
}

void userinterface::servo_off()
{
    ecatslave *v_area_slaves = v_global_ethercat_control.f_get_slave();

    for(int v_area_slave_count = 0; v_area_slave_count < v_global_ethercat_control.f_get_slave_count(); ++v_area_slave_count)
    {
        v_area_slaves[v_area_slave_count].v_global_req_servo_state = REQ_SERVO_OFF;
        d_print("Slave[%d] Set SERVO_OFF \n",v_area_slave_count);
    }
}

void userinterface::servo_run(long tar_position[], int8_t dir[], int32_t joint_offset[], long tar_position_prev[]){
//    d_print("ServoRun .. \n");

//    rt_printf("%ld,%ld,%ld,%ld,%ld,%ld,%ld\n", tar_position[0], tar_position[1], tar_position[2], tar_position[3], tar_position[4], tar_position[5], tar_position[6]);

//    memcpy(v_global_ethercat_control.v_des_pos, tar_position, sizeof(long)*v_global_ethercat_control.f_get_slave_count());
    for(int i = 0; i < v_global_ethercat_control.f_get_slave_count(); i++){
        v_global_ethercat_control.v_des_pos[i] = dir[i]*(tar_position[i] + dir[i]*joint_offset[i]);
    }

    for(int slv_inx=0; slv_inx < v_global_ethercat_control.f_get_slave_count(); slv_inx++)
    {
       v_global_ethercat_control.f_get_slave()[slv_inx].v_global_req_servo_state = REQ_SERVO_RUN;
    }
}

void userinterface::servo_run(int tar_torque[], int8_t dir[]){
//    rt_printf("%ld,%ld,%ld,%ld,%ld,%ld,%ld\n", tar_torque[0], tar_torque[1], tar_torque[2], tar_torque[3], tar_torque[4], tar_torque[5], tar_torque[6]);

    for(int i = 0; i < v_global_ethercat_control.f_get_slave_count(); i++){
        v_global_ethercat_control.v_des_tor[i] = dir[i]*tar_torque[i];
    }

    for(int slv_inx=0; slv_inx < v_global_ethercat_control.f_get_slave_count(); slv_inx++)
    {
       v_global_ethercat_control.f_get_slave()[slv_inx].v_global_req_servo_state = REQ_SERVO_RUN;
    }
}

void userinterface::servo_stop()
{
    for(int slv_inx=0; slv_inx < v_global_ethercat_control.f_get_slave_count(); slv_inx++)
    {
       v_global_ethercat_control.f_get_slave()[slv_inx].v_global_req_servo_state = REQ_SERVO_STOP;
    }
}

void userinterface::set_led(unsigned char mode){
    switch(mode){
        case 0: // off
        {
            v_global_ethercat_control.f_get_slave()[5].f_oper_DIO(0);
            v_global_ethercat_control.f_get_slave()[6].f_oper_DIO(0);
            break;
        }
        case 1: // green
        {
            v_global_ethercat_control.f_get_slave()[5].f_oper_DIO(1);
            v_global_ethercat_control.f_get_slave()[6].f_oper_DIO(0);
            break;
        }
        case 2: // red
        {
            v_global_ethercat_control.f_get_slave()[5].f_oper_DIO(2);
            v_global_ethercat_control.f_get_slave()[6].f_oper_DIO(0);
            break;
        }
        case 3: // blue
        {
            v_global_ethercat_control.f_get_slave()[5].f_oper_DIO(0);
            v_global_ethercat_control.f_get_slave()[6].f_oper_DIO(2);
            break;
        }
    }

}

void userinterface::start()
{
    usleep(1000000);
    on_v_action_pb_ECAT_start_clicked();

    usleep(1000000);
    on_v_action_pb_SERVO_BT_clicked();

    usleep(10000);
    on_v_action_pb_servo_RUN_clicked();
}

void userinterface::stop()
{
    usleep(1000000);
    on_v_action_pb_servo_STOP_clicked();

    usleep(1000000);
    on_v_action_pb_SERVO_BT_clicked();

    usleep(1000000);
    on_v_action_pb_ECAT_start_clicked();
}

void userinterface::servo_mode(int joint_op_mode)
{
    ecatslave *v_area_slaves = v_global_ethercat_control.f_get_slave();

    switch(joint_op_mode){
        case 0:
            for(int v_area_slave_count = 0; v_area_slave_count < v_global_ethercat_control.f_get_slave_count(); ++v_area_slave_count)
            {
                v_area_slaves[v_area_slave_count].f_oper_to_cst();
            }
            break;
        case 3:
            for(int v_area_slave_count = 0; v_area_slave_count < v_global_ethercat_control.f_get_slave_count(); ++v_area_slave_count)
            {
                v_area_slaves[v_area_slave_count].f_oper_to_csp();
            }
            break;
        default:
            break;
    }
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
        for(v_area_slave_count= 0; v_area_slave_count < v_global_ethercat_control.f_get_slave_count(); ++v_area_slave_count )
        {
            v_area_slaves[v_area_slave_count].v_global_req_servo_state = REQ_SERVO_OFF;
        }

        /* MODIFIED */
        v_global_ethercat_control.f_oper_master_initialize(v_global_ethercat_control.f_get_slave_count(), 1, 1000000, 500000, 0, 1, true);
    }
    else
    {
        REF(v_area_req_ecat_state) = 0;
        for(v_area_slave_count= 0; v_area_slave_count < v_global_ethercat_control.f_get_slave_count(); ++v_area_slave_count )
        {
            v_area_slaves[v_area_slave_count].v_global_req_servo_state = REQ_SERVO_OFF;
        }
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
//        d_print("%s\n", str.toStdString().c_str());

//        v_global_status_word = REF(pThis->v_global_ethercat_control.f_get_cur_ecat_state_ptr());
//        REF(v_area_master_state).al_states >= EC_AL_STATES_OP && REF(v_area_domain_state).state.wc_state == EC_WC_COMPLETE

//        for(int count = 0; count < 7; count++)
//        {
//            printf("%d,%d,%ld,%ld,%d,%ld,%d\n", count + 1,
//                   pThis->v_global_ethercat_control.f_get_slave()[count].v_global_cur_status_word,
//                   pThis->v_global_ethercat_control.f_get_slave()[count].v_global_cur_position,
//                   pThis->v_global_ethercat_control.f_get_slave()[count].v_global_cur_velocity,
//                   pThis->v_global_ethercat_control.f_get_slave()[count].v_global_cur_torque_value1,
//                   pThis->v_global_ethercat_control.f_get_slave()[count].v_global_tar_position,
//                   pThis->v_global_ethercat_control.f_get_slave()[count].v_global_tar_torque);
//        }

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
                    if(pThis->v_global_ethercat_control.f_get_slave()[slv_inx].v_global_req_servo_state == REQ_SERVO_ON){
                        pThis->v_global_ethercat_control.f_get_slave()[slv_inx].v_global_req_servo_state = REQ_SERVO_STOP;
                    }
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
