
#include "ecatmaster.h"



ecatmaster::ecatmaster()
{
    v_global_dec_interface = new daininterface();


    v_global_domain_shmem_ptr = NULL;
    v_global_domain_shmem_size = 0;
    v_global_decat_lib_version = 0;

    v_global_ptr_output_offset_haed = 0;
    v_global_ptr_input_offset_haed = 0;

    v_global_domain1_pd = NULL;

}

/**
  * 1. EtherCAT master Initialize part
  *
*/

//1. Slave setup start
int ecatmaster::f_init_slave_initialize(int v_local_slave_count, int v_local_domain_count)
{
    int   v_area_ret = 0;

    rt_print_auto_init(1);

    if (v_area_ret=v_global_dec_interface->f_oper_open() < 0)
    {
        d_print(" Failed to open dec(%d) .. \n", v_area_ret);
        return -1;
    }

    /* 1. slave setup start   */
    v_area_ret = v_global_dec_interface->f_oper_slave_setup_init(
                v_local_slave_count, v_local_domain_count);
    if (v_area_ret < 0 )
    {
        d_print(" Failed to dec_slave_setup_init (%d) .. \n",v_area_ret);
        return -1;
    }

    v_global_slavecount = v_local_slave_count;
    v_global_domaincount = v_local_domain_count;


#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP1
    d_print("[ECATMASTER]:[init]:[f_init_slave_initialize]");
    d_print("[STEP1] : v_global_slavecount : %d\r\n", v_global_slavecount);
    d_print("[STEP1] : v_global_domaincount : %d\r\n", v_global_domaincount);
#endif
    return v_area_ret;
}


//2. Slave configuration
int ecatmaster::f_init_slave_information_insert(uint16_t v_local_slave_number,
                                                uint32_t v_local_vendorID,
                                                uint32_t v_local_productCODE,
                                                uint16_t v_local_alias,
                                                uint16_t v_local_position)
{
    int   v_area_ret = 0;
    /* 2. slave configuration  */

    if ((v_area_ret = v_global_dec_interface->f_oper_slave_config(v_local_slave_number,
                                                                  v_local_vendorID,
                                                                  v_local_productCODE,
                                                                  v_local_alias,
                                                                  v_local_position)) < 0)
    {
        d_print("Failed to dec_slave_config (%d)\n",v_area_ret);
        return -1;
    }

}

//3. pdo List send
int ecatmaster::f_init_PDO_List_send(int v_local_slave_number,
                                     ec_sync_info_t * v_local_ptr_slave_syncs,
                                     bool v_local_servo_flag)
{
    int   v_area_ret = 0;
    ec_sync_info_t *v_area_sync_buf;
    int v_area_sync_c=0;
    int v_area_count;

#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP3
    {
        int v_sub_syncs_count = 0;
        int v_sub_pdos_count = 0;
        int v_sub_entreis_count = 0;

        for(v_sub_syncs_count =0;
            v_local_ptr_slave_syncs[v_sub_syncs_count].index != 0xFF;
            ++v_sub_syncs_count )
        {
            d_print("[ECATMASTER]:[init]:[f_init_PDO_List_send]\r\n");
            d_print("[STEP 3] : SYNCS [%d] \r\n",v_sub_syncs_count);
            d_print("[STEP 3] : SYNCS  : index : 0x%X\r\n",v_local_ptr_slave_syncs[v_sub_syncs_count].index);
            d_print("[STEP 3] : SYNCS  : n_pdos : %d\r\n",v_local_ptr_slave_syncs[v_sub_syncs_count].n_pdos);
            if(v_local_ptr_slave_syncs[v_sub_syncs_count].dir == EC_DIR_INPUT)
                d_print("[STEP 3] : SYNCS  : dir : INPUT \r\n");
            else if(v_local_ptr_slave_syncs[v_sub_syncs_count].dir == EC_DIR_OUTPUT)
                d_print("[STEP 3] : SYNCS  : dir : OUTPUT \r\n");

            for(v_sub_pdos_count = 0;
                v_sub_pdos_count < v_local_ptr_slave_syncs[v_sub_syncs_count].n_pdos;
                ++ v_sub_pdos_count)
            {
                d_print("[STEP 3] : PDOS [%d]\r\n",v_sub_pdos_count);
                d_print("[STEP 3] : PDOS : index : 0x%X\r\n",v_local_ptr_slave_syncs[v_sub_syncs_count].pdos[v_sub_pdos_count].index);
                d_print("[STEP 3] : PDOS : n_entreis : %d\r\n",v_local_ptr_slave_syncs[v_sub_syncs_count].pdos[v_sub_pdos_count].n_entries);

                for(v_sub_entreis_count = 0;
                    v_sub_entreis_count < v_local_ptr_slave_syncs[v_sub_syncs_count].pdos[v_sub_pdos_count].n_entries;
                    ++ v_sub_entreis_count)
                {
                    d_print("[STEP 3] : ENTREIS [%d]\r\n",v_sub_entreis_count);
                    d_print("[STEP 3] : ENTREIS : index : 0x%X\r\n",
                            v_local_ptr_slave_syncs[v_sub_syncs_count].pdos[v_sub_pdos_count].entries[v_sub_entreis_count].index);
                    d_print("[STEP 3] : ENTREIS : subindex : 0x%X\r\n",
                            v_local_ptr_slave_syncs[v_sub_syncs_count].pdos[v_sub_pdos_count].entries[v_sub_entreis_count].subindex);
                }
            }
        }

    }
#endif


    v_area_sync_buf = v_local_ptr_slave_syncs;
    for(v_area_count=0;v_area_count<8;v_area_count++){
        if(v_area_sync_buf->index != 0xff){
            v_area_sync_c++;
            v_area_sync_buf++;
        }
        else
            break;
    }

    if((v_area_ret = v_global_dec_interface->f_oper_slave_config_pdo_list(v_local_slave_number
                                                                          ,23*v_area_sync_c
                                                                          ,v_local_ptr_slave_syncs)) < 0)
    {
        d_print(" Failed to configure out-PDOs (%d)\n",v_area_ret);
        return -1;
    }

    if(v_local_servo_flag == true)
    {
        f_oper_sdo_write(v_local_slave_number,0x6060,0x00,0x08,_8BIT);
    }
}

// 4. make domain and list inforamtion insert
int ecatmaster::f_init_make_domain(int v_local_slave_numbers,
                                   ec_pdo_entry_reg_t *v_local_ptr_slave_domain_out_regs[],
                                   ec_pdo_entry_reg_t *v_local_ptr_slave_domain_in_regs[],
                                   ec_sync_info_t *v_local_ptr_slave_syncs[])
{
    int v_area_slave_count = 0;
    int v_area_pdo_numbers = 0;
    int v_area_pdo_count = 0;
    int v_area_sync_count = 0;
    int v_area_pdo_total_count = 0;
    ec_pdo_entry_reg_t	*v_area_ptr_domain = NULL;
    int v_area_domain_count = 0;
    int   v_area_ret = 0;

#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP4
    {
        int v_sub_syncs_count = 0;
        int v_sub_pdos_count = 0;
        int v_sub_entreis_count = 0;
        d_print("[ECATMASTER] : [init] : [f_init_make_domain] : entries check \r\n");

        for(v_area_slave_count = 0; v_area_slave_count < v_local_slave_numbers; ++v_area_slave_count)
        {
            for(v_sub_syncs_count = 0;
                v_local_ptr_slave_syncs[v_area_slave_count][v_sub_syncs_count].index != 0xFF;
                ++v_sub_syncs_count)
            {
                d_print("[STEP 4] : SYNCS : index : 0x%X\r\n"
                        ,v_local_ptr_slave_syncs[v_area_slave_count][v_sub_syncs_count].index);
                d_print("[STEP 4] : SYNCS : n_pdos : %d\r\n"
                        ,v_local_ptr_slave_syncs[v_area_slave_count][v_sub_syncs_count].n_pdos);

                if(v_local_ptr_slave_syncs[v_area_slave_count][v_sub_syncs_count].dir == EC_DIR_INPUT)
                {
                    d_print("[STEP 4] : SYNCS : dir : INPUT\r\n");
                }
                else if ( v_local_ptr_slave_syncs[v_area_slave_count][v_sub_syncs_count].dir == EC_DIR_OUTPUT )
                {
                    d_print("[STEP 4] : SYNCS : dir : OUTPUT\r\n");
                }

                for(v_sub_pdos_count = 0;
                    v_sub_pdos_count < v_local_ptr_slave_syncs[v_area_slave_count][v_sub_syncs_count].n_pdos;
                    ++ v_sub_pdos_count)
                {
                    d_print("[STEP 4] : PDOS [%d]\r\n",v_sub_pdos_count);
                    d_print("[STEP 4] : PDOS : index : 0x%X\r\n",v_local_ptr_slave_syncs[v_area_slave_count][v_sub_syncs_count].pdos[v_sub_pdos_count].index);
                    d_print("[STEP 4] : PDOS : n_entreis : %d\r\n",v_local_ptr_slave_syncs[v_area_slave_count][v_sub_syncs_count].pdos[v_sub_pdos_count].n_entries);

                    for(v_sub_entreis_count = 0;
                        v_sub_entreis_count < v_local_ptr_slave_syncs[v_area_slave_count][v_sub_syncs_count].pdos[v_sub_pdos_count].n_entries;
                        ++ v_sub_entreis_count)
                    {
                        d_print("[STEP 4] : ENTREIS [%d]\r\n",v_sub_entreis_count);
                        d_print("[STEP 4] : ENTREIS : index : 0x%X\r\n",
                                v_local_ptr_slave_syncs[v_area_slave_count][v_sub_syncs_count].pdos[v_sub_pdos_count].entries[v_sub_entreis_count].index);
                        d_print("[STEP 4] : ENTREIS : subindex : 0x%X\r\n",
                                v_local_ptr_slave_syncs[v_area_slave_count][v_sub_syncs_count].pdos[v_sub_pdos_count].entries[v_sub_entreis_count].subindex);
                    }
                }
            }
        }
    }
#endif



    memset(v_global_domain1_regs,0,sizeof(v_global_domain1_regs));

    //Make output domain
    for(v_area_slave_count = 0; v_area_slave_count < v_local_slave_numbers; ++v_area_slave_count)
    {

        v_area_sync_count = 0;
        v_area_pdo_total_count = 0;
        while(v_local_ptr_slave_syncs[v_area_slave_count][v_area_sync_count].index != 0xFF)
        {
#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP4
            {
                d_print("[ECATMASTER] : [init] : [f_init_make_domain]\r\n");
                d_print("[SYNCS] [%d/%d] \r\n",v_area_slave_count,v_area_sync_count);
                d_print("[STEP 4] : index : 0x%X\r\n",v_local_ptr_slave_syncs[v_area_slave_count][v_area_sync_count].index);
            }
#endif
            if(v_local_ptr_slave_syncs[v_area_slave_count][v_area_sync_count].dir == EC_DIR_OUTPUT)
            {
                v_area_pdo_numbers = 0;
                for(v_area_pdo_count = 0;
                    v_area_pdo_count < v_local_ptr_slave_syncs[v_area_slave_count][v_area_sync_count].n_pdos;
                    ++v_area_pdo_count)
                {
                    v_area_pdo_numbers += v_local_ptr_slave_syncs[v_area_slave_count][v_area_sync_count].pdos[v_area_pdo_count].n_entries;
                }
                v_area_pdo_total_count += v_area_pdo_numbers;
            }
            v_area_sync_count++;
        }
#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP4
        {
            int v_sub_entries_count = 0;
            d_print("[ECATMASTER] : [init] : [f_init_make_domain]\r\n");
            d_print("[STEP 4] : v_area_pdo_total_count : %d \r\n",v_area_pdo_total_count);
            for(v_sub_entries_count = 0; v_sub_entries_count < v_area_pdo_total_count; ++v_sub_entries_count)
            {
                d_print("[STEP 4] : v_local_ptr_slave_domain_in_regs[v_area_slave_count][v_sub_entries_count].index[%d] : 0x%X \r\n",v_sub_entries_count,
                        v_local_ptr_slave_domain_out_regs[v_area_slave_count][v_sub_entries_count].index);
            }
        }
#endif

        if(v_area_pdo_total_count)
        {
            v_area_ptr_domain = (ec_pdo_entry_reg_t *)v_local_ptr_slave_domain_out_regs[v_area_slave_count];

            if(!v_global_ptr_output_offset_haed)
            {
                v_global_ptr_output_offset_haed = v_area_ptr_domain->offset;
            }

            while(v_area_pdo_total_count--)
            {
                *(ec_pdo_entry_reg_t *)&v_global_domain1_regs[v_area_domain_count] = *v_area_ptr_domain;
#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP4
                d_print("[STEP 4] : v_global_domain1_regs[%d] : 0x%X \r\n",v_area_domain_count,v_global_domain1_regs[v_area_domain_count].index);
#endif
                v_area_domain_count++;
                v_area_ptr_domain++;
            }
        }
    }


    //Make input domain
    v_area_ptr_domain = NULL;
    for(v_area_slave_count = 0; v_area_slave_count < v_local_slave_numbers; ++v_area_slave_count)
    {

        v_area_sync_count = 0;
        v_area_pdo_total_count = 0;

        while(v_local_ptr_slave_syncs[v_area_slave_count][v_area_sync_count].index != 0xFF)
        {

#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP4
            {
                d_print("[ECATMASTER] : [init] : [f_init_make_domain]\r\n");
                d_print("[SYNCS] [%d/%d] \r\n",v_area_slave_count,v_area_sync_count);
                d_print("[STEP 4] : index : 0x%X\r\n",v_local_ptr_slave_syncs[v_area_slave_count][v_area_sync_count].index);
            }
#endif
            if(v_local_ptr_slave_syncs[v_area_slave_count][v_area_sync_count].dir == EC_DIR_INPUT)
            {
                v_area_pdo_numbers = 0;
                for(v_area_pdo_count = 0; v_area_pdo_count < v_local_ptr_slave_syncs[v_area_slave_count][v_area_sync_count].n_pdos; ++v_area_pdo_count)
                    v_area_pdo_numbers += v_local_ptr_slave_syncs[v_area_slave_count][v_area_sync_count].pdos[v_area_pdo_count].n_entries;

                v_area_pdo_total_count += v_area_pdo_numbers;
            }
            v_area_sync_count++;
        }

#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP4
        {
            int v_sub_entries_count = 0;
            d_print("[ECATMASTER] : [init] : [f_init_make_domain]\r\n");
            d_print("[STEP 4] : v_area_pdo_total_count : %d \r\n",v_area_pdo_total_count);
            for(v_sub_entries_count = 0; v_sub_entries_count < v_area_pdo_total_count; ++v_sub_entries_count)
            {
                d_print("[STEP 4] : v_local_ptr_slave_domain_in_regs[v_area_slave_count][v_sub_entries_count].index[%d] : 0x%X \r\n",v_sub_entries_count,
                        v_local_ptr_slave_domain_in_regs[v_area_slave_count][v_sub_entries_count].index);
            }
        }
#endif
        if(v_area_pdo_total_count)
        {
            v_area_ptr_domain = (ec_pdo_entry_reg_t *)v_local_ptr_slave_domain_in_regs[v_area_slave_count];

            if(!v_global_ptr_input_offset_haed)
            {
                v_global_ptr_input_offset_haed = v_area_ptr_domain->offset;

            }

            while(v_area_pdo_total_count--)
            {
                *(ec_pdo_entry_reg_t *)&v_global_domain1_regs[v_area_domain_count] = *v_area_ptr_domain;
#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP4
                d_print("[STEP 4] : v_global_domain1_regs[%d] : 0x%X \r\n",v_area_domain_count,v_global_domain1_regs[v_area_domain_count].index);
#endif
                v_area_domain_count++;
                v_area_ptr_domain++;
            }
        }
    }


    d_print("set_domain_regs:  size(%d) * count(%d) = %d \n",
            sizeof(ec_pdo_entry_reg_t), v_area_domain_count, sizeof(ec_pdo_entry_reg_t) * v_area_domain_count);


    if ( (v_area_ret = v_global_dec_interface->f_oper_domain_reg_entry(0, (sizeof(ec_pdo_entry_reg_t)* v_area_domain_count), v_global_domain1_regs)) < 0 )
    {
        d_print(" Failed to  dec_domain_reg_entry (%d) \n",v_area_ret);
        return -1;
    }


    return (sizeof(ec_pdo_entry_reg_t) * v_area_domain_count);
}



// 5. DC Setup
int ecatmaster::f_init_DC_setup(int v_local_slave_number,
                                int v_local_dc_flag,
                                bool v_local_area_servo_drive)
{
    int   v_area_ret = 0;

#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP5

    {
        d_print("[ECATMASTER] : [init] : [f_init_DC_setup]\r\n");
        d_print("[STEP 5] : v_local_slave_number : %d\r\n",v_local_slave_number);
        d_print("[STEP 5] : v_local_dc_flag : %d\r\n",v_local_dc_flag);
        d_print("[STEP 5] : v_area_dc_info.dc_assign_activate : 0x%X\r\n",0x300);
        d_print("[STEP 5] : v_global_sync_ctime[0] : %d\r\n",v_global_sync_ctime[0]);
        d_print("[STEP 5] : v_global_sync_stime[0] : %d\r\n",v_global_sync_stime[0]);

    }

#endif


    if(v_local_area_servo_drive == true)
    {
        v_area_ret = v_global_dec_interface->f_oper_slave_config_dc(v_local_slave_number,
                                                                    v_local_dc_flag,
                                                                    0x300,
                                                                    v_global_sync_ctime[0],
                v_global_sync_stime[0],
                0,
                0,
                v_global_sync_refcnt[0]);

    }
    else
    {
        v_area_ret = v_global_dec_interface->f_oper_slave_config_dc(v_local_slave_number,
                                                                    v_local_dc_flag,
                                                                    0x00,
                                                                    v_global_sync_ctime[0],
                v_global_sync_stime[0],
                0,
                0,
                v_global_sync_refcnt[0]);
    }


    if (v_area_ret  < 0)
    {
        d_print(" Failed to  dec_slave_config_dc (%d) \n",v_area_ret);
    }

    return v_area_ret;
}


// 6. Setup Complete and active
int ecatmaster::f_init_slave_active()
{
    v_global_domain_data_size = v_global_dec_interface->f_oper_slave_active();
#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP6
    d_print("[ECATMASTER] : [init] : [f_init_slave_active]\r\n");
    d_print("[STEP 6] : v_global_domain_data_size : 0x%X\r\n",v_global_domain_data_size);
#endif
}

// 7. data pointer
int ecatmaster::f_init_open_domain_data(uint16_t v_local_size)
{
    v_global_domain1_pd = (uint8_t*)v_global_dec_interface->f_oper_dec_open_domain_data((int)v_local_size,
                                                                                        &v_global_domain_shmem_ptr,
                                                                                        &v_global_domain_shmem_size,
                                                                                        &v_global_decat_lib_version);


#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP7
    d_print("[ECATMASTER] : [init] : [f_init_slave_active]\r\n");
    d_print("[STEP 7] : v_global_domain_shmem_ptr : 0x%X\r\n",v_global_domain_shmem_ptr);
    d_print("[STEP 7] : v_global_domain_shmem_size : %d\r\n",v_global_domain_shmem_size);
    d_print("[STEP 7] : v_global_decat_lib_version : 0x%X\r\n",v_global_decat_lib_version);
#endif
}


// 8. rt task create
int ecatmaster::f_init_create_rt_task()
{
    int v_area_ret = 0;
    mlockall(MCL_CURRENT|MCL_FUTURE);

#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP8
    d_print("[ECATMASTER] : [init] : [f_init_slave_active]\r\n");
    d_print("[STEP 8] : rt_task_create \r\n");
#endif


    /*	9. rt event bind with ethercat master module */
    v_area_ret = rt_event_bind(&v_global_usr_event_desc, "EC_DC_SYNC_EVENT", 1000000000);
    if(v_area_ret < 0) {
        d_print("error!!, usr_event_bind (err_code:0x%x) \r\n", v_area_ret);
    }else{
        v_global_usr_event_flag = true;
    }

    // RT share semaphore
    v_area_ret = rt_sem_bind(&v_global_sem_desc,"EC_DC_SHARE_SEMAPHORE",TM_INFINITE);
    if(v_area_ret){
        d_print("error!!, rt_sem_bind : sem_desc (err_code:0x%x) \r\n", v_area_ret);
    }
    v_area_ret = rt_task_create(&v_global_rt_task_desc,
                                "ecat_task",
                                TASK_STKSZ,
                                90,
                                TASK_MODE);
    if (!v_area_ret)
    {
        rt_task_start(&v_global_rt_task_desc,&f_task_rt_ecat_task,this);
    }
    else
    {
        d_print("rt_task_create error (%d) \r\n", v_area_ret);
    }

    return 0;
}

//9.user task create
int ecatmaster::f_init_create_process_task()
{
    int v_area_ret = 0;
    v_area_ret = pthread_create(&v_global_GetState_thread, NULL,f_task_process_task,(void *)this);	//Get the Master and Slave state thread
    if (v_area_ret < 0)
    {
        perror("thread create error\r\n");
    }
}







/**
  * 2. EtherCAT master servo control part
  *
*/


/*  Function name : get_servo_state
 *  Description   : This function checks the status word of the slave to determine its status.
 *  Return value  : None.
 */

// CiA 402 profile
void ecatmaster::f_oper_get_servo_state(int v_local_slv_inx)
{
    int state;

    state = v_global_ecat_slave[v_local_slv_inx].v_global_cur_servo_state &  ~(0xF);

    if (v_global_ecat_slave[v_local_slv_inx].v_global_cur_status_word & 0x08)  {						//error
        state  |= STATE_SERVO_ERROR;
    }
    else if ((v_global_ecat_slave[v_local_slv_inx].v_global_cur_status_word  & 0x2F)==0x00)	{		// Switch on disabled --> shutdown
        state  |= STATE_SERVO_OFF;
    }
    else if ((v_global_ecat_slave[v_local_slv_inx].v_global_cur_status_word  & 0x2F)==0x20)	{		// Switch on disabled --> shutdown
        state  |= STATE_SERVO_OFF;
    }
    else if ((v_global_ecat_slave[v_local_slv_inx].v_global_cur_status_word  & 0x2F)==0x21)	{		// ready to switch on -->Switch on
        state  |= STATE_SERVO_OFF;
    }
    else if ((v_global_ecat_slave[v_local_slv_inx].v_global_cur_status_word  & 0x2F)==0x23)	{		// switched on --> servo On
        state  |= STATE_SERVO_SW_ON;
    }
    else if ((v_global_ecat_slave[v_local_slv_inx].v_global_cur_status_word  & 0x2F)==0x27)	{		// servo On
        state  |= STATE_SERVO_ON;
    }
    else
    {
        state  |= STATE_SERVO_ERROR;
    }

    if (v_global_ecat_slave[v_local_slv_inx].v_global_cur_servo_state != state)
    {
        v_global_ecat_slave[v_local_slv_inx].v_global_cur_servo_state = state;
    }
}

/*  Function name : set_servo_state
 *  Description   : This function determines the control word value
 *                  according to the slave status word value and writes it.
 *  Return value  : None.
 */

void ecatmaster::f_oper_set_servo_state(int v_local_slv_inx)
{
    long v_area_val = 0;
    static int v_area_task_mode_count[MAX_SLAVES]={0,};

    if (v_global_master_state.al_states < EC_AL_STATES_OP || v_global_domain_state.state.wc_state != EC_WC_COMPLETE)
    {
        return ;
    }



    if (v_global_ecat_slave[v_local_slv_inx].v_global_cur_status_word & 0x08)						//error --> Not send
    {
        v_global_ecat_slave[v_local_slv_inx].v_global_cur_step = step_error;
    }
    else
    {
        if(((v_global_ecat_slave[v_local_slv_inx].v_global_cur_status_word  & 0x2F)==0x00)||((v_global_ecat_slave[v_local_slv_inx].v_global_cur_status_word  & 0x2F)==0x20))
        {
            v_global_ecat_slave[v_local_slv_inx].v_global_cur_step = step_shutdown;
        }
        else if ((v_global_ecat_slave[v_local_slv_inx].v_global_cur_status_word  & 0x2F)==0x21)	 	// ready to switch on -->Switch on
        {
            if (v_global_ecat_slave[v_local_slv_inx].v_global_req_servo_state >= REQ_SERVO_ON)				//Servo On State
            {
                v_global_ecat_slave[v_local_slv_inx].v_global_cur_step = step_switch_on;				//go to next step
            }
            else if (v_global_ecat_slave[v_local_slv_inx].v_global_req_servo_state == REQ_SERVO_OFF)			//Servo Off State
            {
                v_global_ecat_slave[v_local_slv_inx].v_global_cur_step = step_shutdown;				//back to the first step
            }
        }
        else if ((v_global_ecat_slave[v_local_slv_inx].v_global_cur_status_word  & 0x2F)==0x23)    	// switched on --> servo On
        {
            if (v_global_ecat_slave[v_local_slv_inx].v_global_req_servo_state >= REQ_SERVO_ON)				//Servo On State
            {
                v_global_ecat_slave[v_local_slv_inx].v_global_cur_step = step_servo_on;				//go to next step
            }
            else if (v_global_ecat_slave[v_local_slv_inx].v_global_req_servo_state == REQ_SERVO_OFF)			//Servo Off State
            {
                v_global_ecat_slave[v_local_slv_inx].v_global_cur_step = step_shutdown;				//back to the first step
            }
        }
        else if ((v_global_ecat_slave[v_local_slv_inx].v_global_cur_status_word & 0x2F)==0x27)		// Servo on ok --> wirte position
        {
            if (v_global_ecat_slave[v_local_slv_inx].v_global_req_servo_state == REQ_SERVO_RUN)				//Servo Run State
            {
                if (v_global_ecat_slave[v_local_slv_inx].v_global_run_type == RUN_TASK)						//Task drive mode
                {
                    if((v_global_ecat_slave[v_local_slv_inx].v_global_cur_step != step_target_pos_rev) && (v_global_ecat_slave[v_local_slv_inx].v_global_cur_step != step_target_pos_fw)){
                        v_global_ecat_slave[v_local_slv_inx].v_global_cur_step = step_target_pos_fw;
                        v_global_ecat_slave[v_local_slv_inx].v_global_prev_step = v_global_ecat_slave[v_local_slv_inx].v_global_cur_step;
                    }

                    v_area_task_mode_count[v_local_slv_inx]++;
                    if(v_area_task_mode_count[v_local_slv_inx]>50)
                    {
                        v_area_task_mode_count[v_local_slv_inx]=0;
                        v_global_ecat_slave[v_local_slv_inx].v_global_cur_step = (v_global_ecat_slave[v_local_slv_inx].v_global_prev_step  == step_target_pos_fw)?
                                    step_target_pos_rev : step_target_pos_fw;
                        v_global_ecat_slave[v_local_slv_inx].v_global_prev_step = v_global_ecat_slave[v_local_slv_inx].v_global_cur_step;
                        return ;
                    }
                }
                else if (v_global_ecat_slave[v_local_slv_inx].v_global_run_type == RUN_FORWARD)				//Forward drive mode
                {
                    v_global_ecat_slave[v_local_slv_inx].v_global_cur_step = step_target_pos_fw;
                }
                else
                {
                    v_global_ecat_slave[v_local_slv_inx].v_global_cur_step = step_target_pos_rev;	//Reverse drive mode
                }

                v_global_ecat_slave[v_local_slv_inx].v_global_prev_step = v_global_ecat_slave[v_local_slv_inx].v_global_cur_step;
            }
            else if (v_global_ecat_slave[v_local_slv_inx].v_global_req_servo_state == REQ_SERVO_OFF)			//Servo Off State
            {
                v_global_ecat_slave[v_local_slv_inx].v_global_cur_step = step_shutdown;				//back to the first step
            }
            else if (v_global_ecat_slave[v_local_slv_inx].v_global_req_servo_state == REQ_SERVO_STOP)			//Servo Stop State
            {
                return;
            }
        }
    }

    if(v_global_ecat_slave[v_local_slv_inx].v_global_prev_step != v_global_ecat_slave[v_local_slv_inx].v_global_cur_step)
    {
        switch (v_global_ecat_slave[v_local_slv_inx].v_global_cur_step)
        {
        case step_error:	// error
            v_area_val = 0x80;		// fault reset
            d_print("ERROR [%d] cur_status_word = %x\n", v_local_slv_inx, v_global_ecat_slave[v_local_slv_inx].v_global_cur_status_word);
            break;
        case step_shutdown:	// voltage on
            v_area_val = 0x06;		// shutdown
            break;
        case step_switch_on:// switch on
            v_area_val = 0x07;		// switch on
            EC_WRITE_S8(v_global_domain1_pd + v_global_ecat_slave[v_local_slv_inx].f_oper_get_offset_value(MODE_OF_OP) , 0x08);
            break;
        case step_servo_on:	// servo on
            v_area_val = 0x0f;		//enable operation
            v_global_ecat_slave[v_local_slv_inx].v_global_set_target_val = v_global_ecat_slave[v_local_slv_inx].v_global_cur_position;											//Read current position
            d_print("[v_local_slv_inx:%d] = target_val:%ld\r\n", v_local_slv_inx, v_global_ecat_slave[v_local_slv_inx].v_global_set_target_val);	//Print current position

            EC_WRITE_U32(v_global_domain1_pd + v_global_ecat_slave[v_local_slv_inx].f_oper_get_offset_value(TARGET_POSITION)
                         , v_global_ecat_slave[v_local_slv_inx].v_global_set_target_val);		//Set the current position
            break;		// servo on
        case step_target_pos_fw:	//driving forward
        case step_target_pos_rev:	//driving reverse
            break;
        default:
            break;
        }




        if (v_area_val != EC_READ_U16(v_global_domain1_pd + v_global_ecat_slave[v_local_slv_inx].f_oper_get_offset_value(CONTROL_WORD)))	//Read current command
            EC_WRITE_U16(v_global_domain1_pd + v_global_ecat_slave[v_local_slv_inx].f_oper_get_offset_value(CONTROL_WORD), v_area_val);     //Write current command

        v_global_ecat_slave[v_local_slv_inx].v_global_prev_step = v_global_ecat_slave[v_local_slv_inx].v_global_cur_step;
    }
}


int ecatmaster::f_oper_get_servo_valid_state()
{
    int v_area_ax = 0;
    int v_area_state = 0;


    for(v_area_ax = 0; v_area_ax < v_global_slavecount; v_area_ax++)
    {
        if(v_global_ecat_slave[v_area_ax].f_get_is_servo_drive())
        {
            if(v_global_ecat_slave[v_area_ax].v_global_cur_servo_state == STATE_SERVO_ERROR)
            {
                v_area_state = STATE_SERVO_ERROR;
                break;
            }
            else if(v_global_ecat_slave[v_area_ax].v_global_cur_servo_state == STATE_SERVO_OFF)
            {
                v_area_state = STATE_SERVO_OFF;
                break;
            }
            else
            {
                v_area_state = STATE_SERVO_ON;
            }
        }
    }

    return v_area_state;
}


QString ecatmaster::f_oper_get_ecat_state()
{
    switch(v_global_cur_ecat_state)
    {
    case ECAT_STATE_INIT:
        return "Init";
    case ECAT_STATE_PREOP:
        return "Pre-Op";
    case ECAT_STATE_SAFEOP:
        return "Safe-Op";
    case ECAT_STATE_OP:
        return "OP";
    default:
        return "?";
    }

}

/**
  * 3. EtherCAT master start part
  *
*/



int ecatmaster::f_oper_slave_setting_initialize(int v_local_slave_num,
                                                char *v_local_slave_models,
                                                char *v_local_vendor_name,
                                                uint32_t v_local_vendorID,
                                                uint32_t v_local_productCODE,
                                                bool v_local_is_servo_driver)
{

    /*
     * fix !!!
     */


    if(v_local_vendorID == 0x0000066F)
    {
        if(v_local_productCODE == 0x515070A1)
        {
            // make pana slave information
            f_oper_pana1507BA1_make(v_global_ecat_slave[v_local_slave_num].f_get_v_global_slave_syncs_ptr(),
                                    v_global_ecat_slave[v_local_slave_num].f_get_v_global_pdos_ptr(),
                                    v_global_ecat_slave[v_local_slave_num].f_get_v_global_entries_ptr());
            // slave setting
            v_global_ecat_slave[v_local_slave_num].f_init_initialize(v_local_slave_models,
                                                                     v_local_vendor_name,
                                                                     v_local_vendorID,
                                                                     v_local_productCODE,
                                                                     v_local_slave_num,
                                                                     v_local_is_servo_driver);
        }
    }

    if(v_local_vendorID == 0x0000ffff)
    {
        if(v_local_productCODE == 0x00000000)
        {
            // make pana slave information
            f_oper_KITECH_ECAT_make(v_global_ecat_slave[v_local_slave_num].f_get_v_global_slave_syncs_ptr(),
                                    v_global_ecat_slave[v_local_slave_num].f_get_v_global_pdos_ptr(),
                                    v_global_ecat_slave[v_local_slave_num].f_get_v_global_entries_ptr());
            // slave setting
            v_global_ecat_slave[v_local_slave_num].f_init_initialize(v_local_slave_models,
                                                                     v_local_vendor_name,
                                                                     v_local_vendorID,
                                                                     v_local_productCODE,
                                                                     v_local_slave_num,
                                                                     v_local_is_servo_driver);
        }
    }



    if(v_local_vendorID == 0x00007595)
    {
        if(v_local_productCODE == 0x00000000)
        {
            // make pana slave information
            f_oper_LSMPO_make(v_global_ecat_slave[v_local_slave_num].f_get_v_global_slave_syncs_ptr(),
                              v_global_ecat_slave[v_local_slave_num].f_get_v_global_pdos_ptr(),
                              v_global_ecat_slave[v_local_slave_num].f_get_v_global_entries_ptr());

#if ETHERCAT_MASTER_CLASS_DEBUG_SLAVE_SETTING
            {
                int v_sub_syncs_count = 0;
                int v_sub_pdos_count = 0;
                int v_sub_entreis_count = 0;

                ec_sync_info_t * v_area_ptr_syncs = v_global_ecat_slave[v_local_slave_num].f_get_v_global_slave_syncs_ptr();


                for(v_sub_syncs_count = 0;
                    v_area_ptr_syncs[v_sub_syncs_count].index != 0xFF;
                    ++v_sub_syncs_count)
                {
                    d_print("[ECATMASTER]:[start]:[f_oper_master_initialize]\r\n");
                    d_print("[SLAVE_SETTING] : SYNCS [%d] \r\n",v_sub_syncs_count);
                    d_print("[SLAVE_SETTING] : SYNCS  : index : 0x%X\r\n",v_area_ptr_syncs[v_sub_syncs_count].index);
                    d_print("[SLAVE_SETTING] : SYNCS  : n_pdos : %d\r\n",v_area_ptr_syncs[v_sub_syncs_count].n_pdos);
                    if(v_area_ptr_syncs[v_sub_syncs_count].dir == EC_DIR_INPUT)
                        d_print("[SLAVE_SETTING] : SYNCS  : dir : INPUT \r\n");
                    else if(v_area_ptr_syncs[v_sub_syncs_count].dir == EC_DIR_OUTPUT)
                        d_print("[SLAVE_SETTING] : SYNCS  : dir : OUTPUT \r\n");

                    for(v_sub_pdos_count = 0;
                        v_sub_pdos_count < v_area_ptr_syncs[v_sub_syncs_count].n_pdos;
                        ++ v_sub_pdos_count)
                    {
                        d_print("[SLAVE_SETTING] : PDOS [%d]\r\n",v_sub_pdos_count);
                        d_print("[SLAVE_SETTING] : PDOS : index : 0x%X\r\n",v_area_ptr_syncs[v_sub_syncs_count].pdos[v_sub_pdos_count].index);
                        d_print("[SLAVE_SETTING] : PDOS : n_entreis : %d\r\n",v_area_ptr_syncs[v_sub_syncs_count].pdos[v_sub_pdos_count].n_entries);

                        for(v_sub_entreis_count = 0;
                            v_sub_entreis_count < v_area_ptr_syncs[v_sub_syncs_count].pdos[v_sub_pdos_count].n_entries;
                            ++ v_sub_entreis_count)
                        {
                            d_print("[SLAVE_SETTING] : ENTREIS [%d]\r\n",v_sub_entreis_count);
                            d_print("[SLAVE_SETTING] : ENTREIS : index : 0x%X\r\n",
                                    v_area_ptr_syncs[v_sub_syncs_count].pdos[v_sub_pdos_count].entries[v_sub_entreis_count].index);
                            d_print("[SLAVE_SETTING] : ENTREIS : subindex : 0x%X\r\n",
                                    v_area_ptr_syncs[v_sub_syncs_count].pdos[v_sub_pdos_count].entries[v_sub_entreis_count].subindex);
                        }
                    }
                }

            }

#endif




            // slave setting
            v_global_ecat_slave[v_local_slave_num].f_init_initialize(v_local_slave_models,
                                                                     v_local_vendor_name,
                                                                     v_local_vendorID,
                                                                     v_local_productCODE,
                                                                     v_local_slave_num,
                                                                     v_local_is_servo_driver);
        }
    }

}


int ecatmaster::f_oper_master_initialize(int v_local_slave_count,
                                         int v_local_domain_count,
                                         ulong v_local_sync_ctime,
                                         ulong v_local_sync_stime,
                                         ulong v_local_refcnt,
                                         int v_local_dc_flag,
                                         bool v_local_servo_flag)
{
    int v_area_slavenumber = 0;
    ec_pdo_entry_reg_t *v_area_ptr_outreg[MAX_SLAVES];
    ec_pdo_entry_reg_t *v_area_ptr_inreg[MAX_SLAVES];
    ec_sync_info_t *v_area_ptr_syncs[MAX_SYNC];


    for(v_area_slavenumber = 0; v_area_slavenumber < v_local_slave_count; ++v_area_slavenumber)
        v_global_ecat_slave[v_area_slavenumber].f_init_master_insert(this);


    v_global_sync_ctime[0] = v_local_sync_ctime;
    v_global_sync_stime[0] = v_local_sync_stime;
    v_global_sync_refcnt[0] = v_local_refcnt;


#if ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP1
    //step 1 slave ready
    f_init_slave_initialize(v_local_slave_count,v_local_domain_count);
#endif

#if ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP2
    //step 2 slave information insert
    for(v_area_slavenumber = 0; v_area_slavenumber < v_local_slave_count; ++v_area_slavenumber)
    {
        v_area_ptr_outreg[v_area_slavenumber] = v_global_ecat_slave[v_area_slavenumber].f_get_slave_domain_out_regs_ptr();

        f_init_slave_information_insert(v_area_slavenumber,
                                        v_area_ptr_outreg[v_area_slavenumber][0].vendor_id,
                v_area_ptr_outreg[v_area_slavenumber][0].product_code,
                v_area_ptr_outreg[v_area_slavenumber][0].alias,
                v_area_ptr_outreg[v_area_slavenumber][0].position);
#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP2
        d_print("[ECATMASTER]:[start]:[f_oper_master_initialize]\r\n");
        d_print("[STEP 2] : slave number : %d\r\n",v_area_slavenumber);
        d_print("[STEP 2] : v_area_ptr_outreg->vendor_id : 0x%X\r\n",v_area_ptr_outreg[v_area_slavenumber]->vendor_id);
        d_print("[STEP 2] : v_area_ptr_outreg->product_code : 0x%X\r\n",v_area_ptr_outreg[v_area_slavenumber]->product_code);
        d_print("[STEP 2] : v_area_ptr_outreg->alias : 0x%X\r\n",v_area_ptr_outreg[v_area_slavenumber]->alias);
        d_print("[STEP 2] : v_area_ptr_outreg->position : 0x%X\r\n",v_area_ptr_outreg[v_area_slavenumber]->position);
#endif

    }
#endif


#if ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP3
    //step 3 pdo list insert
    for(v_area_slavenumber = 0; v_area_slavenumber < v_local_slave_count  ; ++v_area_slavenumber)
    {

        v_area_ptr_syncs[v_area_slavenumber] = v_global_ecat_slave[v_area_slavenumber].f_get_v_global_slave_syncs_ptr();

        f_init_PDO_List_send(v_area_slavenumber,
                             v_area_ptr_syncs[v_area_slavenumber],
                             v_local_servo_flag);

#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP3
        {
            int v_sub_syncs_count = 0;
            int v_sub_pdos_count = 0;
            int v_sub_entreis_count = 0;

            for(v_sub_syncs_count =0;
                v_area_ptr_syncs[v_area_slavenumber][v_sub_syncs_count].index != 0xFF;
                ++v_sub_syncs_count )
            {
                d_print("[ECATMASTER]:[start]:[f_oper_master_initialize]\r\n");
                d_print("[STEP 3] : SYNCS [%d] \r\n",v_sub_syncs_count);
                d_print("[STEP 3] : SYNCS  : index : 0x%X\r\n",v_area_ptr_syncs[v_area_slavenumber][v_sub_syncs_count].index);
                d_print("[STEP 3] : SYNCS  : n_pdos : %d\r\n",v_area_ptr_syncs[v_area_slavenumber][v_sub_syncs_count].n_pdos);
                if(v_area_ptr_syncs[v_area_slavenumber][v_sub_syncs_count].dir == EC_DIR_INPUT)
                    d_print("[STEP 3] : SYNCS  : dir : INPUT \r\n");
                else if(v_area_ptr_syncs[v_area_slavenumber][v_sub_syncs_count].dir == EC_DIR_OUTPUT)
                    d_print("[STEP 3] : SYNCS  : dir : OUTPUT \r\n");

                for(v_sub_pdos_count = 0;
                    v_sub_pdos_count < v_area_ptr_syncs[v_area_slavenumber][v_sub_syncs_count].n_pdos;
                    ++ v_sub_pdos_count)
                {
                    d_print("[STEP 3] : PDOS [%d]\r\n",v_sub_pdos_count);
                    d_print("[STEP 3] : PDOS : index : 0x%X\r\n",v_area_ptr_syncs[v_area_slavenumber][v_sub_syncs_count].pdos[v_sub_pdos_count].index);
                    d_print("[STEP 3] : PDOS : n_entreis : %d\r\n",v_area_ptr_syncs[v_area_slavenumber][v_sub_syncs_count].pdos[v_sub_pdos_count].n_entries);

                    for(v_sub_entreis_count = 0;
                        v_sub_entreis_count < v_area_ptr_syncs[v_area_slavenumber][v_sub_syncs_count].pdos[v_sub_pdos_count].n_entries;
                        ++ v_sub_entreis_count)
                    {
                        d_print("[STEP 3] : ENTREIS [%d]\r\n",v_sub_entreis_count);
                        d_print("[STEP 3] : ENTREIS : index : 0x%X\r\n",
                                v_area_ptr_syncs[v_area_slavenumber][v_sub_syncs_count].pdos[v_sub_pdos_count].entries[v_sub_entreis_count].index);
                        d_print("[STEP 3] : ENTREIS : subindex : 0x%X\r\n",
                                v_area_ptr_syncs[v_area_slavenumber][v_sub_syncs_count].pdos[v_sub_pdos_count].entries[v_sub_entreis_count].subindex);
                    }
                }
            }

        }
#endif
    }
#endif

#if ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP4
    //step 4 make domain ready
    for(v_area_slavenumber = 0; v_area_slavenumber < v_local_slave_count; ++v_area_slavenumber)
    {
        v_area_ptr_inreg[v_area_slavenumber] = v_global_ecat_slave[v_area_slavenumber].f_get_slave_domain_in_regs_ptr();
        v_area_ptr_outreg[v_area_slavenumber] = v_global_ecat_slave[v_area_slavenumber].f_get_slave_domain_out_regs_ptr();
        v_area_ptr_syncs[v_area_slavenumber] = v_global_ecat_slave[v_area_slavenumber].f_get_v_global_slave_syncs_ptr();


#if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP4
        {
            int v_sub_incount = 0;
            int v_sub_outcount = 0;

            for(v_sub_incount = 0; v_global_ecat_slave[v_area_slavenumber].f_get_slave_domain_in_regs_ptr()[v_sub_incount].index != 0x00; ++ v_sub_incount)
            {
                d_print("[ECATMASTER]:[start]:[f_oper_master_initialize]\r\n");
                d_print("[STEP 4] : inreg [%d]\r\n",v_sub_incount);
                d_print("[STEP 4] : vendor id : 0x%X\r\n",v_global_ecat_slave[v_area_slavenumber].f_get_slave_domain_in_regs_ptr()[v_sub_incount].vendor_id);
                d_print("[STEP 4] : product id : 0x%X\r\n",v_global_ecat_slave[v_area_slavenumber].f_get_slave_domain_in_regs_ptr()[v_sub_incount].product_code);
                d_print("[STEP 4] : index : 0x%X\r\n",v_global_ecat_slave[v_area_slavenumber].f_get_slave_domain_in_regs_ptr()[v_sub_incount].index);
                d_print("[STEP 4] : subindex : 0x%X\r\n",v_global_ecat_slave[v_area_slavenumber].f_get_slave_domain_in_regs_ptr()[v_sub_incount].subindex);
            }

            for(v_sub_outcount = 0; v_global_ecat_slave[v_area_slavenumber].f_get_slave_domain_out_regs_ptr()[v_sub_outcount].index != 0x00; ++ v_sub_outcount)
            {
                d_print("[ECATMASTER]:[start]:[f_oper_master_initialize]\r\n");
                d_print("[STEP 4] :outreg [%d]\r\n",v_sub_outcount);
                d_print("[STEP 4] : vendor id : 0x%X\r\n",v_global_ecat_slave[v_area_slavenumber].f_get_slave_domain_out_regs_ptr()[v_sub_outcount].vendor_id);
                d_print("[STEP 4] : product id : 0x%X\r\n",v_global_ecat_slave[v_area_slavenumber].f_get_slave_domain_out_regs_ptr()[v_sub_outcount].product_code);
                d_print("[STEP 4] : index : 0x%X\r\n",v_global_ecat_slave[v_area_slavenumber].f_get_slave_domain_out_regs_ptr()[v_sub_outcount].index);
                d_print("[STEP 4] : subindex : 0x%X\r\n",v_global_ecat_slave[v_area_slavenumber].f_get_slave_domain_out_regs_ptr()[v_sub_outcount].subindex);
            }
        }
#endif
    }



    //step 4 make domain
    f_init_make_domain(v_area_slavenumber,
                       v_area_ptr_outreg,
                       v_area_ptr_inreg,
                       v_area_ptr_syncs);


#endif


#if ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP5
    //step 5 DC Set
    for(v_area_slavenumber = 0; v_area_slavenumber < v_local_slave_count; ++v_area_slavenumber)
    {
        f_init_DC_setup(v_area_slavenumber,
                        v_local_dc_flag,
                        v_local_servo_flag);
    }
#endif


#if ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP6

    //step 6 Setup complete and active
    f_init_slave_active();
#endif

#if ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP7
    //step 7
    f_init_open_domain_data(C_GLOBAL_MEMORY_MAP_SIZE);
#endif

#if ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP8
    // 8. rt task create
    f_init_create_rt_task();
#endif

#if ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP9
    //9.user task create
    f_init_create_process_task();
#endif
}



int ecatmaster::f_oper_master_deinitialize()
{
    int v_area_slv_inx;

    v_global_ecat_task_run = 0;



}



/**
  * 4. EtherCAT master oper part
  *
*/




//sdo write
// if servo driver mode of operation index send date
int ecatmaster::f_oper_sdo_write(int v_local_slave_number,
                                 uint16_t v_local_index,
                                 uint16_t v_local_subindex,
                                 uint32_t v_local_data,
                                 COMMUNICAT_SIZE_TYPE v_local_datatype)
{
    int   v_area_ret = 0;
    switch(v_local_datatype)
    {
    case _8BIT:
    {
        v_area_ret = v_global_dec_interface->f_oper_sdo_read_8(v_local_slave_number, v_local_index,v_local_subindex,(uint8_t*)&v_local_data);
        if (v_area_ret < 0)
        {
            d_print( "sdo_8 write failed (%x) \n",v_local_index);
        }
    }
        break;
    case _16BIT:
    {
        v_area_ret = v_global_dec_interface->f_oper_sdo_read_16(v_local_slave_number, v_local_index,v_local_subindex,(uint8_t*)&v_local_data);
        if (v_area_ret < 0)
        {
            d_print( "sdo_16 write failed (%x) \n",v_local_index);
        }
    }
        break;
    case _32BIT:
    {
        v_area_ret = v_global_dec_interface->f_oper_sdo_read_32(v_local_slave_number, v_local_index,v_local_subindex,(uint8_t*)&v_local_data);
        if (v_area_ret < 0)
        {
            d_print( "sdo_32 write failed (%x) \n",v_local_index);
        }
    }
        break;
    }
}




/**
  * 5. EtherCAT master task suport part
  *
*/



//task support




/*  Function name : time_diff_print
 *  Description   : This function measurment of time gap.
 *  Return value  : None.
 */
void ecatmaster::time_diff_print(int id, int print_mode)
{
    static uint32_t cur_min[5]={0,}, cur_max[5]={0,};

    static uint32_t ts_start[5], ts_end[5], ts_gap[5];

    static uint32_t n_gap[5]={0,}, n_min[5]={0,}, n_max[5]={0,}, n_tot[5]={0,}, n_avg[5]={0,}, n_cnt[5]={0,}, n_init[5]={0,}, n_loop[5]={0,}, n_dc[5]={0,};



    if(id == 0)
        n_dc[id]=v_global_sync_ctime[0]/1000;
    else
        n_dc[id]=v_global_sync_ctime[0];

    switch(print_mode) {
    case 0:
        ts_start[id] = rt_timer_read();
        break;
    case 1:
        n_cnt[id]++;
        if((n_tot[id] >= 0xFFFF0000) || (n_cnt[id] > 10000)) {
            n_gap[id]=0, n_tot[id]=0,n_avg[id]=0,n_cnt[id]=0;
            n_loop[id]++;
        }
        ts_end[id] = rt_timer_read();
        ts_gap[id] = (ts_end[id] - ts_start[id]);

        if(id == 0){
            n_gap[id] = (ts_gap[id]/1000);
            n_tot[id] += abs((long)n_dc[id]-(long)n_gap[id]);
        }
        else{
            n_gap[id] = (ts_gap[id]);
            n_tot[id] += n_gap[id];
        }

        if(n_cnt[id] >= 1){
            if(id == 0)
                n_avg[id] = n_dc[id]-(n_tot[id]/n_cnt[id]);
            else
                n_avg[id] = (n_tot[id]/n_cnt[id]);
        }

        if(n_loop[id]){
            if((n_min[id]>=n_gap[id])||(!n_min[id]))
                n_min[id] = n_gap[id];
            if(n_max[id]<=n_gap[id])
                n_max[id] = n_gap[id];

            if((cur_min[id]>=n_gap[id])||(!cur_min[id]))
                cur_min[id] = n_gap[id];
            if(cur_max[id]<=n_gap[id])
                cur_max[id] = n_gap[id];
        }
        break;
    case 2:
        if( (n_cnt[id] > 0) && ((n_cnt[id] % 1000) == 0) && (n_loop[id])){
            d_print("ID:%d,%d, avg:%ld, min:%ld(%ld), max:%ld(%ld), cnt:%ld:%ld \r\n",id,n_init[id], n_avg[id], n_min[id], cur_min[id], n_max[id], cur_max[id] , n_loop[id], n_cnt[id]);
            cur_min[id] = 0;
            cur_max[id] = 0;
        }
        break;
    }

}





/*  Function name : f_task_ecat_master_event_wait
 *  Description   : This function waits for the periodic master's rt task event.
 *  Return value  : Returns 0 on success and error code on failure.
 */
static int before_err = 0;
int ecatmaster::f_task_ecat_master_event_wait()
{

    int v_area_err = 0;
    unsigned long mask_ret = 0;
    if(v_global_usr_event_flag) {


        v_area_err = rt_event_wait(&v_global_usr_event_desc,
                                   EVENT_WAIT_APP,
                                   &mask_ret,
                                   EV_ANY, /* Disjunctive wait */
                                   v_global_sync_ctime[0]*2);


        if((v_area_err != 0) && (!before_err)){
            d_print("error!!, rt_event_wait (err_code:0x%x, before_err:0x%x) \r\n", v_area_err, before_err);
        }
        else if((!v_area_err) && (before_err)){
            d_print("cleared!!, rt_event_wait (err_code:0x%x, before_err:0x%x) \r\n", v_area_err, before_err);
        }
        before_err = v_area_err;
        v_area_err = rt_event_clear(&v_global_usr_event_desc,
                                    EVENT_WAIT_APP,
                                    &mask_ret);
    }
    return before_err;
}

/*  Function name : f_task_ecat_master_end_event_send
 *  Description   : This function informs the master that the application's rt task has finished running.
 *  Return value  : Returns 0 on success and error code on failure.
 */
int ecatmaster::f_task_ecat_master_end_event_send()
{
    int v_area_err = 0;

    if(v_global_usr_event_flag) {
        if(v_global_ecat_task_run == 0)
            return -1;
        v_area_err = rt_event_signal(&v_global_usr_event_desc,EVENT_WAIT_END);
        if(v_area_err < 0){
            d_print("error!!, rt_event_signal: EVENT_WAIT_END (err_code:%d) \r\n", v_area_err);
        }
    }

    return v_area_err;
}

/*  Function name : f_task_ecat_master_ready_event_send
 *  Description   : This function informs the master that the application's rt task is ready to run.
 *  Return value  : Returns 0 on success and error code on failure.
 */
int ecatmaster::f_task_ecat_master_ready_event_send()
{
    int v_area_err = 0;

    if(v_global_usr_event_flag) {

#if RT_TASK_FUNCTION_DEBUG
        d_print("[ECATMASTER] : [TASK] : [f_task_ecat_master_ready_event_send]\r\n");
        d_print("READY EVENT \r\n");
        d_print("v_global_ecat_task_run : %d \r\n",v_global_ecat_task_run);
#endif

        if(v_global_ecat_task_run == 0)
        {
#if RT_TASK_FUNCTION_DEBUG
            d_print("[ECATMASTER] : [TASK] : [f_task_ecat_master_ready_event_send]\r\n");
            d_print("error v_global_ecat_task_run : %d \r\n",v_global_ecat_task_run);
#endif
            return -1;
        }
        v_area_err = rt_event_signal(&v_global_usr_event_desc,EVENT_WAIT_READY);
        if(v_area_err < 0){
            d_print("error!!, rt_event_signal: EVENT_WAIT_READY (err_code:%d) \r\n", v_area_err);
        }
    }

    return v_area_err;
}


/**
  * 6. EtherCAT master task function part
  *
*/



// task process
void f_task_rt_ecat_task(void *v_local_ptr_cookie)
{
    // EtherCAT control class ptr
    ecatmaster *v_area_ecat_control = (ecatmaster *)v_local_ptr_cookie;

    // dain interface class ptr value
    daininterface *v_area_dinterface_ptr = v_area_ecat_control->f_get_dec_interface();

    //task control check value address
    int *v_area_ecat_task_run_ptr = v_area_ecat_control->f_get_ecat_task_run_ptr();
    int *v_area_ecat_task_exit_ptr = v_area_ecat_control->f_get_ecat_task_exit_ptr();

    // domain data frame addresss
    uint8_t **v_area_domain1_pd = v_area_ecat_control->f_get_domain1_pd_ptr();
    int *v_area_domain_data_size = v_area_ecat_control->f_get_domain_data_size_ptr();

    // frame out, in head address
    unsigned int ** v_area_out_off_head = v_area_ecat_control->f_get_ptr_output_offset_haed_ptr();
    unsigned int ** v_area_in_off_head = v_area_ecat_control->f_get_ptr_input_offset_haed_ptr();

    //rt event address
    RT_EVENT *v_area_event_desc_ptr = v_area_ecat_control->f_get_usr_event_desc_ptr();
    //tr task address
    RT_TASK *v_area_rt_task_desc = v_area_ecat_control->f_get_rt_task_desc_ptr();


    // cycle time value
    ulong v_area_ctime = (v_area_ecat_control->f_get_sync_ctime())[0];


    // shared memory address
    void **v_area_shared_mem_ptr = v_area_ecat_control->f_get_domain_shmem_ptr();


    dec_master_state_t *v_area_master_state = v_area_ecat_control->f_get_master_state_ptr();
    dec_domain_state_t *v_area_domain_state = v_area_ecat_control->f_get_domain_state_ptr();



    unsigned long v_area_mask_ret = 0;
    int v_area_err = 0;
    dec_ioctl_master_version_t v_area_m_ver={0,};


    REF(v_area_ecat_task_run_ptr) = 1;
    REF(v_area_ecat_task_exit_ptr) = 0;




    if (!REF(v_area_domain1_pd) || !REF(v_area_domain_data_size))
    {
        d_print("error!!, domain_size (%d), domain_pd (0x%x) \r\n", REF(v_area_domain_data_size),REF(v_area_domain1_pd) );
        d_print("rt_ecat_task() exit \r\n");
        REF(v_area_ecat_task_run_ptr) = 0;
        return ;
    }


#if RT_TASK_FUNCTION_DEBUG
    d_print("[ECATMASTER] : [TASK] : [f_task_rt_ecat_task]\r\n");
    d_print("domain address check , domain size check \r\n");
    d_print("REF(v_area_domain1_pd) : 0x%X\r\n",REF(v_area_domain1_pd));
    d_print("REF(v_area_domain_data_size) : %d\r\n",REF(v_area_domain_data_size));
#endif

    /* set sync offset */
    if(REF(v_area_in_off_head))
        REF(v_area_dinterface_ptr).f_oper_set_recv_sync(*REF(v_area_in_off_head),
                                                        (REF(v_area_domain_data_size) - (*REF(v_area_in_off_head))));

#if RT_TASK_FUNCTION_DEBUG
    d_print("[ECATMASTER] : [TASK] : [f_task_rt_ecat_task]\r\n");
    d_print("input offset check , input size check \r\n");
    d_print("*REF(v_area_in_off_head) : 0x%X\r\n",*REF(v_area_in_off_head));
    d_print("REF(v_area_domain_data_size) : %d\r\n",REF(v_area_domain_data_size));
    d_print("(REF(v_area_domain_data_size) - (*REF(v_area_in_off_head))) : %d\r\n",(REF(v_area_domain_data_size) - (*REF(v_area_in_off_head))));
#endif


    if(REF(v_area_out_off_head))
    {
        if(REF(v_area_in_off_head))
            REF(v_area_dinterface_ptr).f_oper_set_send_sync(*REF(v_area_out_off_head), *REF(v_area_in_off_head) /* size */);
        else
            REF(v_area_dinterface_ptr).f_oper_set_send_sync(*REF(v_area_out_off_head),REF(v_area_domain_data_size));
    }

#if RT_TASK_FUNCTION_DEBUG
    d_print("[ECATMASTER] : [TASK] : [f_task_rt_ecat_task]\r\n");
    d_print("input offset check , input size check \r\n");
    d_print("*REF(v_area_out_off_head) : 0x%X\r\n",*REF(v_area_out_off_head));
    d_print("REF(v_area_out_off_head) : %d\r\n",REF(v_area_domain_data_size));
    d_print("(REF(v_area_out_off_head) - (*REF(v_area_in_off_head))) : %d\r\n",(REF(v_area_domain_data_size) - (*REF(v_area_out_off_head))));
#endif


    v_area_ecat_control->f_task_ecat_master_ready_event_send();

    rt_event_clear(v_area_event_desc_ptr, EVENT_WAIT_APP, &v_area_mask_ret);

    REF(v_area_dinterface_ptr).f_oper_get_master_version(&v_area_m_ver);

    d_print("Master Version: %s\n",&v_area_m_ver.str_data);
#if RT_TASK_FUNCTION_DEBUG
    d_print("v_area_ecat_task_run_ptr : %d\r\n",REF(v_area_ecat_task_run_ptr));
#endif

    int indx = 0;
    slave_data_t local_slave_data;

    while (REF(v_area_ecat_task_run_ptr))			//Task run flag
    {
        if(v_area_ecat_control->f_task_ecat_master_event_wait())	//Wait to EtherCAT Master's Real Time Event
        {
            rt_task_sleep(v_area_ctime);
        }

        if (REF(v_area_master_state).al_states >= EC_AL_STATES_OP && REF(v_area_domain_state).state.wc_state == EC_WC_COMPLETE)
        {
            for(int v_sub_count =0; v_sub_count < v_area_ecat_control->f_get_slave_count(); ++v_sub_count)
            {
                v_area_ecat_control->f_get_slave()[v_sub_count].f_oper_recv_process();
                v_area_ecat_control->f_get_slave()[v_sub_count].f_oper_send_process();
            }
        }
        else
        {
            for(int v_sub_count =0; v_sub_count < v_area_ecat_control->f_get_slave_count(); ++v_sub_count)
            {
                v_area_ecat_control->f_get_slave()[v_sub_count].f_oper_recv_process();
            }
        }

        local_slave_data.indx = indx++;

        for(int v_sub_count =0; v_sub_count < v_area_ecat_control->f_get_slave_count(); ++v_sub_count)
        {
            local_slave_data.cur_status_word[v_sub_count] = v_area_ecat_control->f_get_slave()[v_sub_count].v_global_cur_status_word;
            local_slave_data.tar_position[v_sub_count]    = v_area_ecat_control->f_get_slave()[v_sub_count].v_global_tar_position;
            local_slave_data.tar_velocity[v_sub_count]    = v_area_ecat_control->f_get_slave()[v_sub_count].v_global_tar_velocity;
            local_slave_data.tar_torque[v_sub_count]      = v_area_ecat_control->f_get_slave()[v_sub_count].v_global_tar_torque;
            local_slave_data.cur_position[v_sub_count]    = v_area_ecat_control->f_get_slave()[v_sub_count].v_global_cur_position;
            local_slave_data.cur_velocity[v_sub_count]    = v_area_ecat_control->f_get_slave()[v_sub_count].v_global_cur_velocity;
            local_slave_data.cur_torque[v_sub_count]      = v_area_ecat_control->f_get_slave()[v_sub_count].v_global_cur_torque_value1;
        }

        v_area_ecat_control->slave_data.push_back(local_slave_data);
    }

    v_area_ecat_control->f_task_ecat_master_end_event_send();

    pthread_cancel(v_area_ecat_control->f_get_GetState_thread());

    REF(v_area_dinterface_ptr).f_oper_slave_deactive();
    REF(v_area_dinterface_ptr).f_oper_close_domain_date(v_area_shared_mem_ptr, REF(v_area_domain1_pd), C_GLOBAL_MEMORY_MAP_SIZE);

#if DC_RT_SHARE_SEMAPHORE
    v_area_err = rt_sem_p(&sem_desc, TM_INFINITE);
    if (v_area_err) {
        d_print("error!!, rt_sem_p: %d\n", v_area_err);
    }
#endif

    v_area_err = rt_event_unbind(v_area_event_desc_ptr);
    if(v_area_err < 0){
        d_print("error!!, rt_event_unbind (err_code:0x%x) \r\n", v_area_err);
    }
    REF(v_area_dinterface_ptr).f_oper_close();
    d_print("APP deactive \r\n", v_area_err);

    rt_task_delete(v_area_rt_task_desc);
}

void *f_task_process_task(void *v_local_ptr_data)
{

    // EtherCAT control class ptr
    ecatmaster *v_area_ecat_control = (ecatmaster *)v_local_ptr_data;

    // dain interface class ptr value
    daininterface *v_area_dinterface_ptr = v_area_ecat_control->f_get_dec_interface();

    // slave state
    dec_slave_state_t *v_area_slave_state_ptr =  v_area_ecat_control->f_get_slave_state_ptr();

    // master state
    dec_master_state_t *v_area_master_state_ptr = v_area_ecat_control->f_get_master_state_ptr();

    // domain state
    dec_domain_state_t *v_area_domain_state_ptr = v_area_ecat_control->f_get_domain_state_ptr();

    ecatslave *v_area_slave = v_area_ecat_control->f_get_slave();


    // Ethercat master current state
    int * v_area_cur_ecat_state = v_area_ecat_control->f_get_cur_ecat_state_ptr();

    // domain current state
    int *v_area_cur_domain_state = v_area_ecat_control->f_get_cur_domain_state_ptr();


    //slave numbers
    int v_area_slave_numbers = REF(v_area_ecat_control).f_get_slave_count();

    int v_area_slv_inx;
    dec_master_state_t   v_area_get_master_state;
    dec_domain_state_t   v_area_get_domain_state;
    static int loop_count=0;

    v_area_ecat_control->fp = fopen("/mnt/mtd5/daincube/logging_data.csv", "w+");
    unsigned int len = 0;

    while(1)
    {
        REF(v_area_dinterface_ptr).f_oper_get_master_state(&v_area_get_master_state);
        REF(v_area_dinterface_ptr).f_oper_get_domain_state(&v_area_get_domain_state);

        for (v_area_slv_inx = 0; v_area_slv_inx < v_area_slave_numbers; v_area_slv_inx++)
        {
            v_area_slave_state_ptr[v_area_slv_inx].slave_number = v_area_slv_inx;
            REF(v_area_dinterface_ptr).f_oper_get_slave_state(&v_area_slave_state_ptr[v_area_slv_inx]);
        }

        if  (v_area_get_master_state.al_states != REF(v_area_master_state_ptr).al_states )
        {
            memcpy(v_area_master_state_ptr, &v_area_get_master_state,sizeof(dec_master_state_t));

            REF(v_area_cur_ecat_state)  = (int)v_area_get_master_state.al_states;
        }

        if (v_area_get_domain_state.state.wc_state != REF(v_area_domain_state_ptr).state.wc_state)
        {
            memcpy(v_area_domain_state_ptr, &v_area_get_domain_state,sizeof(dec_domain_state_t));
            REF(v_area_cur_domain_state) = (int)v_area_get_domain_state.state.working_counter;
        }

        if(loop_count++ > 10)	//cycle time * 10, cycle time: 10ms = 100mssec
        {
            loop_count = 0;
            for (v_area_slv_inx = 0; v_area_slv_inx < v_area_slave_numbers; v_area_slv_inx++)
            {
                if(v_area_slave[v_area_slv_inx].f_get_is_servo_drive() == true)
                {
                    REF(v_area_ecat_control).f_oper_get_servo_state(v_area_slv_inx);
                    REF(v_area_ecat_control).f_oper_set_servo_state(v_area_slv_inx);
                }
            }
        }

        len = v_area_ecat_control->slave_data.size();
        if(len > 0){
            for(unsigned int i = 0; i < len; i++){
                fprintf(v_area_ecat_control->fp, "%d,", v_area_ecat_control->slave_data.front().indx);
                for(int count = 0; count < v_area_slave_numbers; count++){
                fprintf(v_area_ecat_control->fp, "%d,%ld,%ld,%ld,%ld,%d,%d,",
                        v_area_ecat_control->slave_data.front().cur_status_word[count],
                        v_area_ecat_control->slave_data.front().tar_position[count], v_area_ecat_control->slave_data.front().cur_position[count],
                        v_area_ecat_control->slave_data.front().tar_velocity[count], v_area_ecat_control->slave_data.front().cur_velocity[count],
                        v_area_ecat_control->slave_data.front().tar_torque[count],   v_area_ecat_control->slave_data.front().cur_torque[count]);
                }
                fprintf(v_area_ecat_control->fp, "\n");
                v_area_ecat_control->slave_data.erase(v_area_ecat_control->slave_data.begin());
            }
            len = 0;
        }

        usleep(10000);	//10ms
    }

    fclose(v_area_ecat_control->fp);
}

