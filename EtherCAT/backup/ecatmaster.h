#ifndef ECATMASTER_H
#define ECATMASTER_H
#include "globalstruct.h"
#include "daininterface.h"
#include "ecatslave.h"


//rt include
#include <native/task.h>
#include <native/timer.h>
#include <native/alarm.h>
#include <native/event.h>
#include <native/mutex.h>
#include <native/sem.h>
#include <sys/mman.h>
#include <rtdk.h>

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <vector>
using namespace std;


//fix plz
#include "esi.h"

/* name config
    - first name
        * v : velue
        * f : function
        * s : struct

    - second name
        * function behavior
            ex) : set, get, oper ...

    - third name
        * function name
            ex) : initialize , ....
*/

// rt task function
void f_task_rt_ecat_task(void *v_local_ptr_cookie);
void *f_task_process_task(void *v_local_ptr_data);

class ecatmaster
{
    // velue
    private:
        int v_global_slavecount; // connect slave count
        int v_global_domaincount; // Domain count

        daininterface *v_global_dec_interface; // daincube EtherCAT interface
        ecatslave v_global_ecat_slave[MAX_SLAVES]; // insert slave buffer


        //EtherCAT master frame buffer
        ec_pdo_entry_reg_t  v_global_domain1_regs[30*MAX_SLAVES];


        unsigned int *v_global_ptr_output_offset_haed;
        unsigned int *v_global_ptr_input_offset_haed;

        uint8_t *v_global_domain1_pd;
        int v_global_domain_data_size;

        void*    v_global_domain_shmem_ptr;
        int      v_global_domain_shmem_size;
        int      v_global_decat_lib_version = 0;


        ulong v_global_sync_ctime[2];
        ulong v_global_sync_stime[2];
        ulong v_global_sync_refcnt[2];


        int  v_global_ecat_task_run = 0;
        int  v_global_ecat_task_exit = 1;

        //task value
        RT_TASK  v_global_rt_task_desc;
        RT_EVENT v_global_usr_event_desc;
        pthread_t v_global_GetState_thread;

        // share semaphore
        RT_SEM v_global_sem_desc;
        RT_SEM_INFO v_global_sem_info;


        // task event process value
        int v_global_usr_event_flag = 0;


        // EtherCAT communication status
        dec_master_state_t   v_global_master_state;
        dec_domain_state_t   v_global_domain_state;
        dec_slave_state_t    v_global_slave_state[MAX_SLAVES] = {};
        int   v_global_cur_ecat_state  = 0;     		// Operation Mode( Init, Pre-Op, Safe-Op, OP)
        int   v_global_cur_domain_state   = 0;
        int   v_global_req_ecat_state = 0;    				// is Servo Profile setup complete?
        int   v_global_cur_all_state = 0;

//        vector<int> index;
//        vector<int> cur_status_word;
//        vector<long> cur_position, tar_position, cur_velocity;
//        vector<int> cur



    // class init
    public:
        ecatmaster();

    // get value
    public:
        inline daininterface *f_get_dec_interface()
        {
            return v_global_dec_interface;
        }

        inline ulong *f_get_sync_ctime()
        {
            return v_global_sync_ctime;
        }
        // thread cancle
        inline pthread_t f_get_GetState_thread()
        {
            return v_global_GetState_thread;
        }

        // slave total count
        inline int f_get_slave_count()
        {
            return v_global_slavecount;
        }

        inline ecatslave *f_get_slave()
        {
            return v_global_ecat_slave;
        }


    // get ptr
    public:

        // shared memory address
        inline void **f_get_domain_shmem_ptr()
        {
            return &v_global_domain_shmem_ptr;
        }


        // master state check
        inline dec_master_state_t *f_get_master_state_ptr()
        {
            return &v_global_master_state;
        }

        //domain state check
        inline dec_domain_state_t *f_get_domain_state_ptr()
        {
            return &v_global_domain_state;
        }

        //EtherCAT slave state start address
        inline dec_slave_state_t* f_get_slave_state_ptr()
        {
            return v_global_slave_state;
        }

        // current EtherCAT master state
        inline int* f_get_cur_ecat_state_ptr()
        {
            return &v_global_cur_ecat_state;
        }

        // current EtherCAT domain state
        inline int* f_get_cur_domain_state_ptr()
        {
            return &v_global_cur_domain_state;
        }

        // request ethercat state
        inline int *f_get_req_ecat_state_ptr()
        {
            return &v_global_req_ecat_state;
        }


        //task information
        inline int* f_get_ecat_task_run_ptr()
        {
            return &v_global_ecat_task_run;
        }

        inline int* f_get_ecat_task_exit_ptr()
        {
            return &v_global_ecat_task_exit;
        }

        //domain connect
        inline uint8_t **f_get_domain1_pd_ptr()
        {
            return &v_global_domain1_pd;
        }

        inline int *f_get_domain_data_size_ptr()
        {
            return &v_global_domain_data_size;
        }
        // frame connect
        inline unsigned int **f_get_ptr_output_offset_haed_ptr()
        {
            return &v_global_ptr_output_offset_haed;
        }

        inline unsigned int **f_get_ptr_input_offset_haed_ptr()
        {
            return &v_global_ptr_input_offset_haed;
        }

        // task status process
        inline RT_EVENT *f_get_usr_event_desc_ptr()
        {
            return &v_global_usr_event_desc;
        }
        inline RT_TASK *f_get_rt_task_desc_ptr()
        {
            return &v_global_rt_task_desc;
        }


    // init Function
    private:

        // step 1
        int f_init_slave_initialize(int v_local_slave_count, int v_local_domain_count);

        // step 2
        int f_init_slave_information_insert(uint16_t v_local_slave_number,
                                            uint32_t v_local_vendorID,
                                            uint32_t v_local_productCODE,
                                            uint16_t v_local_alias,
                                            uint16_t v_local_position);

        //step 3

        int f_init_PDO_List_send(int v_local_slave_number,
                                 ec_sync_info_t * v_local_ptr_slave_syncs,
                                 bool v_local_servo_flag);

        //step 4
        int f_init_make_domain(int v_local_slave_numbers,
                                           ec_pdo_entry_reg_t *v_local_ptr_slave_domain_out_regs[],
                                           ec_pdo_entry_reg_t *v_local_ptr_slave_domain_in_regs[],
                                           ec_sync_info_t *v_local_ptr_slave_syncs[]);
        //step 5
        int f_init_DC_setup(int v_local_slave_number,
                            int v_local_dc_flag,
                            bool v_local_area_servo_drive);

        //step 6
        int f_init_slave_active();

        //step 7
        int f_init_open_domain_data(uint16_t v_local_size);

        //step 8
        int f_init_create_rt_task();
        int f_init_create_process_task();




    // task function
    public :
        void time_diff_print(int id, int print_mode);
        int f_task_ecat_master_event_wait();
        int f_task_ecat_master_end_event_send();
        int f_task_ecat_master_ready_event_send();


    // EtherCAT Operating function
    public:

        // master connect slave device add
        int f_oper_slave_setting_initialize(int v_local_slave_num,
                                            char *v_local_slave_models,
                                            char *v_local_vendor_name,
                                            uint32_t v_local_vendorID,
                                            uint32_t v_local_productCODE,
                                            bool v_local_is_servo_driver);

        // master ready
        int f_oper_master_initialize(int v_local_slave_count,
                                     int v_local_domain_count,
                                     ulong v_local_sync_ctime,
                                     ulong v_local_sync_stime,
                                     ulong v_local_refcnt,
                                     int v_local_dc_flag,
                                     bool v_local_servo_flag);

        // master ready
        int f_oper_master_deinitialize();

        //sub oper function
        int f_oper_sdo_write(int v_local_slave_number,
                                         uint16_t v_local_index,
                                         uint16_t v_local_subindex,
                                         uint32_t v_local_data,
                                         COMMUNICAT_SIZE_TYPE v_local_datatype);

        // servo setting
        void f_oper_get_servo_state(int v_local_slv_inx);
        void f_oper_set_servo_state(int v_local_slv_inx);

        //ecat master communication state
        QString f_oper_get_ecat_state();

        //ecat servo on state check
        int f_oper_get_servo_valid_state();


        vector<slave_data_t> slave_data;
        FILE *fp;
};

#endif // ECATMASTER_H
