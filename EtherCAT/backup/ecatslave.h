#ifndef ETHERCATSLAVE_H
#define ETHERCATSLAVE_H
#include "globalstruct.h"


class ecatslave
{
    private:

        void *v_global_ecat_master_ptr;

        bool    v_global_is_servo_drive;

        /////////////////////////////////////////////////////////////////////////////////////

        //ethercat slave information data
        char v_global_vendor_name[MAX_NAME_LEN];
        char v_global_slave_models[MAX_NAME_LEN];
        uint32_t v_global_vendor_id;
        uint32_t v_global_product_id;

        ec_pdo_entry_reg_t slave_domain_out_regs[MAX_ENTRIES];
        ec_pdo_entry_reg_t slave_domain_in_regs[MAX_ENTRIES];

        ec_sync_info_t v_global_slave_syncs[MAX_SYNC];
        ec_pdo_info_t v_global_pdos[MAX_PDOS];
        ec_pdo_entry_info_t v_global_entries[MAX_ENTRIES];
        unsigned int v_global_offset_buffer[MAX_ENTRIES];

    public :
        //PDO entries CiA402 profile
        unsigned long   v_global_cur_step               = 0;
        unsigned long   v_global_prev_step              = 0;
        unsigned long   v_global_send_counter           = 0;
        unsigned long   v_global_receive_counter        = 0;
        long            v_global_set_target_val         = 0;
        int             v_global_cur_status_word        = 0;
        int             v_global_cur_oper_mode          = 0;
        uint8_t         v_global_cur_modes_of_operation = 0;
        uint8_t         v_global_set_modes_of_operation = 0;
        long            v_global_cur_position           = 0;
        long            v_global_tar_position           = 0;
        long            v_global_tar_velocity           = 0;
        int             v_global_tar_torque             = 0;
        int             v_global_cur_error_val          = 0;
        long            v_global_cur_velocity           = 0;
        int             v_global_cur_torque_value1      = 0;
        int             v_global_cur_torque_value2      = 0;
        int             v_global_cur_io_value           = 0;
        uint8_t         v_global_cur_digital_input      = 0;
        uint8_t         v_global_cur_digital_output     = 0;

        // servo state
        int   v_global_cur_servo_state = REQ_SERVO_OFF;  // All ServoOn is Servo On, Off,
        int   v_global_req_servo_state = REQ_SERVO_OFF;  // reqtue servo state


        // servo config
        ulong v_global_run_speed;
        ulong v_global_run_type;  // Forward, Reverse, Task_Run


    public:

        ecatslave();


        // set get
    public :
        inline bool f_get_is_servo_drive()
        {
            return v_global_is_servo_drive;
        }

        inline ec_sync_info_t* f_get_v_global_slave_syncs_ptr()
        {
            return (ec_sync_info_t *)v_global_slave_syncs;
        }

        inline ec_pdo_info_t* f_get_v_global_pdos_ptr()
        {
            return (ec_pdo_info_t *)v_global_pdos;
        }

        inline ec_pdo_entry_info_t* f_get_v_global_entries_ptr()
        {
            return (ec_pdo_entry_info_t *)v_global_entries;
        }


        inline ec_pdo_entry_reg_t *f_get_slave_domain_out_regs_ptr()
        {
            return (ec_pdo_entry_reg_t *)slave_domain_out_regs;
        }

        inline ec_pdo_entry_reg_t *f_get_slave_domain_in_regs_ptr()
        {
            return (ec_pdo_entry_reg_t *)slave_domain_in_regs;
        }

    public:

        // slave setting
        void f_init_initialize(char *v_local_slave_models,
                                char *v_local_vendor_name,
                                uint32_t v_local_vendorID,
                                uint32_t v_local_productCODE,
                                int v_local_slave_position,
                                bool v_local_is_servo_drive);

        void f_init_master_insert(void * v_local_ecat_master);



        //oper
        unsigned int f_oper_get_offset_value(int v_local_index);
        virtual void f_oper_send_process();
        virtual void f_oper_recv_process();

};




#endif // ETHERCATSLAVE_H
