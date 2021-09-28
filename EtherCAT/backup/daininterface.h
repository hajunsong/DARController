#ifndef DAININTERFACE_H
#define DAININTERFACE_H
#include <QObject>
#include "globalstruct.h"

/* name config
    - first name
        * v : velue
            @ local
            @ area
            @ global

        * f : function
        * s : struct
        * C : const velue

    - second name
        * function behavior
            ex) : set, get, oper ...

    - third name
        * function name
            ex) : initialize , ....
*/




// Singleton Class dev_library
class daininterface
{
    private:
        int v_global_dec_fd;


    public:
        daininterface()
        {
            v_global_dec_fd = 0;
        }
        ~daininterface();

        inline int  g_get_dec_fd(){ return v_global_dec_fd; }
        inline void g_set_dec_fd(int velue) { v_global_dec_fd = velue; }

    public:

        /** dec driver open
         */
        // dec_open
        int f_oper_open(void);


        /** dec driver close
         */
        // dec_close
        int f_oper_close(void);

        /**  dec driver is opened
         */
        // dec_is_open
        int f_oper_is_open(void);

        /** Slave Setup
         */
        // dec_slave_setup_init
        int f_oper_slave_setup_init(uint16_t v_local_slave_count,
                                    uint16_t v_local_domain_count);

        /**  slave config setup
         */
        // dec_slave_config
        int f_oper_slave_config(uint16_t v_local_slave_number,
                                uint32_t v_local_vendor_id,
                                uint32_t v_local_product_code,
                                uint16_t v_local_alias,
                                uint16_t v_local_position);

        /**  slave dc setup
         */
        // dec_slave_config_pdo_list
        int f_oper_slave_config_pdo_list(
                                uint16_t v_local_slave_number,
                                uint16_t v_local_syncs_size,
                                ec_sync_info_t *v_local_sync);

        /**  slave domain entry setup
         *   send(recv) pdo through domain data
         */
        // dec_clave_config_dcccc
        int f_oper_slave_config_dc(
                                uint16_t v_local_slave_number,
                                uint16_t v_local_dc_enable,
                                uint16_t v_local_dc_assign_activate,
                                uint32_t v_local_sync0_cycle_time,
                                uint32_t v_local_sync0_shift_time,
                                uint32_t v_local_sync1_cycle_time,
                                uint32_t v_local_sync1_shift_time,
                                uint16_t v_local_ref_count);

        /**  open domain data buff
         */
        // dec_domain_reg_entry
        int f_oper_domain_reg_entry(
                                uint16_t v_local_domain_number,
                                uint16_t v_local_entry_size,
                                ec_pdo_entry_reg_t  *v_local_domain_entry);

        /**  close domain data buff
         */
        // dec_open_domain_data
        uint8_t *f_oper_dec_open_domain_data(uint16_t v_local_size,
                                                void **v_local_ptr_domainshmem,
                                                int *v_local_ptr_domainshsize,
                                                int *v_local_ptr_decatlibversion);

        /** start DC thread and  kernel allocate domain data
         */
        // dec_close_domain_data
        int f_oper_close_domain_date(
                                void **v_local_domain_shmem_ptr,
                                uint8_t * v_local_domain_data,
                                uint16_t v_local_size);

        /** stop DC thread and kernel deallocate domain data
         */
        // dec_slave_active
        int f_oper_slave_active(void);

        /**  for periodically copy kernel domain data to user area.
         *   offset is location of domain-data that get through dec_open_domain_data()
         */
        // dec_slave_deactive
        int f_oper_slave_deactive(void);

        /**  for periodically copy kernel domain data to user area.
         *   offset is location of domain-data that get through dec_open_domain_data()
         */
        //dec_set_recv_sync
        int f_oper_set_recv_sync(
                                uint32_t v_local_offset,
                                uint32_t v_local_size);

        /**  for periodically copy user domain data to kernel area.
         *   offset is location of domain-data that get through dec_open_domain_data()
         */
        //dec_set_send_sync
        int f_oper_set_send_sync(
                                uint32_t v_local_offset,
                                uint32_t v_local_size);

        /** get domain state
         *  if DC Syncronize is error,  wc state is changed..
         */
        //dec_get_domain_state
        int f_oper_get_domain_state(
                                dec_domain_state_t *v_local_state);

        /** get master state
         *  init, pre-op, safe-op , op
         */
        //dec_get_master_state
        int f_oper_get_master_state(
                                dec_master_state_t *v_local_state);

        /** get slave state
         *  init, pre-op, safe-op , op
         */
        //dec_get_slave_state
        int f_oper_get_slave_state(
                                dec_slave_state_t *v_local_state);


        /** get slave information
         *  slave_count, driver_ver, master_ver, info_str
         */
        //dec_get_slave_info
        int f_oper_get_slave_info(
                                dec_info_t *v_local_info);

        /**  SDO Write(mailbox)
         */
        //dec_sdo_write
        int f_oper_sdo_write( uint16_t v_local_slave_number,
                              uint16_t v_local_index,
                              uint16_t v_local_subindex,
                              uint16_t v_local_len,
                              uint8_t *v_local_data);

        /**  SDO Write 8bit
         */
        //dec_sdo_write_8
        int f_oper_sdo_write_8( uint16_t v_local_slave_number,
                                uint16_t v_local_index,
                                uint16_t v_local_subindex,
                                const uint8_t v_local_data);

        /**  SDO Write 16bit
         */
        //dec_sdo_write_16
        int f_oper_sdo_write_16( uint16_t v_local_slave_number,
                                 uint16_t v_local_index,
                                 uint16_t v_local_subindex,
                                 const uint16_t v_local_data);

        /**  SDO Write 32bit
         */
        //dec_sdo_write_32
        int f_oper_sdo_write_32( uint16_t v_local_slave_number,
                                 uint16_t v_local_index,
                                 uint16_t v_local_subindex,
                                 const uint32_t v_local_data);


        /**  SDO read(mailbox)
         */
        //dec_sdo_read
        int f_oper_sdo_read( uint16_t v_local_slave_number,
                                 uint16_t v_local_index,
                                 uint16_t v_local_subindex,
                                 uint16_t v_local_len,
                                 uint8_t *v_local_data);

        /**  SDO read 8bit
         */
        //dec_sdo_read_8
        int f_oper_sdo_read_8( uint16_t v_local_slave_number,
                                 uint16_t v_local_index,
                                 uint16_t v_local_subindex,
                                 const uint8_t *v_local_data);

        /**  SDO read 16bit
         */
        //dec_sdo_read_16
        int f_oper_sdo_read_16( uint16_t v_local_slave_number,
                                 uint16_t v_local_index,
                                 uint16_t v_local_subindex,
                                 const uint8_t *v_local_data);


        /**  SDO read 32bit
         */
        //dec_sdo_read_32
        int f_oper_sdo_read_32( uint16_t v_local_slave_number,
                                 uint16_t v_local_index,
                                 uint16_t v_local_subindex,
                                 const uint8_t *v_local_data);

        /**  SII Write
         */
        //dec_slave_sii_write
        int f_oper_slave_sii_write(
                                dec_slave_sii_t *v_local_slave_sii);

        /**  SII Read
         */
        //dec_slave_sii_read
        int  f_oper_slave_sii_read(
                                dec_slave_sii_t *v_local_slave_sii);


        /**  Set the Debug Print Level
         */
        //dec_set_print_level
        int  f_oper_set_print_level (unsigned int v_local_level);

        /** set slave state
         *  init, pre-op, safe-op , op
         */
        //dec_set_slave_state
        int  f_oper_set_slave_state(dec_ioctl_slave_state_t *v_local_data);


        /** Read a slave's registers.
         *
         * \return Zero on success, otherwise a negative error code.
         */
        //dec_slave_reg_read
        int f_oper_slave_reg_read(dec_ioctl_slave_reg_t *v_local_slave_reg, int *v_local_ptr_decatlibversion);


        /** Write a slave's registers.
         *
         * \return Zero on success, otherwise a negative error code.
         */
        //dec_slave_reg_write
        int f_oper_slave_reg_write(dec_ioctl_slave_reg_t *v_local_slave_reg, int *v_local_ptr_decatlibversion);


        /** IOCTL_DEC_GET_ONLINE_SLAVE_NUM
         */
        //dec_get_online_slave_num
        int f_oper_get_online_slave_num(uint16_t* v_local_value);


        /** IOCTL_DEC_GET_CURRENT_WORKING_CNT
         */
        //dec_get_current_working_cnt
        int f_oper_get_current_working_cnt(uint16_t* v_local_value);


        /** IOCTL_DEC_GET_RT_TASK_STATE
         */
        //dec_get_rt_task_state
        int f_oper_get_rt_task_state(uint16_t* v_local_value);

        /** IOCTL_DEC_GET_RT_TASK_CNT
         */
        //dec_get_rt_task_cnt
        int f_oper_get_rt_task_cnt(uint32_t* v_local_value);


        /** IOCTL_DEC_GET_LOST_PACKETCNT
         */
        //dec_get_lost_packetcnt
        int f_oper_get_lost_packetcnt(uint32_t* v_local_value);


        /** IOCTL_DEC_GET_PACKET_CRC_ERRCNT
         */
        //dec_get_packet_crc_errcnt
        int f_oper_get_packet_crc_errcnt(uint32_t* v_local_value);


        /** IOCTL_DEC_GET_WORKINGCNT_CHANGECNT
         */
        //dec_get_packet_crc_errcnt
        int f_oper_get_workingcnt_changecnt(uint16_t* v_local_value);

        /** IOCTL_DEC_GET_ERROR_STATE
         */
        //dec_get_error_state
        int f_oper_get_error_state(uint32_t* v_local_value);

        /** IOCTL_DEC_GET_ONLINE_SLAVENUM_ERRCNT
         */
        //dec_get_online_slavenum_errcnt
        int f_oper_get_online_slavenum_errcnt(uint16_t* v_local_value);

        /** IOCTL_DEC_GET_RDYEVENT_ERRCNT
         */
        //dec_get_rdyevent_errcnt
        int f_oper_get_rdyevent_errcnt(uint16_t* v_local_value);


        /** IOCTL_DEC_GET_APPEVENT_ERRCNT
         */
        //dec_get_appevent_errcnt
        int f_oper_get_appevent_errcnt(uint16_t* v_local_value);


        /** IOCTL_DEC_GET_ENDEVENT_ERRCNT
         */
        //dec_get_endevent_errcnt
        int f_oper_get_endevent_errcnt(uint16_t* v_local_value);


        /** IOCTL_DEC_GET_WAITPERIOD_ERRCNT
         */
        //dec_get_waitperiod_errcnt
        int f_oper_get_waitperiod_errcnt(uint16_t* v_local_value);


        /** IOCTL_DEC_GET_DEEP_SLEEP_ERRCNT
         */
        //dec_get_deep_sleep_errcnt
        int f_oper_get_deep_sleep_errcnt(uint16_t* v_local_value);


        /** IOCTL_DEC_GET_WAKEUP_LARGE_ERRCNT
         */
        //dec_get_wakeup_large_errcnt
        int f_oper_get_wakeup_large_errcnt(uint16_t* v_local_value);


        /** IOCTL_DEC_GET_MASTER_VERSION
         */
        //dec_get_master_version
        int  f_oper_get_master_version(dec_ioctl_master_version_t *v_local_version);

        /**  get master stats address
         */
        //dec_master_stats_data
        dec_shared_stats_t* f_oper_dec_master_stats_data(
                                        void *v_local_ptr_domainshmem,
                                        int v_local_ptr_decatlibversion);

        /**  read master stats data
         */
        //read_master_stats_data
        void f_oper_read_master_stats_data(dec_shared_stats_t* v_local_ptr_master_stats_data,
                                           void **v_local_ptr_domainshmem,
                                           int *v_local_ptr_decatlibversion);

};

#endif // DAININTERFACE_H
