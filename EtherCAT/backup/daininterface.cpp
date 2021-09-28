#include "daininterface.h"

/** dec driver open
 */
// dec_open
int daininterface::f_oper_open(void)
{
    int v_area_ret;
    if (!v_global_dec_fd)
    {
        v_area_ret = open(DEC_DEV_STRING, O_RDWR);
        if (v_area_ret == -1)
            d_print( "Failed to open %s: %s\n", DEC_DEV_STRING, strerror(errno));
        v_global_dec_fd = v_area_ret;
    }
    return v_global_dec_fd;
}

/** dec driver close
 */
// dec_close
int daininterface::f_oper_close(void)
{
    if (v_global_dec_fd)
    {
        close(v_global_dec_fd);
        v_global_dec_fd = 0;
    }
    d_print( "dec_close \n" );

    return 0;
}

/**  dec driver is opened
 */
// dec_is_open
int daininterface::f_oper_is_open(void)
{
    return v_global_dec_fd;
}

/** Slave Setup
 */
// dec_slave_setup_init
int daininterface::f_oper_slave_setup_init(uint16_t v_local_slave_count,
                            uint16_t v_local_domain_count)
{
    int v_area_ret = 0;
    dec_setup_init_t  v_area_dec_init;

    if (f_oper_open() < 0 ) return -1;

    v_area_dec_init.slave_count  = v_local_slave_count;
    v_area_dec_init.domain_count = v_local_domain_count;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_SETUP_INIT, &v_area_dec_init) == -1)
    {
        d_print( "Failed to dec_slave_setup_init: %s\n", strerror(errno));
        v_area_ret =  -1;
    }

    return 	v_area_ret;
}

/**  slave config setup
 */
// dec_slave_config
int daininterface::f_oper_slave_config(uint16_t v_local_slave_number,
                        uint32_t v_local_vendor_id,
                        uint32_t v_local_product_code,
                        uint16_t v_local_alias,
                        uint16_t v_local_position)
{
    int v_area_ret = 0;
    dec_slave_config_t  v_area_slave;

    if (f_oper_open() < 0 ) return -1;

    v_area_slave.slave_number       = v_local_slave_number;
    v_area_slave.vendor_id          = v_local_vendor_id;
    v_area_slave.product_code       = v_local_product_code;
    v_area_slave.alias              = v_local_alias;
    v_area_slave.position           = v_local_position;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_SLAVE_CONFIG, &v_area_slave) == -1)
    {
        d_print( "Failed to dec_slave_config: %s\n", strerror(errno));
        v_area_ret =  -1;
    }
    return 	v_area_ret;
}

/**  slave dc setup
 */
// dec_slave_config_pdo_list
int daininterface::f_oper_slave_config_pdo_list(
                        uint16_t v_local_slave_number,
                        uint16_t v_local_syncs_size,
                        ec_sync_info_t *v_local_sync)
{
    int v_area_ret = 0;
    dec_slave_pdo_list_t  v_area_pdo_list;

    if (f_oper_open() < 0 ) return -1;
    v_area_pdo_list.slave_number    = v_local_slave_number;
    v_area_pdo_list.syncs_size      = v_local_syncs_size;
    v_area_pdo_list.syncs           = v_local_sync;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_SLAVE_PDO_LIST, &v_area_pdo_list) == -1)
    {
        d_print( "Failed to dec_slave_config_pdo_list: %s\n", strerror(errno));
        v_area_ret =  -1;
    }

    return 	v_area_ret;
}

/**  slave domain entry setup
 *   send(recv) pdo through domain data
 */
// dec_clave_config_dcccc
int daininterface::f_oper_slave_config_dc(
                        uint16_t v_local_slave_number,
                        uint16_t v_local_dc_enable,
                        uint16_t v_local_dc_assign_activate,
                        uint32_t v_local_sync0_cycle_time,
                        uint32_t v_local_sync0_shift_time,
                        uint32_t v_local_sync1_cycle_time,
                        uint32_t v_local_sync1_shift_time,
                        uint16_t v_local_ref_count)
{
    int v_area_ret = 0;
    dec_slave_config_dc_t  v_area_dc_info;

    if (f_oper_open() < 0 ) return -1;

    v_area_dc_info.slave_number             = v_local_slave_number;
    v_area_dc_info.dc_enable                = v_local_dc_enable;
    v_area_dc_info.dc_assign_activate       = v_local_dc_assign_activate;
    v_area_dc_info.dc_sync[0].cycle_time    = v_local_sync0_cycle_time;
    v_area_dc_info.dc_sync[0].shift_time    = v_local_sync0_shift_time;
    v_area_dc_info.dc_sync[1].cycle_time    = v_local_sync1_cycle_time;
    v_area_dc_info.dc_sync[1].shift_time    = v_local_sync1_shift_time;
    v_area_dc_info.ref_count                = v_local_ref_count;


    if (ioctl(v_global_dec_fd, IOCTL_DEC_SLAVE_DC, &v_area_dc_info) == -1)
    {
        d_print( "Failed to dec_slave_config_dc: %s\n", strerror(errno));
        v_area_ret =  -1;
    }
    return 	v_area_ret;
}

/**  open domain data buff
 */
// dec_domain_reg_entry
int daininterface::f_oper_domain_reg_entry(
                        uint16_t v_local_domain_number,
                        uint16_t v_local_entry_size,
                        ec_pdo_entry_reg_t  *v_local_domain_entry)
{
    int v_area_ret = 0;
    dec_domain_list_t v_area_domain_list;
    if (f_oper_open() < 0 ) return -1;

    v_area_domain_list.domain_number = v_local_domain_number;
    v_area_domain_list.entry_size    = v_local_entry_size;
    v_area_domain_list.domain_entry  = v_local_domain_entry;
    #if ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP4
    {
        int v_sub_entry_count = 0;
        d_print("[DAININTERFACE] : [oper] : [f_oper_domain_reg_entry]\r\n");
        d_print("[STEP 4] : v_area_domain_list.domain_number : %d\r\n",v_area_domain_list.domain_number);
        d_print("[STEP 4] : v_area_domain_list.entry_size : %d\r\n",v_area_domain_list.entry_size);
        for(v_sub_entry_count = 0; v_area_domain_list.domain_entry[v_sub_entry_count].index != 0x00; ++v_sub_entry_count)
        {
            d_print("[STEP 4] : v_area_domain_list.domain_entry[%d].index : 0x%X \r\n",v_sub_entry_count,
                    v_area_domain_list.domain_entry[v_sub_entry_count].index);
        }
    }
    #endif

    if (ioctl(v_global_dec_fd, IOCTL_DEC_DOMAIN_LIST, &v_area_domain_list) == -1)
    {
        d_print( "Failed to dec_domain_reg_entry: %s\n", strerror(errno));
        v_area_ret =  -1;
    }

    return 	v_area_ret;
}

/**  close domain data buff
 */
// dec_open_domain_data
uint8_t *daininterface::f_oper_dec_open_domain_data(
                        uint16_t v_local_size,
                        void **v_local_ptr_domainshmem,
                        int *v_local_ptr_domainshsize,
                        int *v_local_ptr_decatlibversion)
{

     if (f_oper_open() < 0 ) return 0;

     // create shared memory
     uint8_t *v_local_ptr_domain_data = NULL;

     if(*v_local_ptr_domainshmem)
     {
         return (uint8_t*)*v_local_ptr_domainshmem;
     }

     if (v_local_size)
     {
         /* try to alloc max size */
         *v_local_ptr_domainshsize = C_GLOBAL_MEMORY_MAP_SIZE;
         v_local_ptr_domain_data = (uint8_t*)mmap(0,
                                         C_GLOBAL_MEMORY_MAP_SIZE,
                                         PROT_READ | PROT_WRITE,
                                         MAP_SHARED,
                                         v_global_dec_fd,
                                         0);

         if (v_local_ptr_domain_data == MAP_FAILED)
         {
             d_print( "Failed to dec_open_domain_data: %s\n", strerror(errno));

             /* retry to alloc given size for  old version */
             *v_local_ptr_domainshsize = v_local_size;
             v_local_ptr_domain_data = (uint8_t*)mmap(0,
                                             v_local_size,
                                             PROT_READ | PROT_WRITE,
                                             MAP_SHARED,
                                             v_global_dec_fd,
                                             0);

             if (v_local_ptr_domain_data == MAP_FAILED)
             {
                 *v_local_ptr_domainshsize = 0;
                 return (uint8_t*)-1;
             }
         }
         else
         {
             *v_local_ptr_decatlibversion = 1;
         }
     }

     *v_local_ptr_domainshmem = v_local_ptr_domain_data;

     return v_local_ptr_domain_data;

}

/** start DC thread and  kernel allocate domain data
 */
// dec_close_domain_data
int daininterface::f_oper_close_domain_date(
                        void **v_local_domain_shmem_ptr,
                        uint8_t * v_local_domain_data,
                        uint16_t v_local_size)
{
#if 1
    int v_local_ret = 0;
    if (f_oper_open() < 0 ) return -1;

    if (v_local_domain_data)
    {
        v_local_ret = munmap(v_local_domain_data, v_local_size);
    }

    REF(v_local_domain_shmem_ptr) = NULL;

    d_print( "dec_close_domain_data (%d)\n", v_local_ret);

    return 0;
#endif
}

/** stop DC thread and kernel deallocate domain data
 */
// dec_slave_active
int daininterface::f_oper_slave_active(void)
{
    int v_local_process_data_size = 0;

    if (f_oper_open() < 0 ) return -1;

    // get process data size
    if (ioctl(v_global_dec_fd, IOCTL_DEC_ACTIVE, &v_local_process_data_size) == -1)
    {
        d_print( "Failed to dec_slave_active: %s\n", strerror(errno));
        return -1;
    }

    return v_local_process_data_size;
}

/**  for periodically copy kernel domain data to user area.
 *   offset is location of domain-data that get through dec_open_domain_data()
 */
// dec_slave_deactive
int daininterface::f_oper_slave_deactive(void)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    d_print( "dec_slave_deactive.. \n");

    if (ioctl(v_global_dec_fd, IOCTL_DEC_DEACTIVE, NULL) == -1)
    {
        d_print( "Failed to dec_slave_deactive: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}

/**  for periodically copy kernel domain data to user area.
 *   offset is location of domain-data that get through dec_open_domain_data()
 */
//dec_set_recv_sync
int daininterface::f_oper_set_recv_sync(
                        uint32_t v_local_offset,
                        uint32_t v_local_size)
{
    int v_local_ret = 0;
    dec_data_sync_t v_local_data_sync;
    v_local_data_sync.offset = v_local_offset;
    v_local_data_sync.size = v_local_size;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_RECV_SYNC, &v_local_data_sync) == -1)
    {
        d_print( "Failed to dec_set_recv_sync: %s\n", strerror(errno));
        v_local_ret =  -1;
    }
    return 	v_local_ret;
}

/**  for periodically copy user domain data to kernel area.
 *   offset is location of domain-data that get through dec_open_domain_data()
 */
//dec_set_send_sync
int daininterface::f_oper_set_send_sync(
                        uint32_t v_local_offset,
                        uint32_t v_local_size)
{
    int v_local_ret = 0;
    dec_data_sync_t v_local_data_sync;

    v_local_data_sync.offset = v_local_offset;
    v_local_data_sync.size = v_local_size;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_SEND_SYNC, &v_local_data_sync) == -1)
    {
        d_print( "Failed to dec_set_send_sync: %s\n", strerror(errno));
        v_local_ret =  -1;
    }
    return 	v_local_ret;
}

/** get domain state
 *  if DC Syncronize is error,  wc state is changed..
 */
//dec_get_domain_state
int daininterface::f_oper_get_domain_state(
                        dec_domain_state_t *v_local_state)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_DOMAIN_STATE, v_local_state) == -1)
    {
        d_print( "Failed to dec_get_domain_state: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}

/** get master state
 *  init, pre-op, safe-op , op
 */
//dec_get_master_state
int daininterface::f_oper_get_master_state(
                        dec_master_state_t *v_local_state)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_MASTER_STATE, v_local_state) == -1)
    {
        d_print( "Failed to dec_get_master_state: %s\n", strerror(errno));
        v_local_ret =  -1;
    }
    return 	v_local_ret;
}

/** get slave state
 *  init, pre-op, safe-op , op
 */
//dec_get_slave_state
int daininterface::f_oper_get_slave_state(
                        dec_slave_state_t *v_local_state)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_SLAVE_STATE, v_local_state) == -1)
    {
        d_print( "Failed to dec_get_slave_state: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/** get slave information
 *  slave_count, driver_ver, master_ver, info_str
 */
//dec_get_slave_info
int daininterface::f_oper_get_slave_info(
                        dec_info_t *v_local_info)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    v_local_ret = ioctl(v_global_dec_fd, IOCTL_DEC_GET_INFO, v_local_info);

    if (v_local_ret == -1)
    {
        d_print( "Failed to dec_get_slave_info: %s\n", strerror(errno));
    }

    return 	v_local_ret;
}


/**  SDO Write(mailbox)
 */
//dec_sdo_write
int daininterface::f_oper_sdo_write( uint16_t v_local_slave_number,
                      uint16_t v_local_index,
                      uint16_t v_local_subindex,
                      uint16_t v_local_len,
                      uint8_t *v_local_data)
{
    int v_local_ret = 0;
    dec_sdo_data_t  v_local_sdo_write;

    if (f_oper_open() < 0 ) return -1;

    v_local_sdo_write.slave_number = v_local_slave_number;
    v_local_sdo_write.index = v_local_index;
    v_local_sdo_write.subindex = v_local_subindex;
    v_local_sdo_write.io_flag = C_GLOBAL_IO_WRITE;
    memcpy(v_local_sdo_write.data, v_local_data, v_local_len);
    v_local_sdo_write.len = v_local_len;

    v_local_ret = ioctl(v_global_dec_fd, IOCTL_DEC_SDO_SET_DATA, &v_local_sdo_write);

    if(v_local_ret)
    {
        d_print( "Failed to dec_sdo_write: %s\n", strerror(errno));
    }

    return 	v_local_ret;
}

/**  SDO Write 8bit
 */
//dec_sdo_write_8
int daininterface::f_oper_sdo_write_8( uint16_t v_local_slave_number,
                        uint16_t v_local_index,
                        uint16_t v_local_subindex,
                        const uint8_t v_local_data)
{
    uint8_t v_local_wr_data = v_local_data;
    int v_local_ret;

    v_local_ret = f_oper_sdo_write(v_local_slave_number, v_local_index, v_local_subindex, 1, &v_local_wr_data);
    return v_local_ret;
}

/**  SDO Write 16bit
 */
//dec_sdo_write_16
int daininterface::f_oper_sdo_write_16( uint16_t v_local_slave_number,
                         uint16_t v_local_index,
                         uint16_t v_local_subindex,
                         const uint16_t v_local_data)
{
#if 0
    uint16_t v_local_wr_data = v_local_data;
    int v_local_ret;

    v_local_ret = f_oper_sdo_write(v_local_slave_number, v_local_index, v_local_subindex, 2, &v_local_wr_data);
    return v_local_ret;
#endif
}

/**  SDO Write 32bit
 */
//dec_sdo_write_32
int daininterface::f_oper_sdo_write_32( uint16_t v_local_slave_number,
                         uint16_t v_local_index,
                         uint16_t v_local_subindex,
                         const uint32_t v_local_data)
{
    uint32_t v_local_wr_data = v_local_data;
    int v_local_ret;

    v_local_ret = f_oper_sdo_write(v_local_slave_number, v_local_index, v_local_subindex, 4, (uint8_t *)&v_local_wr_data);
    return v_local_ret;
}


/**  SDO read(mailbox)
 */
//dec_sdo_read
int daininterface::f_oper_sdo_read( uint16_t v_local_slave_number,
                         uint16_t v_local_index,
                         uint16_t v_local_subindex,
                         uint16_t v_local_len,
                         uint8_t *v_local_data)
{
#if 0
    int v_local_ret = 0;
    dec_sdo_data_t  v_local_sdo_read;
    if (f_oper_open() < 0 ) return -1;

    v_local_sdo_read.slave_number = v_local_slave_number;
    v_local_sdo_read.index    = v_local_index;
    v_local_sdo_read.subindex = v_local_subindex;
    v_local_sdo_read.io_flag  = IO_READ;
    v_local_sdo_read.len 	  = v_local_len;
    v_local_ret = ioctl(v_global_dec_fd, IOCTL_DEC_SDO_GET_DATA, &v_local_sdo_read);

    if(v_local_ret)
    {
        d_print( "Failed to dec_sdo_read: %s\n", strerror(errno));
    }
    else
    {
        memcpy(v_local_data, v_local_sdo_read.data, v_local_len);
    }

    return 	v_local_ret;
#endif
}

/**  SDO read 8bit
 */
//dec_sdo_read_8
int daininterface::f_oper_sdo_read_8( uint16_t v_local_slave_number,
                         uint16_t v_local_index,
                         uint16_t v_local_subindex,
                         const uint8_t *v_local_data)
{
#if 0
    int v_local_ret;

    v_local_ret = f_oper_sdo_read(v_local_slave_number, v_local_index, v_local_subindex, 1, v_local_data);
    return v_local_ret;
#endif
}

/**  SDO read 16bit
 */
//dec_sdo_read_16
int daininterface::f_oper_sdo_read_16( uint16_t v_local_slave_number,
                         uint16_t v_local_index,
                         uint16_t v_local_subindex,
                         const uint8_t *v_local_data)
{
#if 0
    int v_local_ret;

    v_local_ret = f_oper_sdo_read(v_local_slave_number, v_local_index, v_local_subindex, 2, v_local_data);
    return v_local_ret;
#endif
}


/**  SDO read 32bit
 */
//dec_sdo_read_32
int daininterface::f_oper_sdo_read_32( uint16_t v_local_slave_number,
                         uint16_t v_local_index,
                         uint16_t v_local_subindex,
                         const uint8_t *v_local_data)
{
#if 0
    int v_local_ret;

    v_local_ret = f_oper_sdo_read(v_local_slave_number, v_local_index, v_local_subindex, 4, v_local_data);
    return v_local_ret;
#endif
}

/**  SII Write
 */
//dec_slave_sii_write
int daininterface::f_oper_slave_sii_write(
                        dec_slave_sii_t *v_local_slave_sii)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_SLAVE_SII_WRITE, v_local_slave_sii) != 0)
    {
        d_print( "Failed to dec_slave_sii_write: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}

/**  SII Read
 */
//dec_slave_sii_read
int  daininterface::f_oper_slave_sii_read(
                        dec_slave_sii_t *v_local_slave_sii)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_SLAVE_SII_READ, v_local_slave_sii) != 0)
    {
        d_print( "Failed to dec_slave_sii_read: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/** set slave state
 *  init, pre-op, safe-op , op
 */
//dec_set_slave_state
int  daininterface::f_oper_set_slave_state(dec_ioctl_slave_state_t *v_local_data)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_SET_SLAVE_STATE, v_local_data) != 0)
    {
        d_print( "Failed to dec_set_slave_state: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/**  Set the Debug Print Level
 */
//dec_set_print_level
int  daininterface::f_oper_set_print_level (unsigned int v_local_level)
{
    int v_local_ret = 0;
    char v_local_command[50] = {0,};

    if(v_local_level < 0)
        v_local_level = 0;
    else if(v_local_level > 7)
        v_local_level = 7;

    sprintf(v_local_command,"echo \"%d\" > /proc/sys/kernel/printk",v_local_level);
    d_print("%s",v_local_command);
    v_local_ret = system(v_local_command);

    return v_local_ret;
}

/** Read a slave's registers.
 *
 * \return Zero on success, otherwise a negative error code.
 */
//dec_slave_reg_read
int daininterface::f_oper_slave_reg_read(dec_ioctl_slave_reg_t *v_local_slave_reg, int *v_local_ptr_decatlibversion)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;
    if(*v_local_ptr_decatlibversion == 0) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_SLAVE_REG_READ, v_local_slave_reg) != 0)
    {
        d_print( "Failed to dec_slave_reg_read: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/** Write a slave's registers.
 *
 * \return Zero on success, otherwise a negative error code.
 */
//dec_slave_reg_write
int daininterface::f_oper_slave_reg_write(dec_ioctl_slave_reg_t *v_local_slave_reg, int *v_local_ptr_decatlibversion)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;
    if(v_local_ptr_decatlibversion == 0) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_SLAVE_REG_WRITE, v_local_slave_reg) != 0)
    {
        d_print( "Failed to dec_slave_reg_write: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/** IOCTL_DEC_GET_ONLINE_SLAVE_NUM
 */
//dec_get_online_slave_num
int daininterface::f_oper_get_online_slave_num(uint16_t* v_local_value)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_ONLINE_SLAVE_NUM, v_local_value) != 0)
    {
        d_print( "Failed to dec_get_online_slave_num: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/** IOCTL_DEC_GET_CURRENT_WORKING_CNT
 */
//dec_get_current_working_cnt
int daininterface::f_oper_get_current_working_cnt(uint16_t* v_local_value)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_CURRENT_WORKING_CNT, v_local_value) != 0)
    {
        d_print( "Failed to dec_get_current_working_cnt: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/** IOCTL_DEC_GET_RT_TASK_STATE
 */
//dec_get_rt_task_state
int daininterface::f_oper_get_rt_task_state(uint16_t* v_local_value)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_RT_TASK_STATE, v_local_value) != 0)
    {
        d_print( "Failed to dec_get_rt_task_state: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}

/** IOCTL_DEC_GET_RT_TASK_CNT
 */
//dec_get_rt_task_cnt
int daininterface::f_oper_get_rt_task_cnt(uint32_t* v_local_value)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_RT_TASK_CNT, v_local_value) != 0)
    {
        d_print( "Failed to dec_get_rt_task_cnt: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/** IOCTL_DEC_GET_LOST_PACKETCNT
 */
//dec_get_lost_packetcnt
int daininterface::f_oper_get_lost_packetcnt(uint32_t* v_local_value)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_LOST_PACKETCNT, v_local_value) != 0)
    {
        d_print( "Failed to dec_get_lost_packetcnt: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/** IOCTL_DEC_GET_PACKET_CRC_ERRCNT
 */
//dec_get_packet_crc_errcnt
int daininterface::f_oper_get_packet_crc_errcnt(uint32_t* v_local_value)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_PACKET_CRC_ERRCNT, v_local_value) != 0)
    {
        d_print( "Failed to dec_get_packet_crc_errcnt: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/** IOCTL_DEC_GET_WORKINGCNT_CHANGECNT
 */
//dec_get_packet_crc_errcnt
int daininterface::f_oper_get_workingcnt_changecnt(uint16_t* v_local_value)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_WORKINGCNT_CHANGECNT, v_local_value) != 0)
    {
        d_print( "Failed to dec_get_workingcnt_changecnt: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}

/** IOCTL_DEC_GET_ERROR_STATE
 */
//dec_get_error_state
int daininterface::f_oper_get_error_state(uint32_t* v_local_value)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_ERROR_STATE, v_local_value) != 0)
    {
        d_print( "Failed to dec_get_error_state: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}

/** IOCTL_DEC_GET_ONLINE_SLAVENUM_ERRCNT
 */
//dec_get_online_slavenum_errcnt
int daininterface::f_oper_get_online_slavenum_errcnt(uint16_t* v_local_value)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_ONLINE_SLAVENUM_ERRCNT, v_local_value) != 0)
    {
        d_print( "Failed to dec_get_online_slavenum_errcnt: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}

/** IOCTL_DEC_GET_RDYEVENT_ERRCNT
 */
//dec_get_rdyevent_errcnt
int daininterface::f_oper_get_rdyevent_errcnt(uint16_t* v_local_value)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_RDYEVENT_ERRCNT, v_local_value) != 0)
    {
        d_print( "Failed to dec_get_rdyevent_errcnt: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/** IOCTL_DEC_GET_APPEVENT_ERRCNT
 */
//dec_get_appevent_errcnt
int daininterface::f_oper_get_appevent_errcnt(uint16_t* v_local_value)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_APPEVENT_ERRCNT, v_local_value) != 0)
    {
        d_print( "Failed to dec_get_appevent_errcnt: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/** IOCTL_DEC_GET_ENDEVENT_ERRCNT
 */
//dec_get_endevent_errcnt
int daininterface::f_oper_get_endevent_errcnt(uint16_t* v_local_value)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_ENDEVENT_ERRCNT, v_local_value) != 0)
    {
        d_print( "Failed to dec_get_endevent_errcnt: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/** IOCTL_DEC_GET_WAITPERIOD_ERRCNT
 */
//dec_get_waitperiod_errcnt
int daininterface::f_oper_get_waitperiod_errcnt(uint16_t* v_local_value)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_WAITPERIOD_ERRCNT, v_local_value) != 0)
    {
        d_print( "Failed to dec_get_waitperiod_errcnt: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/** IOCTL_DEC_GET_DEEP_SLEEP_ERRCNT
 */
//dec_get_deep_sleep_errcnt
int daininterface::f_oper_get_deep_sleep_errcnt(uint16_t* v_local_value)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_DEEP_SLEEP_ERRCNT, v_local_value) != 0)
    {
        d_print( "Failed to dec_get_deep_sleep_errcnt: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/** IOCTL_DEC_GET_WAKEUP_LARGE_ERRCNT
 */
//dec_get_wakeup_large_errcnt
int daininterface::f_oper_get_wakeup_large_errcnt(uint16_t* v_local_value)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_WAKEUP_LARGE_ERRCNT, v_local_value) != 0)
    {
        d_print( "Failed to dec_get_wakeup_large_errcnt: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

    return 	v_local_ret;
}


/** IOCTL_DEC_GET_MASTER_VERSION
 */
//dec_get_master_version
int  daininterface::f_oper_get_master_version(dec_ioctl_master_version_t *v_local_version)
{
    int v_local_ret = 0;

    if (f_oper_open() < 0 ) return -1;

    if (ioctl(v_global_dec_fd, IOCTL_DEC_GET_MASTER_VERSION, v_local_version) != 0)
    {
        d_print( "Failed to dec_get_wakeup_large_errcnt: %s\n", strerror(errno));
        v_local_ret =  -1;
    }

#if RT_TASK_FUNCTION_DEBUG
     d_print("[DAININTERFACE] : [init] : [f_oper_get_master_version]\r\n");
     d_print("v_local_version->str_data : %s\r\n",v_local_version->str_data);
#endif


    return 	v_local_ret;
}


/**  get master stats address
 */
//dec_master_stats_data
dec_shared_stats_t *daininterface::f_oper_dec_master_stats_data(
                                                                void *v_local_ptr_domainshmem,
                                                                int v_local_ptr_decatlibversion)
{
    // create shared memory
    uint8_t *v_local_shared_data = NULL;

    if(v_local_ptr_domainshmem && v_local_ptr_decatlibversion == 1)
        v_local_shared_data = (uint8_t *)v_local_ptr_domainshmem + C_GLOBAL_MASTER_STATE_MAP_OFFSET;

    return 	(dec_shared_stats_t*)v_local_shared_data;
}



/**  read master stats data
 */
//read_master_stats_data
void daininterface::f_oper_read_master_stats_data(dec_shared_stats_t *v_local_ptr_master_stats_data,
                                                  void **v_local_ptr_domainshmem,
                                                  int *v_local_ptr_decatlibversion)
{
    dec_shared_stats_t *v_local_master_stats_pd;

    v_local_master_stats_pd = f_oper_dec_master_stats_data(*v_local_ptr_domainshmem,*v_local_ptr_decatlibversion);

    if(v_local_master_stats_pd)
        memcpy(v_local_ptr_master_stats_data,v_local_master_stats_pd,sizeof(dec_shared_stats_t));
}
