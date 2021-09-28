#ifndef GLOBALSTRUCT_H
#define GLOBALSTRUCT_H
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h> // O_RDWR
#include <errno.h> // errno
#include <string.h> // strerror()
#include <sys/mman.h> // PROT_READ
#include <linux/ioctl.h> // IOW
#include <unistd.h> // close
#include <sys/ioctl.h>


#define CONTROL_WORD        0x6040
#define MODE_OF_OP          0x6060
#define TARGET_POSITION     0x607A
#define TARGET_VELOCITY     0x60FF
#define PHYSICAL_OUTPUT     0x60FE
#define TARGET_TORQUE       0x6071
#define STATUS_WORD         0x6041
#define POSITION_VALUE      0x6064
#define VELOCITY_VALUE      0x606C
#define TORQUE_VALUE        0x6077


#define	ECAT_STATE_INIT			1
#define	ECAT_STATE_PREOP		2
#define	ECAT_STATE_BOOTSTRAP	3
#define	ECAT_STATE_SAFEOP		4
#define	ECAT_STATE_OP			8
#define	ECAT_STATE_ERROR		16


#define is_new_pos(x,y) (abs(x-y) > 1)
#define TASK_PRIO  50 /* middle RT priority */
#define TASK_MODE  T_FPU  /* No flags */
#define TASK_STKSZ 0  /* Stack size (use default one) */


#define REF(x) (*x) // Reference Operator function
#define DREF(x) (**x) // Reference Operator function



#define PFX						"ec-app: "
#define d_print(fmt, args...)   printf(PFX" " fmt, ##args)
#define d_line()			 	printf(PFX " %s() L:%d \r\n", __FUNCTION__, __LINE__ )

#define  DEC_DEV_STRING 	"/dev/da-ecat"
#define  EC_SYNC_SIGNAL_COUNT   2
#define  DAIN_DEVICE_TYPE       0xda


#define	MAX_VENDOR	3


#define MAX_SLAVES    8
#define MAX_ENTRIES   480
#define MAX_PDOS      16
#define MAX_SYNC      8
#define MAX_NAME_LEN  32

#define CYCLE_NUMS   	8
#define SCYCLE_NUMS   	8
#define REF_NUMS   		8


#define TASK_WAIT_NS   1000000

#define EVENT_INIT        0x0           /* No flags present at init */
#define EVENT_MODE        EV_PRIO       /* Tasks will wait by priority order */

#define EVENT_WAIT_READY  0x1
#define EVENT_WAIT_APP	  0x2
#define EVENT_WAIT_KO	  0x4
#define EVENT_WAIT_END	  0x8
#define EVENT_WAIT_MASK   (0x2) 		/* List of monitored events */
#define EVENT_SIGNAL_MASK (0x2)			/* List of events to send */


#ifndef __KERNEL__

#if __BYTE_ORDER == __LITTLE_ENDIAN

#define le16_to_cpu(x) x
#define le32_to_cpu(x) x
#define le64_to_cpu(x) x

#define cpu_to_le16(x) x
#define cpu_to_le32(x) x
#define cpu_to_le64(x) x

#elif __BYTE_ORDER == __BIG_ENDIAN

#define swap16(x) \
    ((uint16_t)( \
    (((uint16_t)(x) & 0x00ffU) << 8) | \
    (((uint16_t)(x) & 0xff00U) >> 8) ))
#define swap32(x) \
    ((uint32_t)( \
    (((uint32_t)(x) & 0x000000ffUL) << 24) | \
    (((uint32_t)(x) & 0x0000ff00UL) <<  8) | \
    (((uint32_t)(x) & 0x00ff0000UL) >>  8) | \
    (((uint32_t)(x) & 0xff000000UL) >> 24) ))
#define swap64(x) \
    ((uint64_t)( \
    (((uint64_t)(x) & 0x00000000000000ffULL) << 56) | \
    (((uint64_t)(x) & 0x000000000000ff00ULL) << 40) | \
    (((uint64_t)(x) & 0x0000000000ff0000ULL) << 24) | \
    (((uint64_t)(x) & 0x00000000ff000000ULL) <<  8) | \
    (((uint64_t)(x) & 0x000000ff00000000ULL) >>  8) | \
    (((uint64_t)(x) & 0x0000ff0000000000ULL) >> 24) | \
    (((uint64_t)(x) & 0x00ff000000000000ULL) >> 40) | \
    (((uint64_t)(x) & 0xff00000000000000ULL) >> 56) ))

#define le16_to_cpu(x) swap16(x)
#define le32_to_cpu(x) swap32(x)
#define le64_to_cpu(x) swap64(x)

#define cpu_to_le16(x) swap16(x)
#define cpu_to_le32(x) swap32(x)
#define cpu_to_le64(x) swap64(x)

#endif

#define le16_to_cpup(x) le16_to_cpu(*((uint16_t *)(x)))
#define le32_to_cpup(x) le32_to_cpu(*((uint32_t *)(x)))
#define le64_to_cpup(x) le64_to_cpu(*((uint64_t *)(x)))

#endif /* ifndef __KERNEL__ */


/******************************************************************************
 * Read macros
 *****************************************************************************/

/** Read an 8-bit unsigned value from EtherCAT data.
 *
 * \return EtherCAT data value
 */
#define EC_READ_U8(DATA) \
    ((uint8_t) *((uint8_t *) (DATA)))

/** Read an 8-bit signed value from EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \return EtherCAT data value
 */
#define EC_READ_S8(DATA) \
    ((int8_t) *((uint8_t *) (DATA)))

/** Read a 16-bit unsigned value from EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \return EtherCAT data value
 */
#define EC_READ_U16(DATA) \
    ((uint16_t) le16_to_cpup((void *) (DATA)))

/** Read a 16-bit signed value from EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \return EtherCAT data value
 */
#define EC_READ_S16(DATA) \
    ((int16_t) le16_to_cpup((void *) (DATA)))

/** Read a 32-bit unsigned value from EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \return EtherCAT data value
 */
#define EC_READ_U32(DATA) \
    ((uint32_t) le32_to_cpup((void *) (DATA)))

/** Read a 32-bit signed value from EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \return EtherCAT data value
 */
#define EC_READ_S32(DATA) \
    ((int32_t) le32_to_cpup((void *) (DATA)))

/** Read a 64-bit unsigned value from EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \return EtherCAT data value
 */
#define EC_READ_U64(DATA) \
    ((uint64_t) le64_to_cpup((void *) (DATA)))

/** Read a 64-bit signed value from EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \return EtherCAT data value
 */
#define EC_READ_S64(DATA) \
    ((int64_t) le64_to_cpup((void *) (DATA)))

/******************************************************************************
 * Write macros
 *****************************************************************************/

/** Write an 8-bit unsigned value to EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \param VAL new value
 */
#define EC_WRITE_U8(DATA, VAL) \
    do { \
    *((uint8_t *)(DATA)) = ((uint8_t) (VAL)); \
    } while (0)

/** Write an 8-bit signed value to EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \param VAL new value
 */
#define EC_WRITE_S8(DATA, VAL) EC_WRITE_U8(DATA, VAL)

/** Write a 16-bit unsigned value to EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \param VAL new value
 */
#define EC_WRITE_U16(DATA, VAL) \
    do { \
    *((uint16_t *) (DATA)) = cpu_to_le16((uint16_t) (VAL)); \
    } while (0)

/** Write a 16-bit signed value to EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \param VAL new value
 */
#define EC_WRITE_S16(DATA, VAL) EC_WRITE_U16(DATA, VAL)

/** Write a 32-bit unsigned value to EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \param VAL new value
 */
#define EC_WRITE_U32(DATA, VAL) \
    do { \
    *((uint32_t *) (DATA)) = cpu_to_le32((uint32_t) (VAL)); \
    } while (0)

/** Write a 32-bit signed value to EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \param VAL new value
 */
#define EC_WRITE_S32(DATA, VAL) EC_WRITE_U32(DATA, VAL)

/** Write a 64-bit unsigned value to EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \param VAL new value
 */
#define EC_WRITE_U64(DATA, VAL) \
    do { \
    *((uint64_t *) (DATA)) = cpu_to_le64((uint64_t) (VAL)); \
    } while (0)

/** Write a 64-bit signed value to EtherCAT data.
 *
 * \param DATA EtherCAT data pointer
 * \param VAL new value
 */
#define EC_WRITE_S64(DATA, VAL) EC_WRITE_U64(DATA, VAL)

/*****************************************************************************/

/** @} */





const ulong v_global_con_cycle_val[CYCLE_NUMS] = { 0,250000, 500000, 1000000, 2000000, 4000000, 8000000,16000000};
const ulong v_global_con_scycle_val[SCYCLE_NUMS] = { 0, 125000, 250000, 500000, 1000000, 2000000, 4000000, 8000000};
const ulong v_global_con_ref_val[REF_NUMS] = { 0, 1, 2, 4, 8, 10, 16, 20};


//class globalstruct
//{
// define const velue
//public:

static const int C_GLOBAL_ONLINE_SLAVE_NUM_ERR     = 0x01;
static const int C_GLOBAL_READY_EVENT_ERROR        = 0x02;
static const int C_GLOBAL_APP_EVENT_ERROR          = 0x04;
static const int C_GLOBAL_END_EVENT_ERROR          = 0x08;
static const int C_GLOBAL_WAIT_PERIOD_ERROR        = 0x10;
static const int C_GLOBAL_DEEP_SLEEP_ERROR         = 0x20;
static const int C_GLOBAL_WAKEUP_COUNT_LARGE_ERROR = 0x40;
static const int C_GLOBAL_DEC_ERROR_FLAG           = 0x80;


static const int C_GLOBAL_PAGE_SIZE                = 4096;
static const int C_GLOBAL_MEMORY_MAP_PAGE          = 5;
static const int C_GLOBAL_MASTER_STATE_MAP_PAGE    = (C_GLOBAL_MEMORY_MAP_PAGE-1);
static const int C_GLOBAL_MASTER_STATE_MAP_OFFSET  = (C_GLOBAL_MASTER_STATE_MAP_PAGE * C_GLOBAL_PAGE_SIZE);
static const int C_GLOBAL_MEMORY_MAP_SIZE          = (C_GLOBAL_PAGE_SIZE * C_GLOBAL_MEMORY_MAP_PAGE);


static const int C_GLOBAL_IO_READ                  = 0;
static const int C_GLOBAL_IO_WRITE                 = 1;

//enum
//protected:

//EtherCAT Header

/*****************************************************************************/

/** Direction type for PDO assignment functions.
             */
typedef enum {
    EC_DIR_INVALID, /**< Invalid direction. Do not use this value. */
    EC_DIR_OUTPUT, /**< Values written by the master. */
    EC_DIR_INPUT, /**< Values read by the master. */
    EC_DIR_COUNT /**< Number of directions. For internal use only. */
} ec_direction_t;

/*****************************************************************************/

/*****************************************************************************/

/** Watchdog mode for sync manager configuration.
             *
             * Used to specify, if a sync manager's watchdog is to be enabled.
             */
typedef enum {
    EC_WD_DEFAULT, /**< Use the default setting of the sync manager. */
    EC_WD_ENABLE, /**< Enable the watchdog. */
    EC_WD_DISABLE, /**< Disable the watchdog. */
} ec_watchdog_mode_t;

/*****************************************************************************/

/*****************************************************************************/
typedef enum {
    EC_AL_STATES_INIT 	= 1,
    EC_AL_STATES_PREOP	= 2,
    EC_AL_STATES_SAFEOP	= 4,
    EC_AL_STATES_OP		= 8
} ec_master_al_stagus_t;
/*****************************************************************************/

/*****************************************************************************/

typedef enum{
    REG_OUT = 0,
    REG_IN
} ec_pdo_entry_reg_type;

/*****************************************************************************/




typedef enum{
    rx_pdos_servo = 0,
    tx_pdos_servo,
    rx_pdos_input,
    tx_pdos_output,
}PDO_ENTRIES_TYPE;


typedef enum{
    _8BIT = 0,
    _16BIT,
    _32BIT,
}COMMUNICAT_SIZE_TYPE;

typedef enum{
    PANA_1507BA1 = 0,
    LSMPO,
}VENDOR_TYPE;



enum { REQ_SERVO_OFF,REQ_SERVO_ON,REQ_SERVO_RUN,REQ_SERVO_STOP};
enum { STATE_SERVO_OFF,STATE_SERVO_SW_ON, STATE_SERVO_ON, STATE_SERVO_ERROR};
enum { RUN_FORWARD, RUN_REVERSE, RUN_TASK};
enum {
    step_error = 1,
    step_shutdown,
    step_volt_on = step_shutdown,
    step_switch_on,
    step_servo_on,
    step_target_pos_fw,
    step_target_pos_rev,
};

//struct
//protected:



//EtherCAT Header


/*****************************************************************************/

/** Slave configuration state.
             *
             * This is used as an output parameter of ecrt_slave_config_state().
             *
             * \see ecrt_slave_config_state().
             */
typedef struct  {
    unsigned int online : 1; /**< The slave is online. */
    unsigned int operational : 1; /**< The slave was brought into \a OP state
                                                using the specified configuration. */
    unsigned int al_state : 4; /**< The application-layer state of the slave.
                                             - 1: \a INIT
                                             - 2: \a PREOP
                                             - 4: \a SAFEOP
                                             - 8: \a OP

                                             Note that each state is coded in a different
                                             bit! */
} ec_slave_config_state_t;

/*****************************************************************************/

/*****************************************************************************/

/** Master state.
             *
             * This is used for the output parameter of ecrt_master_state().
             *
             * \see ecrt_master_state().
             */
typedef struct {
    unsigned int slaves_responding; /**< Number of slaves in the bus. */
    unsigned int al_states : 4; /**< Application-layer states of all slaves.
                                              The states are coded in the lower 4 bits.
                                              If a bit is set, it means that at least one
                                              slave in the bus is in the corresponding
                                              state:
                                              - Bit 0: \a INIT
                                              - Bit 1: \a PREOP
                                              - Bit 2: \a SAFEOP
                                              - Bit 3: \a OP */
    unsigned int link_up : 1; /**< \a true, if the network link is up. */
} ec_master_state_t;

/*****************************************************************************/

/*****************************************************************************/

/** Domain working counter interpretation.
             *
             * This is used in ec_domain_state_t.
             */
typedef enum {
    EC_WC_ZERO = 0,   /**< No registered process data were exchanged. */
    EC_WC_INCOMPLETE, /**< Some of the registered process data were exchanged. */
    EC_WC_COMPLETE    /**< All registered process data were exchanged. */
} ec_wc_state_t;

/*****************************************************************************/

/*****************************************************************************/

/** Domain state.
             *
             * This is used for the output parameter of ecrt_domain_state().
             */
typedef struct {
    unsigned int working_counter; /**< Value of the last working counter. */
    ec_wc_state_t wc_state; /**< Working counter interpretation. */
    unsigned int redundancy_active; /**< Redundant link is in use. */
} ec_domain_state_t;

/*****************************************************************************/

/*****************************************************************************/

/** PDO entry configuration information.
             *
             * This is the data type of the \a entries field in ec_pdo_info_t.
             *
             * \see ecrt_slave_config_pdos().
             */
typedef struct {
    uint16_t index; /**< PDO entry index. */
    uint8_t subindex; /**< PDO entry subindex. */
    uint8_t bit_length; /**< Size of the PDO entry in bit. */
} ec_pdo_entry_info_t;

/*****************************************************************************/

/*****************************************************************************/

/** PDO configuration information.
             *
             * This is the data type of the \a pdos field in ec_sync_info_t.
             *
             * \see ecrt_slave_config_pdos().
             */
typedef struct {
    uint16_t index; /**< PDO index. */
    unsigned int n_entries; /**< Number of PDO entries in \a entries to map.
                                          Zero means, that the default mapping shall be
                                          used (this can only be done if the slave is
                                          present at bus configuration time). */
    ec_pdo_entry_info_t *entries; /**< Array of PDO entries to map. Can either
                                                be \a NULL, or must contain at
                                                least \a n_entries values. */
} ec_pdo_info_t;

/*****************************************************************************/

/*****************************************************************************/

/** Sync manager configuration information.
             *
             * This can be use to configure multiple sync managers including the PDO
             * assignment and PDO mapping. It is used as an input parameter type in
             * ecrt_slave_config_pdos().
             */
typedef struct {
    uint8_t index; /**< Sync manager index. Must be less
                                 than #EC_MAX_SYNC_MANAGERS for a valid sync manager,
                                 but can also be \a 0xff to mark the end of the list. */
    ec_direction_t dir; /**< Sync manager direction. */
    unsigned int n_pdos; /**< Number of PDOs in \a pdos. */
    ec_pdo_info_t *pdos; /**< Array with PDOs to assign. This must contain
                                        at least \a n_pdos PDOs. */
    ec_watchdog_mode_t watchdog_mode; /**< Watchdog mode. */
} ec_sync_info_t;

/*****************************************************************************/

/*****************************************************************************/

/** List record type for PDO entry mass-registration.
             *
             * This type is used for the array parameter of the
             * ecrt_domain_reg_pdo_entry_list()
             */
typedef struct {
    uint16_t alias; /**< Slave alias address. */
    uint16_t position; /**< Slave position. */
    uint32_t vendor_id; /**< Slave vendor ID. */
    uint32_t product_code; /**< Slave product code. */
    uint16_t index; /**< PDO entry index. */
    uint8_t subindex; /**< PDO entry subindex. */
    unsigned int *offset; /**< Pointer to a variable to store the PDO entry's
                                   (byte-)offset in the process data. */
    unsigned int *bit_position; /**< Pointer to a variable to store a bit
                                              position (0-7) within the \a offset. Can be
                                              NULL, in which case an error is raised if the
                                              PDO entry does not byte-align. */
} ec_pdo_entry_reg_t;

/*****************************************************************************/

/*****************************************************************************/

//DEC Library Header

/***********************************************************/

typedef struct {
    uint16_t			domain_number;
    ec_domain_state_t	state;
}dec_domain_state_t;

/***********************************************************/

typedef  ec_master_state_t  dec_master_state_t;

/***********************************************************/

/***********************************************************/

typedef struct {
    uint16_t				slave_number;
    ec_slave_config_state_t	state;
}dec_slave_state_t;

/***********************************************************/

/***********************************************************/

typedef struct {
    uint32_t  slave_count;
    uint32_t  driver_ver;
    uint32_t  master_ver;
    uint8_t   info_str[248];
}dec_info_t;

/***********************************************************/

/***********************************************************/

typedef struct {
    uint16_t slave_position;
    uint16_t offset;
    uint32_t nwords;
    uint16_t *words;
} dec_slave_sii_t;

/***********************************************************/

/***********************************************************/

typedef struct {
    uint16_t slave_position;
    uint8_t al_state;
} dec_ioctl_slave_state_t;


/***********************************************************/

/***********************************************************/

typedef struct {
    // inputs
    uint16_t slave_position;
    uint16_t address;
    size_t size;
    uint8_t *data;
} dec_ioctl_slave_reg_t;

/***********************************************************/

/***********************************************************/

typedef struct {
    uint8_t str_data[20];
} dec_ioctl_master_version_t;

/***********************************************************/



/***********************************************************/

typedef struct {

    /* shared_stats : Rev.1 Start*/
    uint32_t	dec_cbBytes;                    /* structure size : use for revision checking */
    uint32_t	dec_config;                     /* ? */

    uint32_t	dec_error_state;
    uint32_t	dec_rt_cycletime_avg;
    uint32_t	dec_rt_cycletime_min;
    uint32_t	dec_rt_cycletime_max;
    uint32_t	dec_rt_task_runtime_avg;
    uint32_t	dec_rt_task_runtime_min;
    uint32_t	dec_rt_task_runtime_max;

    uint16_t	working_counter;
    uint16_t	online_slave_num;
    uint16_t	recv_data_sync_flag;
    uint16_t	send_data_sync_flag;
    uint16_t	rt_task_state;
    uint32_t	rt_task_counter;

    uint16_t	working_count_change_c;
    uint16_t	online_slave_num_err_c;
    uint16_t	ready_ev_err_c;
    uint16_t	app_ev_err_c;
    uint16_t	end_ev_err_c;
    uint16_t	wait_period_err_c;
    uint16_t	deep_sleep_err_c;
    uint16_t	wakeup_count_large_err_c;
    uint16_t	recv_data_err_c;
    uint16_t	send_data_err_c;

    uint32_t    netdev_rx_length_errors;
    uint32_t    netdev_rx_crc_errors;
    uint32_t    netdev_rx_fifo_errors;

    uint64_t 	ecdev_tx_count;
    uint64_t 	ecdev_rx_count;
    uint16_t	ecdev_lost_packet;
    uint64_t 	ecdev_tx_bytes;
    uint64_t 	ecdev_rx_bytes;

    uint32_t	dc_time_diag_table[16];             /* [0] : total_count, [1~] : 1us, 2, 5, 10, 20, 50, 100, 200, 500, 500> */
    int32_t		cur_dc_diff_time;
    /* shared_stats : Rev.1 End*/

    /* shared_stats : Rev.2 Start*/
    uint8_t		dec_stats_all_clear_flag;           /* clear stats data for restarting: flag is 0 when accepted */
    uint8_t		dc_time_table_clear_flag;           /* clear master drift error table(dc_time_diag_table) */
    /* shared_stats : Rev.2 End*/

    /* shared_stats : Rev.3 Start*/
    uint64_t	app_start_time;
    uint64_t	app_time;
    uint64_t	start_time;
    uint32_t	sync0_cycle_time;                   /* cycle time : must be the same to CONTROL_CYCLE */
    int32_t		sync0_shift_time;                   /* input shift time */
    uint32_t	remainder;
    uint64_t	actual_start;

    uint32_t	actual_shift_time;                  /* actual shift time */
    uint32_t	system_time_of_next_sync0;
    uint32_t	system_time_of_send_frame;
    /* shared_stats : Rev.3 End*/

} dec_shared_stats_t;

/***********************************************************/

/***********************************************************/
typedef struct {
    uint16_t      slave_count;
    uint16_t      domain_count;
}dec_setup_init_t;
/***********************************************************/

/***********************************************************/

typedef struct {
    uint16_t      slave_number;
    uint32_t      vendor_id;
    uint32_t      product_code;
    uint8_t       alias;
    uint8_t       position;
} dec_slave_config_t;

/***********************************************************/


/***********************************************************/

typedef ec_sync_info_t dec_sync_info_t;

/***********************************************************/


/***********************************************************/

typedef struct {
    uint16_t		  slave_number;
    size_t			  syncs_size;
    dec_sync_info_t  *syncs;
} dec_slave_pdo_list_t;

/***********************************************************/

/***********************************************************/

typedef struct {
    uint32_t cycle_time; /**< Cycle time [ns]. */
    uint32_t shift_time; /**< Shift time [ns]. */
} dec_sync_signal_t;

/***********************************************************/

/***********************************************************/

typedef struct {
    uint16_t			slave_number;
    uint16_t            dc_enable;
    uint16_t 			dc_assign_activate;
    dec_sync_signal_t	dc_sync[EC_SYNC_SIGNAL_COUNT];
    uint16_t			ref_count;
}dec_slave_config_dc_t;

/***********************************************************/

/***********************************************************/

typedef   ec_pdo_entry_reg_t  dec_domain_entry_t;

/***********************************************************/

/***********************************************************/

typedef struct {
    uint16_t			 domain_number;
    size_t			  	 entry_size;
    dec_domain_entry_t  *domain_entry;
} dec_domain_list_t;

/***********************************************************/

/***********************************************************/

typedef struct {
    uint32_t 	  offset;
    uint32_t	  size;
}dec_data_sync_t;
/***********************************************************/

/***********************************************************/

typedef struct {
    uint16_t				slave_number;
    uint16_t				index;
    uint16_t				subindex;
    uint16_t				io_flag;
    uint16_t				len;		/* byte: 1 or 2 or 4 or 8  */
    uint8_t	     			data[16];
}dec_sdo_data_t;

typedef struct {
    uint16_t				slave_number;
    uint16_t				index;
    uint16_t				subindex;
    uint16_t				io_flag;
    uint16_t				len;		/* byte: 1 or 2 or 4 or 8  */
    uint16_t	     		data[16];
}dec_sdo16_data_t;

/***********************************************************/


// Slave header
/***********************************************************/
typedef struct{
    uint32_t vendor_id;
    uint32_t product_id;
    char *vendor_name;
    char *slave_models;
    ec_pdo_entry_reg_t *slave_domain_out_regs;
    ec_pdo_entry_reg_t *slave_domain_in_regs;
    ec_pdo_entry_info_t *slave_pdo_entries;
    ec_pdo_info_t 		*slave_pdos;
    ec_sync_info_t 		*slave_syncs;
    bool				is_servo_drive;
    /*
                void (*Slave_init)(int vendor_inx);
                void (*Domain_value_init)(int slv_inx);
                void (*Slave_send)(int slv_inx);
                void (*Slave_receive)(int slv_inx);
                */
}Vendor_Config;
/***********************************************************/




//public:

/********************************************************************
         *
         *   IOCTL List for communicating with driver
         *
         *******************************************************************/
#define IOCTL_DEC_SETUP_INIT		_IOW(DAIN_DEVICE_TYPE, 	0, 	dec_setup_init_t)
#define IOCTL_DEC_SLAVE_CONFIG		_IOW(DAIN_DEVICE_TYPE,  1, 	dec_slave_config_t)
#define IOCTL_DEC_SLAVE_PDO_LIST	_IOW(DAIN_DEVICE_TYPE,  2, 	dec_slave_pdo_list_t)
#define IOCTL_DEC_DOMAIN_LIST		_IOW(DAIN_DEVICE_TYPE,  3, 	dec_domain_list_t)
#define IOCTL_DEC_SLAVE_DC			_IOW(DAIN_DEVICE_TYPE,  4, 	dec_slave_config_dc_t)
#define IOCTL_DEC_DOMAIN_DATA		_IOW(DAIN_DEVICE_TYPE,  5, 	dec_domain_data_t)
#define IOCTL_DEC_DOMAIN_OFFSET		_IO(DAIN_DEVICE_TYPE, 	6)
#define IOCTL_DEC_DOMAIN_STATE		_IOWR(DAIN_DEVICE_TYPE,  7,  dec_domain_state_t)
#define IOCTL_DEC_MASTER_STATE		_IOWR(DAIN_DEVICE_TYPE,  8,  dec_master_state_t)
#define IOCTL_DEC_SLAVE_STATE		_IOWR(DAIN_DEVICE_TYPE,  9,  dec_slave_state_t)
#define IOCTL_DEC_SDO_SET_DATA		_IOW(DAIN_DEVICE_TYPE,  10,  dec_sdo_data_t)
#define IOCTL_DEC_SDO_GET_DATA		_IOR(DAIN_DEVICE_TYPE,  11,  dec_sdo_data_t)
#define IOCTL_DEC_GET_INFO			_IOR(DAIN_DEVICE_TYPE, 	12,  dec_info_t)
#define IOCTL_DEC_RECV_SYNC			_IOW(DAIN_DEVICE_TYPE, 	13,  dec_data_sync_t)
#define IOCTL_DEC_SEND_SYNC			_IOW(DAIN_DEVICE_TYPE, 	14,  dec_data_sync_t)
#define IOCTL_DEC_ACTIVE			_IOR(DAIN_DEVICE_TYPE, 	15,  size_t)
#define IOCTL_DEC_DEACTIVE			_IO(DAIN_DEVICE_TYPE, 	16)
#define IOCTL_DEC_SET_DEBUG_LEVEL	_IOW(DAIN_DEVICE_TYPE, 	17,	 unsigned int)
#define IOCTL_DEC_SLAVE_SII_READ	_IOWR(DAIN_DEVICE_TYPE, 18,	 dec_slave_sii_t)
#define IOCTL_DEC_SLAVE_SII_WRITE	_IOWR(DAIN_DEVICE_TYPE, 19,	 dec_slave_sii_t)
#define IOCTL_DEC_SET_SLAVE_STATE	_IOWR(DAIN_DEVICE_TYPE, 20,	 dec_ioctl_slave_state_t)
#define IOCTL_DEC_SLAVE_REG_READ	_IOWR(DAIN_DEVICE_TYPE, 21,	 dec_ioctl_slave_reg_t)
#define IOCTL_DEC_SLAVE_REG_WRITE	_IOWR(DAIN_DEVICE_TYPE, 22,	 dec_ioctl_slave_reg_t)

#define IOCTL_DEC_GET_ONLINE_SLAVE_NUM			_IOWR(DAIN_DEVICE_TYPE, 23,	 uint16_t)
#define IOCTL_DEC_GET_CURRENT_WORKING_CNT		_IOWR(DAIN_DEVICE_TYPE, 24,	 uint16_t)
#define IOCTL_DEC_GET_RT_TASK_STATE				_IOWR(DAIN_DEVICE_TYPE, 25,	 uint16_t)
#define IOCTL_DEC_GET_RT_TASK_CNT				_IOWR(DAIN_DEVICE_TYPE, 26,	 uint32_t)

#define IOCTL_DEC_GET_LOST_PACKETCNT			_IOWR(DAIN_DEVICE_TYPE, 27,	 uint32_t)
#define IOCTL_DEC_GET_PACKET_CRC_ERRCNT			_IOWR(DAIN_DEVICE_TYPE, 28,	 uint32_t)
#define IOCTL_DEC_GET_WORKINGCNT_CHANGECNT		_IOWR(DAIN_DEVICE_TYPE, 29,	 uint16_t)

#define IOCTL_DEC_GET_ERROR_STATE				_IOWR(DAIN_DEVICE_TYPE, 30,	 uint32_t)
#define IOCTL_DEC_GET_ONLINE_SLAVENUM_ERRCNT	_IOWR(DAIN_DEVICE_TYPE, 31,	 uint16_t)
#define IOCTL_DEC_GET_RDYEVENT_ERRCNT			_IOWR(DAIN_DEVICE_TYPE, 32,	 uint16_t)
#define IOCTL_DEC_GET_APPEVENT_ERRCNT			_IOWR(DAIN_DEVICE_TYPE, 33,	 uint16_t)
#define IOCTL_DEC_GET_ENDEVENT_ERRCNT			_IOWR(DAIN_DEVICE_TYPE, 34,	 uint16_t)
#define IOCTL_DEC_GET_WAITPERIOD_ERRCNT			_IOWR(DAIN_DEVICE_TYPE, 35,	 uint16_t)
#define IOCTL_DEC_GET_DEEP_SLEEP_ERRCNT			_IOWR(DAIN_DEVICE_TYPE, 36,	 uint16_t)
#define IOCTL_DEC_GET_WAKEUP_LARGE_ERRCNT		_IOWR(DAIN_DEVICE_TYPE, 37,	 uint16_t)

#define IOCTL_DEC_GET_MASTER_VERSION			_IOR(DAIN_DEVICE_TYPE, 38,	 dec_ioctl_master_version_t)
//};
// EtherCAT Slave etries velue start

// EtherCAT Slave etries velue end





//Application Debug FLAG !!!

#define ETHERCAT_SLAVE_CLASS_DEBUG 0 // EtherCAT slave debug flag

//EtherCAT master debug flag
#define ETHERCAT_MASTER_CLASS_DEBUG_SLAVE_SETTING 0

#define ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP1 1
#define ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP1 0

#define ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP2 1
#define ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP2 0

#define ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP3 1
#define ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP3 0

#define ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP4 1
#define ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP4 0

#define ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP5 1
#define ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP5 0

#define ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP6 1
#define ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP6 0

#define ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP7 1
#define ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP7 0

#define ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP8 1
#define ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP8 0

#define ETHERCAT_MASTER_CLASS_ACTION_INIT_STEP9 1
#define ETHERCAT_MASTER_CLASS_DEBUG_INIT_STEP9 0


#define RT_TASK_FUNCTION_DEBUG 0



typedef struct{
    int indx, cur_status_word[MAX_SLAVES];
    long tar_position[MAX_SLAVES], cur_position[MAX_SLAVES];
    long tar_velocity[MAX_SLAVES], cur_velocity[MAX_SLAVES];
    int tar_torque[MAX_SLAVES], cur_torque[MAX_SLAVES];
} slave_data_t;




#endif // GLOBALSTRUCT_H
