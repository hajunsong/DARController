#include "esi.h"



/*
 ** PANASONIC ENTRIES.
 *
 * index 2
 * DIR : output
 * n_pdos : 1
 * pdos
 *   - index 0x1600
 *   - n_entries : 4
 *   - pdo entries
 *      - index0 0x6040
 *      - subindex 0x00
 *      - bitsize : 16
 *
 *      - index1 0x6060
 *      - subindex 0x00
 *      - bitsize : 8
 *
 *      - index2 0x607A
 *      - subindex 0x00
 *      - bitsize : 32
 *
 *      - index3 0x60B8
 *      - subindex 0x00
 *      - bitsize : 16
 *
 * whatchdog : DEFAULT
 *
 *
 * index 3
 * DIR : output
 * n_pdos : 1
 * pdos
 *   - index 0x1A00
 *   - n_entries : 10
 *   - pdo entries
 *
 *      - index0 0x603F
 *      - subindex 0x00
 *      - bitsize : 16
 *
 *      - index1 0x6041
 *      - subindex 0x00
 *      - bitsize : 16
 *
 *      - index2 0x6061
 *      - subindex 0x00
 *      - bitsize : 8
 *
 *      - index3 0x6064
 *      - subindex 0x00
 *      - bitsize : 32
 *
 *      - index4 0x60B9
 *      - subindex 0x00
 *      - bitsize : 16
 *
 *      - index5 0x60BA
 *      - subindex 0x00
 *      - bitsize : 32
 *
 *      - index6 0x60F4
 *      - subindex 0x00
 *      - bitsize : 32
 *
 *      - index7 0x60FB
 *      - subindex 0x00
 *      - bitsize : 32
 *
 *      - index8 0x606C
 *      - subindex 0x00
 *      - bitsize : 32
 *
 *      - index9 0x6077
 *      - subindex 0x00
 *      - bitsize : 16
 */


/*
 ** LSMPO ENTRIES.
 *
 * index 2
 * DIR : output
 * n_pdos : 1
 * pdos
 *   - index 0x1601
 *   - n_entries : 2
 *   - pdo entries
 *      - index0 0x6040
 *      - subindex 0x00
 *      - bitsize : 16
 *
 *      - index2 0x607A
 *      - subindex 0x00
 *      - bitsize : 32
 *
 * whatchdog : DEFAULT
 *
 *
 * index 3
 * DIR : output
 * n_pdos : 1
 * pdos
 *   - index 0x1A01
 *   - n_entries : 2
 *   - pdo entries
 *
 *      - index1 0x6041
 *      - subindex 0x00
 *      - bitsize : 16
 *
 *      - index3 0x6064
 *      - subindex 0x00
 *      - bitsize : 32
 */


int f_oper_KITECH_ECAT_make(ec_sync_info_t *v_local_slave_syncs,
                            ec_pdo_info_t *v_local_pdos,
                            ec_pdo_entry_info_t *v_local_entries)
{
    int v_area_entries_count = 0;

    ec_pdo_entry_info_t v_area_entries_buffer[]={
        {0x6040, 0, 16}, // Controlword
        {0x607A, 0, 32}, // Target Position
        {0x60FF, 0, 32}, // Target Velocity
        {0x6071, 0, 16}, // Target Torque
        {0x60FE, 1, 32}, // Physical Outputs
        {0x6060, 0,  8}, // Modes of Operations

        {0x6041, 0, 16}, // Statusword
        {0x6064, 0, 32}, // Position Actual Value
        {0x606c, 0, 32}, // Velocity Actual Value
        {0x6077, 0, 16}, // Torque Actual Value
        {0x60fd, 0, 32}, // Digital Inputs
        {0x6061, 0,  8}, // Modes of Operation Display
    };

    // PDO entries insert
    for(v_area_entries_count = 0; v_area_entries_count< 12; ++v_area_entries_count)
    {
        v_local_entries[v_area_entries_count].index = v_area_entries_buffer[v_area_entries_count].index;
        v_local_entries[v_area_entries_count].subindex = v_area_entries_buffer[v_area_entries_count].subindex;
        v_local_entries[v_area_entries_count].bit_length = v_area_entries_buffer[v_area_entries_count].bit_length;
    }

    // PDOs insert
    v_local_pdos[0].index = 0x1608;
    v_local_pdos[0].n_entries = 6;
    v_local_pdos[0].entries = v_local_entries + 0;


    v_local_pdos[1].index = 0x1A08;
    v_local_pdos[1].n_entries = 6;
    v_local_pdos[1].entries = v_local_entries + 6;

    // syncmanager insert
    v_local_slave_syncs[0].pdos = v_local_pdos + 0;
    v_local_slave_syncs[0].index = 2;
    v_local_slave_syncs[0].dir = EC_DIR_OUTPUT;
    v_local_slave_syncs[0].n_pdos = 1;
    v_local_slave_syncs[0].watchdog_mode = EC_WD_DEFAULT;

    v_local_slave_syncs[1].pdos = v_local_pdos + 1;
    v_local_slave_syncs[1].index = 3;
    v_local_slave_syncs[1].dir = EC_DIR_INPUT;
    v_local_slave_syncs[1].n_pdos = 1;
    v_local_slave_syncs[1].watchdog_mode = EC_WD_DEFAULT;

    v_local_slave_syncs[2].index = 0xff;

    return  0;
}


int f_oper_pana1507BA1_make(ec_sync_info_t *v_local_slave_syncs,
                            ec_pdo_info_t *v_local_pdos,
                            ec_pdo_entry_info_t *v_local_entries)
{
    int v_area_entries_count = 0;

    ec_pdo_entry_info_t v_area_entries_buffer[]={
        {0x6040, 0, 16},    // Control word
        {0x6060, 0, 8},     // Modes of operation
        {0x607A, 0, 32},    // Target position
        {0x60B8, 0, 16},     // Touch probe function

        {0x603F, 0, 16},    // ERROR Code
        {0x6041, 0, 16},    // Status word
        {0x6061, 0, 8},     // Modes of operation display
        {0x6064, 0, 32},    // Position actual value
        {0x60b9, 0, 16},    // Touch probe status
        {0x60bA, 0, 32},    // Touch probe position 1 positive value
        {0x60f4, 0, 32},    // Following error actual value
        {0x60fd, 0, 32},     // Digital input
        {0x606c, 0, 32},     // Actual Velocity
        {0x6077, 0, 16}     // Actual Torque
    };


    // PDO entries insert

    for(v_area_entries_count = 0; v_area_entries_count< 14; ++v_area_entries_count)
    {
        v_local_entries[v_area_entries_count].index = v_area_entries_buffer[v_area_entries_count].index;
        v_local_entries[v_area_entries_count].subindex = v_area_entries_buffer[v_area_entries_count].subindex;
        v_local_entries[v_area_entries_count].bit_length = v_area_entries_buffer[v_area_entries_count].bit_length;
    }



    // PDOs insert

    v_local_pdos[0].index = 0x1600;
    v_local_pdos[0].n_entries = 4;
    v_local_pdos[0].entries = v_local_entries + 0;


    v_local_pdos[1].index = 0x1A00;
    v_local_pdos[1].n_entries = 10;
    v_local_pdos[1].entries = v_local_entries + 4;




    // syncmanager insert
    v_local_slave_syncs[0].pdos = v_local_pdos + 0;
    v_local_slave_syncs[0].index = 2;
    v_local_slave_syncs[0].dir = EC_DIR_OUTPUT;
    v_local_slave_syncs[0].n_pdos = 1;
    v_local_slave_syncs[0].watchdog_mode = EC_WD_DEFAULT;

    v_local_slave_syncs[1].pdos = v_local_pdos + 1;
    v_local_slave_syncs[1].index = 3;
    v_local_slave_syncs[1].dir = EC_DIR_INPUT;
    v_local_slave_syncs[1].n_pdos = 1;
    v_local_slave_syncs[1].watchdog_mode = EC_WD_DEFAULT;

    v_local_slave_syncs[2].index = 0xff;

    return  0;
}



int f_oper_LSMPO_make(ec_sync_info_t *v_local_slave_syncs,
                            ec_pdo_info_t *v_local_pdos,
                            ec_pdo_entry_info_t *v_local_entries)
{
    int v_area_entries_count = 0;


    ec_pdo_entry_info_t v_area_entries_buffer[]={
        {0x6040, 0, 16},    // Control word
        {0x607A, 0, 32},    // Target position
        {0x6041, 0, 16},    // Status word
        {0x6064, 0, 32},    // Position actual value
    };


    // PDO entries insert
    for(v_area_entries_count = 0; v_area_entries_count< 4; ++v_area_entries_count)
    {
        v_local_entries[v_area_entries_count].index = v_area_entries_buffer[v_area_entries_count].index;
        v_local_entries[v_area_entries_count].subindex = v_area_entries_buffer[v_area_entries_count].subindex;
        v_local_entries[v_area_entries_count].bit_length = v_area_entries_buffer[v_area_entries_count].bit_length;
    }



    // PDOs insert

    v_local_pdos[0].index = 0x1601;
    v_local_pdos[0].n_entries = 2;
    v_local_pdos[0].entries = v_local_entries + 0;


    v_local_pdos[1].index = 0x1A01;
    v_local_pdos[1].n_entries = 2;
    v_local_pdos[1].entries = v_local_entries + 2;


    // syncmanager insert

    v_local_slave_syncs[0].pdos = v_local_pdos + 0;
    v_local_slave_syncs[0].index = 2;
    v_local_slave_syncs[0].dir = EC_DIR_OUTPUT;
    v_local_slave_syncs[0].n_pdos = 1;
    v_local_slave_syncs[0].watchdog_mode = EC_WD_DEFAULT;

    v_local_slave_syncs[1].pdos = v_local_pdos + 1;
    v_local_slave_syncs[1].index = 3;
    v_local_slave_syncs[1].dir = EC_DIR_INPUT;
    v_local_slave_syncs[1].n_pdos = 1;
    v_local_slave_syncs[1].watchdog_mode = EC_WD_DEFAULT;

    v_local_slave_syncs[2].index = 0xff;



    return  0;
}
