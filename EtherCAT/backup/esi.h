#ifndef ESI_H
#define ESI_H
#include "globalstruct.h"
#include "ecatslave.h"
#include <stdlib.h>

int f_oper_pana1507BA1_make(ec_sync_info_t *v_local_slave_syncs,
                            ec_pdo_info_t *v_local_pdos,
                            ec_pdo_entry_info_t *v_local_entries);


int f_oper_KITECH_ECAT_make(ec_sync_info_t *v_local_slave_syncs,
                            ec_pdo_info_t *v_local_pdos,
                            ec_pdo_entry_info_t *v_local_entries);


int f_oper_LSMPO_make(ec_sync_info_t *v_local_slave_syncs,
                            ec_pdo_info_t *v_local_pdos,
                            ec_pdo_entry_info_t *v_local_entries);

#endif // ESI_H

