#include <assert.h>
#include "io_macro.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_can.h"
#include "pcanpro_variant.h"

#include "pcanfd_ucan.h"

const char* UcanCmdStr[] = {
    "CMD_NOP",
    "CMD_RESET_MODE",
    "CMD_NORMAL_MODE",
    "CMD_LISTEN_ONLY_MODE",
    "CMD_TIMING_SLOW",
    "CMD_TIMING_FAST",
    "CMD_SET_STD_FILTER",
    "CMD_RESERVED2",
    "CMD_FILTER_STD",
    "CMD_TX_ABORT",
    "CMD_WR_ERR_CNT",
    "CMD_SET_EN_OPTION",
    "CMD_CLR_DIS_OPTION",
    "CMD_SET_ERR_GEN1",
    "CMD_SET_ERR_GEN2",
    "CMD_DIS_ERR_GEN",
    "CMD_RX_BARRIER",
    "CMD_SET_ERR_GEN_S"
};

const char* UcanCmdClkSet   = "CMD_CLK_SET";
const char* UcanCmdEndColl  = "CMD_END_OF_COLLECTION";
const char* UcanCmdDevidSet = "CMD_DEVID_SET";
#if 0
uint32_t pcan_can_msg_time( const struct t_can_msg *pmsg, uint32_t nt, uint32_t dt ) {
    static uint32_t can_msg_time = 0;
    return ++can_msg_time;
}
int pcan_can_write( int bus, struct t_can_msg *p_msg ) { return 0; }
void pcan_can_set_bitrate( int bus, uint32_t bitrate, int is_data_bitrate ) {}
void pcan_can_set_bus_active( int bus, uint16_t mode ) {}
void pcan_can_set_iso_mode( int bus, uint8_t iso_mode ) {}
void pcan_can_set_silent( int bus, uint8_t silent_mode ) {}
int pcan_can_set_filter_mask( int bus, int num, int format, uint32_t id, uint32_t mask ) { return 0; }
void pcan_can_poll( void ) {}
#endif
