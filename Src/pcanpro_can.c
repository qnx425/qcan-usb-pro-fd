#include <string.h>
#include <assert.h>
#include "io_macro.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_can.h"
#include "pcanpro_variant.h"
#include "pcanfd_ucan.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

#define CAN_TX_FIFO_SIZE (2048)
struct t_can_dev
{
  void *dev;
  uint32_t tx_msgs;
  uint32_t tx_errs;
  uint32_t tx_ovfs;

  uint32_t rx_msgs;
  uint32_t rx_errs;
  uint32_t rx_ovfs;

  struct t_can_msg tx_fifo[CAN_TX_FIFO_SIZE];
  uint32_t tx_head;
  uint32_t tx_tail;
  uint32_t esr_reg;
  int (*rx_isr)( uint8_t, struct  t_can_msg* );
  int (*tx_isr)( uint8_t, struct  t_can_msg* );
  void (*err_handler)( int bus, uint32_t esr );
};

__attribute__((section(".noinit"))) static struct t_can_dev can_dev_array[CAN_BUS_TOTAL];

#define INTERNAL_CAN_IT_FLAGS          (  CAN_IT_TX_MAILBOX_EMPTY |\
                                          CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING |\
                                          CAN_IT_ERROR_WARNING |\
                                          CAN_IT_ERROR_PASSIVE |\
                                          CAN_IT_LAST_ERROR_CODE |\
                                          CAN_IT_ERROR )

#define CAN2_FILTER_START (14u)

#define CAN_WITHOUT_ISR 1

void pcan_can_init( void )
{
	memset( &can_dev_array[CAN_BUS_1], 0, sizeof( struct t_can_dev ) );
	memset( &can_dev_array[CAN_BUS_2], 0, sizeof( struct t_can_dev ) );

	can_dev_array[CAN_BUS_1].dev = &hfdcan1;
	can_dev_array[CAN_BUS_2].dev = &hfdcan2;
}

uint32_t pcan_can_msg_time( const struct t_can_msg *pmsg, uint32_t nt, uint32_t dt )
{
  const uint32_t data_bits = pmsg->size<<3;
  const uint32_t control_bits = ( pmsg->flags & MSG_FLAG_EXT ) ? 67:47;
 
  if( pmsg->flags & MSG_FLAG_BRS )
    return (control_bits*nt) + (data_bits*dt);
  else
    return (control_bits+data_bits)*nt;
}

int pcan_can_set_filter_mask( int bus, int num, int format, uint32_t id, uint32_t mask )
{
  return 0;
}

static int _can_send( FDCAN_HandleTypeDef *p_can, struct t_can_msg *p_msg )
{
  if( HAL_FDCAN_STATE_BUSY != HAL_FDCAN_GetState( p_can ) ) return -1;

  FDCAN_TxHeaderTypeDef msg = {0};

  if( p_msg->flags & MSG_FLAG_EXT )
  {
    msg.Identifier = p_msg->id & 0x1FFFFFFF;
    msg.IdType = FDCAN_EXTENDED_ID;
  }
  else
  {
    msg.Identifier = p_msg->id & 0x7FF;
    msg.IdType = FDCAN_STANDARD_ID;
  }

  static const uint32_t pcan_fd_datalen2code[] =
  {
	FDCAN_DLC_BYTES_0,
	FDCAN_DLC_BYTES_1,
	FDCAN_DLC_BYTES_2,
	FDCAN_DLC_BYTES_3,
	FDCAN_DLC_BYTES_4,
	FDCAN_DLC_BYTES_5,
	FDCAN_DLC_BYTES_6,
	FDCAN_DLC_BYTES_7,
	FDCAN_DLC_BYTES_8,
	FDCAN_DLC_BYTES_12,
	FDCAN_DLC_BYTES_16,
	FDCAN_DLC_BYTES_20,
	FDCAN_DLC_BYTES_24,
	FDCAN_DLC_BYTES_32,
	FDCAN_DLC_BYTES_48,
	FDCAN_DLC_BYTES_64
  };

  extern const uint8_t pcan_fd_len2dlc[];
  msg.DataLength = pcan_fd_datalen2code[ pcan_fd_len2dlc[p_msg->size] ];

  msg.FDFormat = ( p_msg->flags & MSG_FLAG_FD ) ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;

  if( msg.FDFormat == FDCAN_CLASSIC_CAN ) {
	  msg.TxFrameType = (p_msg->flags & MSG_FLAG_RTR) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
  }
  
  msg.BitRateSwitch = ( p_msg->flags & MSG_FLAG_BRS ) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
  msg.ErrorStateIndicator = ( p_msg->flags & MSG_FLAG_ESI ) ? FDCAN_ESI_ACTIVE : FDCAN_ESI_PASSIVE;

  if( HAL_OK != HAL_FDCAN_AddMessageToTxFifoQ( p_can, &msg, p_msg->data ) )
    return -1;

  return 0;
}

static void pcan_can_flush_tx( int bus )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  struct t_can_msg *p_msg;

  /* empty fifo */
  if( p_dev->tx_head == p_dev->tx_tail )
	return;

  if( !p_dev->dev )
    return;

  p_msg = &p_dev->tx_fifo[p_dev->tx_tail];
  if( _can_send( p_dev->dev, p_msg ) < 0 )
    return;

  if( p_dev->tx_isr )
  {
    (void)p_dev->tx_isr( bus, p_msg );
  }

  /* update fifo index */
  p_dev->tx_tail = (p_dev->tx_tail+1)&(CAN_TX_FIFO_SIZE-1);
}

int pcan_can_write( int bus, struct t_can_msg *p_msg )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];

  if( !p_dev )
    return 0;

  if( !p_msg )
    return 0;

  uint32_t  tx_head_next = (p_dev->tx_head+1)&(CAN_TX_FIFO_SIZE-1);
  /* overflow ? just skip it */
  if( tx_head_next == p_dev->tx_tail )
  {
	++p_dev->tx_ovfs;
    return -1;
  }

  p_dev->tx_fifo[p_dev->tx_head] = *p_msg;
  p_dev->tx_head = tx_head_next;

  return 0;
}

void pcan_can_install_rx_callback( int bus, int (*cb)( uint8_t, struct  t_can_msg* ) )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->rx_isr = cb;
}

void pcan_can_install_tx_callback( int bus, int (*cb)( uint8_t, struct  t_can_msg* ) )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->tx_isr = cb;
}

void pcan_can_install_err_callback( int bus, void (*cb)( int , uint32_t ) )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->err_handler = cb;
}

void apply_settings( int bus, uint16_t opt_mask )
{
	FDCAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

	if( !p_can ) return;

	( opt_mask & UCAN_OPTION_ISO_MODE ) ? HAL_FDCAN_EnableISOMode( p_can ) : HAL_FDCAN_DisableISOMode( p_can );

	p_can->Init.FrameFormat = ( opt_mask & UCAN_OPTION_20AB_MODE ) ? FDCAN_FRAME_CLASSIC : FDCAN_FRAME_FD_BRS;

	if( HAL_FDCAN_Init( p_can ) != HAL_OK )
	{
		assert( 0 );
	}
	p_can->Init.FrameFormat == FDCAN_FRAME_CLASSIC ? 	HAL_FDCAN_DisableTxDelayCompensation( p_can ) :
														HAL_FDCAN_EnableTxDelayCompensation ( p_can );
}

void pcan_can_set_silent( int bus, uint8_t silent_mode )
{
  FDCAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

  if( !p_can )
    return;

  p_can->Init.Mode = silent_mode ? FDCAN_MODE_BUS_MONITORING : FDCAN_MODE_NORMAL;
  if( HAL_FDCAN_Init( p_can ) != HAL_OK )
  {
    assert( 0 );
  }
}

void pcan_can_set_iso_mode( int bus, uint8_t iso_mode )
{
  FDCAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

  if( !p_can )
	return;

  iso_mode ? HAL_FDCAN_EnableISOMode( p_can ) : HAL_FDCAN_DisableISOMode( p_can );
}

void pcan_can_set_loopback( int bus, uint8_t loopback )
{
  FDCAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

  if( !p_can )
    return;
  
  p_can->Init.Mode = loopback ? FDCAN_MODE_EXTERNAL_LOOPBACK : FDCAN_MODE_NORMAL;
  if( HAL_FDCAN_Init( p_can ) != HAL_OK )
  {
    assert( 0 );
  }
}

void pcan_can_set_bus_active( int bus, uint16_t mode )
{
  FDCAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

  if( !p_can )
    return;

  mode ? HAL_FDCAN_Start( p_can ) :	HAL_FDCAN_Stop( p_can );
}

/* set predefined best values */
void pcan_can_set_bitrate( int bus, uint32_t bitrate, int is_data_bitrate )
{
}

void pcan_can_set_bitrate_ex( int bus, uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw, int is_data_bitrate )
{
  FDCAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

  if( !p_can )
    return;

  static const uint32_t fdcan_clock_mhz = CAN_CLOCK_MHZ;
  static const uint32_t tdc_const       = 50;

  if( is_data_bitrate ) {
	  p_can->Init.DataPrescaler 		= brp;
	  p_can->Init.DataTimeSeg1 			= tseg1;
	  p_can->Init.DataTimeSeg2 			= tseg2;
	  p_can->Init.DataSyncJumpWidth 	= sjw;

	  uint32_t tdco = tdc_const * brp * ( 1 + tseg1 + tseg2 ) / fdcan_clock_mhz;
	  if( tdco > 127 ) tdco = 127;

	  uint32_t tdcf = tdco + 2;
	  if( tdcf > 127 ) tdcf = 127;

	  HAL_FDCAN_ConfigTxDelayCompensation( p_can, tdco, tdcf );
	  HAL_FDCAN_EnableTxDelayCompensation( p_can );
  }
  else {
	  p_can->Init.NominalPrescaler 		= brp;
	  p_can->Init.NominalTimeSeg1 		= tseg1;
	  p_can->Init.NominalTimeSeg2 		= tseg2;
	  p_can->Init.NominalSyncJumpWidth 	= sjw;

	  HAL_FDCAN_DisableTxDelayCompensation( p_can );
  }

  if( HAL_FDCAN_Init( p_can ) != HAL_OK )
  {
    assert( 0 );
  }
}

static void pcan_can_tx_complete( int bus, int mail_box )
{
  ++can_dev_array[bus].tx_msgs;
}
#if 0
static void pcan_can_tx_err( int bus, int mail_box )
{
  ++can_dev_array[bus].tx_errs;
}
#endif
int pcan_can_stats( int bus, struct t_can_stats *p_stats )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  
  p_stats->tx_msgs = p_dev->tx_msgs;
  p_stats->tx_errs = p_dev->tx_errs;
  p_stats->rx_msgs = p_dev->rx_msgs;
  p_stats->rx_errs = p_dev->rx_errs;
  p_stats->rx_ovfs = p_dev->rx_ovfs;

  return sizeof( struct t_can_stats );
}

void pcan_can_poll( void )
{
#if ( CAN_WITHOUT_ISR == 1 )
  HAL_FDCAN_IRQHandler( &hfdcan1 );
  HAL_FDCAN_IRQHandler( &hfdcan2 );
#endif
  
  pcan_can_flush_tx( CAN_BUS_1 );
  pcan_can_flush_tx( CAN_BUS_2 );
}

/* --------------- HAL PART ------------- */
static int _bus_from_int_dev( FDCAN_GlobalTypeDef *can )
{
  if( can == FDCAN1 )
    return CAN_BUS_1;
  else if( can == FDCAN2 )
    return CAN_BUS_2;
  /* abnormal! */
  return CAN_BUS_1;
}

static void pcan_can_isr_frame( FDCAN_HandleTypeDef *hfdcan, uint32_t fifo )
{
  FDCAN_RxHeaderTypeDef hdr;
  const int bus = _bus_from_int_dev( hfdcan->Instance );
  struct t_can_dev * const p_dev = &can_dev_array[bus];
  struct t_can_msg  msg = { 0 };
  
  if( HAL_FDCAN_GetRxMessage( hfdcan, fifo, &hdr, msg.data ) != HAL_OK )
    return;

  msg.id = hdr.Identifier;

  if( hdr.IdType == FDCAN_EXTENDED_ID )
  {
    msg.flags |= MSG_FLAG_EXT;
  }

  if( hdr.RxFrameType != FDCAN_DATA_FRAME )
  {
    msg.flags |= MSG_FLAG_RTR;
  }

  if( hdr.FDFormat == FDCAN_FD_CAN )
  {
    msg.flags |= MSG_FLAG_FD;
  }

  if( hdr.BitRateSwitch == FDCAN_BRS_ON )
  {
    msg.flags |= MSG_FLAG_BRS;
  }

  extern const uint8_t pcan_fd_dlc2len[];
  msg.size = pcan_fd_dlc2len[ (hdr.DataLength >> 16) & 0x0F ];

  msg.timestamp = pcan_timestamp_us();
  
  if( p_dev->rx_isr )
  {
    if( p_dev->rx_isr( bus, &msg ) < 0 )
    {
      ++p_dev->rx_ovfs;
      return;
    }
  }
  ++p_dev->rx_msgs;
}

void HAL_FDCAN_RxFifo0Callback( FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs )
{
	if( RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE ) {
		pcan_can_isr_frame( hfdcan, FDCAN_RX_FIFO0 );
	}

	if( RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST ) {

	}

	if( RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK ) {
		;
	}

	if( RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL ) {
		;
	}
}

void HAL_FDCAN_TxEventFifoCallback( FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs) {
	if( TxEventFifoITs & FDCAN_IT_TX_EVT_FIFO_NEW_DATA ) {
		pcan_can_tx_complete( _bus_from_int_dev( hfdcan->Instance ), 0 );
	}
}
