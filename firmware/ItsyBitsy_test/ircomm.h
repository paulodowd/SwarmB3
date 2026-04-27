
#ifndef IRCOMM_H
#define IRCOMM_H

#include "ircomm_i2c.h"
#include "sercom_funcs.h"
#include "ir_parser.h"

typedef struct {
  ir_crc_t          crc;
  ir_activity_t     activity;
  ir_saturation_t   saturation;
  ir_errors_t       errors;
  ir_frame_errors_t frame_errors; // this one needs integrating with channel[]
  ir_msg_timings_t  msg_timings;
  ir_byte_timings_t byte_timings;
  ir_vectors_t      vectors;
  ir_bearing_t      bearing;
  ir_sensors_t      sensors;
  ir_tx_timings_t   tx_timings;
} ircomm_metrics_t;

// On this new board, each rx demodulator and
// pair of IR LEDs are attached to independent
// uart interfaces. I think that means that
// they could be set up to transmit different
// length messages, and they could each be
// receiving different length messages.
// Therefore, I think the easiest way to
// represent this in the overall config is
// to make the tx and rx structs into
// arrays, 1 for each uart.
// We will still need a top-most level config
// to decide if the board is going to use all
// tx independently, or in broadcast, etc
typedef struct {
  ir_params_t     general;
  ir_tx_params_t  tx[4];          // 11 bytes
  ir_rx_params_t  rx[4];          // 20 bytes
} ircomm_config_t;


extern ircomm_config_t config;
extern ircomm_metrics_t metrics;
extern IRParser_c parser[4];
extern float bearing_activity[4];
extern uint32_t tx_repeat_count[4];
extern volatile uint8_t tx_buf[4][MAX_TX_BUF];

void resetMetrics();
void setBearingTimestamp();
uint32_t calcBearingDeltaTime();
void zeroBearingActivity();
void setAllMsgTimestamps();
void setMsgTimestamp( int which );
uint32_t calcMsgDeltaTime( int which );
void setAllByteTimestamps();
void setByteTimestamp( int which );
uint32_t calcByteDeltaTime( int which );
void calcAllByteDeltaTime();
void triggerTx( int which );
bool attemptMsgWriteToSerialBuffer( int which );
bool updateTx( int which );
void resetRxBuffers( int which );
void disableDemodulator( int which, DemodState d_state );
void enableDemodulator( int which );
bool triggerDemodDesaturation(int which);
bool updateDemodDesaturation( int which );
void updateBearing();
void handleMsgParsing();
void handleDemodulatorSaturation();
void handleBearingEstimation();
void handleTxBroadcast();
bool recentByteActivity( int which );
void handleTx( int which );
uint32_t getNewTxInterval( int which );
void handleTransmit();

#endif
