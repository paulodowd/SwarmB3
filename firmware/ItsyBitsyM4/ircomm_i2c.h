
/*
   This file contains the data structures and flags
   used to transfer data over i2c.

   The most important thing is that the maximum
   amount that can be transferred in a single
   transaction is 32bytes.

   i2c interaction with this board requires that first
   the correct mode is set by transferring the
   ir_mode struct, then making the corresponding
   request with the correct receiving struct.
*/

#ifndef IRCOMM_IR_H
#define IRCOMM_IR_H

#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include "ircomm_i2c_datatypes.h"
#include "config.h"

extern volatile ir_metrics_t metrics;
extern volatile ir_config_t  config;

extern volatile bool i2c_flag_reset_metrics;
extern volatile bool i2c_flag_full_reset;
extern volatile bool i2c_flag_set_tx_0;
extern volatile bool i2c_flag_set_tx_1;
extern volatile bool i2c_flag_set_tx_2;
extern volatile bool i2c_flag_set_tx_3;
extern volatile bool i2c_flag_set_tx_all;
extern volatile uint8_t i2c_buf[4][MAX_MSG];
extern volatile uint8_t i2c_tx_len[4];

void i2cSetMsgStatusBit( int which );
void i2cClearRxActivityBits();
void i2cClearStatusBits();
void i2cClearMsgStatusBit( int which );
void i2cSetRxActivityBit( int which );
void i2cClearRxActivityBit( int which );
void i2cUpdateFrameErrors();
void i2cInitState();
void i2c_receive(int len);
void i2c_request();



#endif
