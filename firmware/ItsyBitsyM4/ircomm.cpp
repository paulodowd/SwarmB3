#include "ircomm.h"

// Match up 4 instances of the ir parsrer.
//                          right            back         fwd         left
IRParser_c parser[4] = { port_D12_D13, port_D25_D24, port_D18_D15, port_D1_D0 };
float bearing_activity[4];
uint32_t tx_repeat_count[4];


void resetMetrics() {
  memset( (ir_metrics_t*)&metrics, 0, sizeof( ir_metrics_t ));
  resetAllFrameErrorCounts();
  setAllByteTimestamps();
  setAllMsgTimestamps();
  setBearingTimestamp();
  i2cClearStatusBits();
}

// Reading #define from config.h to give the board a default
// configuration.  All settings can be reconfigured over i2c
void configureFromConfigH() {

  memset( (ir_config_t*)&config, 0, sizeof( ir_config_t ));

  // Top level, general config
  config.general.baud                     = BAUD;
  config.general.flags.bits.broadcast     = BROADCAST;
  config.general.flags.bits.bidirectional = BIDIRECTIONAL;
  config.general.bearing_update_us        = BEARING_UPDATE_US;
  config.general.bearing_alpha            = BEARING_ALPHA;
  config.general.preamble_byte            = TX_PREAMBLE_BYTE;

  // Config per receiver/uart
  for ( int i = 0; i < 4; i++ ) {
    config.tx[i].interval_mod       = TX_INTERVAL_MOD;
    config.tx[i].repeat             = TX_REPEAT;
    config.tx[i].predict_multi      = TX_PREDICT_MULTI;
    config.tx[i].defer_multi        = TX_DEFER_MULTI;
    config.tx[i].preamble_repeat    = TX_PREAMBLE_REPEAT;
    config.tx[i].interval_ms        = TX_INTERVAL_MS;
    config.tx[i].base_ms            = TX_BASE_MS;
    config.tx[i].len                = TX_LEN;

    config.rx[i].flags.bits.overrun = RX_OVERRUN;
    config.rx[i].flags.bits.enabled = RX_ENABLED;
    config.rx[i].timeout_multi      = RX_TIMEOUT_MULTI;
    config.rx[i].saturation_us      = RX_SATURATION_US;
    config.rx[i].desaturation_us    = RX_DESATURATION_US;
  }

  // Enable demodulators as specified
  // in config.h
  for ( int i = 0; i < 4; i++ ) {
    digitalWrite( channel[i].demod_pin, config.rx[i].flags.bits.enabled == 1 ? HIGH : LOW );
  }

}

void fullReset() {
  memset( (ir_config_t*)&config, 0, sizeof( ir_config_t ));
  configureFromConfigH();
  resetMetrics();
  i2cInitState();
}



void setBearingTimestamp() {
  metrics.bearing.us_ts = micros();
}
uint32_t calcBearingDeltaTime() {
  return micros() - metrics.bearing.us_ts;
}

void zeroBearingActivity() {
  for ( int i = 0; i < 4; i++ ) bearing_activity[i] = 0.0;
}
void setAllMsgTimestamps() {
  for ( int i = 0; i < 4; i++ ) {
    metrics.msg_timings.ts_ms[i] = millis();
  }
}
void setMsgTimestamp( int which ) {
  if ( which < 0 || which > 3 ) return;
  metrics.msg_timings.ts_ms[which] = millis();
}
uint32_t calcMsgDeltaTime( int which ) {
  if ( which < 0 || which > 3 ) return 0;
  metrics.msg_timings.dt_ms[which] = millis() - metrics.msg_timings.ts_ms[which];
  return metrics.msg_timings.dt_ms[which];
}

void setAllByteTimestamps() {
  for ( int i = 0; i < 4; i++ ) {
    metrics.byte_timings.ts_us[i] = micros();
  }
}
void setByteTimestamp( int which ) {
  if ( which < 0 || which > 3 ) return;
  metrics.byte_timings.ts_us[ which ] = micros();
}
uint32_t calcByteDeltaTime( int which ) {
  if ( which < 0 || which > 3 ) return 0;
  metrics.byte_timings.dt_us[ which ] = micros() - metrics.byte_timings.ts_us[which];
  return metrics.byte_timings.dt_us[which];
}
void calcAllByteDeltaTime() {
  for ( int i = 0; i < 4; i++ ) {
    metrics.byte_timings.dt_us[i] = micros() - metrics.byte_timings.ts_us[i];
  }
}

void triggerTx( int which ) {
  if ( which < 0 || which > 3 ) return;

  // TODO: add check to config for whether this happens
  if ( config.general.flags.bits.bidirectional == 0 ) {
    //    Serial.println("Deactivate!");
    disableDemodulator( which, DemodState::Deactive );
  }

  // Capture when this happened
  metrics.tx_timings.last_ts_ms[which] = millis();

  channel[which].tx_state = TxState::Sending;

  // clear csma multiplier
  config.tx[which].csma_multi = 0;


  // First, if the user has set a preamble we load this
  // into the arduino serial buffer.  For this device
  // (ItsyBitsy M4) I've verified that 349 byte are
  // available, and tx[].repeat is a uint8_t (max 255).
  // We can only call triggerTx() when the Serial was
  // previously completed (buffer empty).
  for ( int i = 0; i < config.tx[which].preamble_repeat; i++ ) {
    channel[which].port->write( (uint8_t)config.general.preamble_byte );
  }

  // Setup our required number of repeats transmissions.
  // It is possible for the user to specify UINT32_MAX
  // number of repeats, which we can't hold in a byte array.
  // So instead we have to iteratively reload the Serial
  // buffer.  We also don't want to get held up filling
  // the buffer with Serial.write(), which will block until
  // the data is loaded into the buffer.
  tx_repeat_count[which] = config.tx[which].repeat;

  attemptMsgWriteToSerialBuffer( which );
  
  // We may still have repeated transmissions to make
  // but these will be handled by the non-blocking
  // updateTx function.

}


// If this function is called with tx_repeat_count[]
// as zero, it will do nothing.  Later, updateTx will
// catch the circumstance and revert the channel
// TxState to Idle *after* any preamble bytes are
// sent.  This means that the user could configure
// the board to only send preamble bytes, and no
// messages. This could be useful to simply configure
// a board to generate interference or bytes.
bool attemptMsgWriteToSerialBuffer( int which ) {
  if ( which < 0 || which > 3 ) return false;

  bool action;

  // Assume no action
  action = false;

  // Next: check if there is still space to add in the
  // current message stored in tx_buf via config.tx[].len
  int bytes = channel[which].port->availableForWrite();

  // This was origionally while(), but I found that filling the UART
  // buffer for each SERCOM was actually quite time expensive.
  // I checked the general full board update time and the slowest was
  // about 24us.  Therefore, I think writing just 1 copy of the 
  // message into the buffer is fine, because we'll get around quick
  // enough to fill it up again.
  if ( (bytes > config.tx[which].len) && (tx_repeat_count[which] > 0 ) ) {

    // We have enough space, load in the message.
    channel[which].port->write( (uint8_t*)config.tx_buf[which], config.tx[which].len);

    // Ask the channel for remaining bytes. I think
    // bytes -= len is not necessarily safe.
    bytes = channel[which].port->availableForWrite();

    // Register this repeat
    tx_repeat_count[which]--;

    action = true;
  }
  
  return action;
}

static unsigned long last_t;
bool updateTx( int which ) {
  if ( which < 0 || which > 3 ) return false;


  // Already complete? Nothing to do.
  if ( channel[which].tx_state != TxState::Sending ) {
    return true;
  }

  // If we need to progress a repeated transmission that
  // didn't fit into the Serial buffer before
  if ( tx_repeat_count[which] > 0 ) {

    attemptMsgWriteToSerialBuffer( which );

    // If we loaded in more bytes, then tx is still
    // not complete.
    return false;
  }
  


  // If here, we're not attempting to load in more
  // message.  We check if tx is finished.
  if ( isUartTxComplete(channel[which].hw ) ) {

    // Set tx flag back to idle
    channel[which].tx_state = TxState::Idle;

    // Since we finished a transmission, we log
    // this instance in the count
    metrics.tx_counts.sent[which]++;

    // Capture duration
    uint32_t dt = millis() - metrics.tx_timings.last_ts_ms[which];
    metrics.tx_timings.duration_ms[which] = (uint16_t)dt;

    if ( config.general.flags.bits.bidirectional == 0 ) {
      //      Serial.println("activate!");
      enableDemodulator( which );
    }

    return true;
  }


  return false;
}

void resetRxBuffers( int which ) {
  if ( which < 0 || which > 3 ) return;

  // clear hw buffer
  clearSercomRxBuffer( channel[which].hw );

  // Clear arduino object buffer
  while ( channel[which].port->available() ) channel[which].port->read();

}

void disableDemodulator( int which, DemodState d_state ) {
  if ( which < 0 || which > 3 ) return;

  channel[which].demod_state = d_state;
  channel[which].demod_ms_ts = micros();
  digitalWrite( channel[which].demod_pin, LOW); // switch off demod
}

void enableDemodulator( int which ) {
  if ( which < 0 || which > 3 ) return;

  // Clear out junk or anything old
  resetRxBuffers( which );

  // renable demodulator
  channel[which].demod_state = DemodState::Active;
  digitalWrite( channel[which].demod_pin, HIGH );
  
}

bool triggerDemodDesaturation(int which) {
  if ( which < 0 || which > 3 ) return false;

  // If saturation time value is 0, disable desat
  if ( config.rx[which].saturation_us == 0 ) return false;

  if ( config.rx[which].desaturation_us == 0 ) return false;

  // Avoid triggering if the rx demodulator is
  // already deactive (either from a desautration
  // or whilst transmitting)
  if ( channel[which].demod_state == DemodState::Deactive ) return false;

  disableDemodulator(which, DemodState::Desaturating );

  return true;
}

bool updateDemodDesaturation( int which ) {
  if ( which < 0 || which > 3 ) return false;

  if ( channel[which].demod_state != DemodState::Desaturating ) return false;

  uint32_t dt_us = micros() - channel[which].demod_ms_ts;
  if ( dt_us > (uint32_t)config.rx[which].desaturation_us ) {

    enableDemodulator( which );

    // Advance the byte timestamp so that we don't
    // trigger another desaturation in the next
    // iteration
    setByteTimestamp( which );
    
    return true;
  }
  return false;
}

// Small wrapper for how to update the recorded activity
// level by some decay rate.
void updateBearing() {


  // We know that the baud rate is 9600, and there
  // are 10 bits per byte on UART (+start & stop bits)
  // Therefore, we expect 960 bytes per second, or
  // 96 bytes per 100ms
  const float bytes_per_us = 960.0 / 1000000.0;
  const float max_bytes = bytes_per_us * config.general.bearing_update_us;


  const float alpha = 0.25;
  metrics.bearing.sum  = 0.0;
  for ( int i = 0; i < 4; i++ ) {

    bearing_activity[i] /= max_bytes;
    metrics.bearing.sum += bearing_activity[i];

    // Normalising and filtering
    metrics.vectors.rx[i] = (metrics.vectors.rx[i] * (1.0 - alpha) ) + ((bearing_activity[i]) * alpha);

  }


  // Update bearing estimate.
  float x = (metrics.vectors.rx[0] - metrics.vectors.rx[2]);
  float y = (metrics.vectors.rx[3] - metrics.vectors.rx[1]);
  metrics.bearing.theta = atan2( y, x );
  metrics.bearing.mag = sqrt( pow(x, 2) + pow(y, 2));


  // We zero activity, because vectors are implemented
  // with decay.
  zeroBearingActivity();
}

void handleMsgParsing() {

  
  // First, check for new bytes on each of the 4 receivers,
  // logging any metrics/errors and handling a complete message.
  for ( int i = 0; i < 4; i++ ) {

    // Skip if the demodulator is deactive because of Tx or
    // desaturation occuring
    if ( channel[i].demod_state == DemodState::Deactive ) continue;

    // Skip if the user has set to deactive in the config
    // TODO: once i2c is implemented, I think this will be
    // redundant. (?)
    if ( config.rx[i].flags.bits.enabled == false ) continue;


    // timeout_multi is 0:255, and we generally get bytes
    // every 1ms.
    uint32_t byte_timeout_us = (uint32_t)config.rx[i].timeout_multi;
    byte_timeout_us *= 100;
    parser_status_t parser_status = parser[i].getNextByte( byte_timeout_us );


    // Log any activity
    if ( parser_status.bytes > 0 ) {

      //      Serial.println(i);

      // If we got a byte, move the timestamp forwards to stop
      // triggering a desaturation.  Also used if tx is set to
      // defer (to activity on receiver)
      setByteTimestamp( i );

      // Continuous log of activity
      metrics.activity.rx[i]++;

      i2cSetRxActivityBit(i);

      // Cyclical log, used to estimate bearing
      // to neighbours
      // increment bearing activity
      bearing_activity[i] += 1.0;

    } else {
      i2cClearRxActivityBit(i);
    }

    // Log any errors
    if ( parser_status.error != NO_ERROR ) {
      metrics.errors.type[i][ parser_status.error ]++;
    }

    // Duplicated logging of this error - fix later?
    if ( parser_status.error == ERR_BAD_CRC ) metrics.crc.fail[i]++;

    // Decide what to do with a message
    // Note, we will always get a return of 1 byte unless
    // it is a CRC-pass message.
    if ( parser_status.bytes > 1 ) {


      // Debug
      //                  Serial.print("Port "); Serial.print(i);
//                        Serial.print(" Got message: ");
//                        Serial.print( (char*)parser[i].msg);
//                        Serial.print(" ");Serial.println( parser[i].msg_len );
      config.msg_len[i] = parser[i].msg_len;
      parser[i].copyMsg( (uint8_t*)config.msg[i] );
      i2cSetMsgStatusBit( i );

      calcMsgDeltaTime(i);
      setMsgTimestamp(i);

      metrics.crc.pass[i]++;

    }
  }
}



void handleDemodulatorSaturation() {
  // Checking for demodulator saturation, calling desaturation
  // Only valid if the demodulator is active in the first place.
  // Demodulator could be disabled by an on-going transmit
  for ( int i = 0; i < 4; i++ ) {

    if ( channel[i].demod_state == DemodState::Deactive ) {
      continue;

    } else if ( channel[i].demod_state == DemodState::Desaturating ) {
      updateDemodDesaturation( i );

    } else if ( channel[i].demod_state == DemodState::Active ) {

      // If there has been no activity for some time, the
      // demodulator is probably saturated (gain at max) so
      // we trigger a desaturation.  Otherwise, just check
      // whether the demodulator needs reactivating
      if ( calcByteDeltaTime(i) > (uint32_t)config.rx[i].saturation_us ) { // 30ms

        triggerDemodDesaturation( i );
        metrics.saturation.rx[i]++;
      }
    }
  }
}

void printStatus() {

  Serial.print("M[0:3], A[4:7]: ");

  for ( int i = 0; i < 8; i++ ) {
    if ( config.status & (1 << i) ) {
      Serial.print("1 ");
    } else {
      Serial.print("0 ");
    }
  }
  Serial.println();
}

void handleBearingEstimation() {

  // At a much slower rate, update the bearing estimate.
  if ( calcBearingDeltaTime() > config.general.bearing_update_us ) {
    setBearingTimestamp();
    updateBearing();
  }
}


uint32_t getTransmissionLength( int which ) {

  if ( which < 0 || which > 3 ) return 0;

  uint32_t total_bytes;
  total_bytes = (uint32_t)config.tx[which].len;
  total_bytes *= (uint32_t)config.tx[which].repeat;
  total_bytes += (uint32_t)config.tx[which].preamble_repeat;
  return total_bytes;
}

// For CSMA/CA back-off, we extend the current tx interval
void extendTxIntervalCSMA(int which) {
  if ( which < 0 || which > 3 ) return;

  uint32_t extension_ms;
  extension_ms = getTransmissionLength(which);
  extension_ms *= (uint32_t)config.tx[which].csma_multi;
  extension_ms = (uint32_t)random(0, extension_ms);
  config.tx[which].interval_ms += extension_ms;

  return;

}


void handleTxBroadcast() {

  if ( config.tx[0].base_ms == 0 ) return;

  if ( config.tx[0].len == 0 ) return;



  // Check if it is time to transmit.
  uint32_t dt_ms;
  dt_ms = millis() - metrics.tx_timings.last_ts_ms[0];

  // Time to send?
  if ( dt_ms > config.tx[0].interval_ms ) {

    // If defer_multi is set, then any activity on
    // a receiver within the threshold will cancel
    // the send process.
    bool defer = false;
    for ( int i = 0; i < 4; i++ ) {
      if ( recentByteActivity(i) ) {
        defer = true;
      }
    }
    
    if ( defer ) {
      // Use tx state to count only the first
      // occuring instance of deferring
      if ( channel[0].tx_state == TxState::Idle ) {
        metrics.tx_counts.deferred[0]++;
        metrics.tx_counts.deferred[1]++;
        metrics.tx_counts.deferred[2]++;
        metrics.tx_counts.deferred[3]++;
        channel[0].tx_state = TxState::Deferred;

        config.tx[0].csma_multi = 1;
        
        // previously deffered.
      } else if ( channel[0].tx_state  == TxState::Deferred ) {

        // exponentially increase the potential
        // csma back-off
        if ( !(config.tx[0].csma_multi & 0b10000000) ) {
          config.tx[0].csma_multi = config.tx[0].csma_multi << 1;
        }
      }

      extendTxIntervalCSMA(0);
      return;
    } 



    // Abort a transmit if any receiver is
    // configured to overrun and is currently
    // receiving a message
    for ( int i = 0; i < 4; i++ ) {

      if ( config.rx[i].flags.bits.overrun ) {
        if ( parser[i].isDecoding() ) {

          // Paul: 10/05/26: I don't think we should log
          //       an overrun as deferred.
          // Use tx state to count only the first
          // occuring instance of deferring
          //          if ( channel[0].tx_state == TxState::Idle ) {
          //            metrics.tx_counts.deferred[0]++;
          //            metrics.tx_counts.deferred[1]++;
          //            metrics.tx_counts.deferred[2]++;
          //            metrics.tx_counts.deferred[3]++;
          //            channel[0].tx_state = TxState::Deferred;
          //          }

          return;
        }
      }
    }

    // ensure that all channels are duplicates of tx 0
    // I feel like this is a little expensive, but when I 
    // measure the elapsed time it takes 12microseconds, 
    // which shouldn't impact the overall performance of 
    // the board. 
    for ( int i = 1; i < 4; i++ ) {
      memcpy( (void*)&config.tx[i], (void*)&config.tx[0], sizeof( config.tx[0] ));
      memcpy( (void*)config.tx_buf[i], (void*)config.tx_buf[0], sizeof( config.tx_buf[0] ));
    }
    



    // TODO: I think the below is a bit risky because I'm
    // disabling the SERCOM tx beneath the arduino serial
    // abstraction, and I'm not quite sure how that will 
    // behave.  I've made my best attempt to ensure that
    // buffers are empty - but I wonder if the 1 byte 
    // shift register in the SERCOM is going to cause 
    // unpredictable blocking.  I've tested this in reality
    // and it seems to remove the frame errors between 
    // the boundary of pairs of transmitting IR LEDs.
    // I'll come back to this later.
    
    // configure next interval from channel 0
    uint32_t new_interval = getNewTxInterval(0);

    // Make sure that any old transmit has 
//    // fully finished
//    for( int i = 0; i < 4; i++ ) {
//      while( isUartTxComplete( channel[i].hw ) == false ){
////        Serial.printf("Waiting for Tx%d to clear\n", i );
//      }
//    }
    
    // Disable all transmitters, so that we can
    // start transmission at the same time as
    // nearly as possible
    for( int i = 0; i < 4; i++ ) disableSercomTx( channel[i].hw );

    // Load buffers
    for ( int i = 0; i < 4; i++ ) {
      // start the send process, this will also
      // set things up to obstruct another call to this
      // function.
      triggerTx(i);
      config.tx[i].interval_ms = new_interval;
    }

    // Renable transmit
    for( int i = 0; i < 4; i++ ) enableSercomTx( channel[i].hw );

    // Wait for sync
    for( int i = 0; i < 4; i++ ) while( !hasSercomSynchronised( channel[i].hw ) );

  }
  
}


void printTxSettings( int which ) {
  Serial.print("Tx "); Serial.print( which ); Serial.println("Settings:");
  Serial.print(" - Repeat: "); Serial.println( config.tx[which].repeat );
  Serial.print(" - P Multi: "); Serial.println( config.tx[which].predict_multi );
  Serial.print(" - D Multi: "); Serial.println( config.tx[which].defer_multi );
  Serial.print(" - P Repeat: "); Serial.println( config.tx[which].preamble_repeat );
  Serial.print(" - Interval ms: "); Serial.println( config.tx[which].interval_ms );
  Serial.print(" - Base ms: "); Serial.println( config.tx[which].base_ms );
  Serial.print(" - Interval mod: "); Serial.println( config.tx[which].interval_mod);
  Serial.print(" - CSMA Multi: "); Serial.println( config.tx[which].csma_multi );
  Serial.print(" - Len: "); Serial.println( config.tx[which].len );
}


bool recentByteActivity( int which ) {
  if ( which < 0 || which > 3 ) return false;

  uint32_t dt = calcByteDeltaTime( which );

  uint32_t threshold = (uint32_t)config.tx[which].defer_multi;

  // scale for microseconds
  threshold *= 1000;

  // If defer_multi is set to 0, then this
  // will always be false disabling the
  // defer functionality
  return ( dt < threshold );
}

void printDemodStatus() {
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( channel[i].demod_state == DemodState::Active ? "1" : "0");
  }
  Serial.println();
}

void handleTx( int which ) {

  if ( which < 0 || which > 3 ) return;

  if ( config.tx[which].base_ms == 0 ) return;

  if ( config.tx[which].len == 0 ) return;



  // Check if it is time to transmit.
  uint32_t dt_ms;
  dt_ms = millis() - metrics.tx_timings.last_ts_ms[which];

  if ( dt_ms > config.tx[which].interval_ms ) {

    // if defer_multi > 0, performs check
    if ( recentByteActivity(which) ) {
      if ( channel[which].tx_state == TxState::Idle ) {
        metrics.tx_counts.deferred[which]++;
        channel[which].tx_state = TxState::Deferred;
        config.tx[which].csma_multi = 1;


        // previously deffered.
      } else if ( channel[which].tx_state  == TxState::Deferred ) {

        // exponentially increase the potential
        // csma back-off
        if ( !(config.tx[which].csma_multi & 0b10000000) ) {
          config.tx[which].csma_multi = config.tx[which].csma_multi << 1;
        }
      }

      extendTxIntervalCSMA(which);

      return;
    }


    if ( config.rx[which].flags.bits.overrun ) {
      if ( parser[which].isDecoding() ) {
        //        if ( channel[which].tx_state == TxState::Idle ) {
        //          metrics.tx_counts.deferred[which]++;
        //          channel[which].tx_state = TxState::Deferred;
        //        }
        return;
      }
    }

    // start the send process, this will also
    // set things up to obstruct another call to this
    // function.
    triggerTx(which);

    // Update the timing interval_ms for
    // the next transmit operation
    config.tx[which].interval_ms = getNewTxInterval( which );
  }
}


// Returns an new timing interval from a config specified
// by which.
uint32_t getNewTxInterval( int which ) {

  if ( which < 0 || which > 3 ) return 0;


  // Assume we use the base value from config
  uint32_t interval_ms = config.tx[which].base_ms;

  // Conveniently, at 9600 baud, 1 byte is very nearly
  // 1ms.
  // We take into account how many repeat transmissions
  // and preamble bytes are occuring to try to predict the
  // tx temporal spacing
  // interval
  if ( config.tx[which].predict_multi > 0 ) {
    uint32_t total_bytes;
    total_bytes = getTransmissionLength(which);

    // scale by requested config
    total_bytes *= (uint32_t)config.tx[which].predict_multi;
    interval_ms = total_bytes;
  }

  if ( config.tx[which].interval_mod > 0 ) {
    float percent_mod = (float)config.tx[which].interval_mod;
    percent_mod /= 100.0;
    percent_mod *= (float)interval_ms;
    percent_mod = (float)random( -percent_mod, percent_mod);
    interval_ms += (uint32_t)percent_mod;
  }

  return interval_ms;
}



void handleTransmit() {

  // First, simply update all channel sending
  // states, which in most cases means doing
  // nothing
  for ( int i = 0; i < 4; i++ ) {
    updateTx(i);
  }

  // First, are we in broadcast mode or not?
  // The only difference is to ensure that the
  // timing information for broadcast the timing
  // information across channels is shared.
  if ( config.general.flags.bits.broadcast ) {

    // If we're in the middle of a send on any
    // channel, abort
    for ( int i = 0; i < 4; i++ ) {
      if ( channel[i].tx_state == TxState::Sending ) {
        return;
      }
    }
    
    // Else, hand over this operation
    handleTxBroadcast();

  } else {

    for ( int i = 0; i < 4; i++ ) {

      // If currently sending, avoid sending
      if ( channel[i].tx_state == TxState::Sending ) continue;

      // else, hand over this process
      handleTx(i);
    }
  }
}

void handleI2cFlags() {

  if ( i2c_flag_reset_metrics ) {
    i2c_flag_reset_metrics = false;
    resetMetrics();
  }

  if ( i2c_flag_full_reset ) {
    i2c_flag_full_reset = false;
    fullReset();
  }


  if ( i2c_flag_set_tx_0 ) {
    config.tx[0].len = parser[0].formatIRMessage( (uint8_t*)config.tx_buf[0], (uint8_t*)i2c_buf[0], i2c_tx_len[0] );
    config.tx[0].interval_ms = getNewTxInterval(0);
    i2c_flag_set_tx_0 = false;
    i2c_tx_len[0] = 0;

  }

  if ( i2c_flag_set_tx_1 ) {
    config.tx[1].len = parser[1].formatIRMessage( (uint8_t*)config.tx_buf[1], (uint8_t*)i2c_buf[1], i2c_tx_len[1] );
    config.tx[1].interval_ms = getNewTxInterval(1);
    i2c_flag_set_tx_1 = false;
    i2c_tx_len[1] = 0;
  }

  if ( i2c_flag_set_tx_2 ) {
    config.tx[2].len = parser[2].formatIRMessage( (uint8_t*)config.tx_buf[2], (uint8_t*)i2c_buf[2], i2c_tx_len[2] );
    config.tx[2].interval_ms = getNewTxInterval(2);
    i2c_flag_set_tx_2 = false;
    i2c_tx_len[2] = 0;

  }

  if ( i2c_flag_set_tx_3 ) {

    config.tx[3].len = parser[3].formatIRMessage( (uint8_t*)config.tx_buf[3], (uint8_t*)i2c_buf[3], i2c_tx_len[3] );
    config.tx[3].interval_ms = getNewTxInterval(3);
    i2c_flag_set_tx_3 = false;
    i2c_tx_len[3] = 0;
  }

  if ( i2c_flag_set_tx_all ) {

    for ( int i = 0; i < 4; i++ ) {
      config.tx[i].len = parser[i].formatIRMessage( (uint8_t*)config.tx_buf[i], (uint8_t*)i2c_buf[0], i2c_tx_len[0] );
      config.tx[i].interval_ms = getNewTxInterval(i);
    }
    i2c_flag_set_tx_all = false;
    i2c_tx_len[0] = 0;
  }

}
