/* To do:
    - implement i2c receive/request calls.
      - decide if we're going to mutex
*/



#include <Arduino.h>
#include <wiring_private.h>
#include <Wire.h>

//#include "sam.h" // for SAMD51 registers

#include "sercom_funcs.h"
#include "ir_parser.h"
#include "ircomm_i2c.h"

// GPIO mappings

#define PROXA_IN_PIN  PIN_PA02
#define PROXB_IN_PIN  PIN_PB08
#define LDRA_IN_PIN   PIN_PB09
#define LDRB_IN_PIN   PIN_PA06
#define LDRC_IN_PIN   PIN_PA07


// Match up 4 instances of the ir parsrer.
//                          right            back         fwd         left
IRParser_c parser[4] = { port_D12_D13, port_D25_D24, port_D18_D15, port_D1_D0 };


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
ircomm_metrics_t metrics;



float bearing_activity[4];

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
ircomm_config_t config;


volatile uint8_t tx_buf[4][MAX_TX_BUF];
int tx_repeat_count[4];


void dumpSercomCtrla(Sercom* hw) {
  Serial.print("CTRLA = 0x");
  Serial.println(hw->USART.CTRLA.reg, HEX);
}

void sercomInvert(Sercom* hw, bool invertTx, bool invertRx) {
  // Disable USART
  hw->USART.CTRLA.reg &= ~SERCOM_USART_CTRLA_ENABLE;
  while (hw->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_ENABLE) {
  }

  uint32_t ctrla = hw->USART.CTRLA.reg;

  // Clear both inversion bits first
  ctrla &= ~(SERCOM_USART_CTRLA_TXINV | SERCOM_USART_CTRLA_RXINV);

  // SAM D5x/E5x erratum:
  // RXINV actually inverts TX
  // TXINV actually inverts RX
  if (invertTx) {
    ctrla |= SERCOM_USART_CTRLA_RXINV;
  }
  if (invertRx) {
    ctrla |= SERCOM_USART_CTRLA_TXINV;
  }

  hw->USART.CTRLA.reg = ctrla;
  while (hw->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_SWRST) {
  }

  // Re-enable USART
  hw->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
  while (hw->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_ENABLE) {
  }
}


// Reading #define from config.h to give the board a default
// configuration.  All settings can be reconfigured over i2c
void configureFromConfigH() {

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
}


void setup() {

  Serial.begin(115200);
  while (!Serial);
  Serial.println("Reset");

  pinMode( DEMOD1_EN_PIN, OUTPUT);
  pinMode( DEMOD2_EN_PIN, OUTPUT);
  pinMode( DEMOD3_EN_PIN, OUTPUT);
  pinMode( DEMOD4_EN_PIN, OUTPUT);

  // 58 kHz output on D4
  setup58kHz();

  // Clear config and set
  memset( (void*)&config, 0, sizeof( config ));
  
  configureFromConfigH();

  for( int i = 0; i < 4; i++ ) {
    digitalWrite( channel[i].demod_pin, config.rx[i].flags.bits.enabled );
  }

  // TODO: update baud from config
  pinPeripheral(0, PIO_SERCOM);
  pinPeripheral(1,  PIO_SERCOM);
  port_D1_D0.begin(BAUD);

  //  // UART4: MOSI TX / SCK RX -> SERCOM1 (SCK is 24, MOSI is 25)
  pinPeripheral(24, PIO_SERCOM_ALT);
  pinPeripheral(25,  PIO_SERCOM_ALT);
  port_D25_D24.begin(BAUD);

  // UART3: D12 TX / D13 RX -> SERCOM5
  pinPeripheral(12, PIO_SERCOM_ALT);
  pinPeripheral(13, PIO_SERCOM_ALT);
  port_D12_D13.begin(BAUD);

  beginSerialA4A1_manual(BAUD);


  // TODO: make function, going to call this often
  // Clear and Setup initial metrics
  memset( (void*)&metrics, 0, sizeof( metrics));
  setAllByteTimestamps();
  setAllMsgTimestamps();
  setBearingTimestamp();


  // Debugging - setup test message for all channels
  for ( int i = 0; i < 4; i++ ) {
    char msg[32];
    memset( (void*)msg, 0, sizeof( msg ));
    memset( (void*)tx_buf[i], 0, sizeof( tx_buf[i] ));
    sprintf((char*)msg, "paul test %d, %lu", i, micros() );
    config.tx[i].len = parser[i].formatIRMessage( (uint8_t*)tx_buf[i], (uint8_t*)msg, strlen(msg));
  }

  //  Serial.println("Setup complete");
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
  if ( !config.general.flags.bits.bidirectional ) {
    disableDemodulator( which, DemodState::Deactive );
  }

  // Capture when this happened
  metrics.tx_timings.last_ts_ms[which] = millis();

  channel[which].tx_state = TxState::Sending;

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
  while ( bytes > config.tx[which].len && tx_repeat_count[which] > 0 ) {

    // We have enough space, load in the message.
    channel[which].port->write( (uint8_t*)tx_buf[which], config.tx[which].len);

    // Ask the channel for remaining bytes. I think
    // bytes -= len is not necessarily safe.
    bytes = channel[which].port->availableForWrite();

    // Register this repeat
    tx_repeat_count[which]--;

    action = true;
  }

  return action;
}

bool updateTx( int which ) {
  if ( which < 0 || which > 3 ) return false;


  // Already complete? Nothing to do.
  if ( channel[which].tx_state == TxState::Idle ) return true;

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

    // Capture duration
    uint32_t dt = millis() - metrics.tx_timings.last_ts_ms[which];
    metrics.tx_timings.duration_ms[which] = (uint16_t)dt;

    if ( !config.general.flags.bits.bidirectional ) {
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


static unsigned long test_tx;


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
    uint32_t byte_timeout_ms = config.rx[i].timeout_multi;
    parser_status_t parser_status = parser[i].getNextByte( byte_timeout_ms );



    // Log any activity
    if ( parser_status.bytes > 0 ) {

      // If we got a byte, move the timestamp forwards to stop
      // triggering a desaturation.  Also used if tx is set to
      // defer (to activity on receiver)
      setByteTimestamp( i );

      // Continuous log of activity
      metrics.activity.rx[i]++;

      // Cyclical log, used to estimate bearing
      // to neighbours
      // increment bearing activity
      bearing_activity[i] += 1.0;

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
      Serial.print("Port "); Serial.print(i);
      Serial.print(" Got message: ");
      Serial.println( (char*)parser[i].msg);

      calcMsgDeltaTime(i);
      setMsgTimestamp(i);

      metrics.crc.pass[i]++;

      // TODO: transfer message, ready for i2c request
      // and so it isn't over-written by the parser with
      // the next full message received

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

void handleBearingEstimation() {

  // At a much slower rate, update the bearing estimate.
  if ( calcBearingDeltaTime() > config.general.bearing_update_us ) {
    setBearingTimestamp();
    updateBearing();
  }
}

void handleTxBroadcast() {

  if ( config.tx[0].base_ms == 0 ) return;

  if ( config.tx[0].len == 0 ) return;

  // If defer_multi is set, then any activity on
  // a receiver within the threshold will cancel
  // the send process.
  for ( int i = 0; i < 4; i++ ) {
    if ( recentByteActivity(i) ) return;

    // Abort a transmit if any receiver is 
    // configured to overrun and is currently
    // receiving a message
    if ( config.rx[i].flags.bits.overrun ) {
      if ( parser[i].isDecoding() ) return;
    }
  }

  // Check if it is time to transmit.
  uint32_t dt_ms;
  dt_ms = millis() - metrics.tx_timings.last_ts_ms[0];

  // Time to send?
  if ( dt_ms > config.tx[0].interval_ms ) {

    // ensure that all channels are duplicates of 0
    // TODO: a bit expensive?
    for ( int i = 1; i < 3; i++ ) {
      memcpy( &config.tx[i], &config.tx[0], sizeof( config.tx[0] ));
      memcpy( (void*)tx_buf[i], (void*)tx_buf[0], sizeof( tx_buf[0] ));
    }

    // configure next interval from channel 0
    uint32_t new_interval = getNewTxInterval(0);

    // Start messaging across all channels
    for ( int i = 0; i < 4; i++ ) {

      // start the send process, this will also
      // set things up to obstruct another call to this
      // function.
      triggerTx(i);

      config.tx[i].interval_ms = new_interval;
    }
  }
}

bool recentByteActivity( int which ) {
  if ( which < 0 || which > 3 ) return false;

  uint32_t dt = calcByteDeltaTime( which );

  // scale for microseconds
  uint32_t threshold = (uint32_t)config.tx[which].defer_multi;
  threshold *= 1000;
  if ( dt < threshold ) return true;

  return false;
}

void handleTx( int which ) {

  if ( which < 0 || which > 3 ) return;

  if ( config.tx[which].base_ms == 0 ) return;

  if ( config.tx[which].len == 0 ) return;

  if ( recentByteActivity(which) ) return;

  if ( config.rx[which].flags.bits.overrun ) {
    if ( parser[which].isDecoding() ) return;
  }

  // Check if it is time to transmit.
  uint32_t dt_ms;
  dt_ms = millis() - metrics.tx_timings.last_ts_ms[which];

  if ( dt_ms > config.tx[which].interval_ms ) {

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
  if ( config.tx[which].predict_multi > 0 ) {
    interval_ms = config.tx[which].len * config.tx[which].predict_multi;
  }

  if ( config.tx[which].interval_mod > 0 ) {
    float percent_mod = (float)config.tx[which].interval_mod;
    percent_mod /= 100.0;
    percent_mod *= (float)interval_ms;
    percent_mod = (float)random( -percent_mod, percent_mod);
    interval_ms += percent_mod;
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
      if ( channel[i].tx_state == TxState::Sending ) return;
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

void loop() {

  handleMsgParsing();
  handleDemodulatorSaturation();
  handleBearingEstimation();
  handleTransmit();

}
