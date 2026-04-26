/* To do:
    - implement the tx process.  We've tested tx working.
     - check config for whether it is broadcast or not.
      - if yes, use channel[0].
      - if not, for loop -> config.tx[]
     - implementing timing/scheduling of tx
      - check config for whether this is
        - randomised (dysnc)
        - predicted or fixed
        - set to 0! (off)
     - implement a check on the saturation to decide
       if saturation is wanted by the user.
    - check through other config params for requirements.
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
    config.tx[i].flags.bits.desync  = TX_DESYNC;
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

  // TODO: set after loading in config
  // Start with all receivers active
  digitalWrite( DEMOD1_EN_PIN, HIGH);
  digitalWrite( DEMOD2_EN_PIN, HIGH);
  digitalWrite( DEMOD3_EN_PIN, HIGH);
  digitalWrite( DEMOD4_EN_PIN, HIGH);

  // 58 kHz output on D4
  setup58kHz();

  // Clear config and set
  memset( (void*)&config, 0, sizeof( config ));
  configureFromConfigH();

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


  // Debugging
  for ( int i = 0; i < 4; i++ ) {
    char msg[32];
    memset( (void*)msg, 0, sizeof( msg ));
    memset( (void*)tx_buf[i], 0, sizeof( tx_buf[i] ));
    sprintf((char*)msg, "paul test %d", i );
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
  disableDemodulator( which, DemodState::Deactive );

  // Capture when this happened
  metrics.tx_timings.last_ts_ms[which] = millis();

  channel[which].tx_state = TxState::Sending;

  // TODO: build in a config check for whether we are 
  // using preamble bytes, and how many repeated 
  // message tranmissions we are making.

  channel[which].port->write( (uint8_t*)tx_buf[which], config.tx[which].len );

}

bool updateTx( int which ) {
  if ( which < 0 || which > 3 ) return false;


  // Already complete? Nothing to do.
  if ( channel[which].tx_state == TxState::Idle ) return true;

  if ( isUartTxComplete(channel[which].hw ) ) {

    // Set tx flag back to idle
    channel[which].tx_state = TxState::Idle;

    // Capture duration
    uint32_t dt = millis() - metrics.tx_timings.last_ts_ms[which];
    metrics.tx_timings.duration_ms[which] = (uint16_t)dt;

    // TODO: check config for whether this is happening
    enableDemodulator( which );

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
    if( config.rx[i].flags.bits.enabled == false ) continue;

    parser_status_t parser_status = parser[i].getNextByte();

    // If we got a byte, move the timestamp forwards to stop
    // triggering a desaturation
    if ( parser_status.bytes > 0 ) setByteTimestamp( i );

    // Log any activity
    if ( parser_status.bytes > 0 ) {

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
  // In broadcast mode, only settings for 
    // channel[0] are used and applied to all
    if( config.tx[0].base_ms == 0 ) return;

    // Check if it is time to transmit.
    uint32_t dt_ms;
    dt_ms = millis() - metrics.msg_timings.ts_ms[0]; 

    if( dt_ms > config.tx[0].interval_ms ) {

      
      // start the send process
      triggerTx(0);

      // Update the timing interval_ms for
      // the next transmit operation
      updateTimingInterval( 0 );
       
    }
}

void updateTimingInterval( int which ) {
  
}

void handleTx( int which ) {
  
}

void handleTransmit() {

  // First, simply update all channel sending
  // states, which in most cases means doing 
  // nothing
  for( int i = 0; i < 4; i++ ) {
    updateTx(i);
  }

  // First, are we in broadcast mode or not?
  if( config.general.flags.bits.broadcast ) {

    // If we're in the middle of a send, abort
    // attempting to send anything again
    if( channel[0].tx_state == TxState::Sending ) return;

    // Else, hand over this operation
    handleTxBroadcast();
     
  } else {
    
    for( int i = 0; i < 4; i++ ) {

      // If currently sending, avoid sending
      // again.
      if( channel[i].tx_state == TxState::Sending ) continue;
      
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





void i2c_receive( int len ) {

}

void i2c_request() {

}
