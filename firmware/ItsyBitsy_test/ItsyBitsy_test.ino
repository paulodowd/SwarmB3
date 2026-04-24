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

#define BAUD 9600


// Match up 4 instances of the ir parsrer.
//                          right            back         fwd         left
IRParser_c parser[4] = { port_D12_D13, port_D25_D24, port_D18_D15, port_D1_D0 };


typedef struct {
  ir_crc_t          crc;
  ir_activity_t     activity;
  ir_saturation_t   saturation;
  ir_errors_t       errors;
  ir_frame_errors_t frame_errors;
  ir_msg_timings_t  msg_timings;
  ir_byte_timings_t  byte_timings;
  ir_vectors_t      vectors;
  ir_bearing_t      bearing;
  ir_sensors_t      sensors;
} ircomm_metrics_t;
ircomm_metrics_t metrics;


uint32_t bearing_ts;
const uint32_t bearing_update_ms = 100;
float bearing_activity[4];

// A record of byte activity per
// receiver which is periodically
// reset to 0.  Allows for the
// estimation of bearing to other
// transmitting boards/robots.
unsigned long tx_ts;     // periodic transmit
unsigned long led_ts;    // LED time stamp


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
typedef struct { // 31 bytes
  ir_tx_params_t  tx[4];          // 11 bytes
  ir_rx_params_t  rx[4];          // 20 bytes
} ircomm_config_t;
ircomm_config_t config;

volatile byte tx_buf[4][MAX_TX_BUF];




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


void setup() {

  Serial.begin(115200);
  while (!Serial);
  Serial.println("Reset");

  //  while (!Serial) {
  //  }

  pinMode( DEMOD1_EN_PIN, OUTPUT);
  pinMode( DEMOD2_EN_PIN, OUTPUT);
  pinMode( DEMOD3_EN_PIN, OUTPUT);
  pinMode( DEMOD4_EN_PIN, OUTPUT);

  digitalWrite( DEMOD1_EN_PIN, HIGH);
  digitalWrite( DEMOD2_EN_PIN, HIGH);
  digitalWrite( DEMOD3_EN_PIN, HIGH);
  digitalWrite( DEMOD4_EN_PIN, HIGH);

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

  // 58 kHz output on D4
  setup58kHz();

  memset( (void*)&metrics, 0, sizeof( metrics));
  setAllByteTimestamps();
  setAllMsgTimestamps();
  setBearingTimestamp();

  //  Serial.println("Setup complete");
}


void setBearingTimestamp() {
  bearing_ts = millis();
}
uint32_t calcBearingDeltaTime() {
  return millis() - bearing_ts;
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

void triggerDemodDesaturation(int which) {
  if ( which < 0 || which > 3 ) return;
  channel[which].demod_state = DemodState::Deactive;
  channel[which].demod_desat_ts = micros();
  digitalWrite( channel[which].demod_pin, LOW); // switch off demod
}

bool updateDemodDesaturation( int which ) {
  if ( which < 0 || which > 3 ) return false;

  if ( channel[which].demod_state == DemodState::Active ) { // nothing to do
    return true;
  }

  uint32_t dt_us = micros() - channel[which].demod_desat_ts;
  if ( dt_us > 2000 ) { // 20ms
    // renable demodulator
    channel[which].demod_state = DemodState::Active;
    digitalWrite( channel[which].demod_pin, HIGH );
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
  const float bytes_per_ms = 960.0 / 1000.0;
  const float max_bytes = bytes_per_ms * bearing_update_ms;


  metrics.bearing.sum  = 0.0;
  for ( int i = 0; i < 4; i++ ) {

    bearing_activity[i] /= max_bytes;
    metrics.bearing.sum += bearing_activity[i];

    // filter this activity,
    if ( bearing_activity[i] > 0.0 ) {

      // Normalising and filtering
      metrics.vectors.rx[i] = (metrics.vectors.rx[i] * 0.3 ) + ((bearing_activity[i]) * 0.7);

    }

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


unsigned long tx_test;
void loop() {

  for ( int i = 0; i < 4; i++ ) {


    parser_status_t parser_status = parser[i].getNextByte();

    // If we got a byte, move the timestamp forwards to stop
    // triggering a desaturation
    if ( parser_status.bytes > 0 ) setByteTimestamp( i );

    // If there has been no activity for some time, the
    // demodulator is probably saturated (gain at max) so
    // we trigger a desaturation.  Otherwise, just check
    // whether the demodulator needs reactivating
    if ( calcByteDeltaTime(i) > 30000 ) { // 30ms
      triggerDemodDesaturation( i );
      metrics.saturation.rx[i]++;
      setByteTimestamp(i);
    } else {
      updateDemodDesaturation( i ); // will take 20ms to complete
    }

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

    // Got a message
    if ( parser_status.bytes > 1 ) {

      calcMsgDeltaTime(i);
      setMsgTimestamp(i);

      metrics.crc.pass[i]++;

      Serial.print(millis());
      Serial.print(",");
      //      Serial.print("Port "); Serial.print(i); Serial.print(": ");
      //      Serial.print( (char*)parser[i].msg );
      Serial.print( bearing_activity[0] );
      Serial.print(",");
      Serial.print( metrics.activity.rx[0] );
      //      Serial.print("(");
      //      Serial.print( parser_status.bytes );
      //      Serial.print(" bytes), fe: ");
      //      Serial.print( *(channel[i].frame_errors) );
      //      Serial.print(" sat: ");
      //      Serial.println( metrics.saturation.rx[i]);
      Serial.println();
    } else if ( parser_status.bytes == 1 ) {
      //      Serial.print("Port "); Serial.print(i); Serial.print(", error: "); Serial.println( parser_status.error );

    } else if ( parser_status.bytes == 0 ) {


    }
  }



  if ( calcBearingDeltaTime() > bearing_update_ms ) {
    setBearingTimestamp();
    updateBearing();
    Serial.println( metrics.bearing.sum, 4 );
  }




  //  for ( int i = 0; i < 4; i++ ) disableSercomRx( channel[i].hw );
  //
  //  for ( int i = 0; i < 4; i++ ) {
  //    char buf[MAX_TX_BUF];
  //    char msg[MAX_TX_BUF];
  //    memset(msg, 0, sizeof( buf ));
  //    memset(buf, 0, sizeof( buf ));
  //    sprintf(msg, "port%d", i);
  //    int len = parser[i].formatIRMessage( (uint8_t*)buf, (uint8_t*)msg, 5);
  //    //Serial.print("Going to tx: "); Serial.println( buf );
  //    channel[i].port->write( buf, len );
  //    channel[i].port->flush();
  //  }
  //
  //  for ( int i = 0; i < 4; i++ ) {
  //    while ( channel[i].port->available() ) channel[i].port->read();
  //    enableSercomRx( channel[i].hw );
  //
  //  }

}



void i2c_receive( int len ) {

}

void i2c_request() {

}
