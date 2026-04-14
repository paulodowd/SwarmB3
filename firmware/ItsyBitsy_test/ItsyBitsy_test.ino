#include <Arduino.h>
#include <wiring_private.h>
#include <Wire.h>

//#include "sam.h" // for SAMD51 registers

#include "sercom_funcs.h"
#include "ir_parser.h"
#include "ircomm_i2c.h"

// GPIO mappings
#define DEMOD1_EN_PIN 11      // Serial1
#define DEMOD2_EN_PIN 10      // SerialA4
#define DEMOD3_EN_PIN 9       // SerialD12
#define DEMOD4_EN_PIN 23      // SerialSPI
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

// A record of byte activity per
// receiver which is periodically
// reset to 0.  Allows for the
// estimation of bearing to other
// transmitting boards/robots.
float bearing_activity[4];
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

  //             tx    rx
  //    Serial.begin(115200);
  //    while (!Serial);
  //  Serial.println("Reset");

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

  //  pinPeripheral(0, PIO_SERCOM);
  //  pinPeripheral(1,  PIO_SERCOM);
  //  port_D1_D0.begin(BAUD);
  //
  //  // UART4: MOSI TX / SCK RX -> SERCOM1 (SCK is 24, MOSI is 25)
  //  pinPeripheral(24, PIO_SERCOM_ALT);
  //  pinPeripheral(25,  PIO_SERCOM_ALT);
  //  port_D25_D24.begin(BAUD);
  //
  //  // UART3: D12 TX / D13 RX -> SERCOM5
  pinPeripheral(12, PIO_SERCOM_ALT);
  pinPeripheral(13, PIO_SERCOM_ALT);
  port_D12_D13.begin(BAUD);
  //
  //  pinMode( 12, OUTPUT);
  //  digitalWrite(12, LOW);
  pinMode( 25, OUTPUT);
  digitalWrite(25, LOW);
  pinMode( 1, OUTPUT);
  digitalWrite(1, LOW);
  //  pinMode( A4, OUTPUT );
  //  digitalWrite( A4, LOW );
  //
  beginSerialA4A1_manual(BAUD);
//  configureSercom8N1(SERCOM0);

  // 58 kHz output on D4
  setup58kHz();


  #define INVERT false
  
  //  configureSercomInvert(SERCOM3, false, false); // Serial1 on SERCOM3
  configureSercomInvert(SERCOM0, INVERT, false); // SerialA4
  configureSercomInvert(SERCOM5, INVERT, false); // SerialD12
  //  configureSercomInvert(SERCOM1, false, false); // SerialSPI

  //  resetAllFrameErrorCounts();

  // I2C: default Wire on SDA/SCL -> SERCOM2
  //  Wire.begin();
  //  Wire.setClock(400000);

  //  Wire.onReceive( i2c_receive );
  //  Wire.onRequest( i2c_request );


  // Analog inputs
  //  pinMode( LDRA_IN_PIN, INPUT );
  //  pinMode( LDRB_IN_PIN, INPUT );
  //  pinMode( LDRC_IN_PIN, INPUT );
  //  pinMode( PROXA_IN_PIN, INPUT );
  //  pinMode( PROXB_IN_PIN, INPUT );

  //  Serial.println("Setup complete");
}

unsigned long tx_test;

void loop() {
  //  for ( int i = 2; i < 3; i++ ) {
  char buf[MAX_TX_BUF];
  char msg[MAX_TX_BUF];
  memset(msg, 0, sizeof( buf ));
  memset(buf, 0, sizeof( buf ));

  for ( int i = 0; i < 10; i++ ) buf[i] = 0x6A;
  //  buf[10] = 'p';
  //  buf[11] = 'a';
  //  buf[12] = 'u';
  //  buf[13] = 'l';
  //sprintf( msg, "tx%d-test", 0 );
  //int l = parser[ 0 ].formatIRMessage( (uint8_t*)buf, (uint8_t*)msg, strlen(msg) );
  //    uartSendAndRelease( channel[2], (uint8_t*)buf, l);
  uartSendAndRelease( port_D18_D15, SERCOM0,  18, (uint8_t*)buf, 10, INVERT);
  //  }
//  port_D18_D15.write(buf, 10);
  delay(20);
}

void loop2() {
  //    while (Serial.available()) {
  //      SerialA4A5.write(Serial.read());
  //    }

  //
  //  //  Serial.print("*** "); Serial.print(millis()); Serial.println(" ***");
  for ( int i = 0; i < 4; i++ ) {
    int retval = parser[i].getNextByte();
    if ( retval > 1 ) {
      Serial.print(millis());
      Serial.print(" ");
      Serial.print("Port "); Serial.print(i); Serial.print(": ");
      Serial.print( (char*)parser[i].msg );
      Serial.print("(");
      Serial.print(retval);
      Serial.println(" bytes)");
    }


  }

  // Time to transmit?
  if ( millis() - tx_test > 20 ) {
    tx_test = millis();
    for ( int i = 0; i < 4; i++ ) {
      if ( channel[i].tx_phase == TxReleasePhase::Idle ) {


        //      Serial.println("Doing Send!");

        char buf[MAX_TX_BUF];
        char msg[MAX_TX_BUF];
        memset(msg, 0, sizeof( buf ));
        memset(buf, 0, sizeof( buf ));
        sprintf( msg, "tx%d-test", i );
        int l = parser[ i ].formatIRMessage( (uint8_t*)buf, (uint8_t*)msg, strlen(msg) );
        uartSendAndReleaseAsync( channel[ i ], (uint8_t*)buf, l);
      }
    }
  }

  for ( int i = 0; i < 4; i++ ) {
    // Can check when tx is finished here
    if ( channel[i].tx_release_done ) {

      // Reset flag.
      channel[i].tx_release_done = false;
    }
  }


  //  delay(4);
}



void i2c_receive( int len ) {

}

void i2c_request() {

}
