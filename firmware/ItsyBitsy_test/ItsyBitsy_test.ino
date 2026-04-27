/* To do:
    - implement i2c receive/request calls.
      - decide if we're going to mutex
      - remember to populate frame errors
*/



#include <Arduino.h>
#include <wiring_private.h>
#include <Wire.h>

//#include "sam.h" // for SAMD51 registers
#include "ircomm.h"
#include "sercom_funcs.h"
#include "ir_parser.h"
#include "ircomm_i2c.h"

// GPIO mappings
#define PROXA_IN_PIN  PIN_PA02
#define PROXB_IN_PIN  PIN_PB08
#define LDRA_IN_PIN   PIN_PB09
#define LDRB_IN_PIN   PIN_PA06
#define LDRC_IN_PIN   PIN_PA07


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
  //  while (!Serial);
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

  // Enable demodulators as specified
  // in config.h
  for ( int i = 0; i < 4; i++ ) {
    digitalWrite( channel[i].demod_pin, config.rx[i].flags.bits.enabled == 1 ? HIGH : LOW );
  }

  // Activate SERCOM units. Note, slightly
  // different config for each.
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

  // Had trouble with this one!
  beginSerialA4A1_manual(BAUD);


  resetMetrics();

  //  Serial.println("Setup complete");
}

void setTestMessage() {
  for ( int i = 0; i < 4; i++ ) {
    char msg[32];
    memset( (void*)msg, 0, sizeof( msg ));
    memset( (void*)tx_buf[i], 0, sizeof( tx_buf[i] ));
    sprintf((char*)msg, "test %d, %lu", i, micros() );
    config.tx[i].len = parser[i].formatIRMessage( (uint8_t*)tx_buf[i], (uint8_t*)msg, strlen(msg));
  }

}

static unsigned long test_tx;
void loop() {

  if ( millis() - test_tx > 100 ) {

    test_tx = millis();
  }

  handleMsgParsing();
  handleDemodulatorSaturation();
  handleBearingEstimation();
  handleTransmit();

}
