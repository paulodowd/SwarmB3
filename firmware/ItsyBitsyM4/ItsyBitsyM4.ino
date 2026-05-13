/* To do:
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





void setup() {

  Serial.begin(115200);

  // Paul: debugging 10/05/26
//  while (!Serial);
//  Serial.println("Reset");


  randomSeed( generateRandomSeed() );


  pinMode( DEMOD1_EN_PIN, OUTPUT);
  pinMode( DEMOD2_EN_PIN, OUTPUT);
  pinMode( DEMOD3_EN_PIN, OUTPUT);
  pinMode( DEMOD4_EN_PIN, OUTPUT);

  // 58 kHz output on D4
  setup58kHz();

  fullReset();

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



  Wire.begin(IRCOMM_I2C_ADDR);
  Wire.onReceive( i2c_receive );
  Wire.onRequest( i2c_request );

  //  // Paul: debugging 10/05/26
//  for ( int i = 0; i < 4; i++ ) {
//    config.tx[0].repeat = UINT32_MAX;
//    config.tx[0].preamble_repeat = 0;
//  }
//  config.general.flags.bits.broadcast = 1;
//  setTestMessage(4);
//    Serial.println("Setup complete");

}

uint32_t generateRandomSeed() {
  uint32_t seed = 0x00000000;
  pinMode(A3, INPUT);

  // Generate an unsigned long as a seed by reading
  // the lsb of an analogRead.
  for ( int i = 0; i < 32; i++ ) {
    uint32_t sample = (uint32_t)analogRead(A3);
    sample = (sample & 0x00000001) << i;
    seed |= sample;
    delay(1);
  }
  return seed;
}

void setTestMessage(int len) {

  char msg[34];
  memset( (char*)msg, 0, sizeof( msg ));
  for ( int j = 0; j < len; j++ ) {

    char c;
    do {
      c = (char)random(0, 255);
    } while ( c == '~' || c == '^' );
    msg[j] = c;
  }
  for ( int i = 0; i < 4; i++ ) {
    memset( (uint8_t*)config.tx_buf[i], 0, sizeof( config.tx_buf[i] ));
    config.tx[i].len = parser[i].formatIRMessage( (uint8_t*)config.tx_buf[i], (uint8_t*)msg, strlen(msg));
  }


}


void loop() {
  handleI2cFlags();
  handleMsgParsing();
  handleDemodulatorSaturation();
  handleBearingEstimation();
  handleTransmit();

}
