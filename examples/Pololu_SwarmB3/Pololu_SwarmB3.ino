#include <Wire.h>
#include "ircomm_i2c_datatypes.h"
#include "SwarmB3.h"

SwarmB3_c SwarmB3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
//  while (!Serial);
//  Serial.println("Reset");

  Wire.begin();
  Wire.setClock(400000);
  delay(10);

  unsigned long s;
  unsigned long e;


  SwarmB3.init();
  delay(10);
  for ( int i = 0; i < 4; i++ ) {
    SwarmB3.printRxSettings(i);
  }

  for ( int i = 0; i < 4; i++ ) {
    SwarmB3.printTxSettings(i);
  }

  

}

unsigned long test_ts;
void loop() {

  if ( millis() - test_ts > 250 ) {
    test_ts = millis();

    char buf[32];
    for ( int i = 0; i < 4; i++ ) {
      memset(buf, 0, sizeof(buf));
      sprintf((char*)buf, "robot2 tx%d %lu", i, millis());
      SwarmB3.setIRMessage( (uint8_t*)buf, strlen(buf), i);
      delay(1);
    }

  }

//  ir_status_t status = SwarmB3.getStatus();
//  SwarmB3.printStatus();

    SwarmB3.printAnyMessage();

  delay(10);
}
