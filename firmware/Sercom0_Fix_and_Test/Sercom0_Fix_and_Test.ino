#include <Arduino.h>
#include <wiring_private.h>
#include <Wire.h>

#pragma pack(push,1)

#define ROBOT_I2C_ADDR    0x08

// Flag values to change request context
#define REQUEST_NULL        0
#define REQUEST_SURFACE     1
#define REQUEST_BUMP        2
#define REQUEST_POSE        3
#define SET_POSE            4
#define SET_MOTORS          5
#define SET_BUZZER          6
#define SET_SENSOR_CONFIG   7

//
//
// Size: 1 byte
typedef struct {
  uint8_t value;
} RequestType_t;

//
//
// Size: 10 byte
typedef struct {
  uint16_t  reading[5];
} RobotSurface_t;


// Size: 2 byte
#define EMIT_LINE     0
#define EMIT_BUMP     1
#define EMIT_OFF      2
#define MODE_MICROS   0
#define MODE_ADC      1
typedef struct {
  uint8_t   mode;
  uint8_t   emit;
} RobotSensorConfig_t;

#pragma pack(pop);

// RX first, TX second
Uart SerialA4A5(&sercom0, PIN_A1, PIN_A4, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// UART4: MOSI TX / SCK RX -> SERCOM1
Uart SerialSPI(&sercom1, PIN_SPI_SCK, PIN_SPI_MOSI, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// UART3: D12 TX / D13 RX -> SERCOM5
Uart SerialD12(&sercom5, 13, 12, SERCOM_RX_PAD_1, UART_TX_PAD_0);


void SERCOM0_0_Handler() {
  SerialA4A5.IrqHandler();
}
void SERCOM0_1_Handler() {
  SerialA4A5.IrqHandler();
}
void SERCOM0_2_Handler() {
  SerialA4A5.IrqHandler();
}
void SERCOM0_3_Handler() {
  SerialA4A5.IrqHandler();
}


void SERCOM1_0_Handler() {
  //  countFrameError( SERCOM1, SerialSPI_frame_errors);
  SerialSPI.IrqHandler();
}
void SERCOM1_1_Handler() {
  //  countFrameError( SERCOM1, SerialSPI_frame_errors);
  SerialSPI.IrqHandler();
}
void SERCOM1_2_Handler() {
  //  countFrameError( SERCOM1, SerialSPI_frame_errors);
  SerialSPI.IrqHandler();
}
void SERCOM1_3_Handler() {
  //  countFrameError( SERCOM1, SerialSPI_frame_errors);
  SerialSPI.IrqHandler();
}



void SERCOM5_0_Handler() {
  //  countFrameError( SERCOM5, SerialD12_frame_errors);
  SerialD12.IrqHandler();
}
void SERCOM5_1_Handler() {
  //  countFrameError( SERCOM5, SerialD12_frame_errors);
  SerialD12.IrqHandler();
}
void SERCOM5_2_Handler() {
  //  countFrameError( SERCOM5, SerialD12_frame_errors);
  SerialD12.IrqHandler();
}
void SERCOM5_3_Handler() {
  //  countFrameError( SERCOM5, SerialD12_frame_errors);
  SerialD12.IrqHandler();
}

static void beginSerialA4A5_manual(uint32_t baud)
{
  // A4 = TX = PAD0
  // A5 = RX = PAD2
  pinPeripheral(PIN_A4, PIO_SERCOM_ALT);
  pinPeripheral(PIN_A1, PIO_SERCOM_ALT);

  sercom0.initUART(UART_INT_CLOCK, SAMPLE_RATE_x16, baud);
  sercom0.initFrame(UART_CHAR_SIZE_8_BITS,
                    LSB_FIRST,
                    SERCOM_NO_PARITY,
                    SERCOM_STOP_BIT_1);
  sercom0.initPads(UART_TX_PAD_0, SERCOM_RX_PAD_1);
  sercom0.enableUART();
}


// -----------------------------------------------------------------------------
// 58 kHz clock on D7 using TCC1
// D7 = PA18 = TCC1/WO[2]
// -----------------------------------------------------------------------------
static void setup58kHz() {
  // D7 on ItsyBitsy M4 = PA18
  // PA18 can be muxed to TCC1/WO[2]

  // Configure PA18 for peripheral output
  PORT->Group[0].DIRSET.reg = (1 << 18);
  PORT->Group[0].PMUX[18 >> 1].bit.PMUXE = MUX_PA18F_TCC1_WO2;
  PORT->Group[0].PINCFG[18].bit.PMUXEN = 1;

  // Enable bus clock for TCC1
  MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC1;

  // Route GCLK0 to TCC1
  // On the ItsyBitsy M4, the board runs at 120 MHz. If GCLK0 is 120 MHz,
  // this gives the frequency calculation below.
  GCLK->PCHCTRL[TCC1_GCLK_ID].reg =
    GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
  while (!(GCLK->PCHCTRL[TCC1_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));

  // Disable TCC1 before configuration
  TCC1->CTRLA.bit.ENABLE = 0;
  while (TCC1->SYNCBUSY.bit.ENABLE);

  // Prescaler = 1
  TCC1->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1;

  // Normal PWM mode
  TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  while (TCC1->SYNCBUSY.bit.WAVE);

  // 120 MHz / (2068 + 1) = 57,999 Hz
  TCC1->PER.reg = 2068;
  while (TCC1->SYNCBUSY.bit.PER);

  // ~50% duty cycle
  TCC1->CC[2].reg = 1034;
  while (TCC1->SYNCBUSY.bit.CC2);

  // Enable TCC1
  TCC1->CTRLA.bit.ENABLE = 1;
  while (TCC1->SYNCBUSY.bit.ENABLE);
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Start");

  // UART1: built-in Serial1 on D1/D0
  Serial1.begin(9600);


  // UART4: MOSI TX / SCK RX -> SERCOM1 (SCK is 24, MOSI is 25)
  pinPeripheral(PIN_SPI_MOSI, PIO_SERCOM_ALT);
  pinPeripheral(PIN_SPI_SCK,  PIO_SERCOM_ALT);
  SerialSPI.begin(9600);

  // UART3: D12 TX / D13 RX -> SERCOM5
  pinPeripheral(12, PIO_SERCOM_ALT);
  pinPeripheral(13, PIO_SERCOM_ALT);
  SerialD12.begin(9600);

  beginSerialA4A5_manual(9600);


  //  Wire.begin();
  //  Wire.setClock(100000);
  //
  //    Wire.onReceive( i2c_receive );
  //    Wire.onRequest( i2c_request );

  Serial.println("SERCOM0 ready");
  SerialA4A5.println("Hello from SERCOM0 A4/A5");

  //  setup58kHz();

  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(23,OUTPUT);
  
  digitalWrite(9,LOW);    // D12
  digitalWrite(10,HIGH);  
  digitalWrite(11,LOW);   // S1
  digitalWrite(23,LOW);   // SPI

  delay(100);
  //  setSensorConfig( 0, 0 );
}


void setSensorConfig( uint8_t mode, uint8_t emit ) {

  //  if ( mode > ADC ) return;
  //  if ( emit > EMIT_OFF ) return;

  RequestType_t request;
  request.value = SET_SENSOR_CONFIG;
  Wire.beginTransmission( ROBOT_I2C_ADDR );
  Wire.write( (byte*)&request, sizeof( request ));
  Wire.endTransmission();

  RobotSensorConfig_t new_config;
  new_config.emit = emit;
  new_config.mode = mode;

  Wire.beginTransmission( ROBOT_I2C_ADDR );
  Wire.write( (byte*)&new_config, sizeof( new_config ));
  Wire.endTransmission();
}
//
//void i2c_receive( int len ) {
//
//
//}
//
//void i2c_request( ) {
//
//}

static unsigned long update_ts;
static unsigned long i2c_ts;
void loop() {
  //  while (Serial.available()) {
  //    SerialA4A5.write(Serial.read());
  //  }


  if ( millis() - update_ts > 500 ) {
    update_ts = millis();

    Serial.println("A4: ");
    while (SerialA4A5.available()) {
      Serial.print((char)SerialA4A5.read());
    }

    Serial.println("\nSPI: ");
    while ( SerialSPI.available() ) {
      Serial.print( (char)SerialSPI.read() );
    }

    Serial.println("\nD12: ");
    while ( SerialD12.available() ) {
      Serial.print( (char)SerialD12.read() );
    }

    Serial.println("\nS1: ");
    while ( Serial1.available() ) {
      Serial.print( (char)Serial1.read() );
    }

    Serial.println("\n******\n");
//
//    SerialA4A5.print("A4 ");
//    SerialA4A5.println(millis());
//
//
    SerialSPI.print("SPI ");
    SerialSPI.println(millis());
//
//
//    SerialD12.print("D12 ");
//    SerialD12.println(millis());
//
//    Serial1.print("S1 ");
//    Serial1.println(millis());
  }

  if ( millis() - i2c_ts > 100 ) {
    i2c_ts = millis();
    //    getSurfaceSensors();
  }
}

void getSurfaceSensors() {
  RobotSurface_t surface;
  memset( &surface, 0, sizeof( surface ));
  RequestType_t request;
  request.value = REQUEST_SURFACE;
  Wire.beginTransmission( ROBOT_I2C_ADDR );
  Wire.write( (byte*)&request, sizeof( request ));
  Wire.endTransmission();

  Wire.requestFrom( ROBOT_I2C_ADDR, sizeof( surface ));
  Wire.readBytes((byte*)&surface, sizeof( surface ) );

  for ( int i = 0; i < 5; i++ ) {
    Serial.print( surface.reading[i] );
    Serial.print(",");

  }
  Serial.println();

}
